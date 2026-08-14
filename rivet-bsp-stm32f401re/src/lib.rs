//! Rivet RTOS board support: STM32F401RE Nucleo-64 (Cortex-M4), real
//! hardware — ST-LINK/V2-1 on-board debugger (SWD flashing/reset) and its
//! USB CDC-ACM virtual COM port, wired directly to USART2 (PA2 TX / PA3
//! RX) at the hardware level, so "the board's console" and "the terminal
//! you get by opening the ST-LINK's `/dev/ttyACM*`" are the same physical
//! UART — no semihosting, no separate debug transport for output.
//!
//! Runs at the reset-state HSI clock (16 MHz, no PLL configured) — same
//! philosophy as `rivet-bsp-lm3s6965`: don't touch RCC beyond what's
//! actually needed (USART2/GPIOA clock enables), so the board's timing is
//! whatever a freshly-reset chip actually does, not a value that silently
//! stops matching reality if some other init step changes.
//!
//! Unlike the RISC-V boards this workspace ported before it (QEMU virt,
//! the ESP32-C3/S3/C6), Cortex-M's tick/reschedule/IRQ mechanisms
//! (SysTick, PendSV, NVIC) are architecturally fixed — `rivet-arch-
//! cortex-m`'s `systick`/`nvic` modules already implement all of it
//! generically, watchdog-on-tick included. This board only needs to
//! supply: USART2 console, IWDG watchdog, GPIO clock/alt-function setup
//! for the two console pins, and the board IRQ number map.

#![no_std]

pub mod gpio;
pub mod i2c;

/// Board IRQ number map (plan.md Phase 13/26 follow-up): which NVIC IRQ
/// number is which peripheral, per the `stm32f401` PAC's own `Interrupt`
/// enum (cross-checked, not guessed).
pub mod irq {
    pub const USART2: u32 = 38;
    /// I2C1 event interrupt (`SB`/`ADDR`/`BTF`/`TXE`/`RXNE`) — position
    /// 31 in the `stm32f401` PAC's `Interrupt` enum.
    pub const I2C1_EV: u32 = 31;
    /// I2C1 error interrupt (`AF`/`BERR`/`ARLO`/`OVR`) — position 32.
    pub const I2C1_ER: u32 = 32;
}

// Everything below is genuinely Cortex-M specific (real asm!/naked_asm!
// blocks transitively, via rivet-arch-cortex-m) and target-gated so
// `cargo test -p rivet-bsp-stm32f401re` on host still type-checks the
// rest of the crate — mirrors rivet-bsp-lm3s6965's own split.
#[cfg(target_arch = "arm")]
mod board {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Reset-state HSI clock — no PLL/RCC clock-tree configuration is
    /// done anywhere in this crate, so this is simply what the chip
    /// actually runs at from power-on.
    const SYSCLK_HZ: u32 = 16_000_000;

    fn uart_irq_handler() {
        // SAFETY: fixed USART2 register block on this chip.
        let usart2 = unsafe { &*stm32f4::stm32f401::USART2::ptr() };
        loop {
            let sr = usart2.sr().read();
            let cr1 = usart2.cr1().read();
            // `SR.TXE`/`SR.RXNE` are *raw* level flags — unlike the
            // PL011's `MIS` (already ANDed with `IMSC`) or the CMSDK
            // UART's write-1-to-clear `INTSTATUS`, this peripheral has no
            // masked-status register at all. `TXE` in particular is
            // cleared only by writing `DR`, never by masking `TXEIE` —
            // so checking the raw bit alone here means the instant the TX
            // ring goes empty, this loop takes the `None` arm, clears
            // `TXEIE`, re-reads `SR`, finds `TXE` still set, and takes
            // `None` again — forever. Root-caused on real hardware: this
            // ISR runs at priority 0xFF (same as PendSV/SysTick, an
            // equal-priority exception can't preempt an already-running
            // one), so the spin doesn't just stall the console, it stalls
            // *everything* — no tick, no context switch, no fault, a
            // silent total freeze indistinguishable from a scheduler bug
            // until caught by disassembling the wedged PC. ANDing with
            // the enable bit (which *does* correctly reflect "this ISR
            // still owns the condition") is what the PL011/CMSDK masked-
            // status reads were already doing implicitly.
            if sr.txe().bit_is_set() && cr1.txeie().bit_is_set() {
                match rivet::console::tx_irq_next_byte() {
                    Some(b) => {
                        // SAFETY: writing DR clears TXE itself (the FIFO
                        // now has data) — no separate ack step needed.
                        usart2.dr().write(|w| unsafe { w.dr().bits(b as u16) });
                    }
                    None => {
                        usart2.cr1().modify(|_, w| w.txeie().clear_bit());
                    }
                }
            } else if sr.rxne().bit_is_set() && cr1.rxneie().bit_is_set() {
                // SAFETY: reading DR clears RXNE.
                let b = usart2.dr().read().dr().bits() as u8;
                rivet::console::on_rx_byte(b);
            } else {
                break;
            }
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_console_kick_tx() {
        // SAFETY: fixed USART2 register block.
        unsafe {
            (&*stm32f4::stm32f401::USART2::ptr())
                .cr1()
                .modify(|_, w| w.txeie().set_bit());
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_init() {
        // SAFETY: fixed RCC/GPIOA/USART2 register blocks; runs once, at
        // the very start of board init, before anything else touches
        // these peripherals.
        unsafe {
            let rcc = &*stm32f4::stm32f401::RCC::ptr();
            rcc.ahb1enr().modify(|_, w| w.gpioaen().set_bit());
            rcc.apb1enr().modify(|_, w| w.usart2en().set_bit());

            let gpioa = &*stm32f4::stm32f401::GPIOA::ptr();
            // PA2 (TX) / PA3 (RX) -> alternate function mode, AF7
            // (USART2) — Nucleo-64's own fixed ST-LINK VCP wiring, not a
            // pick this crate makes.
            gpioa.moder().modify(|_, w| w.moder2().bits(0b10).moder3().bits(0b10));
            gpioa.afrl().modify(|_, w| w.afrl2().bits(7).afrl3().bits(7));

            let usart2 = &*stm32f4::stm32f401::USART2::ptr();
            // 115200 8N1 at 16 MHz PCLK1 (no PLL configured, APB1
            // prescaler is 1 at reset): BRR = fPCLK / baud, oversampling
            // by 16 (OVER8 left clear) — mantissa/fraction split per the
            // reference manual's own worked example shape.
            let usartdiv = (SYSCLK_HZ * 100) / (16 * 115_200); // x100 fixed-point
            let mantissa = usartdiv / 100;
            let fraction = ((usartdiv % 100) * 16 + 50) / 100; // round to nearest /16th
            usart2
                .brr()
                .write(|w| w.div_mantissa().bits(mantissa as u16).div_fraction().bits(fraction as u8));
            usart2.cr1().write(|w| w.ue().set_bit().te().set_bit().re().set_bit());

            // `register_untraced`, not `register`: this ISR drains the
            // same UART the trace stream itself rides on (once `trace`
            // routes through `console`'s interrupt-driven TX ring — see
            // `__rivet_board_trace_write`'s own docs) — tracing it would
            // queue more bytes on every invocation, re-arming its own
            // interrupt forever.
            rivet::irq::register_untraced(super::irq::USART2, uart_irq_handler).unwrap();
            rivet::irq::set_priority(super::irq::USART2, 0xFF);
            rivet::irq::enable(super::irq::USART2);
            usart2.cr1().modify(|_, w| w.rxneie().set_bit());
            rivet::console::enable_irq_tx();
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_now_us() -> u64 {
        // Sub-tick precision (see `systick::now_micros_precise`'s own
        // docs): a Rivet Debugger trace timeline is useless if every event
        // between two 1kHz ticks reports the exact same timestamp — this
        // still costs nothing scheduling-critical since `sleep_ms`/
        // deadlines only ever needed whole-tick granularity anyway.
        rivet_arch_cortex_m::systick::now_micros_precise()
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_tick_start(hz: u32) {
        let reload_ticks = SYSCLK_HZ / hz;
        let period_us = 1_000_000 / hz;
        rivet_arch_cortex_m::systick::init(reload_ticks, period_us);
    }

    #[no_mangle]
    unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
        // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port contract.
        let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
        // SAFETY: fixed USART2 register block.
        let usart2 = unsafe { &*stm32f4::stm32f401::USART2::ptr() };
        for &b in bytes {
            while usart2.sr().read().txe().bit_is_clear() {
                core::hint::spin_loop();
            }
            usart2.dr().write(|w| unsafe { w.dr().bits(b as u16) });
        }
    }

    #[cfg(feature = "trace")]
    #[no_mangle]
    unsafe extern "Rust" fn __rivet_board_trace_write(ptr: *const u8, len: usize) {
        // Same physical wire as the console (this board's only easily
        // reachable UART is the ST-LINK's own USART2 VCP — a second UART
        // would need extra wiring this Nucleo doesn't expose by default).
        //
        // Routed through `rivet::console`'s own interrupt-driven TX ring
        // (`__rivet_board_init` already calls `enable_irq_tx()`), NOT a
        // raw polling write — this used to block here directly, byte by
        // byte, for ~87µs/byte at 115200 baud (~1.8ms for a whole frame).
        // Every trace call site in `rivet::trace` fires from inside
        // `preempt::on_tick`, i.e. from *inside the PendSV exception
        // handler* — no Thread-mode code, on any task, runs again until
        // that handler returns. A real, confirmed bug: two equal-priority
        // tasks that round-robin every tick each paid that ~1.8ms
        // *every single dispatch*, on a 1ms tick — strictly more than
        // their entire timeslice, so neither task ever advanced past its
        // own entry point (verified live: `PC` stuck at the function's
        // first instruction after several real seconds of uptime, via
        // GDB, non-invasively). Pushing into the ring is a few instructions
        // (bounded, no hardware wait); the actual bytes drain later via
        // `uart_irq_handler`'s own TXE-driven loop, fully outside this
        // call and outside any interrupt context that matters for
        // scheduling. `trace_demo` never calls `console::write_str`, so
        // nothing text-shaped ever shares this ring with the binary
        // frames — the wire stays pure trace, same as before.
        let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
        rivet::console::write_bytes(bytes);
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_reset() -> ! {
        rivet_arch_cortex_m::system_reset()
    }

    /// No semihosting on this board (see the module docs: the ST-LINK's
    /// USB CDC port *is* the console UART, not a separate debug
    /// transport) — print a marker and halt. A human (or the test
    /// harness) reads the marker directly off the same serial port the
    /// rest of the test's output already went to.
    #[no_mangle]
    extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
        if code == 0 {
            rivet::console::write_str("\nRIVET_EXIT_OK\n");
        } else {
            rivet::console::write_str("\nRIVET_FAILURE code=");
            print_dec(code);
            rivet::console::write_str("\n");
        }
        // Same reasoning as rivet-bsp-lm3s6965's own `__rivet_board_exit`:
        // this print must not depend on an interrupt that may never fire
        // again if this halt happens from a context (e.g. a fault
        // handler) that permanently blocks the TX-ISR.
        rivet::console::flush_sync();
        loop {
            core::hint::spin_loop();
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
        WDT_PERIOD_US.store(period_us, Ordering::Release);
        if period_us == 0 {
            return;
        }
        // SAFETY: fixed IWDG register block; IWDG is clocked by the
        // always-on ~32 kHz LSI RC, independent of any RCC enable bit —
        // it just needs to be started (key 0xCCCC) and programmed.
        unsafe {
            let iwdg = &*stm32f4::stm32f401::IWDG::ptr();
            iwdg.kr().write(|w| w.key().bits(0x5555)); // unlock PR/RLR
            // Prescaler /4 (PR=0): counter clock = 32 kHz / 4 = 8 kHz,
            // giving a 1/8000 s per-tick resolution and a reload range up
            // to RLR's 12-bit max (4095) / 8000 ≈ 511 ms — comfortably
            // covers this workspace's watchdog tests (hundreds of ms).
            iwdg.pr().write(|w| w.pr().bits(0));
            let reload = ((period_us as u64 * 8_000) / 1_000_000).clamp(1, 0xFFF) as u16;
            iwdg.rlr().write(|w| w.rl().bits(reload));
            iwdg.kr().write(|w| w.key().bits(0xAAAA)); // reload from RLR
            iwdg.kr().write(|w| w.key().bits(0xCCCC)); // start counting
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_feed() {
        if WDT_PERIOD_US.load(Ordering::Acquire) != 0 {
            // SAFETY: fixed IWDG KR register — 0xAAAA reloads the
            // countdown from RLR, the documented "feed" sequence.
            unsafe {
                (&*stm32f4::stm32f401::IWDG::ptr()).kr().write(|w| w.key().bits(0xAAAA));
            }
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_check() {
        // No-op: IWDG is a real hardware watchdog, counting down
        // autonomously off the LSI RC — there is no software deadline to
        // check (matches rivet-bsp-lm3s6965's own reasoning).
    }

    static WDT_PERIOD_US: AtomicU32 = AtomicU32::new(0);

    /// The board's `SysTick` exception vector (referenced directly by the
    /// linker script's vector table): bridges to the arch crate's generic
    /// tick mechanism.
    ///
    /// # Safety
    /// Exception entry point; never called directly.
    #[no_mangle]
    pub unsafe extern "C" fn SysTick() {
        rivet_arch_cortex_m::systick::handler();
    }

    fn print_dec(mut n: u32) {
        if n == 0 {
            rivet::console::write_str("0");
            return;
        }
        let mut digits = [0u8; 10];
        let mut i = 0;
        while n > 0 {
            digits[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        let mut buf = [0u8; 10];
        for j in 0..i {
            buf[j] = digits[i - 1 - j];
        }
        if let Ok(s) = core::str::from_utf8(&buf[..i]) {
            rivet::console::write_str(s);
        }
    }
}
