//! Rivet RTOS board support: Raspberry Pi Pico (RP2040), real hardware —
//! Cortex-M0+ (ARMv6-M, see `rivet-arch-cortex-m0`), dual-core (this port
//! only brings up core 0 — see that crate's own docs). Two consoles run
//! simultaneously: a real UART0 (GP0 TX / GP1 RX, needs an external
//! USB-serial adapter or SWD probe to observe) and a USB CDC-ACM virtual
//! serial port over the Pico's own USB connector — the same cable used to
//! flash it. Both receive every byte `rivet::console` writes; either one
//! alone is enough to see the kernel's output. See the `usb` module below for why the
//! CDC device exists at all (the Pico has no onboard USB-serial bridge
//! chip like the Nucleo's ST-LINK) and how it's wired in.
//!
//! # Boot: the boot2 stage
//!
//! Unlike every other Cortex-M board in this workspace, RP2040's boot ROM
//! does not jump straight to a vector table at the start of flash. On
//! cold boot it reads the *first 256 bytes* of flash, checksums them, and
//! — if valid — executes them as a small "stage 2" bootloader whose job
//! is to bring the external QSPI flash chip into fast execute-in-place
//! (XIP) mode (the flash chip itself needs a command sequence to leave
//! its default slow/serial mode) before handing off to the real
//! application. Hand-writing this from scratch would mean re-deriving a
//! flash-chip-specific SPI command sequence AND getting the boot ROM's
//! checksum algorithm exactly right — not a reasonable thing to attempt
//! without real hardware to iterate against, so this crate uses the
//! well-known `rp2040-boot2` crate's pre-built, widely-used blob for the
//! Pico's onboard W25Q080 flash chip (see `boot2.rs`) instead of
//! reinventing it. This is the same category of "don't hand-roll what a
//! focused, correctness-critical crate already gets right" as this
//! workspace's use of `cortex-m`/`stm32f4`/`rp2040-pac` themselves.
//!
//! # Clocks
//!
//! RP2040 resets with `clk_sys` running from the internal ring oscillator
//! (ROSC) — deliberately imprecise (no crystal, drifts with process/
//! voltage/temperature), fine for the boot2 stage but not for a UART baud
//! rate. Unlike this workspace's STM32/LM3S boards (whose reset-state
//! HSI/PIOSC oscillators are factory-trimmed and precise enough to leave
//! alone), RP2040 genuinely needs real clock-tree bring-up: the Pico's
//! 12 MHz crystal (XOSC) as the reference, the system PLL locked to
//! 125 MHz (the standard Pico frequency — `REFDIV=1, FBDIV=125,
//! POSTDIV1=6, POSTDIV2=2`, giving a 1.5 GHz VCO / 12 = 125 MHz, matching
//! every other RP2040 SDK's default), and `clk_sys`/`clk_peri` switched
//! onto it. See `__rivet_board_init`'s own comments for the exact,
//! datasheet-standard sequence.

#![no_std]

pub mod gpio;

/// Board IRQ number map (RP2040 datasheet §2.3.2's fixed NVIC vector
/// order — 26 external IRQ lines, fewer than a typical ARMv7-M chip
/// since ARMv6-M's NVIC only implements what a given chip actually
/// needs).
pub mod irq {
    pub const TIMER_IRQ_0: u32 = 0;
    pub const TIMER_IRQ_1: u32 = 1;
    pub const TIMER_IRQ_2: u32 = 2;
    pub const TIMER_IRQ_3: u32 = 3;
    pub const PWM_IRQ_WRAP: u32 = 4;
    pub const USBCTRL_IRQ: u32 = 5;
    pub const XIP_IRQ: u32 = 6;
    pub const PIO0_IRQ_0: u32 = 7;
    pub const PIO0_IRQ_1: u32 = 8;
    pub const PIO1_IRQ_0: u32 = 9;
    pub const PIO1_IRQ_1: u32 = 10;
    pub const DMA_IRQ_0: u32 = 11;
    pub const DMA_IRQ_1: u32 = 12;
    pub const IO_IRQ_BANK0: u32 = 13;
    pub const IO_IRQ_QSPI: u32 = 14;
    pub const SIO_IRQ_PROC0: u32 = 15;
    pub const SIO_IRQ_PROC1: u32 = 16;
    pub const CLOCKS_IRQ: u32 = 17;
    pub const SPI0_IRQ: u32 = 18;
    pub const SPI1_IRQ: u32 = 19;
    pub const UART0_IRQ: u32 = 20;
    pub const UART1_IRQ: u32 = 21;
    pub const ADC_IRQ_FIFO: u32 = 22;
    pub const I2C0_IRQ: u32 = 23;
    pub const I2C1_IRQ: u32 = 24;
    pub const RTC_IRQ: u32 = 25;
}

/// The stage-2 bootloader blob — see this crate's own module docs. Placed
/// at the very start of flash by `link-rp2040.ld`'s `.boot2` section;
/// `#[used]` because nothing in this crate's own control flow ever reads
/// it (the boot ROM does, before any Rust code runs at all), so it would
/// otherwise look dead to the linker and get stripped.
#[used]
#[link_section = ".boot2"]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[cfg(target_arch = "arm")]
mod board {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// Standard Pico system clock: `REFDIV=1, FBDIV=125, POSTDIV1=6,
    /// POSTDIV2=2` → VCO 1.5 GHz / 12 = 125 MHz — see this crate's own
    /// module docs for the full reasoning. `rp2040_hal::clocks::
    /// init_clocks_and_plls` (used below) hard-codes this exact same
    /// value as `PLL_SYS_125MHZ`, confirming it by construction rather
    /// than by two independent derivations happening to agree.
    const SYSCLK_HZ: u32 = 125_000_000;
    /// `clk_peri` (what the UART's baud generator counts against) is
    /// wired straight to `clk_sys` — see `__rivet_board_init`'s clocks
    /// section.
    const PERI_HZ: u32 = SYSCLK_HZ;

    /// USB CDC-ACM console — see this crate's own module docs for why it
    /// exists (no onboard USB-serial bridge, unlike the Nucleo). Runs
    /// alongside UART0, not instead of it: both receive every byte
    /// `rivet::console` writes.
    mod usb {
        use core::cell::UnsafeCell;

        use rp2040_hal::usb::UsbBus;
        use usb_device::bus::UsbBusAllocator;
        use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
        use usbd_serial::SerialPort;

        // The allocator itself needs `'static` storage — `SerialPort`/
        // `UsbDevice` below hold references into it, not ownership of it.
        // Written exactly once, from `init`, before any interrupt that
        // could observe it is enabled — never touched concurrently.
        #[allow(static_mut_refs)]
        static mut BUS_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;

        struct State {
            device: UsbDevice<'static, UsbBus>,
            serial: SerialPort<'static, UsbBus>,
        }
        // SAFETY: every access goes through `rivet::critical::enter`
        // (PRIMASK masked, reentrant — see `critical.rs`'s own docs), so
        // there is no concurrent access even though this is reached from
        // both task context (`write_best_effort`) and the USBCTRL IRQ
        // (`irq_handler`).
        struct StateCell(UnsafeCell<Option<State>>);
        unsafe impl Sync for StateCell {}
        static STATE: StateCell = StateCell(UnsafeCell::new(None));

        pub fn init(
            regs: rp2040_pac::USBCTRL_REGS,
            dpram: rp2040_pac::USBCTRL_DPRAM,
            usb_clock: rp2040_hal::clocks::UsbClock,
            resets: &mut rp2040_pac::RESETS,
        ) {
            // `force_vbus_detect_bit = true`: this port doesn't configure
            // GP24 as the VBUS-sense input (the Pico's board design wires
            // it there, but using it needs its own GPIO/pad setup this
            // crate doesn't otherwise need) — forcing the controller to
            // treat VBUS as always-present is the standard fallback every
            // minimal RP2040 USB bring-up that skips VBUS sensing uses.
            let bus = UsbBus::new(regs, dpram, usb_clock, true, resets);
            // SAFETY: written exactly once, here, before USBCTRL_IRQ is
            // enabled below — nothing else can observe `BUS_ALLOC` until
            // then.
            #[allow(static_mut_refs)]
            let alloc: &'static UsbBusAllocator<UsbBus> = unsafe {
                BUS_ALLOC = Some(UsbBusAllocator::new(bus));
                BUS_ALLOC.as_ref().unwrap()
            };

            let serial = SerialPort::new(alloc);
            // VID:PID: Raspberry Pi's vendor ID with the generic example
            // product ID every rp2040-hal USB-serial demo uses — not a
            // real product registration, just a stable, recognizable pair
            // for a development board's own debug console.
            let device = UsbDeviceBuilder::new(alloc, UsbVidPid(0x2e8a, 0x000a))
                .strings(&[StringDescriptors::default()
                    .manufacturer("Rivet RTOS")
                    .product("Rivet Debug Console")
                    .serial_number("RIVET-RP2040")])
                .expect("USB string descriptors")
                // Standard IAD device class (0xEF/0x02/0x01) — what a
                // composite CDC-ACM device (separate control + data
                // interfaces) needs for host OS drivers to bind correctly
                // without a custom driver.
                .composite_with_iads()
                .build();

            rivet::critical::enter(|| {
                // SAFETY: guarded by critical::enter.
                unsafe { *STATE.0.get() = Some(State { device, serial }) };
            });

            rivet::irq::register_untraced(super::super::irq::USBCTRL_IRQ, irq_handler).unwrap();
            rivet::irq::set_priority(super::super::irq::USBCTRL_IRQ, 0xFF);
            rivet::irq::enable(super::super::irq::USBCTRL_IRQ);
        }

        fn irq_handler() {
            rivet::critical::enter(|| {
                // SAFETY: guarded by critical::enter.
                if let Some(s) = unsafe { &mut *STATE.0.get() } {
                    s.device.poll(&mut [&mut s.serial]);
                }
            });
        }

        /// Best-effort, non-blocking: `SerialPort::write` copies into the
        /// IN endpoint's buffer and returns immediately (or `WouldBlock`
        /// if that buffer is still occupied by an unacknowledged
        /// transfer) — it never spins waiting for the host to actually
        /// read, so this is safe to call from inside a critical section.
        /// Matches `rivet::console::on_rx_byte`'s own drop-rather-than-
        /// block policy: before enumeration, or if the host isn't
        /// actually reading, bytes are silently dropped rather than
        /// stalling the caller.
        pub fn write_best_effort(bytes: &[u8]) {
            rivet::critical::enter(|| {
                // SAFETY: guarded by critical::enter.
                if let Some(s) = unsafe { &mut *STATE.0.get() } {
                    let _ = s.serial.write(bytes);
                }
            });
            // Poll repeatedly for a bounded stretch of real wall-clock
            // time, outside the critical section between calls — the USB
            // SIE's own hardware state machine is autonomous (doesn't
            // need CPU interrupts to make wire-level progress), so this
            // gives it every chance to actually complete a transfer
            // before this function gives up. Confirmed necessary on real
            // hardware: `USBCTRL_IRQ`'s `BUFF_STATUS` interrupt alone
            // (this module's `irq_handler`, still registered below) never
            // once drove a CDC data transfer to completion in testing —
            // only direct, repeated, synchronous `poll()` calls did, even
            // though the *same* interrupt path correctly completes every
            // enumeration control transfer on EP0. Root cause not fully
            // isolated (a real, open question for whoever revisits this —
            // possibly a `BUFF_STATUS` edge specific to non-zero
            // endpoints on this rp2040-hal version); this bounded
            // synchronous fallback is what actually works, verified live.
            // 200_000 iterations is empirically generous — a diagnostic
            // build looping 10x longer (2,000,000) reliably delivered
            // bytes within a couple of print calls, not near its budget.
            for _ in 0..200_000u32 {
                rivet::critical::enter(|| {
                    // SAFETY: guarded by critical::enter.
                    if let Some(s) = unsafe { &mut *STATE.0.get() } {
                        s.device.poll(&mut [&mut s.serial]);
                    }
                });
                core::hint::spin_loop();
            }
        }
    }

    /// One-time bring-up: clocks (XOSC/PLL_SYS/PLL_USB via `rp2040-hal`
    /// — see its own doc below for why that one piece isn't hand-rolled
    /// like the rest of this file), RESETS for the blocks this crate
    /// drives directly, GPIO mux (UART0 on GP0/GP1, LED on GP25), UART0,
    /// and the USB CDC-ACM console (`usb::init`). Order matters
    /// throughout — see each block's own comment for why.
    #[no_mangle]
    extern "Rust" fn __rivet_board_init() {
        // `Peripherals::take()`, not raw `::ptr()` access, for exactly
        // the handful of blocks `rp2040_hal::clocks::init_clocks_and_plls`
        // and `usb::init` need *owned* (they're written once, at boot,
        // through APIs that want ownership as a way to prove nothing
        // else is concurrently touching the same registers — not a
        // real concurrency concern on this single-hart-at-boot port, but
        // the API shape the well-tested crate exists for isn't worth
        // fighting). Everything else in this function keeps using plain
        // `::ptr()` raw access, same as every other board in this
        // workspace.
        let mut pac = rp2040_pac::Peripherals::take().expect("Peripherals::take() called twice");

        // ── Clocks: XOSC + PLL_SYS + PLL_USB, via rp2040-hal ────────
        // Unlike the rest of this file's hand-rolled register sequences,
        // this one piece uses `rp2040_hal::clocks::init_clocks_and_plls`
        // rather than a hand-derived one: it needs to configure *two*
        // PLLs (PLL_SYS for `clk_sys`, PLL_USB for the 48 MHz `clk_usb`
        // the USB controller requires) and hand back a `UsbClock` token
        // whose only public constructor lives inside `rp2040-hal`'s own
        // clock-management types — there's no way to build one from a
        // hand-rolled register sequence and still satisfy `usb::init`'s
        // `UsbBus::new` call below. Since this call already does
        // everything this function used to hand-roll for `clk_ref`/
        // `clk_sys`/`clk_peri`/the watchdog tick generator too (confirmed
        // by reading its source: identical XOSC startup-delay formula,
        // identical `PLL_SYS_125MHZ` constant, identical
        // `watchdog.enable_tick_generation(xosc_mhz)` call), it fully
        // replaces that code rather than running alongside it.
        let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);
        let clocks = rp2040_hal::clocks::init_clocks_and_plls(
            12_000_000, // Pico's XOSC crystal
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut pac.RESETS,
            &mut watchdog,
        )
        .ok()
        .expect("clock init failed");

        // ── USB CDC-ACM console ──────────────────────────────────────
        usb::init(pac.USBCTRL_REGS, pac.USBCTRL_DPRAM, clocks.usb_clock, &mut pac.RESETS);

        // SAFETY: fixed peripheral base addresses (RP2040's memory map is
        // architected the same on every chip); this runs once, before
        // anything else touches these blocks.
        unsafe {
            let resets = &*rp2040_pac::RESETS::ptr();
            let io_bank0 = &*rp2040_pac::IO_BANK0::ptr();
            let pads_bank0 = &*rp2040_pac::PADS_BANK0::ptr();
            let sio = &*rp2040_pac::SIO::ptr();
            let uart0 = &*rp2040_pac::UART0::ptr();

            // ── RESETS: release what we need out of reset ──────────
            // RESETS.RESET: a set bit means "held in reset" — clear the
            // bits for the blocks this board touches directly (PLL_SYS
            // is already released by `init_clocks_and_plls` above;
            // repeating it here is a harmless no-op, kept for this
            // block's own readability). Everything else stays reset,
            // matching every board in this workspace's "only enable
            // what's actually used" discipline.
            resets.reset().modify(|_, w| {
                w.io_bank0()
                    .clear_bit()
                    .pads_bank0()
                    .clear_bit()
                    .uart0()
                    .clear_bit()
            });
            while resets.reset_done().read().io_bank0().bit_is_clear() {}
            while resets.reset_done().read().pads_bank0().bit_is_clear() {}
            while resets.reset_done().read().uart0().bit_is_clear() {}

            // ── GPIO mux: UART0 on GP0/GP1, LED (SIO) on GP25 ───────
            io_bank0.gpio(0).gpio_ctrl().write(|w| w.funcsel().uart());
            io_bank0.gpio(1).gpio_ctrl().write(|w| w.funcsel().uart());
            io_bank0.gpio(25).gpio_ctrl().write(|w| w.funcsel().sio());
            // GP0 (TX): output driver enabled, input disabled (we never
            // read it). GP1 (RX): input enabled, output disabled.
            pads_bank0.gpio(0).modify(|_, w| w.od().clear_bit().ie().set_bit());
            pads_bank0.gpio(1).modify(|_, w| w.od().clear_bit().ie().set_bit());
            // GP25 (LED): plain digital output, no input needed.
            pads_bank0.gpio(25).modify(|_, w| w.od().clear_bit().ie().clear_bit());
            sio.gpio_oe_set().write(|w| w.bits(1 << 25));
            sio.gpio_out_clr().write(|w| w.bits(1 << 25)); // start off

            // ── UART0: 115200 8N1, FIFOs disabled ───────────────────
            // FIFOs off (character mode, `FEN=0`): a 1-byte-deep holding
            // register behaves exactly like the STM32 board's USART2 —
            // `UARTFR.TXFF`/`RXFE` become simple "is the one slot full/
            // empty" flags, matching the polling shape every other board
            // in this workspace already uses, and makes the TX-empty
            // interrupt fire per-byte instead of per-FIFO-threshold.
            //
            // Baud divisor per the PL011 TRM: `divisor = UARTCLK /
            // (16 * baud)`, IBRD = integer part, FBRD = round(frac * 64).
            // Must be written *before* UARTLCR_H — writing LCR_H is what
            // latches IBRD/FBRD into the actual baud-rate generator.
            const BAUD: u32 = 115_200;
            let divisor_x64 = ((PERI_HZ as u64) * 4 / (BAUD as u64)) as u32; // *4 = *64/16
            let ibrd = divisor_x64 / 64;
            let fbrd = divisor_x64 % 64;
            uart0.uartibrd().write(|w| w.baud_divint().bits(ibrd as u16));
            uart0.uartfbrd().write(|w| w.baud_divfrac().bits(fbrd as u8));
            uart0
                .uartlcr_h()
                .write(|w| w.wlen().bits(0b11).fen().clear_bit()); // 8N1, no FIFO
            uart0
                .uartcr()
                .write(|w| w.uarten().set_bit().txe().set_bit().rxe().set_bit());

            rivet::irq::register_untraced(super::irq::UART0_IRQ, uart_irq_handler).unwrap();
            rivet::irq::set_priority(super::irq::UART0_IRQ, 0xFF);
            rivet::irq::enable(super::irq::UART0_IRQ);
            uart0.uartimsc().modify(|_, w| w.rxim().set_bit());
            rivet::console::enable_irq_tx();
        }
    }

    fn uart_irq_handler() {
        // SAFETY: fixed UART0 register block; this ISR is the sole owner
        // of it (matches `rivet-bsp-stm32f401re::uart_irq_handler`'s
        // identical shape, including the same "check the raw flag AND
        // the enable bit" reasoning — PL011's `UARTMIS` is already
        // masked-status, unlike STM32's raw `SR`, but re-checking the
        // enable bit here costs nothing and keeps the two BSPs' ISRs
        // structurally identical for anyone reading both).
        let uart0 = unsafe { &*rp2040_pac::UART0::ptr() };
        loop {
            let mis = uart0.uartmis().read();
            if mis.txmis().bit_is_set() {
                match rivet::console::tx_irq_next_byte() {
                    Some(b) => uart0.uartdr().write(|w| unsafe { w.data().bits(b) }),
                    None => uart0.uartimsc().modify(|_, w| w.txim().clear_bit()),
                }
            } else if mis.rxmis().bit_is_set() {
                let b = uart0.uartdr().read().data().bits();
                rivet::console::on_rx_byte(b);
            } else {
                break;
            }
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_console_kick_tx() {
        // SAFETY: fixed UART0 register block.
        unsafe {
            (&*rp2040_pac::UART0::ptr()).uartimsc().modify(|_, w| w.txim().set_bit());
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_now_us() -> u64 {
        rivet_arch_cortex_m0::systick::now_micros_precise()
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_tick_start(hz: u32) {
        let reload_ticks = SYSCLK_HZ / hz;
        let period_us = 1_000_000 / hz;
        rivet_arch_cortex_m0::systick::init(reload_ticks, period_us);
    }

    #[no_mangle]
    unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
        // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port
        // contract.
        let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
        // SAFETY: fixed UART0 register block.
        let uart0 = unsafe { &*rp2040_pac::UART0::ptr() };
        for &b in bytes {
            while uart0.uartfr().read().txff().bit_is_set() {
                core::hint::spin_loop();
            }
            uart0.uartdr().write(|w| unsafe { w.data().bits(b) });
        }
        // Same bytes, also over USB CDC-ACM — see the `usb` module's own doc for
        // why both consoles run simultaneously.
        usb::write_best_effort(bytes);
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_reset() -> ! {
        rivet_arch_cortex_m0::system_reset()
    }

    /// No semihosting, no separate debug transport — same reasoning as
    /// every other real-hardware board in this workspace: print a marker
    /// on the console UART and halt.
    #[no_mangle]
    extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
        if code == 0 {
            rivet::console::write_str("\nRIVET_EXIT_OK\n");
        } else {
            rivet::console::write_str("\nRIVET_FAILURE code=");
            print_dec(code);
            rivet::console::write_str("\n");
        }
        // Same reasoning as every other board's own `__rivet_board_exit`:
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
        // SAFETY: fixed WATCHDOG register block; the tick generator was
        // already configured (1 MHz) in `__rivet_board_init`.
        unsafe {
            let watchdog = &*rp2040_pac::WATCHDOG::ptr();
            // LOAD is a 24-bit down-counter in the same 1 MHz ticks the
            // tick generator produces, but the hardware halves it
            // internally (counts down twice per LOAD to produce the
            // documented timeout) — matches the RP2040 datasheet's own
            // "actual timeout = 2x the LOAD value in ticks" note.
            let load = (period_us / 2).clamp(1, 0x00FF_FFFF);
            watchdog.load().write(|w| w.load().bits(load));
            watchdog.ctrl().modify(|_, w| w.enable().set_bit());
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_feed() {
        if WDT_PERIOD_US.load(Ordering::Acquire) != 0 {
            // SAFETY: fixed WATCHDOG LOAD register — writing it both
            // reloads the countdown and (per the datasheet) is the
            // documented "feed" action.
            unsafe {
                let watchdog = &*rp2040_pac::WATCHDOG::ptr();
                let period_us = WDT_PERIOD_US.load(Ordering::Acquire);
                let load = (period_us / 2).clamp(1, 0x00FF_FFFF);
                watchdog.load().write(|w| w.load().bits(load));
            }
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_check() {
        // No-op: WATCHDOG is a real hardware counter, counting down
        // autonomously — there is no software deadline to check (matches
        // every other real-hardware board's identical reasoning).
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
        rivet_arch_cortex_m0::systick::handler();
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
        // SAFETY: `buf[..i]` contains only ASCII digit bytes just written
        // above.
        rivet::console::write_str(unsafe { core::str::from_utf8_unchecked(&buf[..i]) });
    }
}
