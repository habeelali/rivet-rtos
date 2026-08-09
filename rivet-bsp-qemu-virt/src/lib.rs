//! Rivet RTOS board support: QEMU RISC-V `virt` machine (rv32).
//!
//! Implements the Group B (`rivet::port::board`) contract for QEMU's
//! `-machine virt -cpu rv32 -bios none`: CLINT at `0x0200_0000` (10 MHz
//! `mtime`), NS16550 UART at `0x1000_0000`, and the `riscv.sifive.test`
//! device at `0x0010_0000` for exit/reset (`0x5555` = pass, `0x3333 |
//! (code << 16)` = fail with a distinguishable code, `0x7777` = system
//! reset). `virt` has no hardware watchdog, so [`__rivet_board_wdt_init`]
//! arms [`rivet_bsp_support::sw_watchdog`] instead — **not independent of
//! the CPU**: a tick-driven check cannot catch a hang that stops ticks
//! (e.g. interrupts disabled in a spin loop). That independence is only
//! validatable on a board with real watchdog hardware (see
//! `rivet-bsp-lm3s6965`).

#![no_std]

const CLINT_BASE: usize = 0x0200_0000;
const MTIME_HZ: u64 = 10_000_000;
const UART0_DATA: usize = 0x1000_0000;
const SIFIVE_TEST_BASE: *mut u32 = 0x0010_0000 as *mut u32;
const SIFIVE_TEST_PASS: u32 = 0x5555;
const SIFIVE_TEST_FAIL: u32 = 0x3333;
const SIFIVE_TEST_RESET: u32 = 0x7777;
/// PLIC, verified empirically via QEMU monitor `info mtree`
/// (`riscv.sifive.plic` at this base) — see plan.md Phase 13.
const PLIC_BASE: usize = 0x0c00_0000;

/// Board IRQ number map (plan.md Phase 13): which PLIC source is which
/// peripheral. `UART0` = 10 is QEMU's `hw/riscv/virt.c` `UART0_IRQ`
/// constant — a long-stable, widely-relied-on value in the RV32
/// bare-metal ecosystem (used unchanged by, e.g., xv6-riscv's own `virt`
/// port), not just an assumption made here.
pub mod irq {
    pub const UART0: u32 = 10;
}

// ── NS16550 interrupt-driven console (plan.md Phase 14) ────────────
//
// Standard NS16550 register layout beyond the data register already used
// for polling writes: IER (offset 1, interrupt enable — bit0 = RX data
// available, bit1 = THR empty), IIR (offset 2, read-only interrupt
// identification), LSR (offset 5, line status).
const UART_IER: *mut u8 = (UART0_DATA + 1) as *mut u8;
const UART_IIR: *const u8 = (UART0_DATA + 2) as *const u8;
const IER_RX_AVAILABLE: u8 = 1 << 0;
const IER_THR_EMPTY: u8 = 1 << 1;

fn uart_irq_handler() {
    loop {
        // SAFETY: fixed NS16550 registers on the QEMU virt machine.
        let iir = unsafe { core::ptr::read_volatile(UART_IIR) };
        if iir & 1 != 0 {
            break; // no interrupt pending
        }
        // Standard 16550 IIR interrupt-ID field (bits 3:1, no FIFO):
        // 001 = THR empty, 010 = received data available, 011 = receiver
        // line status, 000 = modem status.
        match (iir >> 1) & 0b111 {
            0b001 => {
                // THR empty: drain one more byte from the ring, or
                // disable the interrupt if there's nothing left to send.
                match rivet::console::tx_irq_next_byte() {
                    Some(b) => unsafe {
                        core::ptr::write_volatile(UART0_DATA as *mut u8, b)
                    },
                    None => unsafe {
                        let ier = core::ptr::read_volatile(UART_IER);
                        core::ptr::write_volatile(UART_IER, ier & !IER_THR_EMPTY);
                    },
                }
            }
            0b010 => {
                // RX data available.
                // SAFETY: fixed NS16550 data register.
                let b = unsafe { core::ptr::read_volatile(UART0_DATA as *const u8) };
                rivet::console::on_rx_byte(b);
            }
            _ => {
                // Some other cause (line status, modem status): nothing
                // this driver acts on, but IIR must still be re-read on
                // the next loop iteration to avoid spinning on a cause we
                // don't clear — reading LSR/MSR acks those specifically.
                break;
            }
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {
    // SAFETY: fixed NS16550 IER register.
    unsafe {
        let ier = core::ptr::read_volatile(UART_IER);
        core::ptr::write_volatile(UART_IER, ier | IER_THR_EMPTY);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    rivet_arch_riscv::clint::configure(CLINT_BASE, MTIME_HZ);
    rivet_arch_riscv::plic::configure(PLIC_BASE);

    rivet::irq::register(irq::UART0, uart_irq_handler).unwrap();
    rivet::irq::enable(irq::UART0);
    // SAFETY: fixed NS16550 IER register; RX enabled from boot, TX left
    // off (kick_tx turns it on only when there's something queued).
    unsafe { core::ptr::write_volatile(UART_IER, IER_RX_AVAILABLE) };
    rivet::console::enable_irq_tx();
}

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    rivet_arch_riscv::clint::now_micros()
}

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    rivet_arch_riscv::clint::tick_start(hz);
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port contract.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    // SAFETY: `UART0_DATA` is the fixed NS16550 data register on the QEMU
    // virt machine.
    unsafe { rivet_bsp_support::ns16550::write_bytes(UART0_DATA, bytes) };
}

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    // SAFETY: `SIFIVE_TEST_BASE` is the fixed `riscv.sifive.test` device on
    // the QEMU virt machine; writing 0x7777 requests a system reset.
    unsafe { core::ptr::write_volatile(SIFIVE_TEST_BASE, SIFIVE_TEST_RESET) };
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    let value = if code == 0 {
        SIFIVE_TEST_PASS
    } else {
        SIFIVE_TEST_FAIL | (code << 16)
    };
    // SAFETY: `SIFIVE_TEST_BASE` is the fixed `riscv.sifive.test` device on
    // the QEMU virt machine; the guest writes the pass/fail pattern to
    // terminate the simulation with a distinguishable exit status.
    unsafe { core::ptr::write_volatile(SIFIVE_TEST_BASE, value) };
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
    rivet_bsp_support::sw_watchdog::init(period_us, rivet_arch_riscv::clint::now_micros());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    rivet_bsp_support::sw_watchdog::feed(rivet_arch_riscv::clint::now_micros());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {
    let now = rivet_arch_riscv::clint::now_micros();
    if rivet_bsp_support::sw_watchdog::expired(now) {
        rivet::console::write_str("RIVET WATCHDOG TIMEOUT\n");
        // Through the safe wrapper (not the raw `__rivet_board_reset`
        // symbol directly): it flushes the interrupt-driven console's TX
        // ring first (plan.md Phase 14) so this message actually reaches
        // the console before the guest resets.
        rivet::port::board::reset();
    }
}
