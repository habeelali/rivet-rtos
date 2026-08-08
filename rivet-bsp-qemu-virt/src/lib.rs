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

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    rivet_arch_riscv::clint::configure(CLINT_BASE, MTIME_HZ);
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
        __rivet_board_reset();
    }
}
