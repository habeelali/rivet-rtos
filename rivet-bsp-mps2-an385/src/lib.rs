//! Rivet RTOS board support: ARM MPS2 AN385 FPGA image (Cortex-M3), as
//! modeled by QEMU's `mps2-an385` machine.
//!
//! Implements the Group B (`rivet::port::board`) contract. Deliberately a
//! **different** memory map and peripheral set from `rivet-bsp-lm3s6965`
//! (verified via `qemu-system-arm -M mps2-an385 -monitor stdio -S`, `info
//! mtree` — see `link-mps2-an385.ld`'s header comment for the raw dump):
//! boot flash-equivalent (SSRAM1) at `0x0000_0000`, RAM (SSRAM23) at
//! `0x2000_0000`, a CMSDK APB UART at `0x4000_4000` (register layout is
//! DATA/STATE/CTRL/INTSTATUS/BAUDDIV — nothing like the LM3S6965's PL011),
//! a CMSDK APB watchdog (SP805-compatible) at `0x4000_8000`. This is the
//! proof that the arch/board boundary is in the right place: this crate
//! is the *only* thing that changed to add this board — `rivet`,
//! `rivet-arch-cortex-m`, and `rivet-rt` are untouched.

#![no_std]

use core::sync::atomic::{AtomicU32, Ordering};

/// Board IRQ number map (plan.md Phase 13). `UART0_TX = 1` verified
/// empirically (enabled a probe range of IRQs, confirmed which one fires
/// on a genuine UART0 TX-empty condition). `UART0_RX = 0` follows the
/// same CMSDK convention (RX/TX on adjacent, RX-first IRQ lines) but is
/// *not* independently verified the same way — this board's QEMU model
/// has no easy way to inject RX bytes from the host without a real
/// terminal, so Phase 14's RX-interrupt work should re-confirm this
/// before relying on it.
pub mod irq {
    pub const UART0_RX: u32 = 0;
    pub const UART0_TX: u32 = 1;
}

/// MPS2 default system clock on QEMU (SYSCLK from the board's SCC,
/// unconfigured/reset-state — matches what real AN385 firmware boots at).
const SYSCLK_HZ: u32 = 25_000_000;

// ── CMSDK APB UART (verified register layout: ARM CMSDK Technical
// Reference Manual — DATA/STATE/CTRL/INTSTATUS/BAUDDIV, distinct from the
// LM3S6965's PL011) ──────────────────────────────────────────────────
const UART0_BASE: u32 = 0x4000_4000;
const UART_DATA: *mut u32 = UART0_BASE as *mut u32;
const UART_STATE: *mut u32 = (UART0_BASE + 0x04) as *mut u32;
const UART_CTRL: *mut u32 = (UART0_BASE + 0x08) as *mut u32;
const UART_BAUDDIV: *mut u32 = (UART0_BASE + 0x10) as *mut u32;
const UART_STATE_TX_FULL: u32 = 1 << 0;
const UART_CTRL_TX_EN: u32 = 1 << 0;

// ── CMSDK APB watchdog (SP805-compatible register layout) ─────────────
const WDT_BASE: usize = 0x4000_8000;
const WDOGLOAD: *mut u32 = WDT_BASE as *mut u32;
const WDOGCONTROL: *mut u32 = (WDT_BASE + 0x08) as *mut u32;
const WDOGINTCLR: *mut u32 = (WDT_BASE + 0x0C) as *mut u32;
const WDOGLOCK: *mut u32 = (WDT_BASE + 0xC00) as *mut u32;
const WDT_UNLOCK: u32 = 0x1ACC_E551;

static WDT_PERIOD_US: AtomicU32 = AtomicU32::new(0);

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    // SAFETY: fixed CMSDK UART registers (volatile); board-exclusive.
    unsafe {
        core::ptr::write_volatile(UART_BAUDDIV, 16); // minimum divider
        core::ptr::write_volatile(UART_CTRL, UART_CTRL_TX_EN);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    #[cfg(target_arch = "arm")]
    {
        rivet_arch_cortex_m::systick::now_micros()
    }
    #[cfg(not(target_arch = "arm"))]
    0
}

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    #[cfg(target_arch = "arm")]
    {
        let reload_ticks = SYSCLK_HZ / hz;
        let period_us = 1_000_000 / hz;
        rivet_arch_cortex_m::systick::init(reload_ticks, period_us);
    }
    #[cfg(not(target_arch = "arm"))]
    let _ = hz;
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port contract.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    for &b in bytes {
        // SAFETY: `UART_STATE`/`UART_DATA` are the fixed CMSDK UART
        // registers (memory-mapped, volatile).
        while unsafe { core::ptr::read_volatile(UART_STATE) } & UART_STATE_TX_FULL != 0 {
            core::hint::spin_loop();
        }
        // SAFETY: as above — UART data-register write.
        unsafe { core::ptr::write_volatile(UART_DATA, b as u32) };
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    #[cfg(target_arch = "arm")]
    {
        rivet_arch_cortex_m::system_reset()
    }
    #[cfg(not(target_arch = "arm"))]
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    #[cfg(target_arch = "arm")]
    if code == 0 {
        rivet_arch_cortex_m::semihosting::exit_success();
    }
    rivet::console::write_str("\nRIVET_FAILURE code=");
    print_dec(code);
    rivet::console::write_str("\n");
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
    // MPS2's SCC leaves the peripheral clock at a known, always-on rate
    // (unlike the LM3S6965, which needs an explicit RCC write) — no
    // extra clock bring-up needed before arming the watchdog.
    let ticks = (period_us as u64 * (SYSCLK_HZ as u64 / 1_000_000)).max(1) as u32;
    // SAFETY: fixed memory-mapped CMSDK watchdog registers (volatile).
    unsafe {
        core::ptr::write_volatile(WDOGLOCK, WDT_UNLOCK);
        core::ptr::write_volatile(WDOGLOAD, ticks);
        core::ptr::write_volatile(WDOGCONTROL, 0b11); // INTEN | RESEN
        core::ptr::write_volatile(WDOGINTCLR, 1);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    if WDT_PERIOD_US.load(Ordering::Acquire) != 0 {
        // SAFETY: fixed WDT interrupt-clear register (reloads the
        // countdown from WDOGLOAD).
        unsafe {
            core::ptr::write_volatile(WDOGINTCLR, 1);
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {
    // No-op: this is a real hardware watchdog, counting down autonomously.
}

/// The board's `SysTick` exception vector (referenced directly by the
/// linker script's vector table).
///
/// # Safety
/// Exception entry point; never called directly.
#[no_mangle]
pub unsafe extern "C" fn SysTick() {
    #[cfg(target_arch = "arm")]
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
