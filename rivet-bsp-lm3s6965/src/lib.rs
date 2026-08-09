//! Rivet RTOS board support: TI/Luminary LM3S6965 (Stellaris) Cortex-M3,
//! as modeled by QEMU's `lm3s6965evb` machine.
//!
//! Implements the Group B (`rivet::port::board`) contract: PL011 UART at
//! `0x4000_C000`, the real `luminary-watchdog` hardware block at
//! `0x4000_0000` (a genuine hardware watchdog — it keeps counting even if
//! the CPU wedges with interrupts off, and QEMU models reset-on-expiry),
//! ARM semihosting for exit. Runs at the board's default 12 MHz system
//! clock.

#![no_std]

pub mod gpio;

/// Board IRQ number map (plan.md Phase 13): which NVIC IRQ number is
/// which peripheral. `UART0 = 5` verified empirically (not assumed) by
/// enabling a probe range of IRQs and confirming which one fires on a
/// genuine UART0 TX-empty condition — matches the Stellaris LM3S6965
/// datasheet's exception table (GPIO ports A-E occupy positions 16-20,
/// i.e. IRQ 0-4; UART0 is position 21, IRQ 5).
pub mod irq {
    pub const UART0: u32 = 5;
}

// Everything below is genuinely Cortex-M specific (real asm!/naked_asm!
// blocks transitively, via rivet-arch-cortex-m) and target-gated so
// `cargo test -p rivet-bsp-lm3s6965` on host still runs `gpio`'s
// typestate compile-check without needing an ARM target/runner — mirrors
// the pattern the GPIO module used before it lived in the kernel crate
// (`#[cfg(any(target_arch = "arm", test))]`).
#[cfg(target_arch = "arm")]
mod board {
    use core::sync::atomic::{AtomicU32, Ordering};

    /// LM3S6965 default system clock on QEMU (no PLL/RCC configuration
    /// needed for this to hold — it's the reset-state clock).
    const SYSCLK_HZ: u32 = 12_000_000;

    const UART0_BASE: u32 = 0x4000_C000;
    const UART0_DR: *mut u32 = UART0_BASE as *mut u32;
    const UART0_FR: *const u32 = (UART0_BASE + 0x18) as *const u32;
    const UART_FR_BUSY: u32 = 1 << 5;

    /// `luminary-watchdog` register block.
    const WDT_BASE: usize = 0x4000_0000;
    const WDTLOAD: *mut u32 = WDT_BASE as *mut u32;
    const WDTCTL: *mut u32 = (WDT_BASE + 0x08) as *mut u32;
    const WDTICR: *mut u32 = (WDT_BASE + 0x0C) as *mut u32;
    const WDTLOCK: *mut u32 = (WDT_BASE + 0xC00) as *mut u32;
    const WDT_UNLOCK: u32 = 0x1ACC_E551;

    /// Watchdog period in microseconds, tracked locally so `feed()` knows
    /// whether the watchdog was ever armed (mirrors the `period_us == 0`
    /// "disabled" convention from the port contract).
    static WDT_PERIOD_US: AtomicU32 = AtomicU32::new(0);

    #[no_mangle]
    extern "Rust" fn __rivet_board_init() {}

    #[no_mangle]
    extern "Rust" fn __rivet_board_now_us() -> u64 {
        rivet_arch_cortex_m::systick::now_micros()
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
        for &b in bytes {
            // SAFETY: `UART0_FR`/`UART0_DR` point at the LM3S6965 PL011 UART
            // registers (fixed, memory-mapped, volatile).
            while unsafe { core::ptr::read_volatile(UART0_FR) } & UART_FR_BUSY != 0 {
                core::hint::spin_loop();
            }
            // SAFETY: as above — UART data-register write.
            unsafe { core::ptr::write_volatile(UART0_DR, b as u32) };
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_reset() -> ! {
        rivet_arch_cortex_m::system_reset()
    }

    /// QEMU ARM semihosting has no simple "exit with code N" path, so
    /// failure prints a distinguishable marker instead and halts; the
    /// QEMU test harness asserts on the marker text rather than an exit
    /// code.
    #[no_mangle]
    extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
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
        // The LM3S6965's system clock (and therefore the watchdog's
        // WDOGCLK) stays at zero until the guest programs the System
        // Control RCC register — QEMU's ssys models the clock from RCC,
        // and a zero clock leaves the WDT ptimer permanently disabled
        // ("Timer with period zero, disabling"). Program a known main-
        // oscillator/SYSDIV configuration first. Scoped to only-when-armed
        // (rather than unconditionally at board init) so tests that never
        // touch the watchdog see the untouched reset-state clock that
        // `SYSCLK_HZ` above assumes.
        // SAFETY: fixed SSYS register (volatile).
        unsafe {
            core::ptr::write_volatile(0x400F_E060 as *mut u32, 0x078E_3AC1);
        }
        // LM3S6965 system clock is 12 MHz on QEMU -> period in ticks (us x
        // 12 ticks/us; u32 holds ~358s of period).
        let ticks = (period_us as u64 * 12).max(1) as u32;
        // SAFETY: fixed memory-mapped WDT registers (volatile).
        unsafe {
            core::ptr::write_volatile(WDTLOCK, WDT_UNLOCK);
            core::ptr::write_volatile(WDTLOAD, ticks);
            core::ptr::write_volatile(WDTCTL, 0b11); // INTEN | RESEN
            core::ptr::write_volatile(WDTICR, 0);
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_feed() {
        if WDT_PERIOD_US.load(Ordering::Acquire) != 0 {
            // SAFETY: fixed WDT interrupt-clear register (reloads the
            // countdown from WDTLOAD).
            unsafe {
                core::ptr::write_volatile(WDTICR, 0);
            }
        }
    }

    #[no_mangle]
    extern "Rust" fn __rivet_board_wdt_check() {
        // No-op: this is a real hardware watchdog, counting down
        // autonomously — there is no software deadline to check.
    }

    /// The board's `SysTick` exception vector (referenced directly by
    /// every board linker script's vector table): bridges to the arch
    /// crate's generic tick mechanism. `extern "C"`/`#[no_mangle]` because
    /// it's an exception vector, not a port-contract symbol.
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
