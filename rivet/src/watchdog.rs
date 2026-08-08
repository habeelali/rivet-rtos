//! Watchdog (plan.md §3.5).
//!
//! - **Cortex-M:** the real `luminary-watchdog` hardware block on
//!   lm3s6965evb (WDTLOAD/WDTCTL/WDTICR/WDTLOCK at 0x40000000). A genuine
//!   hardware watchdog: it keeps counting even if the CPU wedges with
//!   interrupts off, and QEMU models reset-on-expiry (after two expiries:
//!   the first sets the interrupt status, the second resets — the CMSDK
//!   two-stage model).
//! - **RISC-V:** `virt` has no WDT, so a *software* watchdog on the tick:
//!   [`feed`] re-arms a deadline; the tick handler checks it and resets
//!   via `riscv.sifive.test` (0x7777). Clearly flagged: a tick-driven
//!   watchdog cannot catch a hang that stops ticks — that independence is
//!   only validatable on Cortex-M.
//!
//! Task-level watchdogs: a task that calls [`checkin`] in its main loop
//! opts in; the tick handler flags it if it goes silent longer than
//! [`enable_checkins`].

use core::sync::atomic::{AtomicU32, Ordering};

/// Watchdog period in microseconds (fits u32: max ~71 min). 0 = disabled.
static PERIOD_US: AtomicU32 = AtomicU32::new(0);

/// Software-watchdog deadline (RISC-V): `feed()` sets it to now + period;
/// the tick checks it.
#[cfg(target_arch = "riscv32")]
static DEADLINE: AtomicU32 = AtomicU32::new(0);

/// Task checkin timeout in microseconds (0 = task checkins disabled).
static CHECKIN_TIMEOUT_US: AtomicU32 = AtomicU32::new(0);

/// Initialize the watchdog with the given period. On Cortex-M this
/// programs the hardware WDT; on RISC-V it arms the software deadline.
pub fn init(period: crate::time::Duration) {
    let us = period.as_micros() as u32;
    PERIOD_US.store(us, Ordering::Release);
    #[cfg(target_arch = "arm")]
    {
        // The LM3S6965's system clock (and therefore the watchdog's
        // WDOGCLK) stays at zero until the guest programs the System
        // Control RCC register — QEMU's ssys models the clock from RCC,
        // and a zero clock leaves the WDT ptimer permanently disabled
        // ("Timer with period zero, disabling"). Program a known main-
        // oscillator / SYSDIV configuration first.
        // SAFETY: fixed SSYS register (volatile).
        unsafe {
            core::ptr::write_volatile(0x400F_E060 as *mut u32, 0x078E_3AC1);
        }

        // luminary-watchdog at 0x40000000.
        const WDT_BASE: usize = 0x4000_0000;
        const WDTLOAD: *mut u32 = WDT_BASE as *mut u32;
        const WDTCTL: *mut u32 = (WDT_BASE + 0x08) as *mut u32;
        const WDTICR: *mut u32 = (WDT_BASE + 0x0C) as *mut u32;
        const WDTLOCK: *mut u32 = (WDT_BASE + 0xC00) as *mut u32;
        const UNLOCK: u32 = 0x1ACC_E551;
        // LM3S6965 system clock is 12 MHz on QEMU → period in ticks.
        // (us × 12 ticks/µs; u32 holds ~358 s of period.)
        let ticks = (us as u64 * 12).max(1) as u32;
        // SAFETY: fixed memory-mapped WDT registers (volatile).
        unsafe {
            core::ptr::write_volatile(WDTLOCK, UNLOCK);
            core::ptr::write_volatile(WDTLOAD, ticks);
            core::ptr::write_volatile(WDTCTL, 0b11); // INTEN | RESEN
            core::ptr::write_volatile(WDTICR, 0);
        }
    }
    // RISC-V: arm the software deadline immediately.
    #[cfg(target_arch = "riscv32")]
    {
        DEADLINE.store(
            crate::arch::now_micros().wrapping_add(us as u64) as u32,
            Ordering::Release,
        );
    }
}

/// Kick the watchdog. Call periodically from the main loop / a high-
/// priority task.
pub fn feed() {
    let us = PERIOD_US.load(Ordering::Acquire);
    if us != 0 {
        #[cfg(target_arch = "arm")]
        {
            // SAFETY: fixed WDT interrupt-clear register (reloads the
            // countdown from WDTLOAD).
            unsafe {
                core::ptr::write_volatile((0x4000_0000usize + 0x0C) as *mut u32, 0);
            }
        }
        #[cfg(target_arch = "riscv32")]
        {
            DEADLINE.store(
                crate::arch::now_micros().wrapping_add(us as u64) as u32,
                Ordering::Release,
            );
        }
    }
}

/// Called by the arch tick handler every tick. RISC-V: check the software
/// deadline and reset on expiry. Cortex-M: nothing to do (hardware WDT).
pub fn on_tick() {
    #[cfg(target_arch = "riscv32")]
    {
        let deadline = DEADLINE.load(Ordering::Acquire);
        let period = PERIOD_US.load(Ordering::Acquire);
        let now = crate::arch::now_micros() as u32;
        if period != 0 && now.wrapping_sub(deadline) < u32::MAX / 2 {
            crate::arch::debug_print("RIVET WATCHDOG TIMEOUT\n");
            crate::arch::system_reset();
        }
    }
    check_task_checkins();
}

/// Opt a task into the task-level watchdog: record the current time as its
/// last checkin. Called periodically from the task's own main loop.
pub fn checkin() {
    if CHECKIN_TIMEOUT_US.load(Ordering::Acquire) == 0 {
        return;
    }
    if let Some(id) = crate::preempt::sched::current() {
        if let Some(t) = crate::preempt::tcb::get(id) {
            t.last_checkin
                .store(crate::arch::now_micros() as u32, Ordering::Release);
        }
    }
}

/// Enable task-level checkin monitoring with the given timeout.
pub fn enable_checkins(timeout: crate::time::Duration) {
    CHECKIN_TIMEOUT_US.store(timeout.as_micros() as u32, Ordering::Release);
}

/// Scan tasks that opted into checkins; reset if any has been silent too
/// long. (Per-task *isolation* of an unresponsive task is Phase 6 work;
/// here the recovery is a diagnosed reset.)
fn check_task_checkins() {
    let timeout = CHECKIN_TIMEOUT_US.load(Ordering::Acquire);
    if timeout == 0 {
        return;
    }
    let now = crate::arch::now_micros() as u32;
    for (id, t) in crate::preempt::tcb::TASKS.iter().enumerate() {
        if !t.used.load(Ordering::Acquire) {
            continue;
        }
        let last = t.last_checkin.load(Ordering::Acquire);
        if last != 0 && now.wrapping_sub(last) > timeout {
            crate::arch::debug_print("RIVET TASK CHECKIN TIMEOUT task=");
            print_dec(id);
            crate::arch::debug_print("\n");
            crate::arch::system_reset();
        }
    }
}

fn print_dec(mut n: usize) {
    if n == 0 {
        crate::arch::debug_print("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut out = [0u8; 10];
    for j in 0..i {
        out[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&out[..i]) {
        crate::arch::debug_print(s);
    }
}

/// Test-only reset (host).
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    PERIOD_US.store(0, Ordering::Release);
    #[cfg(target_arch = "riscv32")]
    DEADLINE.store(0, Ordering::Release);
    CHECKIN_TIMEOUT_US.store(0, Ordering::Release);
}
