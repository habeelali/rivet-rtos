//! Stock SysTick tick source — identical role to
//! `rivet-arch-cortex-m::systick`, duplicated rather than shared because
//! this is a separate crate (ARMv6-M has no MPU/DWT, so it isn't just a
//! `#[cfg]` inside the ARMv7-M port — see this crate's own module docs).
//! SysTick itself is architected the same way on Cortex-M0/M0+ as on
//! M3/M4/M7 (same registers, same fixed base) — only the *reload value*
//! is board-specific.

use rivet::sync::atomic::{AtomicU32, Ordering};

/// Tick counter. Counts *ticks*, not microseconds: a u32 tick counter at a
/// typical 1 kHz wraps in ~49 days, versus ~71 minutes for a u32
/// microsecond counter. Conversion happens at the API boundary in
/// [`now_micros`]. The tick handler is the only writer.
static SYSTEM_TICKS: AtomicU32 = AtomicU32::new(0);
/// Tick period in microseconds, set by [`init`] from the kernel's
/// configured `TICK_HZ` (not necessarily 1000 — configurable via
/// `RIVET_TICK_HZ`).
static TICK_PERIOD_US: AtomicU32 = AtomicU32::new(1000);

/// Configure SysTick's reload value, but deliberately do NOT enable it
/// here (ENABLE/TICKINT bits left clear) — see
/// `rivet-arch-cortex-m::systick::init`'s identical reasoning: PendSV
/// must not fire before PSP is valid. [`enable`] is called only once PSP
/// is safely set up (from `__rivet_arch_start_first_task`).
///
/// `reload_ticks` is the board-computed `sysclk_hz / tick_hz`;
/// `tick_period_us` is `1_000_000 / tick_hz`, used to convert the tick
/// count to microseconds in [`now_micros`].
pub fn init(reload_ticks: u32, tick_period_us: u32) {
    TICK_PERIOD_US.store(tick_period_us, Ordering::Release);
    // SAFETY: `SYST::PTR` is the statically-known SysTick peripheral base,
    // valid on every Cortex-M (including M0/M0+); register writes are
    // volatile MMIO accesses.
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    unsafe { syst.csr.write(0) }; // disable while configuring
    unsafe { syst.rvr.write(reload_ticks - 1) };
    unsafe { syst.cvr.write(0) }; // clear current value
}

/// Enable SysTick (ENABLE + TICKINT). Call only once PSP has been set up
/// — see [`init`] for why.
pub fn enable() {
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    unsafe {
        syst.csr.write(
            (1 << 0)  // ENABLE
            | (1 << 1)  // TICKINT
            | (1 << 2), // CLKSOURCE (system clock)
        )
    };
}

/// Override the SysTick reload value (in system-clock ticks) after
/// [`init`]. Safe to call before `run()` (the countdown starts from the
/// new value when SysTick is enabled); also resets the current value so
/// the first underflow uses the new period.
pub fn reload(ticks: u32) {
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    unsafe {
        syst.rvr.write(ticks);
        syst.cvr.write(0);
    }
}

/// Test hook: seed the tick counter so a test can start near the u32
/// boundary and observe a wrap crossing without running days of simulated
/// time. Harmless in production: it merely rewinds/advances the monotonic
/// tick count.
pub fn seed_ticks(v: u32) {
    SYSTEM_TICKS.store(v, Ordering::Release);
}

/// Call from the board's `SysTick` exception handler: advance system time,
/// wake expired `Sleep` futures, then request a reschedule opportunity via
/// PendSV (never switches stacks directly).
pub fn handler() {
    let tick = SYSTEM_TICKS.fetch_add(1, Ordering::Release) + 1;
    rivet::watchdog::on_tick();
    let period_us = TICK_PERIOD_US.load(Ordering::Acquire) as u64;
    rivet::timer::poll_timers((tick as u64) * period_us);
    super::__rivet_arch_request_reschedule();
}

/// Current system time in microseconds: tick count x tick period,
/// converted at the API boundary.
pub fn now_micros() -> u64 {
    (SYSTEM_TICKS.load(Ordering::Acquire) as u64) * (TICK_PERIOD_US.load(Ordering::Acquire) as u64)
}

/// Same as [`now_micros`] but with sub-tick resolution: reads SysTick's
/// own down-counter (`CVR`) for how far into the *current* tick period we
/// are — see `rivet-arch-cortex-m::systick::now_micros_precise`'s
/// identical docs (duplicated here, not shared, for the same
/// separate-crate reason as the rest of this module).
pub fn now_micros_precise() -> u64 {
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    loop {
        let t0 = SYSTEM_TICKS.load(Ordering::Acquire);
        let cvr = syst.cvr.read();
        let t1 = SYSTEM_TICKS.load(Ordering::Acquire);
        if t0 != t1 {
            continue;
        }
        let rvr = syst.rvr.read();
        let period_us = TICK_PERIOD_US.load(Ordering::Acquire) as u64;
        let elapsed = (rvr.saturating_sub(cvr)) as u64;
        let frac_us = elapsed.saturating_mul(period_us) / (rvr as u64 + 1);
        return (t0 as u64) * period_us + frac_us;
    }
}
