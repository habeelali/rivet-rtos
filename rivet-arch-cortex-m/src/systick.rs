//! Stock SysTick tick source.
//!
//! SysTick itself is architectural (fixed address `0xE000E010` on every
//! Cortex-M) — but the *reload value* that makes it fire at the kernel's
//! configured tick rate depends on the board's clock frequency, which only
//! the board knows. Call [`init`] with a reload value computed by the
//! board (`sysclk_hz / tick_hz`) from `__rivet_board_tick_start`; this
//! module handles the mechanism (counting ticks — not microseconds, so a
//! u32 counter wraps in ~49 days instead of the ~71 minutes a microsecond
//! counter would give) and the timing-sensitive enable-after-PSP-is-valid
//! ordering.

use core::sync::atomic::{AtomicU32, Ordering};

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
/// here (ENABLE/TICKINT bits left clear). If SysTick (and therefore
/// PendSV) could fire this early, it could land while still on the plain
/// boot stack with PSP never set — PendSV's asm unconditionally does `mrs
/// r0, psp; stmia r0, {r4-r11}` assuming PSP is valid, so an uninitialized
/// PSP there faults immediately. [`enable`] is called only once PSP is
/// safely set up (from `__rivet_arch_start_first_task`).
///
/// `reload_ticks` is the board-computed `sysclk_hz / tick_hz`;
/// `tick_period_us` is `1_000_000 / tick_hz`, used to convert the tick
/// count to microseconds in [`now_micros`].
pub fn init(reload_ticks: u32, tick_period_us: u32) {
    TICK_PERIOD_US.store(tick_period_us, Ordering::Release);
    // SAFETY: `SYST::PTR` is the statically-known SysTick peripheral base,
    // valid on every Cortex-M; register writes are volatile MMIO accesses.
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    // SAFETY: peripheral register write to the valid SysTick block; the
    // peripheral is only ever accessed here and in `enable`.
    unsafe { syst.csr.write(0) }; // disable while configuring
    unsafe { syst.rvr.write(reload_ticks - 1) };
    unsafe { syst.cvr.write(0) }; // clear current value
}

/// Enable SysTick (ENABLE + TICKINT). Call only once PSP has been set up
/// — see [`init`] for why.
pub fn enable() {
    // SAFETY: SYST::PTR is the statically-known SysTick base (see `init`).
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    // SAFETY: volatile memory-mapped write to the SysTick control
    // register; the peripheral is exclusively owned by this module.
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
    // SAFETY: SYST::PTR is the statically-known SysTick base (see `init`);
    // RVR/CVR writes are volatile MMIO accesses.
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
