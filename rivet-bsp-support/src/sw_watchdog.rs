//! Software watchdog fallback: for boards with no real watchdog hardware,
//! arm a deadline and check it on every tick. **Not independent of the
//! CPU**: unlike a real hardware watchdog, this cannot catch a hang that
//! stops the tick itself (e.g. a spin loop with interrupts disabled) —
//! document that limitation prominently in any BSP that uses this.

use core::sync::atomic::{AtomicU32, Ordering};

static PERIOD_US: AtomicU32 = AtomicU32::new(0);
// Not a native atomic on RV32 (no AtomicU64); `feed()` can race
// `expired()` (one from a fed task, one from tick/ISR context), but a
// torn read here only ever produces an early or late expiry check by one
// tick's worth of time — never a wildly wrong one (`now_us`/period are
// both bounded, ordinary values, not a "wrap the future in" kind of
// hazard) — an acceptable trade for a fallback that's already documented
// as best-effort, not hardware-grade. Guarded with a critical section
// instead of the mtime driver's hi/lo/hi-recheck protocol for simplicity.
static mut DEADLINE_US: u64 = 0;

/// Arm the deadline at `now_us() + period_us`. `period_us == 0` disables
/// the watchdog.
pub fn init(period_us: u32, now_us: u64) {
    PERIOD_US.store(period_us, Ordering::Release);
    rivet::critical::enter(|| {
        // SAFETY: read-modify-write under a critical section.
        unsafe { DEADLINE_US = now_us.wrapping_add(period_us as u64) };
    });
}

/// Re-arm the deadline. Call periodically from the fed task's main loop.
pub fn feed(now_us: u64) {
    let period = PERIOD_US.load(Ordering::Acquire);
    if period != 0 {
        rivet::critical::enter(|| {
            // SAFETY: read-modify-write under a critical section.
            unsafe { DEADLINE_US = now_us.wrapping_add(period as u64) };
        });
    }
}

/// Check whether the deadline has passed. Call every tick; the board
/// decides what to do on expiry (typically: print a diagnostic, reset).
pub fn expired(now_us: u64) -> bool {
    let period = PERIOD_US.load(Ordering::Acquire);
    if period == 0 {
        return false;
    }
    let deadline = rivet::critical::enter(|| {
        // SAFETY: read under a critical section.
        unsafe { DEADLINE_US }
    });
    now_us >= deadline
}
