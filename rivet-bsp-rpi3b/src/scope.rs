//! GPIO markers for measuring kernel operations with a scope.
//!
//! Each pin carries one operation and is high for exactly as long as that
//! operation runs. The measurement is the pulse width, read off a single
//! channel, so nothing has to be lined up against anything else.
//!
//! The earlier arrangement put a start edge on one pin and a stop edge on
//! another, which meant every reading was a cursor placed on two traces.
//! Width on one trace is the same information without the arithmetic, and
//! it also gives the period for free: for a periodic signal the rising
//! edges are the release times, so one channel shows both how long the
//! work took and how regularly it was started.
//!
//! # What this costs
//!
//! A pin is driven by a store to Device memory, which is a trip to the
//! peripheral bus. Two of them bracket the operation, so **the width you
//! measure is the operation plus the cost of marking it**, and the cost
//! lands inside the window rather than outside. For the tick handler,
//! which is a few hundred nanoseconds, that is not a rounding error.
//! Compare widths against each other, and use `rt_bench` when the
//! absolute number matters.
//!
//! # Why this is behind a feature
//!
//! The kernel markers are compiled out unless `scope-pins` is on, and that
//! is not caution, it is a measurement. Leaving them in cost 52 ns on the
//! cheapest tick and 156 ns on the mean, and took the count of ticks over
//! a microsecond from 6 in 30000 to 748.
//!
//! The reason is the one already written up for the timer sweep in
//! `docs/rpi3b-benchmarks.md`: the pin numbers live in statics that the
//! tick path would otherwise never touch, they are read once per tick and
//! not otherwise, and that is exactly the access pattern that sits in a
//! shared L2 and gets evicted between ticks. Two more cold lines on that
//! path is two more misses per tick. Having just removed six of them from
//! `poll_timers`, adding two back for instrumentation nobody had enabled
//! would have been a poor trade.
//!
//! [`Marker`] is always available, because application code calling it is
//! already paying for its own working set.

use core::sync::atomic::{AtomicU32, Ordering};

/// No pin assigned. Chosen over `Option<u8>` so the whole thing stays a
/// lock-free atomic load on the interrupt path.
const NONE: u32 = u32::MAX;

static TICK: AtomicU32 = AtomicU32::new(NONE);
static DOORBELL: AtomicU32 = AtomicU32::new(NONE);

/// Mark the timer interrupt: high on entry, low on exit.
///
/// # Safety
/// The pin must be configured as an output and driven by nothing else.
pub unsafe fn set_tick_pin(pin: u8) {
    TICK.store(pin as u32, Ordering::Release);
}

/// Mark the doorbell: high when the interrupt is taken, low when the task
/// it woke has run. The width is the whole wake path.
///
/// # Safety
/// The pin must be configured as an output and driven by nothing else.
pub unsafe fn set_doorbell_pin(pin: u8) {
    DOORBELL.store(pin as u32, Ordering::Release);
}

#[cfg_attr(not(feature = "scope-pins"), allow(dead_code))]
fn raise(cell: &AtomicU32) {
    // Relaxed: this is a pin number, not a lock. Nothing is published
    // through it, so the acquire barrier a stronger ordering would emit
    // is pure cost on a path that runs ten thousand times a second.
    let p = cell.load(Ordering::Relaxed);
    if p != NONE {
        // SAFETY: the pin was configured as an output by whoever
        // registered it, per the setter's contract.
        unsafe { crate::gpio::raise(p as u8) };
    }
}

#[cfg_attr(not(feature = "scope-pins"), allow(dead_code))]
fn lower(cell: &AtomicU32) {
    let p = cell.load(Ordering::Relaxed);
    if p != NONE {
        // SAFETY: as above.
        unsafe { crate::gpio::lower(p as u8) };
    }
}

#[cfg(feature = "scope-pins")]
pub(crate) fn tick_begin() {
    raise(&TICK);
}
#[cfg(feature = "scope-pins")]
pub(crate) fn tick_end() {
    lower(&TICK);
}
#[cfg(feature = "scope-pins")]
pub(crate) fn doorbell_begin() {
    raise(&DOORBELL);
}

#[cfg(not(feature = "scope-pins"))]
pub(crate) fn tick_begin() {}
#[cfg(not(feature = "scope-pins"))]
pub(crate) fn tick_end() {}
#[cfg(not(feature = "scope-pins"))]
pub(crate) fn doorbell_begin() {}

/// Drop the doorbell marker. Called by the task the doorbell woke, which
/// is what makes the width the wake path rather than the interrupt alone.
pub fn doorbell_end() {
    lower(&DOORBELL);
}

/// Holds a pin high for as long as it is alive.
///
/// For marking application work rather than kernel internals:
///
/// ```ignore
/// loop {
///     let _m = scope::Marker::new(PIN);   // pin goes high
///     do_the_control_loop();
///     drop(_m);                           // pin goes low
///     sleep_until(next);
/// }
/// ```
///
/// On a periodic task this is the useful one: pulse width is the
/// execution time and the gap between rising edges is the release
/// interval, so release jitter and execution time come off one channel.
pub struct Marker(u8);

impl Marker {
    /// # Safety
    /// The pin must be configured as an output and driven by nothing else.
    pub unsafe fn new(pin: u8) -> Self {
        // SAFETY: forwarded from this function's contract.
        unsafe { crate::gpio::raise(pin) };
        Marker(pin)
    }
}

impl Drop for Marker {
    fn drop(&mut self) {
        // SAFETY: the pin was an output when the marker was made and
        // nothing else drives it.
        unsafe { crate::gpio::lower(self.0) };
    }
}
