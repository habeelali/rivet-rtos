//! Critical section abstraction, built on the Group A `port::arch`
//! interrupt-mask primitives (local, per-hart) plus a genuine cross-hart
//! spinlock (plan.md Phase 19).
//!
//! # Why a lock is needed at all
//!
//! Before Phase 19, `enter` was purely local-interrupt-disable: sound
//! because "this hart's interrupts are off" and "nobody else can touch
//! this data" were the same fact on a single hart. On real SMP that stops
//! being true — two harts can each be inside their own trap handler, each
//! with local interrupts off, and still race on shared kernel state
//! (the scheduler's ready bitmap, the timer queue, `exec_time`'s
//! counters, everything else in this crate that calls [`enter`]).
//! Layering a real spinlock underneath the existing local-mask primitive
//! closes that gap for every one of those call sites at once, without
//! auditing each one individually.
//!
//! # Reentrant by construction, not by accident
//!
//! A **plain** spinlock would deadlock a hart against itself the moment
//! any call site nests `enter` calls — and while no call site does that
//! *today*, ruling it out permanently (rather than re-auditing the whole
//! crate on every future change) is worth the small extra cost: a
//! per-hart nesting counter, incremented/decremented around a spinlock
//! that's only actually acquired/released at the outermost depth. This
//! is the same shape FreeRTOS SMP uses for `portENTER_CRITICAL`, chosen
//! for the same reason.
//!
//! # Single-hart boards are unaffected
//!
//! [`crate::port::arch::hart_id`] is hardwired to `0` on every arch/board
//! except RISC-V (where it reads the real `mhartid`, always `0` unless
//! `RIVET_MAX_HARTS > 1`) — so on every board except a `-smp`-enabled
//! RISC-V `virt` build, the spinlock below is always uncontended (the
//! CAS never has a second hart to race) and the control flow is
//! identical to the pre-Phase-19 code. Verified: the full golden suite
//! is byte-identical on all three boards with this change in place.

use crate::sync::atomic::{AtomicIsize, AtomicU32, Ordering};

const MAX_HARTS: usize = crate::config::MAX_HARTS;

/// Per-hart critical-section nesting depth. Each hart only ever reads/
/// writes its own index — by the time this is touched, local interrupts
/// are already masked by the `port::arch::critical_section` call this
/// closure runs inside, so no *local* concurrent access is possible
/// either — hence `Relaxed` throughout.
#[cfg(not(loom))]
static NESTING: [AtomicU32; MAX_HARTS] = [const { AtomicU32::new(0) }; MAX_HARTS];
#[cfg(loom)]
loom::lazy_static! {
    static ref NESTING: [AtomicU32; MAX_HARTS] = core::array::from_fn(|_| AtomicU32::new(0));
}

/// `-1` = unlocked, else the hart id currently holding the lock.
#[cfg(not(loom))]
static LOCK_OWNER: AtomicIsize = AtomicIsize::new(-1);
#[cfg(loom)]
loom::lazy_static! {
    static ref LOCK_OWNER: AtomicIsize = AtomicIsize::new(-1);
}

/// Run a closure with interrupts disabled on this hart *and* mutual
/// exclusion against every other hart. Nested calls compose correctly on
/// the same hart (the nesting counter makes the lock acquire/release a
/// no-op below the outermost depth); see the module docs for why that
/// matters and how single-hart boards are unaffected.
#[inline]
pub fn enter<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    enter_locked(f)
}

fn enter_locked<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    // Local interrupt mask first (outermost layer): this is what makes
    // the hart-id read and the nesting-depth check below themselves
    // race-free against this *same* hart's own interrupt handlers.
    crate::port::arch::critical_section(|| {
        // plan.md Phase 30 (§11 outlier root cause): the latency-histogram
        // timestamps used to be taken by the *caller* of `enter_locked`,
        // outside this `critical_section` closure entirely — i.e. before
        // interrupts were actually masked, and again after they'd already
        // been restored. Interrupts are still live in both of those
        // shoulder windows, so a rare interrupt (SysTick, tail-chained
        // into a full PendSV context switch) landing in either one had
        // its *entire* handler duration folded into the "critical
        // section" measurement, even though the calling hart was really
        // off servicing an interrupt, not executing the section body.
        // That fully explains the rare 2^15-bucket outliers `stress_load_
        // bench`/`critsec_isolate_bench` saw across all three boards: not
        // a real unbounded cost in `PriorityMutexGuard::drop`'s unlock
        // path (code review never found one because there isn't one), but
        // a measurement artifact from timing a window wider than the
        // region that's actually interrupt-masked. Taking both timestamps
        // in here instead — strictly inside `critical_section`'s closure,
        // i.e. after interrupts are masked and before they're restored —
        // closes that gap: nothing can preempt this hart between them by
        // construction, matching this histogram's own documented
        // assumption (see module docs on `Kind::CriticalSection`).
        #[cfg(feature = "latency-histograms")]
        let start = crate::port::arch::cycle_count();

        let hart = crate::port::arch::hart_id();
        let depth = NESTING[hart].load(Ordering::Relaxed);
        if depth == 0 {
            // Uncontended on every single-hart board (see module docs):
            // this CAS succeeds on the first try whenever `hart` is the
            // only hart that ever runs this code.
            while LOCK_OWNER
                .compare_exchange_weak(-1, hart as isize, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
            {
                core::hint::spin_loop();
            }
        }
        NESTING[hart].store(depth + 1, Ordering::Relaxed);
        let r = f();
        NESTING[hart].store(depth, Ordering::Relaxed);
        if depth == 0 {
            LOCK_OWNER.store(-1, Ordering::Release);
        }

        #[cfg(feature = "latency-histograms")]
        crate::latency::record(
            crate::latency::Kind::CriticalSection,
            crate::port::arch::cycle_count().wrapping_sub(start),
        );

        r
    })
}

#[cfg(all(test, not(loom)))]
mod tests {
    use super::*;

    #[test]
    fn nested_enter_does_not_deadlock() {
        let r = enter(|| enter(|| enter(|| 42)));
        assert_eq!(r, 42);
    }

    #[test]
    fn lock_is_released_after_outermost_exit() {
        enter(|| {});
        assert_eq!(LOCK_OWNER.load(Ordering::Acquire), -1);
        assert_eq!(NESTING[crate::port::arch::hart_id()].load(Ordering::Relaxed), 0);
    }
}
