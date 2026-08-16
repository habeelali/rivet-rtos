//! Fixed-size timer queue backing [`crate::time::Sleep`].
//!
//! A task calling `Sleep::<MICROS>::new().await` registers a deadline here
//! instead of busy-polling — the arch timer ISR (`riscv::timer_tick` /
//! `cortex_m::systick_handler`) calls [`poll_timers`] on every tick, which
//! wakes any task whose deadline has passed. This is what makes
//! `port::arch::idle()` (WFI) a real power-saving wait instead of a spin loop:
//! between ticks, no task is marked ready, so the executor actually sleeps.
//!
//! Slots are `u64`-deadline `UnsafeCell`s guarded by [`crate::critical`]
//! rather than atomics, since RV32 has no native 64-bit atomic ops (even
//! with the `A` extension, which is 32-bit/pointer-width only).

use core::cell::UnsafeCell;

/// Maximum number of outstanding timers (RIVET_MAX_TIMERS; one per task
/// blocked in `Sleep`, so this should be >= `MAX_TASKS` if every task might
/// sleep concurrently).
pub const MAX_TIMERS: usize = crate::config::MAX_TIMERS;

struct TimerSlot {
    /// Deadline in microseconds. 0 = slot unused.
    deadline: UnsafeCell<u64>,
    task: UnsafeCell<crate::task::TaskId>,
}

// Safety: all access goes through `critical::enter`, which disables
// interrupts (single-core), so there is no concurrent access.
unsafe impl Sync for TimerSlot {}

// Inline const avoids a named `const` item with interior mutability
// (clippy::declare_interior_mutable_const).
static TIMER_SLOTS: [TimerSlot; MAX_TIMERS] = [const {
    TimerSlot {
        deadline: UnsafeCell::new(0),
        task: UnsafeCell::new(crate::task::TaskId::new(0, 0)),
    }
}; MAX_TIMERS];

/// Earliest deadline armed in either array, or `NO_DEADLINE` for none.
///
/// Without this, every tick sweeps both arrays in full whether or not
/// anything can possibly have expired. At the default sizes that is six
/// cache lines, touched once per tick and not otherwise, which is exactly
/// the access pattern that lives in L2 rather than L1. On a part whose L2
/// is shared with cores running another OS, those lines are evicted
/// between ticks and every tick pays to fetch them back.
///
/// Measured on a Pi 3B at a 10 kHz tick with Linux on the other three
/// cores: the sweep cost 520 ns with them idle and 781 ns with them
/// saturated, while a single-cache-line control in the same handler moved
/// only 52 ns. The ratio of the two deltas tracked the ratio of cache
/// lines touched, which is what identified the sweep itself rather than
/// anything it was doing with the data.
///
/// Understating this costs a redundant sweep. Overstating it would skip a
/// wake, so it is only ever raised inside the same critical section that
/// recomputed it from the arrays.
static NEXT_DEADLINE: NextDeadline = NextDeadline(UnsafeCell::new(NO_DEADLINE));
const NO_DEADLINE: u64 = u64::MAX;

struct NextDeadline(UnsafeCell<u64>);

// Safety: every access is inside `critical::enter`, as with the arrays it
// summarises. A 64-bit atomic would be the obvious alternative and does
// not exist on the 32-bit targets this kernel also runs on.
unsafe impl Sync for NextDeadline {}

/// Note that something is armed for `deadline_us`.
///
/// # Safety
/// Caller must hold `critical::enter`.
unsafe fn arm_next_deadline(deadline_us: u64) {
    // SAFETY: forwarded from this function's contract.
    unsafe {
        if deadline_us < *NEXT_DEADLINE.0.get() {
            *NEXT_DEADLINE.0.get() = deadline_us;
        }
    }
}

/// Handle to a registered timer slot; carries the slot index and the
/// registered deadline so a stale [`cancel_deadline`] (after the slot was
/// already freed and reused) is a harmless no-op (plan.md [B7]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerHandle {
    slot: u8,
    deadline: u64,
}

/// Register a wake-up for `(priority, index)` at `deadline_us`.
/// Called by `Sleep::poll` on first poll.
///
/// Returns a handle that can be used to cancel the registration (plan.md
/// [B7]: a dropped `Sleep` must not leak its slot), or
/// [`TimerQueueFull`] when every slot is in use.
///
/// Exposed beyond `pub(crate)` only under `feature = "test-support"`, so
/// the property tests (plan.md §1.4) can drive the queue directly.
#[cfg(not(feature = "test-support"))]
pub(crate) fn register_deadline(
    deadline_us: u64,
    task: crate::task::TaskId,
) -> Result<TimerHandle, TimerQueueFull> {
    register_deadline_impl(deadline_us, task)
}

/// See [`register_deadline`].
#[cfg(feature = "test-support")]
pub fn register_deadline(
    deadline_us: u64,
    task: crate::task::TaskId,
) -> Result<TimerHandle, TimerQueueFull> {
    register_deadline_impl(deadline_us, task)
}

fn register_deadline_impl(
    deadline_us: u64,
    task: crate::task::TaskId,
) -> Result<TimerHandle, TimerQueueFull> {
    crate::critical::enter(|| {
        for (i, slot) in TIMER_SLOTS.iter().enumerate() {
            // SAFETY: all access to TIMER_SLOTS goes through
            // `critical::enter` (interrupts disabled on single-core
            // targets), so no concurrent access is possible.
            unsafe {
                if *slot.deadline.get() == 0 {
                    *slot.task.get() = task;
                    *slot.deadline.get() = deadline_us;
                    // SAFETY: inside critical::enter.
                    arm_next_deadline(deadline_us);
                    return Ok(TimerHandle {
                        slot: i as u8,
                        deadline: deadline_us,
                    });
                }
            }
        }
        Err(TimerQueueFull)
    })
}

/// Cancel a registered deadline. Safe to call with a stale handle: the
/// slot is only cleared if its deadline still matches the handle's
/// (plan.md [B7] — a cancelled `Sleep` must not free a *new* registration
/// that reused its slot).
pub(crate) fn cancel_deadline(handle: TimerHandle) {
    if let Some(slot) = TIMER_SLOTS.get(handle.slot as usize) {
        crate::critical::enter(|| {
            // SAFETY: guarded by critical::enter (see register).
            unsafe {
                if *slot.deadline.get() == handle.deadline {
                    *slot.deadline.get() = 0;
                }
            }
        });
    }
}

/// Scan for expired timers and wake their tasks. Call from the platform
/// timer ISR on every tick.
pub fn poll_timers(now_us: u64) {
    let mut woke_cooperative = false;
    let mut woke_preemptive = false;

    // Both arrays are swept in one critical section rather than two, so
    // the summary written at the end cannot be stale: nothing can arm a
    // nearer deadline between the sweep observing the arrays and the
    // write recording what it found.
    let swept = crate::critical::enter(|| {
        // SAFETY: all access to TIMER_SLOTS, PTASK_DEADLINES and
        // NEXT_DEADLINE goes through `critical::enter`, so there is no
        // concurrent access.
        unsafe {
            if now_us < *NEXT_DEADLINE.0.get() {
                // Nothing can have expired. This is the whole point: the
                // common tick reads this one value and no array at all.
                return false;
            }

            let mut earliest = NO_DEADLINE;

            for slot in &TIMER_SLOTS {
                let d = *slot.deadline.get();
                if d == 0 {
                    continue;
                }
                if now_us >= d {
                    *slot.deadline.get() = 0;
                    crate::waker::mark_ready(*slot.task.get());
                    woke_cooperative = true;
                } else if d < earliest {
                    earliest = d;
                }
            }

            for (task, slot) in PTASK_DEADLINES.iter().enumerate() {
                let d = *slot.deadline.get();
                if d == 0 {
                    continue;
                }
                if now_us >= d {
                    *slot.deadline.get() = 0;
                    crate::preempt::sched::unblock(task);
                    woke_preemptive = true;
                } else if d < earliest {
                    earliest = d;
                }
            }

            *NEXT_DEADLINE.0.get() = earliest;
        }
        true
    });

    if !swept {
        return;
    }

    // Only one hart ever calls this, whichever owns the periodic tick,
    // but the task a waker just marked ready could be hosted by the async
    // executor on any hart, including one idling in `wfi` waiting for
    // exactly this news. `mark_ready` only flips bitmap flags; without
    // the broadcast, an executor task idling on a non-tick-owning core
    // never notices its `Sleep` expired. Found on real dual-core
    // hardware: `smp_test.rs`'s monitor task hung this way, while the
    // single-core case passed, where the tick-owning hart and the only
    // hart are trivially the same one.
    //
    // No per-task hart affinity is tracked, so this goes to every other
    // hart rather than one. A spurious wake on an idle hart is a harmless
    // no-op; a real wake never delivered is not.
    if woke_cooperative {
        crate::waker::broadcast_reschedule();
    }

    // Same reasoning for a preemptive task's own blocking timeout
    // (`PriorityMutex::lock_timeout` and friends), which can unblock a
    // task that is not current on this hart at all.
    if woke_preemptive {
        let hart = crate::port::arch::hart_id();
        for other in 0..crate::config::MAX_HARTS {
            if other != hart {
                crate::port::arch::request_reschedule_on(other);
            }
        }
    }
}

// ── Preemptive-task block-with-timeout deadlines ────────────────────
//
// Backs `PriorityMutex::lock_timeout` (and later `Semaphore`/`Channel`
// timeouts): one deadline slot per preemptive task id (indexed by id), so
// a blocked task is unblocked by the tick when its deadline passes. The
// cooperative tier has its own wake mechanism (the waker bitmap); this is
// the preemptive-tier analog, keyed by task id and unblocking via
// `sched::unblock`.

/// Deadline slots, indexed by task id (0 = no deadline registered).
struct PtaskDeadline {
    deadline: UnsafeCell<u64>,
}

// SAFETY: all access goes through `critical::enter` (interrupts disabled,
// single-core), so there is no concurrent access.
unsafe impl Sync for PtaskDeadline {}

// Inline const avoids a named `const` item with interior mutability
// (clippy::declare_interior_mutable_const).
static PTASK_DEADLINES: [PtaskDeadline; crate::preempt::tcb::MAX_PTASKS] = [const {
    PtaskDeadline {
        deadline: UnsafeCell::new(0),
    }
};
    crate::preempt::tcb::MAX_PTASKS];

/// Register a wake-up deadline for a blocked preemptive task. Replaces any
/// previous registration for the same task.
pub(crate) fn register_ptask_deadline(deadline_us: u64, task: usize) -> Result<(), TimerQueueFull> {
    let Some(slot) = PTASK_DEADLINES.get(task) else {
        return Err(TimerQueueFull);
    };
    crate::critical::enter(|| {
        // SAFETY: guarded by critical::enter; single writer per task slot
        // (the blocking task), reader is the tick ISR.
        unsafe {
            *slot.deadline.get() = deadline_us;
            arm_next_deadline(deadline_us);
        }
    });
    Ok(())
}

/// Cancel a preemptive task's block deadline (e.g. it acquired the
/// resource before the deadline). No-op if none registered.
pub(crate) fn cancel_ptask_deadline(task: usize) {
    if let Some(slot) = PTASK_DEADLINES.get(task) {
        crate::critical::enter(|| {
            // SAFETY: guarded by critical::enter (see register).
            unsafe {
                *slot.deadline.get() = 0;
            }
        });
    }
}

/// Queue-full error returned by timer registration APIs (plan.md §4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimerQueueFull;

/// Test-only: clear every timer slot. Part of the global reset done by
/// [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    crate::critical::enter(|| {
        for slot in &TIMER_SLOTS {
            // SAFETY: all access to TIMER_SLOTS goes through
            // `critical::enter` (interrupts disabled, single-core), so no
            // concurrent access is possible here.
            unsafe {
                *slot.deadline.get() = 0;
            }
        }
        for slot in &PTASK_DEADLINES {
            // SAFETY: same guard as above.
            unsafe {
                *slot.deadline.get() = 0;
            }
        }
    });
}

/// Count of timer slots currently in use. Used by host-test reset/
/// inspection helpers and by [`crate::report`].
pub fn slots_in_use() -> usize {
    crate::critical::enter(|| {
        TIMER_SLOTS
            .iter()
            // SAFETY: all access to TIMER_SLOTS goes through
            // `critical::enter` (interrupts disabled), so reads are
            // exclusive.
            .filter(|slot| unsafe { *slot.deadline.get() != 0 })
            .count()
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_and_expire() {
        crate::kernel_test! {
            register_deadline(1000, crate::task::TaskId::new(3, 2)).unwrap();
            poll_timers(500); // not yet
            assert_eq!(crate::waker::next_ready(), None);

            poll_timers(1000); // now expired
            assert_eq!(crate::waker::next_ready(), Some(crate::task::TaskId::new(3, 2)));
        }
    }

    #[test]
    fn multiple_timers_independent() {
        crate::kernel_test! {
            register_deadline(100, crate::task::TaskId::new(1, 0)).unwrap();
            register_deadline(200, crate::task::TaskId::new(2, 0)).unwrap();

            poll_timers(150);
            assert_eq!(crate::waker::next_ready(), Some(crate::task::TaskId::new(1, 0)));
            assert_eq!(crate::waker::next_ready(), None);

            poll_timers(250);
            assert_eq!(crate::waker::next_ready(), Some(crate::task::TaskId::new(2, 0)));
        }
    }
}
