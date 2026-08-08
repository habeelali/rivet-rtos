//! Fixed-size timer queue backing [`crate::time::Sleep`].
//!
//! A task calling `Sleep::<MICROS>::new().await` registers a deadline here
//! instead of busy-polling — the arch timer ISR (`riscv::timer_tick` /
//! `cortex_m::systick_handler`) calls [`poll_timers`] on every tick, which
//! wakes any task whose deadline has passed. This is what makes
//! `arch::sleep()` (WFI) a real power-saving wait instead of a spin loop:
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
    crate::critical::enter(|| {
        for slot in &TIMER_SLOTS {
            // SAFETY: all access to TIMER_SLOTS goes through
            // `critical::enter` (interrupts disabled on single-core
            // targets), so no concurrent access is possible.
            unsafe {
                let d = *slot.deadline.get();
                if d != 0 && now_us >= d {
                    *slot.deadline.get() = 0;
                    crate::waker::mark_ready(*slot.task.get());
                }
            }
        }
    });
    poll_ptask_deadlines(now_us);
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

/// Wake preemptive tasks whose block deadline has passed.
fn poll_ptask_deadlines(now_us: u64) {
    crate::critical::enter(|| {
        for (task, slot) in PTASK_DEADLINES.iter().enumerate() {
            // SAFETY: guarded by critical::enter (see register).
            unsafe {
                let d = *slot.deadline.get();
                if d != 0 && now_us >= d {
                    *slot.deadline.get() = 0;
                    crate::preempt::sched::unblock(task);
                }
            }
        }
    });
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

/// Test-only: count of timer slots currently in use. Part of the global
/// reset/inspection helpers behind `feature = "test-support"` (host tests).
#[cfg(feature = "test-support")]
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
