//! Periods, drift-corrected periodic wake, and CPU-budget enforcement
//! (plan.md Phase 11).
//!
//! Both are configured per task by id via [`set_period_us`]/
//! [`set_budget_us`] (wired up through [`crate::preempt::TaskHandle`]).
//! `period_us` is consumed by [`wait_period`], which a periodic task calls
//! once per iteration instead of `sleep_ms` — the deadline is computed
//! from the *previous* deadline, not `now`, so per-iteration jitter never
//! accumulates into long-term drift (the same technique `soak`/`drift_test`
//! already exercise for the timer queue itself). `budget_us` is checked
//! every tick against the task's accumulated busy time *within the current
//! period* (via [`crate::exec_time`]); exceeding it raises
//! [`crate::fault::FaultKind::BudgetExceeded`] through the normal fault
//! policy (`Panic` or `IsolateTask` — no new fault-handling path needed).
//!
//! Deadline/snapshot storage follows the exact pattern already established
//! by `crate::timer`'s `PTASK_DEADLINES` (`UnsafeCell` array, `unsafe impl
//! Sync`, every access under `critical::enter`) since RV32/ARMv7-M have no
//! native 64-bit atomics.

use core::cell::UnsafeCell;

use crate::preempt::tcb::MAX_PTASKS;
use crate::sync::atomic::{AtomicU32, Ordering};

struct PeriodSlot {
    /// Next period deadline, in microseconds since boot (0 = not yet
    /// anchored — the first `wait_period()` call anchors it to `now`).
    next_us: UnsafeCell<u64>,
    /// `exec_time::busy_cycles(id)` snapshot taken at the start of the
    /// current period, so budget checks compare *this period's* busy
    /// time, not the task's lifetime total.
    budget_start_cycles: UnsafeCell<u64>,
}

// SAFETY: every field access goes through `critical::enter` (interrupts
// disabled, single-core), so there is no concurrent access.
unsafe impl Sync for PeriodSlot {}

#[cfg(not(loom))]
static PERIOD_US: [AtomicU32; MAX_PTASKS] = [const { AtomicU32::new(0) }; MAX_PTASKS];
#[cfg(loom)]
loom::lazy_static! {
    static ref PERIOD_US: [AtomicU32; MAX_PTASKS] = core::array::from_fn(|_| AtomicU32::new(0));
}

#[cfg(not(loom))]
static BUDGET_US: [AtomicU32; MAX_PTASKS] = [const { AtomicU32::new(0) }; MAX_PTASKS];
#[cfg(loom)]
loom::lazy_static! {
    static ref BUDGET_US: [AtomicU32; MAX_PTASKS] = core::array::from_fn(|_| AtomicU32::new(0));
}
static SLOTS: [PeriodSlot; MAX_PTASKS] = [const {
    PeriodSlot {
        next_us: UnsafeCell::new(0),
        budget_start_cycles: UnsafeCell::new(0),
    }
}; MAX_PTASKS];

/// Configure task `id`'s period (microseconds). `0` disables
/// [`wait_period`] for that task (it returns immediately).
pub fn set_period_us(id: usize, period_us: u32) {
    if let Some(slot) = PERIOD_US.get(id) {
        slot.store(period_us, Ordering::Release);
    }
}

/// Configure task `id`'s per-period CPU budget (microseconds, estimated —
/// see [`crate::exec_time::estimate_us_from_cycles`]). `0` disables budget
/// enforcement for that task.
pub fn set_budget_us(id: usize, budget_us: u32) {
    if let Some(slot) = BUDGET_US.get(id) {
        slot.store(budget_us, Ordering::Release);
    }
}

pub fn period_us(id: usize) -> u32 {
    PERIOD_US.get(id).map_or(0, |s| s.load(Ordering::Acquire))
}

pub fn budget_us(id: usize) -> u32 {
    BUDGET_US.get(id).map_or(0, |s| s.load(Ordering::Acquire))
}

/// Block the calling preemptive task until its next period boundary.
/// Drift-corrected: the deadline is `previous_deadline + period`, not
/// `now + period`, so a task that occasionally runs a bit late never
/// permanently shifts its schedule. No-op if the calling task has no
/// period configured ([`set_period_us`] not called, or called with `0`)
/// or isn't a preemptive task.
pub fn wait_period() {
    let Some(me) = crate::preempt::sched::current() else {
        return;
    };
    let period = period_us(me) as u64;
    if period == 0 {
        return;
    }
    let Some(slot) = SLOTS.get(me) else {
        return;
    };
    let next = crate::critical::enter(|| {
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe {
            let prev = *slot.next_us.get();
            let next = if prev == 0 {
                crate::port::board::now_us().wrapping_add(period)
            } else {
                prev.wrapping_add(period)
            };
            *slot.next_us.get() = next;
            *slot.budget_start_cycles.get() = crate::exec_time::busy_cycles(me);
            next
        }
    });
    crate::preempt::sleep_until(next);
}

/// Called from [`crate::preempt::on_tick`] for the currently-running task:
/// true if it has exceeded its configured budget within the current
/// period. Always false if no budget is configured for `id`.
pub(crate) fn check_budget(id: usize) -> bool {
    let budget = budget_us(id);
    if budget == 0 {
        return false;
    }
    let Some(slot) = SLOTS.get(id) else {
        return false;
    };
    // SAFETY: guarded by `critical::enter` (see module docs).
    let start = crate::critical::enter(|| unsafe { *slot.budget_start_cycles.get() });
    // `busy_cycles_live`, not `busy_cycles`: called from `on_tick` for the
    // task that's still mid-dispatch right now (see its docs) — a task
    // that never yields must still be checkable.
    let used_cycles = crate::exec_time::busy_cycles_live(id).wrapping_sub(start);
    let used_us = crate::exec_time::estimate_us_from_cycles(used_cycles);
    used_us > budget as u64
}

/// Test-only: reset every period/budget slot. Part of the global reset
/// done by [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    for s in PERIOD_US.iter() {
        s.store(0, Ordering::Relaxed);
    }
    for s in BUDGET_US.iter() {
        s.store(0, Ordering::Relaxed);
    }
    crate::critical::enter(|| {
        for slot in SLOTS.iter() {
            // SAFETY: guarded by `critical::enter` (see module docs).
            unsafe {
                *slot.next_us.get() = 0;
                *slot.budget_start_cycles.get() = 0;
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wait_period_noop_without_config() {
        crate::kernel_test! {
            // No current task, no period configured: must not panic.
            wait_period();
        }
    }

    #[test]
    fn check_budget_false_without_config() {
        crate::kernel_test! {
            assert!(!check_budget(0));
        }
    }

    #[test]
    fn set_and_read_period_budget() {
        crate::kernel_test! {
            set_period_us(2, 5000);
            set_budget_us(2, 1000);
            assert_eq!(period_us(2), 5000);
            assert_eq!(budget_us(2), 1000);
        }
    }
}
