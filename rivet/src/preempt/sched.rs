//! Priority + round-robin scheduler for the preemptive tier.
//!
//! O(1) selection (plan.md §4.2 / [B13]): a `READY_BITMAP` (bit p =
//! priority p has a ready task) plus per-priority `QUEUES` words (bit i =
//! ptask i ready at that priority). `schedule()` is two bit operations —
//! `31 - leading_zeros` then `trailing_zeros` with an RR rotation — with
//! NO array walk, so worst-case scheduling latency is independent of
//! `MAX_PTASKS`.
//!
//! Queue membership is authoritative for "ready": a task is in the queue
//! of its *effective* priority while `Ready`; Running/Blocked tasks are
//! not in any queue. Every state transition keeps the queues consistent
//! (see `tcb::set_state`, `tcb::set_effective_priority`, `register`).

use crate::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use super::tcb::{TaskState, MAX_PTASKS, NO_TASK, TASKS};

/// Bit p set => priority p has at least one ready ptask.
#[cfg(not(loom))]
static READY_BITMAP: AtomicU32 = AtomicU32::new(0);
#[cfg(loom)]
loom::lazy_static! {
    static ref READY_BITMAP: AtomicU32 = AtomicU32::new(0);
}

/// Bit i set => ptask i is ready, queued at its *effective* priority.
#[cfg(not(loom))]
static QUEUES: [AtomicU32; 32] = [const { AtomicU32::new(0) }; 32];
#[cfg(loom)]
loom::lazy_static! {
    static ref QUEUES: [AtomicU32; 32] = core::array::from_fn(|_| AtomicU32::new(0));
}

/// Currently running task id, or `NO_TASK` if the preemptive tier hasn't
/// started yet (still in cooperative-only / boot context).
#[cfg(not(loom))]
static CURRENT: AtomicUsize = AtomicUsize::new(NO_TASK);
#[cfg(loom)]
loom::lazy_static! {
    static ref CURRENT: AtomicUsize = AtomicUsize::new(NO_TASK);
}

/// Round-robin rotation offset (the last-dispatched id + 1, plan.md
/// [B14]); see [`on_dispatch`].
#[cfg(not(loom))]
static RR_OFFSET: AtomicUsize = AtomicUsize::new(0);
#[cfg(loom)]
loom::lazy_static! {
    static ref RR_OFFSET: AtomicUsize = AtomicUsize::new(0);
}

pub fn current() -> Option<usize> {
    let c = CURRENT.load(Ordering::Acquire);
    if c == NO_TASK {
        None
    } else {
        Some(c)
    }
}

pub fn set_current(id: usize) {
    CURRENT.store(id, Ordering::Release);
}

/// Add task `id` to its effective-priority ready queue (called by
/// `tcb::set_state(Ready)` and `register`). Interrupts must be off or the
/// caller must be the sole mutator.
pub fn ready_add(id: usize) {
    if id >= MAX_PTASKS {
        return;
    }
    let prio = TASKS[id].effective_priority.load(Ordering::Acquire) as usize;
    QUEUES[prio].fetch_or(1u32 << id, Ordering::Release);
    READY_BITMAP.fetch_or(1u32 << prio, Ordering::Release);
}

/// Remove task `id` from its ready queue (called by
/// `tcb::set_state(Running/Blocked)`).
pub fn ready_remove(id: usize) {
    if id >= MAX_PTASKS {
        return;
    }
    let prio = TASKS[id].effective_priority.load(Ordering::Acquire) as usize;
    let prev = QUEUES[prio].fetch_and(!(1u32 << id), Ordering::AcqRel);
    if prev == (1u32 << id) {
        // The queue word is now empty — clear the priority bit.
        READY_BITMAP.fetch_and(!(1u32 << prio), Ordering::AcqRel);
    }
}

/// Move a *Ready* task between queues after its effective priority
/// changed (priority inheritance, plan.md [B11]). No-op for Running or
/// Blocked tasks (they are not queued). `old_prio` is the priority the
/// task was queued under (read *before* the caller stored the new value —
/// reading it back now would clear the wrong queue word and leave a stale
/// bit behind).
pub fn on_effective_priority_change(id: usize, old_prio: u8, new_prio: u8) {
    if id >= MAX_PTASKS || old_prio == new_prio {
        return;
    }
    if TASKS[id].state() != TaskState::Ready {
        return;
    }
    let bit = 1u32 << id;
    // Clear the OLD queue word; drop the priority bit if it emptied.
    let prev = QUEUES[old_prio as usize].fetch_and(!bit, Ordering::AcqRel);
    if prev == bit {
        READY_BITMAP.fetch_and(!(1u32 << old_prio), Ordering::AcqRel);
    }
    // Add to the NEW queue word and raise its priority bit.
    QUEUES[new_prio as usize].fetch_or(bit, Ordering::Release);
    READY_BITMAP.fetch_or(1u32 << new_prio, Ordering::Release);
}

/// Select the next task to run in O(1): highest effective-priority queue
/// with a ready task, round-robin within it via a rotation.
///
/// Pure: has NO side effects (plan.md [B14]); the rotation offset only
/// advances via [`on_dispatch`] at real context switches.
pub fn schedule() -> Option<usize> {
    let bitmap = READY_BITMAP.load(Ordering::Acquire);
    if bitmap == 0 {
        return None;
    }
    let prio = (31 - bitmap.leading_zeros()) as usize;
    let word = QUEUES[prio].load(Ordering::Acquire);
    if word == 0 {
        // Queue drained between the loads; retry once.
        return schedule_retry(bitmap, prio, word);
    }
    let rr = (RR_OFFSET.load(Ordering::Relaxed) & 31) as u32;
    let rotated = word.rotate_right(rr);
    let idx = rotated.trailing_zeros();
    let id = ((idx + rr) & 31) as usize;
    if id >= MAX_PTASKS {
        // Bit outside the registry width (shouldn't happen); fall back to
        // a scan of the word.
        return word
            .trailing_zeros()
            .checked_rem(MAX_PTASKS as u32)
            .map(|i| i as usize);
    }
    Some(id)
}

fn schedule_retry(_bitmap: u32, prio: usize, _word: u32) -> Option<usize> {
    // The queue word was drained concurrently (ISR path); re-read.
    let word = QUEUES[prio].load(Ordering::Acquire);
    if word == 0 {
        return None;
    }
    let rr = (RR_OFFSET.load(Ordering::Relaxed) & 31) as u32;
    let rotated = word.rotate_right(rr);
    let idx = rotated.trailing_zeros();
    let id = ((idx + rr) & 31) as usize;
    (id < MAX_PTASKS).then_some(id)
}

/// Record that task `id` was actually dispatched: advance the round-robin
/// rotation past it (plan.md [B14] — only at real context switches).
pub fn on_dispatch(id: usize) {
    RR_OFFSET.store(id + 1, Ordering::Relaxed);
}

/// Should a tick-time reschedule actually switch away from the running
/// task? True if `candidate` is a *different*, ready task whose effective
/// priority is >= the running task's (strictly-higher preempts; equal
/// priority round-robins on tick, matching classic RTOS tick semantics).
pub fn should_preempt(candidate: usize, running: usize) -> bool {
    if candidate == running {
        return false;
    }
    let cand_prio = TASKS[candidate].effective_priority.load(Ordering::Acquire);
    let run_prio = TASKS[running].effective_priority.load(Ordering::Acquire);
    cand_prio >= run_prio
}

/// Mark task `id` ready (e.g. after a mutex/semaphore unblocks it).
pub fn unblock(id: usize) {
    if let Some(tcb) = super::tcb::get(id) {
        tcb.set_state(id, TaskState::Ready);
    }
}

/// Mark the currently running task blocked. Does not switch context —
/// caller must trigger a reschedule separately.
pub fn block_current() {
    if let Some(id) = current() {
        if let Some(tcb) = super::tcb::get(id) {
            tcb.set_state(id, TaskState::Blocked);
        }
    }
}

/// Test-only: reset scheduler globals. Part of the global reset done by
/// [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    CURRENT.store(NO_TASK, Ordering::Release);
    RR_OFFSET.store(0, Ordering::Release);
    READY_BITMAP.store(0, Ordering::Release);
    for q in QUEUES.iter() {
        q.store(0, Ordering::Release);
    }
}

/// Test-only: peek at the ready bitmap (host tests).
#[cfg(feature = "test-support")]
pub fn ready_bitmap_for_test() -> u32 {
    READY_BITMAP.load(Ordering::Acquire)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preempt::tcb;

    #[test]
    fn schedule_picks_highest_priority() {
        crate::kernel_test! {
            let _a = tcb::register(0x1000, 0).unwrap();
            let b = tcb::register(0x2000, 5).unwrap();
            let _c = tcb::register(0x3000, 2).unwrap();
            assert_eq!(schedule(), Some(b));
        }
    }

    #[test]
    fn schedule_round_robins_same_priority() {
        crate::kernel_test! {
            let a = tcb::register(0x1000, 1).unwrap();
            let b = tcb::register(0x2000, 1).unwrap();

            let mut saw_a = false;
            let mut saw_b = false;
            for _ in 0..10 {
                let id = schedule().unwrap();
                match id {
                    id if id == a => saw_a = true,
                    id if id == b => saw_b = true,
                    _ => {}
                }
                on_dispatch(id); // an actual switch occurred
            }
            assert!(saw_a && saw_b, "both tied tasks must be dispatched");
        }
    }

    #[test]
    fn rr_advances_only_on_dispatch() {
        crate::kernel_test! {
            let a = tcb::register(0x1000, 1).unwrap();
            let b = tcb::register(0x2000, 1).unwrap();

            // No switch yet: repeated schedule() calls must keep selecting
            // the same task (plan.md [B14]).
            assert_eq!(schedule(), Some(a));
            assert_eq!(schedule(), Some(a));

            // After an actual dispatch of `a`, the rotation moves past it.
            on_dispatch(a);
            assert_eq!(schedule(), Some(b));
            on_dispatch(b);
            assert_eq!(schedule(), Some(a));
        }
    }

    #[test]
    fn blocked_tasks_not_scheduled() {
        crate::kernel_test! {
            let a = tcb::register(0x1000, 3).unwrap();
            let b = tcb::register(0x2000, 1).unwrap();
            tcb::get(a).unwrap().set_state(a, TaskState::Blocked);
            assert_eq!(schedule(), Some(b));
        }
    }

    #[test]
    fn should_preempt_logic() {
        crate::kernel_test! {
            let low = tcb::register(0x1000, 1).unwrap();
            let high = tcb::register(0x2000, 5).unwrap();
            assert!(should_preempt(high, low));
            assert!(!should_preempt(low, high));
            assert!(!should_preempt(low, low));
        }
    }

    #[test]
    fn effective_priority_used_for_scheduling() {
        crate::kernel_test! {
            let low = tcb::register(0x1000, 1).unwrap();
            let _high = tcb::register(0x2000, 5).unwrap();
            // Boost `low`'s effective priority above `_high` and move it
            // between queues (simulating priority inheritance).
            tcb::get(low)
                .unwrap()
                .set_effective_priority(low, 9);
            assert_eq!(schedule(), Some(low));
        }
    }

    #[test]
    fn ready_queue_consistent_with_state() {
        crate::kernel_test! {
            let a = tcb::register(0x1000, 2).unwrap();
            let b = tcb::register(0x2000, 2).unwrap();
            // Both ready: bitmap has priority 2.
            assert_ne!(ready_bitmap_for_test() & (1 << 2), 0);
            // Block b: its queue bit clears.
            tcb::get(b).unwrap().set_state(b, TaskState::Blocked);
            assert_eq!(schedule(), Some(a));
            // Running tasks are not queued: dispatch a.
            tcb::get(a).unwrap().set_state(a, TaskState::Running);
            assert_eq!(schedule(), None, "nothing ready");
            tcb::get(b).unwrap().set_state(b, TaskState::Ready);
            assert_eq!(schedule(), Some(b));
        }
    }
}
