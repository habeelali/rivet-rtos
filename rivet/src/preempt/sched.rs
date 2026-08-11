//! Priority + round-robin scheduler for the preemptive tier.
//!
//! Priority selection is O(1) (plan.md §4.2 / [B13]): a `READY_BITMAP`
//! (bit p = priority p has a ready task) picks the winning priority level
//! via `31 - leading_zeros`, no array walk. Round-robin fairness *within*
//! a priority level (plan.md [B14]) costs a bounded scan of that level's
//! `QUEUES` word — see [`DISPATCH_SEQ`]'s docs for why a rotating-cursor
//! O(1) scheme was replaced with this after it proved unable to
//! guarantee fairness under concurrent multi-hart dispatch.
//!
//! Queue membership is authoritative for "ready": a task is in the queue
//! of its *effective* priority while `Ready`; Running/Blocked tasks are
//! not in any queue. Every state transition keeps the queues consistent
//! (see `tcb::set_state`, `tcb::set_effective_priority`, `register`).

use crate::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use super::tcb::{TaskState, MAX_PTASKS, NO_TASK, TASKS};

const MAX_HARTS: usize = crate::config::MAX_HARTS;

// plan.md Phase 12: cycle stamp of when each task most recently became
// Ready, consumed by `on_dispatch` to record `SchedulingWake` latency
// (ready → actually running). A plain array, not `UnsafeCell`-guarded
// like `timer.rs`'s deadline slots, because every write/read here is
// already inside a `critical::enter`-protected caller (`ready_add`,
// `on_dispatch`) and each slot is only ever touched by that single path —
// but stored as `u32` cycles (not `u64`) specifically to sidestep RV32/
// ARMv7-M's missing `AtomicU64`, matching every other cross-tick
// timestamp in this codebase.
#[cfg(feature = "latency-histograms")]
static READY_AT_CYCLE: [AtomicU32; MAX_PTASKS] = [const { AtomicU32::new(0) }; MAX_PTASKS];

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

/// Currently running task id **per hart**, or `NO_TASK` if that hart's
/// preemptive tier hasn't started yet (plan.md Phase 19). Indexed by
/// [`crate::port::arch::hart_id`] — always index 0 on every board except
/// RISC-V `virt` under `-smp`, so this is a single-element array with
/// identical behavior to the pre-Phase-19 single global everywhere else.
/// The ready queue itself (`READY_BITMAP`/`QUEUES` above) stays a single
/// shared structure, not per-hart: this is a global-run-queue SMP design
/// (every hart picks its next task from one shared pool, serialized by
/// `critical::enter`'s cross-hart lock at the call sites in
/// `preempt::on_tick`/`start`), not per-hart independent run queues.
#[cfg(not(loom))]
static CURRENT: [AtomicUsize; MAX_HARTS] = [const { AtomicUsize::new(NO_TASK) }; MAX_HARTS];
#[cfg(loom)]
loom::lazy_static! {
    static ref CURRENT: [AtomicUsize; MAX_HARTS] = core::array::from_fn(|_| AtomicUsize::new(NO_TASK));
}

/// Global monotonic counter, incremented once per real dispatch (any
/// priority). Source of [`DISPATCH_SEQ`]'s values.
#[cfg(not(loom))]
static DISPATCH_COUNTER: AtomicU32 = AtomicU32::new(0);
#[cfg(loom)]
loom::lazy_static! {
    static ref DISPATCH_COUNTER: AtomicU32 = AtomicU32::new(0);
}

/// Per-task "last actually dispatched at" sequence number (plan.md
/// [B14]); see [`on_dispatch`] and [`schedule`].
///
/// plan.md Phase 30: replaces an earlier "rotating offset, nearest bit"
/// round-robin (both a single global offset, and a later per-priority-
/// level refinement of it). Both were vulnerable to the same class of
/// bug: with 3+ tasks tied at one priority and irregular ready/blocked
/// toggling (e.g. two `PriorityMutex` contenders plus a third task that
/// never blocks), and — critically — with *multiple harts* concurrently
/// pulling from the same shared ready queue, the "nearest bit from a
/// shared cursor" heuristic could settle into a small cycle between
/// just two of the ready ids (typically whichever two tasks keep
/// re-queuing each other fastest) and never reach a third that's
/// genuinely waiting, no matter how many ticks pass. Confirmed as a
/// real, reproducible, deterministic starvation on both QEMU (kernel-
/// wide, not board-specific) and real ESP32-S3 hardware — worse under
/// real dual-core concurrent dispatch than under a single hart, and
/// worse still the higher the ready-queue "pressure" from a never-
/// blocking sibling (real 240MHz hardware showing near-total
/// starvation where slower QEMU emulation only showed partial bias).
///
/// This replaces the whole rotating-cursor approach with true
/// least-recently-dispatched selection: every real dispatch stamps the
/// task with the next value from [`DISPATCH_COUNTER`], and `schedule()`
/// picks, among the ready bits at the winning priority, whichever has
/// the *smallest* stamp — i.e. whichever has waited longest since it
/// last ran. This is provably starvation-free for any *bounded* number
/// of ready siblings regardless of hart count or toggle pattern: a task
/// that hasn't run in a while can only get "older" relative to its
/// siblings, so it is eventually the unique minimum and must be picked.
/// The cost is an O(popcount) scan of the winning priority's queue word
/// instead of O(1) — `MAX_PTASKS` is a small, fixed, compile-time bound
/// (already scanned linearly elsewhere in this crate, e.g.
/// `PriorityMutex::highest_waiter_priority`), so this trades a few extra
/// bounded cycles for a real fairness guarantee, which correctness comes
/// first here.
#[cfg(not(loom))]
static DISPATCH_SEQ: [AtomicU32; MAX_PTASKS] = [const { AtomicU32::new(0) }; MAX_PTASKS];
#[cfg(loom)]
loom::lazy_static! {
    static ref DISPATCH_SEQ: [AtomicU32; MAX_PTASKS] = core::array::from_fn(|_| AtomicU32::new(0));
}

/// The calling hart's currently-running task, or `None` if that hart's
/// preemptive tier hasn't started yet.
pub fn current() -> Option<usize> {
    let c = CURRENT[crate::port::arch::hart_id()].load(Ordering::Acquire);
    if c == NO_TASK {
        None
    } else {
        Some(c)
    }
}

/// Set the *calling hart's* currently-running task.
pub fn set_current(id: usize) {
    CURRENT[crate::port::arch::hart_id()].store(id, Ordering::Release);
}

/// Add task `id` to its effective-priority ready queue (called by
/// `tcb::set_state(Ready)` and `register`). Interrupts must be off or the
/// caller must be the sole mutator.
pub fn ready_add(id: usize) {
    if id >= MAX_PTASKS {
        return;
    }
    #[cfg(feature = "latency-histograms")]
    READY_AT_CYCLE[id].store(
        crate::port::arch::cycle_count() as u32,
        Ordering::Relaxed,
    );
    let prio = TASKS[id].effective_priority.load(Ordering::Acquire) as usize;
    QUEUES[prio].fetch_or(1u32 << id, Ordering::Release);
    READY_BITMAP.fetch_or(1u32 << prio, Ordering::Release);
    wake_other_harts();
}

/// plan.md Phase 19: a task becoming ready doesn't necessarily do so on
/// the hart that should run it — with this crate's global run queue, any
/// hart could be the right one, including one that's currently idling
/// with no ready work and (per `rivet-arch-riscv::clint`'s single-tick-
/// owner design) no timer of its own to eventually notice. Broadcasting a
/// reschedule IPI to every *other* hart on every `ready_add` is the
/// simple, always-correct choice over tracking which hart is actually
/// idle — some of these traps will find nothing to do and just resume
/// what they were already running, which is a bounded, cheap cost.
/// `MAX_HARTS > 1` is a compile-time-constant `false` on every
/// single-hart board, so the loop is dead code there (zero runtime cost,
/// confirmed identical golden output).
#[inline]
fn wake_other_harts() {
    if MAX_HARTS > 1 {
        let me = crate::port::arch::hart_id();
        for hart in 0..MAX_HARTS {
            if hart != me {
                crate::port::arch::request_reschedule_on(hart);
            }
        }
    }
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

/// Select the next task to run: highest effective-priority queue with a
/// ready task, least-recently-dispatched within it (plan.md [B14]/[B13]
/// — see [`DISPATCH_SEQ`]'s docs for why this replaced a rotating-cursor
/// scheme).
///
/// Pure: has NO side effects; dispatch sequence numbers only advance via
/// [`on_dispatch`] at real context switches.
pub fn schedule() -> Option<usize> {
    let mut bitmap = READY_BITMAP.load(Ordering::Acquire);
    loop {
        if bitmap == 0 {
            return None;
        }
        let prio = (31 - bitmap.leading_zeros()) as usize;
        let word = QUEUES[prio].load(Ordering::Acquire);
        if word == 0 {
            // Stale bitmap bit: `READY_BITMAP` and `QUEUES` are two
            // separate atomics (`ready_add`/`ready_remove` each touch
            // both, but not as one atomic RMW), so a reader can observe
            // the bitmap bit set for an instant after the last task at
            // that priority already left the queue — normally
            // self-correcting on the very next read. But if a caller
            // ever sets `READY_BITMAP`'s bit without a matching queue
            // entry surviving (a real bug elsewhere, not a case this
            // function can prevent), the old code returned `None` here
            // permanently: every future `schedule()` call would keep
            // hitting the same empty word at the same top-priority bit
            // and never look lower, wedging the *entire* scheduler even
            // though lower-priority tasks are genuinely ready. Self-heal
            // instead: clear the stale bit here (a plain diagnostic
            // correction, not a real state change — nothing was ever
            // actually queued at `prio`) and fall through to whatever
            // priority is next, exactly as if the stale bit had never
            // been set. A single scheduling call degrading to "skip one
            // empty priority level" is a far cheaper failure mode than
            // "the scheduler never recovers".
            bitmap = READY_BITMAP.fetch_and(!(1u32 << prio), Ordering::AcqRel) & !(1u32 << prio);
            continue;
        }
        // Least-recently-dispatched among this priority's ready bits
        // (plan.md Phase 30): a bounded scan of at most `MAX_PTASKS` set
        // bits, not the O(1) rotating-cursor lookup this replaced — see
        // `DISPATCH_SEQ`'s docs for why the trade is worth it.
        let mut best_id = None;
        let mut best_seq = u32::MAX;
        let mut w = word;
        while w != 0 {
            let id = w.trailing_zeros() as usize;
            w &= w - 1; // clear the lowest set bit
            if id < MAX_PTASKS {
                let seq = DISPATCH_SEQ[id].load(Ordering::Relaxed);
                if seq < best_seq {
                    best_seq = seq;
                    best_id = Some(id);
                }
            }
        }
        if best_id.is_some() {
            return best_id;
        }
        // Every set bit was `>= MAX_PTASKS` (shouldn't happen); fall back
        // to a scan of the word, matching the old code's own fallback.
        return word
            .trailing_zeros()
            .checked_rem(MAX_PTASKS as u32)
            .map(|i| i as usize);
    }
}

/// Record that task `id` was actually dispatched: stamp it as most
/// recently dispatched (plan.md [B14] — only at real context switches;
/// see [`DISPATCH_SEQ`]'s docs).
pub fn on_dispatch(id: usize) {
    if id < MAX_PTASKS {
        let seq = DISPATCH_COUNTER.fetch_add(1, Ordering::Relaxed).wrapping_add(1);
        DISPATCH_SEQ[id].store(seq, Ordering::Relaxed);
    }
    #[cfg(feature = "latency-histograms")]
    if id < MAX_PTASKS {
        let ready_at = READY_AT_CYCLE[id].load(Ordering::Relaxed);
        // `ready_at == 0` means never recorded (shouldn't happen once a
        // task has gone through at least one Ready transition, but a
        // task's very first dispatch straight from `register()` could
        // race the histogram feature's own bookkeeping order — skip
        // rather than record a bogus latency).
        if ready_at != 0 {
            // 32-bit truncated subtraction (matches `ready_at`'s stored
            // width): correct under wraparound as long as the actual
            // ready-to-running latency never exceeds 2^32 cycles, which a
            // scheduling-latency measurement always satisfies in practice.
            let now = crate::port::arch::cycle_count() as u32;
            crate::latency::record(
                crate::latency::Kind::SchedulingWake,
                now.wrapping_sub(ready_at) as u64,
            );
        }
    }
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
    for c in CURRENT.iter() {
        c.store(NO_TASK, Ordering::Release);
    }
    DISPATCH_COUNTER.store(0, Ordering::Release);
    for s in DISPATCH_SEQ.iter() {
        s.store(0, Ordering::Release);
    }
    READY_BITMAP.store(0, Ordering::Release);
    for q in QUEUES.iter() {
        q.store(0, Ordering::Release);
    }
    #[cfg(feature = "latency-histograms")]
    for s in READY_AT_CYCLE.iter() {
        s.store(0, Ordering::Release);
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

/// Kani proof harnesses (plan.md Phase 17): the core scheduler invariant
/// that `schedule()`'s O(1) selection depends on — "priority `p`'s bit in
/// `READY_BITMAP` is set if and only if `QUEUES[p]` is non-empty" — proven
/// for the two operations that mutate it, not just tested against a
/// handful of example sequences (the `#[test]`s above). `MAX_PTASKS` is
/// fixed at compile time via `RIVET_*` env vars (default 16); these
/// harnesses run against whatever the default build configures, which is
/// enough to exercise every code path in `ready_add`/`ready_remove` (the
/// "word becomes empty, clear the bitmap bit" branch included) regardless
/// of the exact width.
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// `ready_add`/`ready_remove` never observe a bit set in
    /// `READY_BITMAP` whose `QUEUES` word is actually empty, or vice
    /// versa — checked after every mutation, not just at the end, so a
    /// harness that fails pins down exactly which operation broke it.
    fn assert_bitmap_matches_queues() {
        let bitmap = READY_BITMAP.load(Ordering::Acquire);
        for prio in 0..32usize {
            let word = QUEUES[prio].load(Ordering::Acquire);
            let bit_set = (bitmap & (1u32 << prio)) != 0;
            assert_eq!(
                bit_set,
                word != 0,
                "READY_BITMAP bit {prio} = {bit_set} but QUEUES[{prio}] = {word:#x}"
            );
        }
    }

    /// One `ready_add` then one `ready_remove` of the same task, at an
    /// arbitrary (in-range) id and priority, preserves the invariant at
    /// every step — including the empty-initial-state precondition,
    /// which `reset_for_test` establishes deterministically so Kani
    /// doesn't have to reason about whatever prior harness state might
    /// otherwise leak in.
    #[kani::proof]
    fn ready_add_remove_preserves_invariant() {
        reset_for_test();
        assert_bitmap_matches_queues();

        let id: usize = kani::any();
        kani::assume(id < MAX_PTASKS);
        let prio: u8 = kani::any();
        kani::assume((prio as usize) < 32);
        TASKS[id].effective_priority.store(prio, Ordering::Release);

        ready_add(id);
        assert_bitmap_matches_queues();

        ready_remove(id);
        assert_bitmap_matches_queues();
    }

    /// Two distinct tasks at arbitrary (possibly equal) priorities,
    /// added and removed in an arbitrary order — the invariant must hold
    /// after every single mutation, including the "two tasks share a
    /// priority word, one leaves, the bitmap bit must survive" case that
    /// a single-task harness can't reach.
    #[kani::proof]
    fn two_task_interleaving_preserves_invariant() {
        reset_for_test();

        let id_a: usize = kani::any();
        let id_b: usize = kani::any();
        kani::assume(id_a < MAX_PTASKS);
        kani::assume(id_b < MAX_PTASKS);
        kani::assume(id_a != id_b);
        let prio_a: u8 = kani::any();
        let prio_b: u8 = kani::any();
        kani::assume((prio_a as usize) < 32);
        kani::assume((prio_b as usize) < 32);
        TASKS[id_a]
            .effective_priority
            .store(prio_a, Ordering::Release);
        TASKS[id_b]
            .effective_priority
            .store(prio_b, Ordering::Release);

        ready_add(id_a);
        assert_bitmap_matches_queues();
        ready_add(id_b);
        assert_bitmap_matches_queues();
        ready_remove(id_a);
        assert_bitmap_matches_queues();
        ready_remove(id_b);
        assert_bitmap_matches_queues();
    }
}
