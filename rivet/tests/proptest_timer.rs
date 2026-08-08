//! Property-based timer-queue tests (plan.md §1.4).
//!
//! Random sequences of `Register(deadline, prio, idx)` / `Advance(dt)`
//! ops are played against the real [`rivet::timer`] queue. Invariants:
//!
//! - every registered deadline fires **exactly once**, at the first
//!   `poll_timers(now)` with `now >= deadline`;
//! - after firing, the slot returns to free (registering
//!   [`rivet::timer::MAX_TIMERS`] fresh timers always succeeds);
//! - no spurious wakes (nothing fires before its deadline).
//!
//! The `drop`-related leak invariant ([B7], cancelled `Sleep`s must not
//! leak slots) is added in Phase 2.5 when `Sleep`/`TimerHandle` gain
//! cancellation; the queue is exercised here in its current form.

use std::collections::HashMap;

use proptest::prelude::*;

const MAX_TIMERS: usize = 16;

#[derive(Debug, Clone)]
enum Op {
    /// Register a wake-up for (prio, idx) at a deadline in [0, 10_000).
    Register(u8, u8, u64),
    /// Advance simulated time by dt and poll the queue.
    Advance(u64),
}

fn op_strategy() -> impl Strategy<Value = Op> {
    prop_oneof![
        // Deadlines are >= 1: the queue uses 0 as its "slot free" sentinel,
        // so a 0 deadline could never fire.
        3 => (0..=31u8, 0..=15u8, 1..10_000u64).prop_map(|(p, i, d)| Op::Register(p, i, d)),
        5 => (1..=1_000u64).prop_map(Op::Advance),
    ]
}

fn run_ops(ops: &[Op]) {
    rivet::kernel_test! {
        // Model: pending (prio, idx) -> deadline.
        let mut pending: HashMap<(u8, u8), u64> = HashMap::new();
        let mut now: u64 = 0;

        for op in ops {
            match *op {
                Op::Register(prio, idx, deadline) => {
                    // Model discipline: a task only sleeps once at a time —
                    // skip re-registering a still-pending (prio, idx) in
                    // BOTH the model and the real queue (a real task's
                    // `Sleep` future registers exactly once per sleep).
                    if pending.contains_key(&(prio, idx)) {
                        continue;
                    }
                    if pending.len() >= MAX_TIMERS {
                        // Real queue would panic on overflow; the model
                        // just skips to stay in-bounds.
                        continue;
                    }
                    rivet::timer::register_deadline(deadline, rivet::task::TaskId::new(prio, idx))
                        .expect("timer queue full during model");
                    pending.insert((prio, idx), deadline);
                }
                Op::Advance(dt) => {
                    now = now.saturating_add(dt);
                    rivet::timer::poll_timers(now);

                    // Expected wakes: first poll with now >= deadline.
                    let expected: Vec<(u8, u8)> = pending
                        .iter()
                        .filter(|(_, &d)| d <= now)
                        .map(|(&k, _)| k)
                        .collect();

                    // Actual wakes: drain the waker bitmap.
                    let mut actual = Vec::new();
                    while let Some(id) = rivet::waker::next_ready() {
                        actual.push((id.priority(), id.index()));
                    }

                    // Invariants:
                    // (a) nothing fires before its deadline.
                    for &(p, i) in &actual {
                        let d = pending[&(p, i)];
                        assert!(
                            d <= now,
                            "spurious wake: ({p}, {i}) fired at {now}, deadline {d}"
                        );
                    }
                    // (b) exactly the due tasks fire. The multiset
                    // comparison catches a double-fire of the same task
                    // (real queue would wake it twice, model once).
                    let mut sorted_expected = expected.clone();
                    sorted_expected.sort();
                    let mut sorted_actual = actual.clone();
                    sorted_actual.sort();
                    assert_eq!(
                        sorted_actual, sorted_expected,
                        "wake set mismatch at now={now}: actual {actual:?}, expected {expected:?}"
                    );
                    for &(p, i) in &expected {
                        pending.remove(&(p, i));
                    }
                    // (c) fired slots return to free: the number of
                    // still-pending entries matches the real slot count.
                    assert_eq!(
                        rivet::timer::slots_in_use(),
                        pending.len(),
                        "slot leak at now={now}"
                    );
                }
            }
        }

        // Final flush: advance far past every remaining deadline and
        // confirm everything still pending fires exactly once.
        let max_deadline = pending.values().copied().max().unwrap_or(0);
        now = now.max(max_deadline).saturating_add(1);
        rivet::timer::poll_timers(now);
        while let Some(id) = rivet::waker::next_ready() {
            assert!(
                pending.remove(&(id.priority(), id.index())).is_some(),
                "unexpected final wake ({}, {})",
                id.priority(),
                id.index()
            );
        }
        assert!(pending.is_empty(), "deadlines never fired: {pending:?}");
        assert_eq!(
            rivet::timer::slots_in_use(),
            0,
            "slots leaked after final flush"
        );
    }
}

proptest! {
    #[test]
    fn timer_queue_fires_each_deadline_exactly_once(
        ops in prop::collection::vec(op_strategy(), 0..300),
    ) {
        run_ops(&ops);
    }
}
