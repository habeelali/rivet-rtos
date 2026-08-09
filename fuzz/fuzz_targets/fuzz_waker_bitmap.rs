//! Fuzz target: waker bitmap invariants (plan.md §1.5).
//!
//! Input: byte stream of mark/dequeue ops. Invariants: every marked
//! (priority, index) dequeues exactly once; nothing dequeues that was not
//! marked; dequeue order is strictly highest-priority-first.

#![no_main]

use libfuzzer_sys::fuzz_target;

use rivet::task::TaskId;
use rivet::waker;

fuzz_target!(|data: &[u8]| {
    rivet::test_support::reset_all();

    // Tracks marked-but-not-yet-dequeued (prio, idx) pairs. This set is the
    // single source of truth for the model (no parallel arrays that can
    // drift).
    let mut pending: std::collections::HashSet<(u8, u8)> = std::collections::HashSet::new();

    for (i, &b) in data.iter().enumerate() {
        let next = data.get(i.wrapping_add(1)).copied().unwrap_or(0);
        if b % 2 == 0 {
            // Mark ready.
            let prio = next % 32;
            let idx = data.get(i.wrapping_add(2)).copied().unwrap_or(0) % 32;
            waker::mark_ready(TaskId::new(prio, idx));
            pending.insert((prio, idx));
        } else {
            // Dequeue one.
            let got = waker::next_ready();
            if pending.is_empty() {
                assert_eq!(got, None, "dequeued something with nothing pending");
                continue;
            }
            let id = got.expect("pending work but next_ready returned None (lost wakeup)");
            let (prio, idx) = (id.priority(), id.index());
            // Must be the highest priority with pending marks — compare
            // against the max BEFORE removal (the dequeued task is the
            // one being removed).
            let max_pending_prio = pending.iter().map(|&(p, _)| p).max();
            if let Some(max) = max_pending_prio {
                assert_eq!(
                    prio as u8, max,
                    "not highest priority first: dequeued ({prio}, {idx}), highest pending is {max}"
                );
            }
            // Must be a pending task.
            assert!(
                pending.remove(&(prio, idx)),
                "dequeued ({prio}, {idx}) which was never marked (or double-dequeued)"
            );
        }
    }

    // Final drain: everything still pending must dequeue exactly once.
    while let Some(id) = waker::next_ready() {
        let (p, i) = (id.priority(), id.index());
        assert!(
            pending.remove(&(p, i)),
            "final drain dequeued ({p}, {i}) which was never marked (or double-dequeued)"
        );
    }
    assert!(pending.is_empty(), "marks lost: {pending:?}");
});
