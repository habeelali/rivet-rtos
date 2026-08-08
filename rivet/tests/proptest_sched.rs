//! Property-based scheduler tests (plan.md §1.4).
//!
//! A naive reference scheduler — collect `(id, effective_priority)` for
//! `Ready` tasks, pick max, no bitmaps, no RR counter — is driven against
//! the real [`rivet::preempt::sched::schedule`] through random operation
//! sequences. Properties:
//!
//! - (i) `schedule()` never returns a non-Ready / unused task id;
//! - (ii) the returned task's effective priority equals the reference max;
//! - (iii) over K consecutive dispatches at a tied priority, every tied
//!   task is dispatched at least once — this is the property plan.md [B14]
//!   violates, so the test is `#[ignore]`d until Phase 2.6's scheduler
//!   fairness fix lands (then the `#[ignore]` is removed).
//!
//! Each case runs inside [`rivet::kernel_test!`], which serializes kernel
//! tests and resets all kernel globals.

use proptest::prelude::*;
use rivet::preempt::sched;
use rivet::preempt::tcb;

const MAX_PTASKS: usize = tcb::MAX_PTASKS;

/// One random scheduler operation.
#[derive(Debug, Clone)]
enum Op {
    /// Register a new task at the given priority (id assigned by the
    /// registry — first free slot).
    Register(u8),
    /// Mark a task ready (id valid only if the slot is in use).
    SetReady(usize),
    /// Mark a task blocked.
    SetBlocked(usize),
    /// Overwrite a task's effective priority (simulating inheritance).
    SetEff(usize, u8),
    /// Ask the scheduler for the next task (a tick).
    Tick,
}

fn op_strategy() -> impl Strategy<Value = Op> {
    prop_oneof![
        3 => (0..=31u8).prop_map(Op::Register),
        2 => (0..MAX_PTASKS).prop_map(Op::SetReady),
        2 => (0..MAX_PTASKS).prop_map(Op::SetBlocked),
        2 => (0..MAX_PTASKS, 0..=31u8).prop_map(|(id, p)| Op::SetEff(id, p)),
        5 => Just(Op::Tick),
    ]
}

/// Reference model: mirrors `(base, eff, ready)` per slot; `None` = free.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Slot {
    base: u8,
    eff: u8,
    ready: bool,
}

struct RefSched {
    slots: [Option<Slot>; MAX_PTASKS],
    used: usize,
}

impl RefSched {
    fn new() -> Self {
        Self {
            slots: [None; MAX_PTASKS],
            used: 0,
        }
    }

    /// Predict the id `tcb::register` will assign (first free slot) and
    /// apply the registration. Returns `None` if the registry is full.
    fn register(&mut self, prio: u8) -> Option<usize> {
        for (id, slot) in self.slots.iter_mut().enumerate() {
            if slot.is_none() {
                *slot = Some(Slot {
                    base: prio,
                    eff: prio,
                    ready: true,
                });
                self.used += 1;
                return Some(id);
            }
        }
        None
    }

    fn apply(&mut self, id: usize, f: impl FnOnce(&mut Slot)) -> bool {
        match &mut self.slots[id] {
            Some(slot) => {
                f(slot);
                true
            }
            None => false,
        }
    }

    /// Reference `schedule()`: max effective priority among ready tasks.
    fn schedule(&self) -> Option<(usize, u8)> {
        let mut best: Option<(usize, u8)> = None;
        for (id, slot) in self.slots.iter().enumerate() {
            if let Some(s) = slot {
                if s.ready && best.is_none_or(|(_, p)| s.eff > p) {
                    best = Some((id, s.eff));
                }
            }
        }
        best
    }
}

fn run_ops(ops: &[Op]) {
    rivet::kernel_test! {
        let mut ref_sched = RefSched::new();

        for op in ops {
            match *op {
                Op::Register(prio) => {
                    let real = tcb::register(0x4000 + 0x100 * ref_sched.used, prio);
                    let predicted = ref_sched.register(prio);
                    assert_eq!(real, predicted, "register id mismatch at op {op:?}");
                }
                Op::SetReady(id) => {
                    if ref_sched.apply(id, |s| s.ready = true) {
                        sched::unblock(id);
                    }
                }
                Op::SetBlocked(id) => {
                    if ref_sched.apply(id, |s| s.ready = false) {
                        tcb::get(id).unwrap().set_state(id, tcb::TaskState::Blocked);
                    }
                }
                Op::SetEff(id, p) => {
                    if ref_sched.apply(id, |s| s.eff = p) {
                        // Use the queue-aware API (the kernel's priority-
                        // inheritance path) so the O(1) queues stay
                        // consistent with the model.
                        tcb::get(id).unwrap().set_effective_priority(id, p);
                    }
                }
                Op::Tick => {
                    let real = sched::schedule();
                    let reference = ref_sched.schedule();

                    match (real, reference) {
                        (None, None) => {}
                        (Some(id), Some((_, ref_eff))) => {
                            // (i) returned id is used and ready.
                            let t = tcb::get(id).unwrap_or_else(|| {
                                panic!("schedule() returned unused id {id} at op {op:?}")
                            });
                            assert_eq!(
                                t.state(),
                                tcb::TaskState::Ready,
                                "schedule() returned non-Ready task {id}"
                            );
                            // (ii) returned priority == reference max.
                            let eff = t.effective_priority.load(core::sync::atomic::Ordering::Acquire);
                            assert_eq!(
                                eff, ref_eff,
                                "schedule() returned priority {eff}, reference max is {ref_eff} (op {op:?})"
                            );
                        }
                        (Some(id), None) => panic!(
                            "schedule() returned {id} but reference has no ready task (op {op:?})"
                        ),
                        (None, Some(_)) => panic!(
                            "schedule() returned None but reference has a ready task (op {op:?})"
                        ),
                    }
                }
            }
        }
    }
}

proptest! {
    #[test]
    fn schedule_always_picks_max_priority_ready_task(ops in prop::collection::vec(op_strategy(), 0..200)) {
        run_ops(&ops);
    }
}

// ── [B14] fairness property ──────────────────────────────────────────
//
// Round-robin phase must be driven by *actual dispatch*, not by every
// schedule() call (which fires on ticks that do not switch, e.g. when the
// only lower-priority candidate is the async idle task). The old
// `RR_COUNTER % MAX_PTASKS` start advanced on every call, so a task pair
// tied at adjacent ids got aliased: the rotation window is 16 slots, not
// 2, and non-switching ticks advanced the phase, letting one tied task
// win far more than its fair share. Phase 2.6 fixed it (advance only on
// switch, via `sched::on_dispatch`); this property test enforces it.

#[derive(Debug, Clone)]
enum DispatchOp {
    /// Add a task tied at priority P.
    AddTied(u8),
    /// A tick: schedule, and switch if the candidate differs and has
    /// priority >= running (mirrors `preempt::on_tick` + `should_preempt`).
    Tick,
}

fn fairness_op_strategy() -> impl Strategy<Value = DispatchOp> {
    prop_oneof![
        2 => (1..=31u8).prop_map(DispatchOp::AddTied),
        5 => Just(DispatchOp::Tick),
    ]
}

/// Run the real dispatch flow (schedule + should_preempt + switch) and
/// record the sequence of actual dispatches, mirroring `on_tick`'s
/// decision logic exactly. Returns the dispatch sequence plus a snapshot
/// of the final task registry `(id, base_priority)` pairs, both captured
/// under the kernel-test lock.
fn dispatch_sequence(ops: &[DispatchOp], rounds: usize) -> (Vec<usize>, Vec<(usize, u8)>) {
    rivet::kernel_test! {
        let mut running: Option<usize> = None;
        let mut sequence = Vec::new();

        for op in ops {
            match *op {
                DispatchOp::AddTied(prio) => {
                    let _ = tcb::register(0x5000, prio);
                }
                DispatchOp::Tick => {
                    let Some(candidate) = sched::schedule() else { continue };
                    let switch = match running {
                        None => true, // first dispatch
                        Some(r) => {
                            if candidate == r {
                                false
                            } else {
                                sched::should_preempt(candidate, r)
                            }
                        }
                    };
                    if switch {
                        if let Some(r) = running {
                            tcb::get(r).unwrap().set_state(r, tcb::TaskState::Ready);
                        }
                        tcb::get(candidate).unwrap().set_state(candidate, tcb::TaskState::Running);
                        running = Some(candidate);
                        sched::on_dispatch(candidate);
                        sequence.push(candidate);
                    }
                }
            }
        }

        // Drive `rounds` further ticks against the final task set so the
        // tied tasks have a fair window to all appear.
        for _ in 0..rounds {
            let Some(candidate) = sched::schedule() else { break };
            let switch = match running {
                None => true,
                Some(r) => candidate != r && sched::should_preempt(candidate, r),
            };
            if switch {
                if let Some(r) = running {
                    tcb::get(r).unwrap().set_state(r, tcb::TaskState::Ready);
                }
                tcb::get(candidate).unwrap().set_state(candidate, tcb::TaskState::Running);
                running = Some(candidate);
                sched::on_dispatch(candidate);
                sequence.push(candidate);
            }
        }

        let snapshot = tcb::TASKS
            .iter()
            .enumerate()
            .filter(|(_, t)| t.used.load(core::sync::atomic::Ordering::Acquire))
            .map(|(id, t)| {
                (
                    id,
                    t.base_priority.load(core::sync::atomic::Ordering::Acquire),
                )
            })
            .collect::<Vec<_>>();

        (sequence, snapshot)
    }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 128, .. ProptestConfig::default() })]

    /// Over enough ticks with a tied pair, both tasks must be dispatched —
    /// violates on current code (plan.md [B14]); Phase 2.6 removes the
    /// `#[ignore]`.
    #[test]
    fn tied_tasks_are_fairly_dispatched(
        ops in prop::collection::vec(fairness_op_strategy(), 4..60),
    ) {
        let (seq, snapshot) = dispatch_sequence(&ops, 64);

        // Find a priority level with at least two tasks. The fairness
        // property only applies when that level is the *highest* priority
        // in the system: a legitimately higher-priority task preempting
        // everything is not a fairness violation.
        let mut counts: [usize; 32] = [0; 32];
        for &(_, prio) in &snapshot {
            counts[prio as usize] += 1;
        }
        let Some(prio) = counts.iter().position(|&c| c >= 2) else {
            // No tied level — nothing to check.
            return Ok(());
        };
        let max_prio = snapshot.iter().map(|&(_, p)| p).max().unwrap_or(0);
        if prio as u8 != max_prio {
            // A higher-priority task legitimately starves the tied level.
            return Ok(());
        }

        // Gather the dispatched ids of tasks at that priority.
        let mut seen: Vec<usize> = Vec::new();
        for id in &seq {
            if snapshot.iter().any(|&(sid, p)| sid == *id && p as usize == prio)
                && !seen.contains(id)
            {
                seen.push(*id);
            }
        }
        let expected: Vec<usize> = snapshot
            .iter()
            .filter(|&&(_, p)| p as usize == prio)
            .map(|&(id, _)| id)
            .collect();
        assert!(
            seen.len() >= expected.len(),
            "fairness violation: tied tasks {expected:?} at priority {prio}, \
             only {seen:?} were ever dispatched over {} ticks (plan.md [B14])",
            seq.len()
        );
    }
}
