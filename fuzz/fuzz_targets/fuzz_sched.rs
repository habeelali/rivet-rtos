//! Fuzz target: preemptive scheduler invariants (plan.md §1.5).
//!
//! Input: an opcode byte stream. Invariants (same as the proptest model):
//! (i) `schedule()` never returns a non-Ready / unused task id;
//! (ii) the returned task's effective priority equals the reference max
//! among ready tasks.

#![no_main]

use libfuzzer_sys::fuzz_target;

use rivet::preempt::tcb;
use rivet::preempt::sched;

const MAX_PTASKS: usize = tcb::MAX_PTASKS;

#[derive(Clone, Copy)]
enum Op {
    Register(u8),
    SetReady(u8),
    SetBlocked(u8),
    SetEff(u8, u8),
    Tick,
}

fn decode(stream: &[u8]) -> Vec<Op> {
    let mut ops = Vec::new();
    for (i, &b) in stream.iter().enumerate() {
        let op = match b % 5 {
            0 => Op::Register(stream.get(i.wrapping_add(1)).copied().unwrap_or(0) % 32),
            1 => Op::SetReady(stream.get(i.wrapping_add(1)).copied().unwrap_or(0) % MAX_PTASKS as u8),
            2 => Op::SetBlocked(stream.get(i.wrapping_add(1)).copied().unwrap_or(0) % MAX_PTASKS as u8),
            3 => Op::SetEff(
                stream.get(i.wrapping_add(1)).copied().unwrap_or(0) % MAX_PTASKS as u8,
                stream.get(i.wrapping_add(2)).copied().unwrap_or(0) % 32,
            ),
            _ => Op::Tick,
        };
        ops.push(op);
    }
    ops
}

fuzz_target!(|data: &[u8]| {
    rivet::test_support::reset_all();
    let ops = decode(data);

    // Reference model: Option<(eff, ready)> per slot.
    let mut ref_slots: [Option<(u8, bool)>; MAX_PTASKS] = [None; MAX_PTASKS];
    let mut used = 0usize;

    for op in ops {
        match op {
            Op::Register(prio) => {
                let real = tcb::register(0x4000 + 0x100 * used, prio);
                // Predict first-free-slot id.
                let mut predicted = None;
                for (id, slot) in ref_slots.iter_mut().enumerate() {
                    if slot.is_none() {
                        *slot = Some((prio, true));
                        used += 1;
                        predicted = Some(id);
                        break;
                    }
                }
                assert_eq!(real, predicted, "register id mismatch");
            }
            Op::SetReady(id) => {
                if let Some(slot) = ref_slots.get_mut(id as usize).and_then(|s| s.as_mut()) {
                    slot.1 = true;
                    sched::unblock(id as usize);
                }
            }
            Op::SetBlocked(id) => {
                if let Some(slot) = ref_slots.get_mut(id as usize).and_then(|s| s.as_mut()) {
                    slot.1 = false;
                    if let Some(t) = tcb::get(id as usize) {
                        t.set_state(id as usize, tcb::TaskState::Blocked);
                    }
                }
            }
            Op::SetEff(id, p) => {
                if let Some(slot) = ref_slots.get_mut(id as usize).and_then(|s| s.as_mut()) {
                    slot.0 = p;
                    if let Some(t) = tcb::get(id as usize) {
                        let old_prio = t
                            .effective_priority
                            .load(rivet::sync::atomic::Ordering::Acquire);
                        t.effective_priority.store(
                            p,
                            rivet::sync::atomic::Ordering::Release,
                        );
                        // A raw store alone leaves `sched`'s ready queues
                        // pointing at the *old* priority bucket — every
                        // real effective-priority change (priority
                        // inheritance in `mutex.rs`) goes through this,
                        // which is what actually keeps `QUEUES`/
                        // `READY_BITMAP` consistent with the field
                        // (documented invariant in `sched.rs`: "every
                        // state transition keeps the queues consistent").
                        sched::on_effective_priority_change(id as usize, old_prio, p);
                    }
                }
            }
            Op::Tick => {
                let real = sched::schedule();
                let mut best: Option<(usize, u8)> = None;
                for (id, slot) in ref_slots.iter().enumerate() {
                    if let Some((eff, ready)) = slot {
                        if *ready && best.is_none_or(|(_, p)| *eff > p) {
                            best = Some((id, *eff));
                        }
                    }
                }
                match (real, best) {
                    (None, None) => {}
                    (Some(id), Some((_, ref_eff))) => {
                        let t = tcb::get(id)
                            .unwrap_or_else(|| panic!("schedule() returned unused id {id}"));
                        assert_eq!(t.state(), tcb::TaskState::Ready, "non-Ready task returned");
                        let eff = t
                            .effective_priority
                            .load(rivet::sync::atomic::Ordering::Acquire);
                        assert_eq!(eff, ref_eff, "priority != reference max");
                    }
                    (Some(id), None) => panic!("schedule() returned {id}, nothing ready"),
                    (None, Some(_)) => panic!("schedule() None but ready task exists"),
                }
            }
        }
    }
});
