//! Latency histograms (plan.md Phase 12).
//!
//! Four fixed-size, zero-allocation histograms — 16 log2-scaled buckets
//! each (`[AtomicU32; 16]`), bucket `b` covers `[2^b, 2^(b+1))` cycles
//! (bucket 0 covers 0-1) — tracking:
//!
//! - [`Kind::IrqEntry`]: cycles from an interrupt firing to
//!   [`crate::preempt::on_tick`] actually running. Recorded by the arch
//!   trap entry (whichever `rivet-arch-*` crate captures a cycle stamp as
//!   early as possible in the handler).
//! - [`Kind::DispatchDecision`]: cycles spent *inside* `on_tick` itself —
//!   the scheduling decision's own cost.
//! - [`Kind::CriticalSection`]: cycles held between
//!   [`crate::critical::enter`]'s entry and exit — a proxy for
//!   interrupt-latency impact (nothing can preempt the calling hart while
//!   held).
//! - [`Kind::SchedulingWake`]: cycles from a task becoming ready
//!   (`sched::unblock` / `ready_add`) to actually being dispatched
//!   Running.
//!
//! Gated behind the `latency-histograms` feature (off by default — see
//! `rivet/Cargo.toml`): recording a sample is one `cycle_count()` read
//! plus one atomic increment, cheap but not free, and a cost-sensitive
//! board that never reads the histograms shouldn't pay it unasked.

use crate::sync::atomic::{AtomicU32, Ordering};

pub const BUCKETS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Kind {
    IrqEntry,
    DispatchDecision,
    CriticalSection,
    SchedulingWake,
}

const KINDS: usize = 4;

fn index(kind: Kind) -> usize {
    match kind {
        Kind::IrqEntry => 0,
        Kind::DispatchDecision => 1,
        Kind::CriticalSection => 2,
        Kind::SchedulingWake => 3,
    }
}

#[cfg(not(loom))]
static HISTOGRAMS: [[AtomicU32; BUCKETS]; KINDS] =
    [const { [const { AtomicU32::new(0) }; BUCKETS] }; KINDS];
#[cfg(loom)]
loom::lazy_static! {
    static ref HISTOGRAMS: [[AtomicU32; BUCKETS]; KINDS] =
        core::array::from_fn(|_| core::array::from_fn(|_| AtomicU32::new(0)));
}

/// Exact worst-case-observed cycle count per `Kind`, alongside the
/// bucketed histogram above — the top histogram bucket (`[2^15, ∞)`) is
/// unbounded above by construction, so it alone cannot answer "what was
/// the actual worst cycle count seen," only "at least 32768." Needed for
/// WCET reporting, where an open-ended top bucket isn't an exact figure.
#[cfg(not(loom))]
static MAX_CYCLES: [AtomicU32; KINDS] = [const { AtomicU32::new(0) }; KINDS];
#[cfg(loom)]
loom::lazy_static! {
    static ref MAX_CYCLES: [AtomicU32; KINDS] = core::array::from_fn(|_| AtomicU32::new(0));
}

#[cfg(any(feature = "latency-histograms", test))]
fn bucket_of(cycles: u64) -> usize {
    if cycles == 0 {
        return 0;
    }
    // 63 - leading_zeros gives floor(log2(cycles)); clamp to the last
    // bucket for anything huge (a stalled/very-first sample) rather than
    // panicking or silently discarding it.
    let b = 63 - cycles.leading_zeros() as usize;
    b.min(BUCKETS - 1)
}

/// Record one sample of `cycles` duration for `kind`. No-op unless the
/// `latency-histograms` feature is enabled.
#[cfg(feature = "latency-histograms")]
pub fn record(kind: Kind, cycles: u64) {
    HISTOGRAMS[index(kind)][bucket_of(cycles)].fetch_add(1, Ordering::Relaxed);
    let capped = cycles.min(u32::MAX as u64) as u32;
    MAX_CYCLES[index(kind)].fetch_max(capped, Ordering::Relaxed);
}

/// See the feature-gated [`record`] above; a no-op stub keeps call sites
/// (arch crates, `on_tick`, `critical::enter`, `sched::unblock`) free of
/// `#[cfg]` clutter when the feature is off.
#[cfg(not(feature = "latency-histograms"))]
#[inline(always)]
pub fn record(_kind: Kind, _cycles: u64) {}

/// Snapshot of one histogram's 16 bucket counts.
pub fn snapshot(kind: Kind) -> [u32; BUCKETS] {
    core::array::from_fn(|b| HISTOGRAMS[index(kind)][b].load(Ordering::Relaxed))
}

/// Exact worst-case-observed cycle count for `kind` (0 if never recorded)
/// — see [`MAX_CYCLES`]'s own doc for why this exists alongside the
/// bucketed histogram.
pub fn max_cycles(kind: Kind) -> u32 {
    MAX_CYCLES[index(kind)].load(Ordering::Relaxed)
}

/// Human-readable name, for `report()`.
pub fn name(kind: Kind) -> &'static str {
    match kind {
        Kind::IrqEntry => "irq_entry",
        Kind::DispatchDecision => "dispatch",
        Kind::CriticalSection => "critsec",
        Kind::SchedulingWake => "sched_wake",
    }
}

pub const ALL_KINDS: [Kind; KINDS] = [
    Kind::IrqEntry,
    Kind::DispatchDecision,
    Kind::CriticalSection,
    Kind::SchedulingWake,
];

#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    for hist in HISTOGRAMS.iter() {
        for b in hist.iter() {
            b.store(0, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bucket_of_boundaries() {
        assert_eq!(bucket_of(0), 0);
        assert_eq!(bucket_of(1), 0);
        assert_eq!(bucket_of(2), 1);
        assert_eq!(bucket_of(3), 1);
        assert_eq!(bucket_of(4), 2);
        assert_eq!(bucket_of(u64::MAX), BUCKETS - 1);
    }

    #[cfg(feature = "latency-histograms")]
    #[test]
    fn record_and_snapshot() {
        crate::kernel_test! {
            record(Kind::IrqEntry, 5);
            record(Kind::IrqEntry, 5);
            record(Kind::IrqEntry, 100);
            let snap = snapshot(Kind::IrqEntry);
            assert_eq!(snap[bucket_of(5)], 2);
            assert_eq!(snap[bucket_of(100)], 1);
        }
    }
}
