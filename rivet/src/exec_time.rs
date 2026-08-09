//! Per-task execution-time accounting (plan.md Phase 10).
//!
//! Built on the Group A cycle counter ([`crate::port::arch::cycle_count`]):
//! at every *actual* context switch, the cycles since the outgoing task's
//! last dispatch are added to its running total. No accounting happens on
//! no-switch ticks (the common case) — this only costs a `cycle_count()`
//! read plus one add, and only when a switch was already going to happen
//! anyway.
//!
//! RV32 and ARMv7-M both lack native 64-bit atomics (no `AtomicU64` in
//! `core` for either target — the same gap `rivet-arch-riscv::clint`
//! documents for `MTIME_HZ`/`TICK_PERIOD`), so the cycle totals below are
//! plain `static mut u64`s rather than atomics. [`on_switch`] is only ever
//! called from [`crate::preempt::on_tick`], itself only reached from trap/
//! exception context — structurally single-writer, since a hart can't take
//! a second tick trap while already inside one. The one remaining hazard
//! is a task-context reader (`report()`) observing a torn write if a tick
//! lands mid-read; every read and write below is wrapped in
//! [`crate::critical::enter`] to rule that out (a harmless no-op nesting
//! when called from within `on_tick`, which already runs with interrupts
//! effectively masked).
//!
//! Single global "last dispatch" stamp, matching the kernel's current
//! single-`CURRENT`-task model (plan.md Phase 19 upgrades both together
//! for SMP).

use crate::preempt::tcb::MAX_PTASKS;
use core::sync::atomic::{AtomicBool, Ordering};

static mut BUSY_CYCLES: [u64; MAX_PTASKS] = [0; MAX_PTASKS];
static mut LAST_DISPATCH: u64 = 0;
static mut BOOT_CYCLE: u64 = 0;
static mut WALLCLOCK_BOOT_US: u64 = 0;
static STARTED: AtomicBool = AtomicBool::new(false);

/// Record the very first dispatch (called once, from [`crate::preempt::start`]).
pub fn on_first_dispatch() {
    crate::critical::enter(|| {
        let now = crate::port::arch::cycle_count();
        let now_us = crate::port::board::now_us();
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe {
            BOOT_CYCLE = now;
            LAST_DISPATCH = now;
            WALLCLOCK_BOOT_US = now_us;
        }
        STARTED.store(true, Ordering::Release);
    });
}

/// Record an actual context switch away from `outgoing` (its id in
/// `preempt::tcb::TASKS`), crediting it with the cycles since the last
/// dispatch and resetting the stamp for whichever task runs next. No-op
/// if accounting hasn't started yet ([`on_first_dispatch`] not called).
pub fn on_switch(outgoing: usize) {
    if !STARTED.load(Ordering::Acquire) {
        return;
    }
    crate::critical::enter(|| {
        let now = crate::port::arch::cycle_count();
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe {
            let elapsed = now.wrapping_sub(LAST_DISPATCH);
            LAST_DISPATCH = now;
            if outgoing < MAX_PTASKS {
                BUSY_CYCLES[outgoing] = BUSY_CYCLES[outgoing].wrapping_add(elapsed);
            }
        }
    });
}

/// Total cycles task `id` has spent running, since boot, **as of its last
/// completed dispatch** — a task that is still running right now (never
/// yet switched away from) is not included until its next switch. Budget
/// enforcement needs the up-to-the-moment figure for whichever task is
/// *currently* running; see [`busy_cycles_live`].
pub fn busy_cycles(id: usize) -> u64 {
    if id >= MAX_PTASKS {
        return 0;
    }
    // SAFETY: guarded by `critical::enter` (see module docs).
    crate::critical::enter(|| unsafe { BUSY_CYCLES[id] })
}

/// [`busy_cycles`] plus the in-progress dispatch's elapsed cycles, if `id`
/// is the currently-running task (checked by the caller — this function
/// just adds "cycles since the single global `LAST_DISPATCH` stamp",
/// which is only meaningful for whoever is actually running right now).
/// Needed by [`crate::deadlines::check_budget`]: a task that never yields
/// would otherwise never accumulate any *completed*-dispatch cycles at
/// all, and its budget would never be checked.
pub fn busy_cycles_live(id: usize) -> u64 {
    let completed = busy_cycles(id);
    let in_progress = crate::critical::enter(|| {
        let now = crate::port::arch::cycle_count();
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe { now.wrapping_sub(LAST_DISPATCH) }
    });
    completed.wrapping_add(in_progress)
}

/// Cycles elapsed since the scheduler's first dispatch (the denominator
/// for a `%busy` figure). Zero if the preemptive tier hasn't started.
pub fn cycles_since_boot() -> u64 {
    if !STARTED.load(Ordering::Acquire) {
        return 0;
    }
    crate::critical::enter(|| {
        let now = crate::port::arch::cycle_count();
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe { now.wrapping_sub(BOOT_CYCLE) }
    })
}

/// Wall-clock microseconds elapsed since the scheduler's first dispatch.
/// Zero if the preemptive tier hasn't started.
pub fn wallclock_us_since_boot() -> u64 {
    if !STARTED.load(Ordering::Acquire) {
        return 0;
    }
    crate::critical::enter(|| {
        let now_us = crate::port::board::now_us();
        // SAFETY: guarded by `critical::enter` (see module docs).
        unsafe { now_us.wrapping_sub(WALLCLOCK_BOOT_US) }
    })
}

/// Convert a cycle count into an estimated microsecond duration, using the
/// aggregate cycles-per-microsecond rate measured since boot
/// (`cycles_since_boot() / wallclock_us_since_boot()`) — no board-declared
/// clock-rate constant needed, and self-calibrating against whatever the
/// cycle source actually is (real `mcycle`/DWT counting, or the
/// microsecond-resolution SysTick fallback, in which case this is
/// trivially exact). Used by [`crate::deadlines`]'s budget enforcement to
/// turn a `budget_us` into a comparison against [`busy_cycles`] without
/// needing a `__rivet_arch_cycle_count` call-rate contract beyond
/// "monotonic". Zero before the preemptive tier starts, or if no time has
/// passed yet (avoids a divide-by-zero).
pub fn estimate_us_from_cycles(cycles: u64) -> u64 {
    let total_cycles = cycles_since_boot();
    if total_cycles == 0 {
        return 0;
    }
    let total_us = wallclock_us_since_boot();
    cycles.saturating_mul(total_us) / total_cycles
}

/// Integer percentage (0-100) of `cycles_since_boot()` that task `id` has
/// spent running. `0` before the preemptive tier starts or if the task
/// hasn't been dispatched yet.
pub fn busy_percent(id: usize) -> u8 {
    let total = cycles_since_boot();
    if total == 0 {
        return 0;
    }
    let busy = busy_cycles(id);
    (busy.saturating_mul(100) / total).min(100) as u8
}

#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    crate::critical::enter(|| {
        // SAFETY: guarded by `critical::enter` (see module docs); test-only
        // reset runs under `kernel_test!`'s serialization lock too.
        unsafe {
            // Raw-pointer writes (not `.iter_mut()`) so this never forms a
            // `&mut` over the whole static, which `static_mut_refs` (2024
            // edition lint) flags even though there is no concurrent
            // access here (guarded by `critical::enter` + the test-only
            // caller already holding `kernel_test!`'s serialization lock).
            let base = core::ptr::addr_of_mut!(BUSY_CYCLES) as *mut u64;
            for i in 0..MAX_PTASKS {
                base.add(i).write(0);
            }
            LAST_DISPATCH = 0;
            BOOT_CYCLE = 0;
            WALLCLOCK_BOOT_US = 0;
        }
    });
    STARTED.store(false, Ordering::Relaxed);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn busy_percent_zero_before_start() {
        crate::kernel_test! {
            assert_eq!(busy_percent(0), 0);
        }
    }

    #[test]
    fn accounts_switch_time() {
        crate::kernel_test! {
            on_first_dispatch();
            // Simulate some running time on task 0, then a switch to task 1.
            for _ in 0..10 {
                crate::port::arch::cycle_count();
            }
            on_switch(0);
            assert!(busy_cycles(0) > 0);
            assert_eq!(busy_cycles(1), 0);
        }
    }
}
