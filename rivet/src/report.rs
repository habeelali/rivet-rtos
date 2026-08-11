//! `rivet::report()` — a single call that dumps kernel-wide state to the
//! console: every live task's priority (base/effective), state, stack
//! watermark, and `%busy` execution-time share, plus registry-wide
//! timer/task slot usage.
//!
//! Scope note (plan.md Phase 8, extended by Phase 10): `%busy` is backed
//! by [`crate::exec_time`], itself built on the Group A cycle counter.
//! Periods/budgets (plan.md Phase 11) don't get their own report column —
//! a budget overrun raises [`crate::fault::FaultKind::BudgetExceeded`]
//! immediately through the normal fault policy rather than being tallied
//! silently, so there's no "miss count" to display.

use core::sync::atomic::Ordering;

use crate::preempt::tcb::{self, TaskState};

fn print_dec(mut n: usize) {
    if n == 0 {
        crate::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 20];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut buf = [0u8; 20];
    for j in 0..i {
        buf[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&buf[..i]) {
        crate::console::write_str(s);
    }
}

/// Print a full kernel state dump to the console. Safe to call from any
/// task context (not ISR-safe — it does blocking console writes, same as
/// [`crate::console::write_str`] in general; see [`crate::log`] for the
/// ISR-safe alternative when you need to trace from interrupt context).
pub fn report() {
    crate::console::write_str("=== rivet::report() ===\n");

    let mut used_count = 0usize;
    for (id, t) in tcb::TASKS.iter().enumerate() {
        if !t.used.load(Ordering::Acquire) {
            continue;
        }
        used_count += 1;

        crate::console::write_str("task ");
        print_dec(id);

        let base = t.base_priority.load(Ordering::Acquire);
        let eff = t.effective_priority.load(Ordering::Acquire);
        crate::console::write_str(" prio=");
        print_dec(base as usize);
        if eff != base {
            crate::console::write_str("(eff=");
            print_dec(eff as usize);
            crate::console::write_str(")");
        }

        crate::console::write_str(" state=");
        if t.exited.load(Ordering::Acquire) {
            crate::console::write_str("exited");
        } else {
            crate::console::write_str(match t.state() {
                TaskState::Ready => "ready",
                TaskState::Running => "running",
                TaskState::Blocked => "blocked",
            });
        }

        let base_addr = t.stack_base.load(Ordering::Acquire);
        let size = t.stack_size.load(Ordering::Acquire);
        if base_addr != 0 && size != 0 {
            // On Cortex-M, another task's pool-allocated stack is outside
            // the *currently running* task's MPU region-7 window (region
            // 6 denies the rest of the whole pool by design) — reading it
            // needs the same scratch-window primitive `preempt::spawn`/
            // `stack_pool::release_stack` already use for the same reason
            // (a no-op on arches without a whole-pool deny region, e.g.
            // RISC-V). Opening the window disables that deny for the
            // *entire* pool, not just this stack, so — matching every
            // other scratch_open/close use in the kernel — it must run
            // under a critical section: a context switch mid-window would
            // leave every other task's stack briefly unguarded too. Keep
            // the window to just the byte-scan; the console write below
            // happens after closing it, off the critical section.
            let used = crate::critical::enter(|| {
                crate::port::arch::scratch_open(base_addr, size);
                // SAFETY: reading a registered task's own stack range from
                // outside that task is safe for watermarking purposes —
                // the bytes below the high-water mark are never written
                // again once touched, and report() doesn't rely on the
                // *current* contents above it, only on how far the 0xAA
                // fill pattern has been overwritten. The scratch window
                // (opened above, under this critical section) ensures
                // this is also *permitted* by the MPU, not just logically
                // sound.
                let stack = unsafe { core::slice::from_raw_parts(base_addr as *const u8, size) };
                let used = crate::preempt::stack_usage(stack);
                crate::port::arch::scratch_close();
                used
            });
            crate::console::write_str(" stack=");
            print_dec(used);
            crate::console::write_str("/");
            print_dec(size);
        }

        let held = t.held_count.load(Ordering::Acquire);
        if held != 0 {
            crate::console::write_str(" held_mutexes=");
            print_dec(held as usize);
        }

        crate::console::write_str(" busy=");
        print_dec(crate::exec_time::busy_percent(id) as usize);
        crate::console::write_str("%");

        crate::console::write_str("\n");
    }

    crate::console::write_str("ptask slots: ");
    print_dec(used_count);
    crate::console::write_str("/");
    print_dec(tcb::MAX_PTASKS);
    crate::console::write_str("\ntimer slots: ");
    print_dec(crate::timer::slots_in_use());
    crate::console::write_str("/");
    print_dec(crate::timer::MAX_TIMERS);
    crate::console::write_str("\n");

    crate::console::write_str("log: ");
    print_dec(crate::log::dropped_frames());
    crate::console::write_str(" dropped frame(s)\n");

    #[cfg(feature = "latency-histograms")]
    print_latency_histograms();

    crate::console::write_str("=== end report ===\n");
}

/// Print each latency histogram's non-empty buckets as `2^b:count` pairs
/// (plan.md Phase 12). Only compiled with `latency-histograms`.
#[cfg(feature = "latency-histograms")]
fn print_latency_histograms() {
    crate::console::write_str("latency (cycles, log2 buckets):\n");
    for kind in crate::latency::ALL_KINDS {
        crate::console::write_str("  ");
        crate::console::write_str(crate::latency::name(kind));
        crate::console::write_str(": ");
        let snap = crate::latency::snapshot(kind);
        let mut any = false;
        for (b, count) in snap.iter().enumerate() {
            if *count == 0 {
                continue;
            }
            any = true;
            crate::console::write_str("2^");
            print_dec(b);
            crate::console::write_str(":");
            print_dec(*count as usize);
            crate::console::write_str(" ");
        }
        if !any {
            crate::console::write_str("(no samples)");
        } else {
            crate::console::write_str("max=");
            print_dec(crate::latency::max_cycles(kind) as usize);
        }
        crate::console::write_str("\n");
    }
}
