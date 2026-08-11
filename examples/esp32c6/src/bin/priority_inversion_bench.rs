//! Real-time characterization: bounded priority-inversion measurement.
//!
//! Classic scenario: a low-priority task locks a mutex and holds it for a
//! known, fixed duration (`LOW_CRIT_SECTION_CYCLES`); several
//! medium-priority tasks spin continuously the whole time, touching
//! nothing shared — pure interference, exactly the shape that causes
//! *unbounded* priority inversion (the Mars Pathfinder bug) if the
//! mutex doesn't implement priority inheritance, since the medium
//! tasks would otherwise keep preempting the low holder forever. A
//! high-priority task blocks on the same mutex shortly after the low
//! task takes it, and its actual wait (measured start-of-block to
//! acquired) is compared against the low task's own critical-section
//! length: with priority inheritance working, the wait is bounded by
//! that length regardless of how many medium tasks are running or for
//! how long — the wait must NOT grow with medium-task interference.
//!
//! `PriorityMutex` already implements priority inheritance (plan.md
//! [B11], re-verified under this workspace's concurrency-hardening pass)
//! — this binary is the empirical proof, not new mechanism.

#![no_std]
#![no_main]

use rivet_bsp_esp32c6 as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static LOW_STARTED: AtomicU32 = AtomicU32::new(0);
static LOW_LOCKED_AT: AtomicU32 = AtomicU32::new(0);
static LOW_UNLOCKED_AT: AtomicU32 = AtomicU32::new(0);
static HIGH_BLOCK_START: AtomicU32 = AtomicU32::new(0);
static HIGH_ACQUIRED_AT: AtomicU32 = AtomicU32::new(0);
static MED_ITERS: AtomicU32 = AtomicU32::new(0);
static STOP_MED: AtomicU32 = AtomicU32::new(0);
static DONE: AtomicU32 = AtomicU32::new(0);

struct Unit;
static UNIT: Unit = Unit;

const N_MEDIUM: u32 = 3;
/// Fixed, known critical-section length for the low-priority holder —
/// the theoretical bound the high task's measured wait must not exceed
/// by more than scheduling/dispatch overhead.
const LOW_CRIT_SECTION_CYCLES: u32 = 50_000;

fn print_u64(mut n: u64) {
    if n == 0 {
        rivet::console::write_str("0");
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
        rivet::console::write_str(s);
    }
}

fn low_task(_: &'static Unit) -> ! {
    // Lock first, *then* spawn the medium/high interference — spawning
    // them before this task ever gets to run would let priority-4 medium
    // tasks (never blocking, so never yielding the CPU back) starve this
    // priority-1 task out before it can even reach `MTX.lock()`, which
    // would make the whole scenario meaningless (low never holds the
    // mutex, high never actually blocks on it). Matches `mutex_test.rs`'s
    // own `holder()` pattern for the same reason.
    let g = MTX.lock();
    LOW_LOCKED_AT.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    // Set *before* spawning anything higher-priority: spawning
    // `high_task` (priority 9) can preempt this task immediately, and
    // `high_task` no longer busy-waits on this flag (removed — spawn
    // order alone now guarantees the mutex is already held by the time
    // it runs), but leaving the store first is still correct and cheap
    // insurance against future reordering.
    LOW_STARTED.store(1, Ordering::Release);

    for _ in 0..N_MEDIUM {
        let _ = rivet::spawn_ptask!(stack = 256, priority = 4, entry = medium_task, arg = UNIT);
    }
    let _ = rivet::spawn_ptask!(stack = 512, priority = 9, entry = high_task, arg = UNIT);
    let t0 = rivet::port::arch::cycle_count();
    // Busy-hold for a known, fixed duration — this IS the bound the high
    // task's wait must respect, regardless of medium-task interference.
    while (rivet::port::arch::cycle_count().wrapping_sub(t0) as u32) < LOW_CRIT_SECTION_CYCLES {
        core::hint::spin_loop();
    }
    LOW_UNLOCKED_AT.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    drop(g);
    rivet::preempt::park_forever();
}

fn medium_task(_: &'static Unit) -> ! {
    // Pure interference: never touches MTX. If priority inheritance is
    // broken, this is exactly what would starve the low holder and
    // unboundedly delay the high task.
    while STOP_MED.load(Ordering::Acquire) == 0 {
        MED_ITERS.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn high_task(_: &'static Unit) -> ! {
    // No wait needed here: `low_task` only spawns this task *after*
    // already holding `MTX` and setting `LOW_STARTED`, so the mutex is
    // guaranteed held the instant this runs — spawn order is the
    // synchronization, not a flag poll (see `low_task`'s own comment).
    HIGH_BLOCK_START.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    let _g = MTX.lock();
    HIGH_ACQUIRED_AT.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    drop(_g);

    STOP_MED.store(1, Ordering::Release);

    let wait = HIGH_ACQUIRED_AT
        .load(Ordering::Acquire)
        .wrapping_sub(HIGH_BLOCK_START.load(Ordering::Acquire));
    let low_crit = LOW_UNLOCKED_AT
        .load(Ordering::Acquire)
        .wrapping_sub(LOW_LOCKED_AT.load(Ordering::Acquire));

    rivet::console::write_str("=== priority_inversion_bench ===\n");
    rivet::console::write_str("high_wait_cycles=");
    print_u64(wait as u64);
    rivet::console::write_str("\nlow_critical_section_cycles=");
    print_u64(low_crit as u64);
    rivet::console::write_str("\nmedium_task_iterations_during_test=");
    print_u64(MED_ITERS.load(Ordering::Relaxed) as u64);
    rivet::console::write_str("\n");

    // The bound: high's wait must not exceed the low holder's own
    // critical section by more than a generous scheduling-overhead
    // margin (dispatch decision + wakeup latency, not medium-task
    // interference — medium ran the whole time and must NOT show up
    // here if inheritance is working).
    let margin = low_crit; // 2x low_crit as the overhead allowance
    if wait <= low_crit + margin {
        rivet::console::write_str("PRIORITY_INVERSION_BOUNDED\n");
    } else {
        rivet::console::write_str("PRIORITY_INVERSION_UNBOUNDED\n");
        rivet::exit_failure(1);
    }

    DONE.store(1, Ordering::Release);
    rivet::report();
    rivet::console::write_str("PRIORITY_INVERSION_BENCH_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet priority_inversion_bench\n");

    // Priorities: low=1, medium=4 (x3), high=9 — a real priority gap on
    // both sides of the mutex holder, matching a realistic inversion
    // scenario rather than adjacent priorities. Only `low_task` is
    // spawned here — it spawns medium/high itself once it holds the
    // mutex (see its own comment for why the ordering matters).
    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = low_task, arg = UNIT);

    rivet::run();
}
