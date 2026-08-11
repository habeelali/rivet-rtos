//! Periods + CPU-budget enforcement test (plan.md Phase 11).
//!
//! Two independent checks:
//! - `periodic_task` calls [`rivet::deadlines::wait_period`] 5 times with a
//!   20ms period and measures the total elapsed wall-clock time — proving
//!   drift-corrected periodic wake actually schedules on roughly the right
//!   cadence, not just that the API doesn't panic.
//! - `budget_hog` never yields and never calls `wait_period`, with a tiny
//!   configured budget — proving `on_tick`'s budget check actually raises
//!   `FaultKind::BudgetExceeded` through the real fault-isolation path
//!   (same mechanism `fault_isolate.rs` already exercises for stack
//!   overflow), not just that the accounting arithmetic works in a unit
//!   test.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use rivet::fault::FaultKind;

// Low 32 bits of `now_us()` is plenty for this test's sub-second runtime
// (no risk of wraparound); RV32 has no native `AtomicU64` (same gap
// documented in `rivet-arch-riscv::clint`).
static PERIOD_START_US: AtomicU32 = AtomicU32::new(0);
static PERIOD_END_US: AtomicU32 = AtomicU32::new(0);
static PERIOD_DONE: AtomicBool = AtomicBool::new(false);
static BUDGET_FAULT_SEEN: AtomicBool = AtomicBool::new(false);

const PERIOD_US: u32 = 20_000;
const ITERATIONS: u32 = 5;

fn periodic_task(_: &'static ()) -> ! {
    PERIOD_START_US.store(rivet::port::board::now_us() as u32, Ordering::Release);
    for _ in 0..ITERATIONS {
        rivet::deadlines::wait_period();
    }
    PERIOD_END_US.store(rivet::port::board::now_us() as u32, Ordering::Release);
    PERIOD_DONE.store(true, Ordering::Release);
    rivet::preempt::park_forever();
}

fn budget_hog(_: &'static ()) -> ! {
    loop {
        core::hint::spin_loop();
    }
}

fn supervisor(_: &'static ()) -> ! {
    // Give both tasks time to finish (5 periods of 20ms = 100ms, plus the
    // hog should fault within the first couple of ticks).
    rivet::preempt::sleep_ms(500);

    let ok_period = PERIOD_DONE.load(Ordering::Acquire);
    let elapsed =
        (PERIOD_END_US.load(Ordering::Acquire).wrapping_sub(PERIOD_START_US.load(Ordering::Acquire)))
            as u64;
    let expected = (PERIOD_US as u64) * (ITERATIONS as u64 - 1);
    // Generous tolerance (2x expected): this proves periodic wake actually
    // happens on the right order of magnitude, not sub-tick precision —
    // the tick rate itself bounds how exact this can be.
    let period_ok = ok_period && elapsed >= expected / 2 && elapsed <= expected * 2;

    rivet::console::write_str("PERIOD_ELAPSED_US=");
    print_dec(elapsed as usize);
    rivet::console::write_str(" EXPECTED_US=");
    print_dec(expected as usize);
    rivet::console::write_str("\n");

    if !period_ok {
        rivet::console::write_str("PERIOD_FAIL\n");
        rivet::exit_failure(1);
    }
    rivet::console::write_str("PERIOD_OK\n");

    if !BUDGET_FAULT_SEEN.load(Ordering::Acquire) {
        rivet::console::write_str("BUDGET_FAIL: no fault observed\n");
        rivet::exit_failure(2);
    }
    rivet::console::write_str("BUDGET_OK\n");

    rivet::console::write_str("DEADLINE_TEST_OK\n");
    rivet::exit_success();
}

fn print_dec(mut n: usize) {
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

#[rivet::main]
fn main() -> ! {
    rivet::fault::set_policy(rivet::fault::FaultPolicy::IsolateTask);
    rivet::fault::set_on_task_fault(|_id, info| {
        if info.kind == FaultKind::BudgetExceeded {
            BUDGET_FAULT_SEEN.store(true, Ordering::Release);
        }
    });
    rivet::console::write_str("Rivet deadline_test: periods + budget enforcement\n");

    // Spawning (and configuring via the returned handle) happens before
    // `rivet::run()` starts the preemptive scheduler, so there's no race
    // between "task starts running" and "task's period/budget is set".
    let hog = rivet::spawn_ptask!(stack = 512, priority = 3, entry = budget_hog, arg = ())
        .unwrap_or_else(|_| rivet::exit_failure(9));
    hog.set_budget_us(2_000);

    let periodic = rivet::spawn_ptask!(stack = 512, priority = 2, entry = periodic_task, arg = ())
        .unwrap_or_else(|_| rivet::exit_failure(9));
    periodic.set_period_us(PERIOD_US);

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = supervisor, arg = ());
    rivet::run();
}
