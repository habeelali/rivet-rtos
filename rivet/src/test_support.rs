//! Host-test support: serialized, reset-able kernel state.
//!
//! Host tests exercise kernel logic that lives in *global* statics (`TASKS`,
//! waker bitmaps, `TIMER_SLOTS`, `CURRENT`/`RR_COUNTER`), and `cargo test`
//! runs test functions on parallel threads. Without serialization, tests
//! racing on those statics are flaky (observed: 1/8 consecutive runs of
//! `cargo test -p rivet` failed in `schedule_round_robins_same_priority`).
//!
//! Every kernel test must run inside [`macro@crate::kernel_test`], which
//! acquires a global [`std::sync::Mutex`] and resets all kernel state to
//! boot values before running the body. This is deliberately *not*
//! `--test-threads=1`: that would hide the problem, and silently stops
//! applying if anyone runs a subset of the suite.
//!
//! Only compiled with `feature = "test-support"` (enabled exclusively from
//! this crate's `[dev-dependencies]`), so it never affects embedded builds.

use std::sync::Mutex;

/// Serializes all kernel tests: exactly one test mutates kernel globals at
/// a time, so no two tests can interleave their state.
static KERNEL_TEST_LOCK: Mutex<()> = Mutex::new(());

/// Acquire the global kernel-test lock. Held for the duration of the
/// [`macro@crate::kernel_test`] body.
pub fn acquire() -> std::sync::MutexGuard<'static, ()> {
    KERNEL_TEST_LOCK.lock().unwrap_or_else(|e| e.into_inner())
}

/// Reset every kernel global to its boot-time state. Called by
/// [`macro@crate::kernel_test`] before each test body runs.
pub fn reset_all() {
    crate::waker::reset();
    crate::timer::reset_for_test();
    crate::preempt::tcb::reset_for_test();
    crate::preempt::sched::reset_for_test();
    crate::executor::reset_for_test();
    crate::preempt::stack_pool::reset_for_test();
    crate::watchdog::reset_for_test();
    crate::exec_time::reset_for_test();
    crate::port::host::reset_test_clock();
}
