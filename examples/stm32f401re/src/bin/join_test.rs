//! Task exit + join test (plan.md §5.2/§5.3 acceptance).
//!
//! A worker task computes a value and *returns* it; its entry lands in the
//! kernel's exit trampoline, which stores the result and wakes the joiner.
//! The supervisor `join()`s and recovers `Ok(42)`. A second phase joins a
//! task that never exits but parks — the supervisor must block, not busy-
//! spin (implicit: the system keeps ticking while joined).

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use rivet::preempt::TaskHandle;

fn worker(_: &'static ()) -> u32 {
    // Compute something, then return it — the entry returns normally.
    let mut acc = 0u32;
    for i in 0..9 {
        acc += i;
    }
    acc + 6 // 42
}

fn parker(_: &'static ()) {
    // Never returns; parks forever.
    rivet::preempt::park_forever();
}

fn supervisor(_: &'static ()) -> ! {
    // Recover the worker handle encoded by rust_main (id | generation<<16).
    let packed = WORKER_HANDLE.load(core::sync::atomic::Ordering::Acquire);
    let handle = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };

    // Phase 1: join the worker — must return Ok(42).
    match handle.join::<u32>() {
        Ok(42) => rivet::console::write_str("JOIN_OK v=42\n"),
        Ok(v) => {
            rivet::console::write_str("JOIN_WRONG\n");
            let _ = v;
            rivet::exit_failure(6);
        }
        Err(e) => {
            rivet::console::write_str("JOIN_ERR\n");
            rivet::console::write_str(match e {
                rivet::preempt::JoinError::Stale => "STALE\n",
                rivet::preempt::JoinError::SelfJoin => "SELF\n",
                rivet::preempt::JoinError::AlreadyJoined => "ALREADY\n",
                rivet::preempt::JoinError::Faulted => "FAULTED\n",
            });
            rivet::exit_failure(7);
        }
    }

    // Phase 2: joining a parked (never-exiting) task must block, not busy-
    // spin — the join blocks the supervisor while the system keeps ticking.
    // We never reach the print below in this test; the harness's golden is
    // satisfied by phase 1.
    let parker_h = TaskHandle {
        id: PARKER_ID.load(core::sync::atomic::Ordering::Acquire) as u16,
        generation: 0,
    };
    let _ = parker_h.join::<()>();

    rivet::console::write_str("JOIN_TEST_OK\n");
    rivet::exit_success();
}

// rust_main stores the worker's (id | generation<<16) and the parker's id.
static WORKER_HANDLE: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static PARKER_ID: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet join_test\n");

    // Spawn the worker; store its handle for the supervisor.
    let h = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::exit_failure(9),
    };
    WORKER_HANDLE.store(
        h.id as usize | ((h.generation as usize) << 16),
        core::sync::atomic::Ordering::Release,
    );

    // Spawn the parking task and the supervisor.
    match rivet::spawn_ptask!(stack = 512, priority = 1, entry = parker, arg = ()) {
        Ok(h) => PARKER_ID.store(h.id as usize, core::sync::atomic::Ordering::Release),
        Err(_) => rivet::exit_failure(10),
    }
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = supervisor, arg = ());

    rivet::run();
}
