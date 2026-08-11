//! Spawn-while-running stress (plan.md §2.4 / [B2] acceptance).
//!
//! A running task spawns `MAX_PTASKS` worker tasks in a tight loop (ticks
//! landing mid-registration), then one more which must return `None`
//! (registry full). Under the old `tcb::register`, a tick between the
//! `used` CAS and the `sp` store could context-switch into a half-
//! initialized task (`sp == 0`) — a use-before-init race. The fix
//! publishes `used = true` last, so the scheduler never sees a partial
//! slot. Every worker must run exactly once.

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use rivet::preempt::Stack;
use rivet::time::Sleep;

const MAX_PTASKS: usize = rivet::preempt::tcb::MAX_PTASKS;

static mut STACKS: [Stack<1024>; MAX_PTASKS] = [const { Stack::new() }; MAX_PTASKS];
static RAN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static SPAWNER_DONE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

struct Unit;
static UNIT: Unit = Unit;

fn worker(_: &'static Unit) -> ! {
    RAN.fetch_add(1, core::sync::atomic::Ordering::AcqRel);
    rivet::preempt::park_forever();
}

fn spawner(_: &'static Unit) -> ! {
    // `rivet::init()` registered the async idle task and this spawner
    // itself occupies a slot, so exactly MAX_PTASKS - 2 slots remain.
    // Spawn them all from a live task (ticks interleave mid-registration).
    // SAFETY: STACKS is only ever touched from this single spawner task;
    // each stack is handed to exactly one worker for the worker's lifetime.
    // addr_of_mut! avoids creating a reference to the static itself
    // (static_mut_refs lint).
    unsafe {
        // Indexing is required: addr_of_mut! per element (the iterator
        // form would create a reference to the mutable static).
        #[allow(clippy::needless_range_loop)]
        for i in 0..(MAX_PTASKS - 2) {
            let stack = &mut (*core::ptr::addr_of_mut!(STACKS[i])).0;
            let id = rivet::preempt::spawn(stack, 2, worker, &UNIT);
            assert!(id.is_ok(), "spawn failed");
        }
    }
    // One more must be rejected — registry full.
    let extra = unsafe { rivet::preempt::spawn(&mut STACKS[0].0, 2, worker, &UNIT) };
    assert_eq!(
        extra,
        Err(rivet::preempt::SpawnError::RegistryFull),
        "spawn past MAX_PTASKS must fail"
    );
    rivet::console::write_str("SPAWNER_FULL_OK\n");
    SPAWNER_DONE.store(true, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
}

#[rivet::task(priority = 0, stack = 512)]
async fn finisher() {
    loop {
        if SPAWNER_DONE.load(core::sync::atomic::Ordering::Acquire)
            && RAN.load(core::sync::atomic::Ordering::Acquire) == (MAX_PTASKS - 2) as u32
        {
            rivet::console::write_str("SPAWN_STRESS_OK\n");
            rivet::exit_success();
        }
        Sleep::<10_000>::new().await;
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet stress_spawn (B2 publish ordering)\n");

    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = spawner, arg = UNIT);

    rivet::run();
}
