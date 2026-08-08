//! Respawn + pause/resume test (plan.md §5.4/§5.5 acceptance).
//!
//! Phase 1: spawn a worker that returns 42; join it; despawn it (slot +
//! stack released to the pool); respawn a new worker into the recycled
//! slot — the OLD handle must be stale (`is_valid()` false, join →
//! Stale), the NEW handle joins to Ok(7).
//! Phase 2: spawn a chattering task, pause it (progress stops), resume it
//! (progress resumes).

#![no_std]
#![no_main]

use rivet_bsp_mps2_an385 as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicUsize, Ordering};
use rivet::preempt::{JoinError, TaskHandle};

fn worker42(_: &'static ()) -> u32 {
    42
}

fn worker7(_: &'static ()) -> u32 {
    7
}

fn chatter(_: &'static ()) {
    loop {
        CHATTER_TICKS.fetch_add(1, Ordering::Relaxed);
        rivet::yield_now();
    }
}

static CHATTER_TICKS: AtomicUsize = AtomicUsize::new(0);
static W1: AtomicUsize = AtomicUsize::new(0); // packed worker-1 handle
static CHATTER_H: AtomicUsize = AtomicUsize::new(0);

fn supervisor(_: &'static ()) -> ! {
    // Phase 1: join worker1 → Ok(42).
    let packed = W1.load(Ordering::Acquire);
    let h1 = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };
    match h1.join::<u32>() {
        Ok(42) => rivet::console::write_str("JOIN1_OK\n"),
        _ => rivet::exit_failure(11),
    }

    // Despawn worker1: slot + stack released.
    if !h1.despawn() {
        rivet::console::write_str("DESPAWN_FAIL\n");
        rivet::exit_failure(12);
    }
    rivet::console::write_str("DESPAWN_OK\n");

    // Respawn a new worker — must reuse the freed slot (id equal).
    let h2 = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker7, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::exit_failure(13),
    };
    // The old handle must now be stale (generation bumped on reuse).
    if h1.is_valid() {
        rivet::console::write_str("STALE_CHECK_FAIL\n");
        rivet::exit_failure(14);
    }
    rivet::console::write_str("STALE_CHECK_OK\n");
    match h1.join::<u32>() {
        Err(JoinError::Stale) => rivet::console::write_str("STALE_JOIN_OK\n"),
        _ => rivet::exit_failure(15),
    }
    match h2.join::<u32>() {
        Ok(7) => rivet::console::write_str("JOIN2_OK\n"),
        _ => rivet::exit_failure(16),
    }
    if !h2.despawn() {
        rivet::exit_failure(17);
    }

    // Phase 2: pause/resume.
    let packed = CHATTER_H.load(Ordering::Acquire);
    let hc = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };
    // Let the chatter accumulate some ticks.
    rivet::preempt::sleep_ms(10);
    let before = CHATTER_TICKS.load(Ordering::Relaxed);
    if before == 0 {
        rivet::console::write_str("CHATTER_NEVER_RAN\n");
        rivet::exit_failure(18);
    }
    if !hc.pause() {
        rivet::console::write_str("PAUSE_FAIL\n");
        rivet::exit_failure(19);
    }
    rivet::console::write_str("PAUSE_OK\n");
    // While paused, the chatter must not advance (we sleep; it's skipped).
    rivet::preempt::sleep_ms(10);
    let after = CHATTER_TICKS.load(Ordering::Relaxed);
    if after != before {
        rivet::console::write_str("PAUSED_STILL_RAN\n");
        rivet::exit_failure(20);
    }
    rivet::console::write_str("PAUSED_STILL\n");
    if !hc.resume() {
        rivet::console::write_str("RESUME_FAIL\n");
        rivet::exit_failure(21);
    }
    rivet::console::write_str("RESUME_OK\n");
    rivet::preempt::sleep_ms(10);
    if CHATTER_TICKS.load(Ordering::Relaxed) <= after {
        rivet::console::write_str("RESUMED_NOT_RUNNING\n");
        rivet::exit_failure(22);
    }
    rivet::console::write_str("RESPAWN_TEST_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet respawn_test\n");

    let h1 = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker42, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::exit_failure(23),
    };
    W1.store(
        h1.id as usize | ((h1.generation as usize) << 16),
        Ordering::Release,
    );

    let hc = match rivet::spawn_ptask!(stack = 512, priority = 1, entry = chatter, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::exit_failure(24),
    };
    CHATTER_H.store(
        hc.id as usize | ((hc.generation as usize) << 16),
        Ordering::Release,
    );

    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = supervisor, arg = ());

    rivet::run();
}
