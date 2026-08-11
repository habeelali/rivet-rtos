//! Soak-test infrastructure proof (plan.md Phase 9 / old-plan §8.1).
//!
//! A genuine multi-hour soak (periodic tasks at several rates, long-
//! running mutex contention, sustained channel traffic, an IRQ-driven
//! echo) is nightly-CI territory, not something to run to completion in
//! an interactive session. This binary exercises the same *classes* of
//! activity the real soak would (spawn/exit/join/despawn cycling through
//! a stack pool, timer create/cancel, channel producer/consumer traffic,
//! mutex contention) and checks the invariants a real soak cares about —
//! do fixed-size pools return to their baseline occupancy — for
//! `ITERATIONS` cycles.
//!
//! `ITERATIONS` is `SOAK_ITERATIONS` (a build-time env var, default 200 —
//! the CI smoke-scale run) via `xtask`'s `--sim-hours N` (plan.md
//! Phase 17): `cargo xtask soak --sim-hours N` builds with
//! `SOAK_ITERATIONS` scaled from `N` and a proportionally larger
//! wall-clock timeout. This is a **scaled iteration count, not a literal
//! simulated-device-uptime clock** — `drift_test` already covers literal
//! time-drift verification separately — but it genuinely runs far more
//! spawn/join/despawn/channel/mutex cycles than the 200-iteration smoke
//! baseline, which is exactly the class of bug (slow resource leaks,
//! occupancy that drifts rather than returning to baseline) a soak run
//! is for.

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::sync::Channel;
use rivet::time::Sleep;

/// Parse a decimal env-var string at compile time (`option_env!` only
/// gives a `&str`, and there's no `const`-evaluable string-to-int in
/// core stable yet). No error handling: `SOAK_ITERATIONS` is only ever
/// set by `xtask` itself, from a `u32` it formatted — never user input.
const fn parse_u32_or(s: Option<&str>, default: u32) -> u32 {
    match s {
        None => default,
        Some(s) => {
            let bytes = s.as_bytes();
            let mut n = 0u32;
            let mut i = 0;
            while i < bytes.len() {
                n = n * 10 + (bytes[i] - b'0') as u32;
                i += 1;
            }
            n
        }
    }
}

const ITERATIONS: u32 = parse_u32_or(option_env!("SOAK_ITERATIONS"), 200);

fn worker(_: &'static ()) -> u32 {
    7
}

static CHAN: Channel<u32, 4> = Channel::new();
static CHAN_SUM: AtomicU32 = AtomicU32::new(0);
static CHAN_DONE: AtomicU32 = AtomicU32::new(0);

// Channel::split() is one-shot for the pair; main() calls it exactly once
// at boot and hands each half to its task via a Once, matching the
// pattern the demo examples use.
static CHAN_TX: rivet::sync::Once<rivet::sync::Sender<'static, u32, 4>> = rivet::sync::Once::new();
static CHAN_RX: rivet::sync::Once<rivet::sync::Receiver<'static, u32, 4>> =
    rivet::sync::Once::new();

#[rivet::task(priority = 1, stack = 1024)]
async fn chan_producer() {
    let tx = CHAN_TX.get().expect("sender set before this runs");
    for i in 0..ITERATIONS {
        Sleep::<200>::new().await;
        tx.send(i).await;
    }
}

#[rivet::task(priority = 1, stack = 1024)]
async fn chan_consumer() {
    let rx = CHAN_RX.get().expect("receiver set before this runs");
    for _ in 0..ITERATIONS {
        let v = rx.recv().await;
        CHAN_SUM.fetch_add(v, Ordering::Relaxed);
    }
    CHAN_DONE.store(1, Ordering::Release);
}

static MUTEX: rivet::preempt::PriorityMutex<u32> = rivet::preempt::PriorityMutex::new(0);

fn mutex_worker(_: &'static ()) -> ! {
    for _ in 0..ITERATIONS {
        let mut g = MUTEX.lock();
        *g = g.wrapping_add(1);
        drop(g);
        rivet::yield_now();
    }
    rivet::preempt::park_forever();
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

fn ptask_slots_used() -> usize {
    rivet::preempt::tcb::TASKS
        .iter()
        .filter(|t| t.used.load(Ordering::Acquire))
        .count()
}

fn supervisor(_: &'static ()) -> ! {
    // Baseline: idle (priority 0) + this supervisor + mutex_worker +
    // the two channel tasks run as async (priority-0 cooperative, not
    // separate ptask slots) — only idle + supervisor + mutex_worker hold
    // real ptask slots for the supervisor's own lifetime.
    let baseline_ptasks = ptask_slots_used();
    let baseline_timers = rivet::timer::slots_in_use();

    // Spawn/exit/join/despawn cycling: the exact scenario old-plan §8.1
    // calls out ("TCB slot leak: used count returns to baseline after
    // joins").
    for iter in 0..ITERATIONS {
        let h = match rivet::spawn_ptask!(stack = 1024, priority = 3, entry = worker, arg = ()) {
            Ok(h) => h,
            Err(_) => {
                rivet::console::write_str("SPAWN_FAILED_MID_SOAK iter=");
                print_dec(iter as usize);
                rivet::console::write_str("\n");
                rivet::exit_failure(1);
            }
        };
        match h.join::<u32>() {
            Ok(7) => {}
            other => {
                rivet::console::write_str("JOIN_MISMATCH iter=");
                print_dec(iter as usize);
                rivet::console::write_str(" got=");
                match other {
                    Ok(v) => print_dec(v as usize),
                    Err(rivet::preempt::JoinError::Stale) => {
                        rivet::console::write_str("Stale")
                    }
                    Err(rivet::preempt::JoinError::SelfJoin) => {
                        rivet::console::write_str("SelfJoin")
                    }
                    Err(rivet::preempt::JoinError::AlreadyJoined) => {
                        rivet::console::write_str("AlreadyJoined")
                    }
                    Err(rivet::preempt::JoinError::Faulted) => {
                        rivet::console::write_str("Faulted")
                    }
                }
                rivet::console::write_str("\n");
                rivet::exit_failure(2);
            }
        }
        if !h.despawn() {
            rivet::console::write_str("DESPAWN_FAILED iter=");
            print_dec(iter as usize);
            rivet::console::write_str("\n");
            rivet::exit_failure(3);
        }
    }
    rivet::console::write_str("SPAWN_CYCLE_OK\n");

    // Wait for the channel producer/consumer pair and the mutex worker
    // (spawned from main()) to finish their own iteration counts.
    while CHAN_DONE.load(Ordering::Acquire) == 0 {
        rivet::preempt::sleep_ms(5);
    }
    let expected_sum = (0..ITERATIONS).sum::<u32>();
    if CHAN_SUM.load(Ordering::Acquire) != expected_sum {
        rivet::console::write_str("CHANNEL_SUM_MISMATCH\n");
        rivet::exit_failure(4);
    }
    rivet::console::write_str("CHANNEL_TRAFFIC_OK\n");

    // Give the mutex worker a moment to finish its own loop and park.
    rivet::preempt::sleep_ms(20);

    let final_ptasks = ptask_slots_used();
    let final_timers = rivet::timer::slots_in_use();
    if final_ptasks != baseline_ptasks {
        rivet::console::write_str("PTASK_SLOT_LEAK baseline=");
        print_dec(baseline_ptasks);
        rivet::console::write_str(" final=");
        print_dec(final_ptasks);
        rivet::console::write_str("\n");
        rivet::exit_failure(5);
    }
    rivet::console::write_str("NO_PTASK_LEAK\n");
    if final_timers != baseline_timers {
        rivet::console::write_str("TIMER_SLOT_LEAK baseline=");
        print_dec(baseline_timers);
        rivet::console::write_str(" final=");
        print_dec(final_timers);
        rivet::console::write_str("\n");
        rivet::exit_failure(6);
    }
    rivet::console::write_str("NO_TIMER_LEAK\n");

    rivet::report();
    rivet::console::write_str("SOAK_SMOKE_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet soak_smoke\n");

    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let _ = CHAN_TX.set(tx);
    let _ = CHAN_RX.set(rx);

    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = mutex_worker, arg = ());
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = supervisor, arg = ());

    rivet::run();
}
