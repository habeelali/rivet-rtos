//! Real-time characterization: worst-case latency under combined load.
//!
//! Unlike `latency_bench` (uncontended, near-idle system), this runs
//! several real sources of interference concurrently for a fixed
//! duration and lets the `latency-histograms` feature (IrqEntry/
//! DispatchDecision/CriticalSection/SchedulingWake) capture the actual
//! worst case observed under that load — the number the "under load"
//! part of the testing methodology mindmap asks for, as distinct from
//! the best-case/uncontended numbers `latency_bench` reports:
//!
//! - a high-priority periodic task (`Sleep`-based, simulating a
//!   deadline-driven workload)
//! - two same-priority tasks hammering a shared `PriorityMutex`
//!   (contention, not the uncontended fast path)
//! - a channel producer/consumer pair (`try_send`/`try_recv` traffic)
//! - a low-priority busy task (pure background interference)
//! - the periodic tick itself, firing the whole time regardless
//!
//! Must build with `--features latency-histograms` to get the actual
//! worst-case dump; without it, this still runs the load (useful as a
//! stress/soak-adjacent smoke check) but has nothing new to report over
//! `rivet::report()`'s normal fields.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;
use rivet::sync::Channel;
use rivet::time::Sleep;

const DURATION_ITERS: u32 = 5_000;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static CHAN: Channel<u32, 8> = Channel::new();
static CHAN_TX: rivet::sync::Once<rivet::sync::Sender<'static, u32, 8>> = rivet::sync::Once::new();
static CHAN_RX: rivet::sync::Once<rivet::sync::Receiver<'static, u32, 8>> =
    rivet::sync::Once::new();
static STOP: AtomicU32 = AtomicU32::new(0);
static HIGH_TICKS: AtomicU32 = AtomicU32::new(0);
static MTX_ITERS: AtomicU32 = AtomicU32::new(0);
static CHAN_SENT: AtomicU32 = AtomicU32::new(0);
static CHAN_RECV: AtomicU32 = AtomicU32::new(0);
static LOW_ITERS: AtomicU32 = AtomicU32::new(0);

struct Unit;
static UNIT: Unit = Unit;

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

fn high_periodic(_: &'static Unit) -> ! {
    for _ in 0..DURATION_ITERS {
        // A short, fixed-period sleep — the "deadline-driven" task whose
        // wakeup latency the tick/dispatch histograms below characterize
        // under the load the other tasks generate concurrently.
        rivet::preempt::sleep_ms(1);
        HIGH_TICKS.fetch_add(1, Ordering::Relaxed);
    }
    STOP.store(1, Ordering::Release);
    rivet::preempt::park_forever();
}

fn mutex_contender(_: &'static Unit) -> ! {
    while STOP.load(Ordering::Acquire) == 0 {
        let mut g = MTX.lock();
        *g = g.wrapping_add(1);
        core::hint::black_box(&*g);
        drop(g);
        MTX_ITERS.fetch_add(1, Ordering::Relaxed);
    }
    rivet::preempt::park_forever();
}

fn channel_producer(_: &'static Unit) -> ! {
    let tx = CHAN_TX.get().expect("sender set before this runs");
    let mut n: u32 = 0;
    while STOP.load(Ordering::Acquire) == 0 {
        if tx.try_send(n).is_ok() {
            n = n.wrapping_add(1);
            CHAN_SENT.fetch_add(1, Ordering::Relaxed);
        }
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn channel_consumer(_: &'static Unit) -> ! {
    let rx = CHAN_RX.get().expect("receiver set before this runs");
    loop {
        if let Some(_v) = rx.try_recv() {
            CHAN_RECV.fetch_add(1, Ordering::Relaxed);
        } else if STOP.load(Ordering::Acquire) != 0 {
            break;
        }
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn low_busy(_: &'static Unit) -> ! {
    while STOP.load(Ordering::Acquire) == 0 {
        LOW_ITERS.fetch_add(1, Ordering::Relaxed);
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet stress_load_bench\n");

    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let _ = CHAN_TX.set(tx);
    let _ = CHAN_RX.set(rx);

    // Mutex contenders, channel producer/consumer all share priority 2:
    // none of these ever voluntarily block/yield, so under strict
    // fixed-priority scheduling a *higher*-priority never-yielding task
    // would starve a lower one completely (correct scheduler behavior,
    // but it would silently reduce this to a single-source-of-load test)
    // — putting them at the same level lets tick-driven round-robin
    // actually share the CPU across all four, so channel traffic and
    // mutex contention genuinely overlap the way real combined load
    // would. `low_busy` at priority 1 is expected to starve completely
    // under this — that's fine, it's pure background filler, not a
    // metric this binary reports on.
    let _ = rivet::spawn_ptask!(stack = 256, priority = 1, entry = low_busy, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = channel_producer, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = channel_consumer, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = mutex_contender, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = mutex_contender, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 8, entry = high_periodic, arg = UNIT);

    rivet::run();
}

// Finisher: a cooperative async task polls for STOP and prints the
// summary + histogram dump once the high-priority periodic task has run
// its full course.
#[rivet::task(priority = 0, stack = 512)]
async fn finisher() {
    loop {
        if STOP.load(Ordering::Acquire) != 0 {
            // Give the other ptasks a moment to notice STOP and park.
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
            rivet::console::write_str("=== stress_load_bench summary ===\n");
            rivet::console::write_str("high_periodic_ticks=");
            print_u64(HIGH_TICKS.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\nmutex_contender_iters=");
            print_u64(MTX_ITERS.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\nchannel_sent=");
            print_u64(CHAN_SENT.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\nchannel_recv=");
            print_u64(CHAN_RECV.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\nlow_busy_iters=");
            print_u64(LOW_ITERS.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\n");
            rivet::report();
            rivet::console::write_str("STRESS_LOAD_BENCH_OK\n");
            rivet::exit_success();
        }
        Sleep::<50_000>::new().await;
    }
}
