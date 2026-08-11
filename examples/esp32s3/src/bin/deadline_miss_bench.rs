//! Real-time characterization: deadline-miss testing under realistic load.
//!
//! Distinct from `deadline_test.rs` (which proves the *mechanism* —
//! `wait_period`'s drift-corrected cadence and `BudgetExceeded` fault
//! delivery — in an otherwise near-idle system): this runs a
//! periodic, deadline-bound high-priority task for many periods **while**
//! the same combined interference `stress_load_bench` uses (mutex
//! contention, channel producer/consumer traffic, a low-priority busy
//! task) runs concurrently, and directly measures each period's actual
//! inter-wake interval against the required deadline. This is the "hard
//! real-time" definition from the testing methodology this binary is
//! built against: for this explicitly-defined workload, are deadline
//! misses bounded at (ideally) zero, not just bounded-and-nonzero.
//!
//! A "miss" here is defined as: the interval between consecutive wakes
//! of the periodic task exceeded `PERIOD_US + DEADLINE_SLACK_US`. The
//! task's own priority (highest in the system) means the scheduler
//! should never let anything lower-priority delay it past that slack,
//! which is itself generous headroom over one dispatch-decision's own
//! cost, not the deadline itself.

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;
use rivet::sync::Channel;

const PERIODS: u32 = 500;
const PERIOD_US: u32 = 2_000;
/// Generous but real headroom over one period — a genuine hard-real-time
/// deployment would tune this to its own dispatch-latency budget; this is
/// deliberately loose (2x the period) so the pass/fail bar is "is this
/// bounded at all under load", not a tightly-tuned production margin.
const DEADLINE_SLACK_US: u32 = PERIOD_US * 2;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static CHAN: Channel<u32, 8> = Channel::new();
static CHAN_TX: rivet::sync::Once<rivet::sync::Sender<'static, u32, 8>> = rivet::sync::Once::new();
static CHAN_RX: rivet::sync::Once<rivet::sync::Receiver<'static, u32, 8>> =
    rivet::sync::Once::new();
static STOP: AtomicU32 = AtomicU32::new(0);
static MISSES: AtomicU32 = AtomicU32::new(0);
static WORST_LATE_US: AtomicU32 = AtomicU32::new(0);

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

fn mutex_contender(_: &'static Unit) -> ! {
    while STOP.load(Ordering::Acquire) == 0 {
        let mut g = MTX.lock();
        *g = g.wrapping_add(1);
        core::hint::black_box(&*g);
        drop(g);
    }
    rivet::preempt::park_forever();
}

fn channel_producer(_: &'static Unit) -> ! {
    let tx = CHAN_TX.get().expect("sender set before this runs");
    let mut n: u32 = 0;
    while STOP.load(Ordering::Acquire) == 0 {
        if tx.try_send(n).is_ok() {
            n = n.wrapping_add(1);
        }
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn channel_consumer(_: &'static Unit) -> ! {
    let rx = CHAN_RX.get().expect("receiver set before this runs");
    loop {
        if rx.try_recv().is_none() && STOP.load(Ordering::Acquire) != 0 {
            break;
        }
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn low_busy(_: &'static Unit) -> ! {
    while STOP.load(Ordering::Acquire) == 0 {
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn periodic_deadline_task(_: &'static Unit) -> ! {
    let mut last_us = rivet::port::board::now_us();
    for _ in 0..PERIODS {
        rivet::deadlines::wait_period();
        let now = rivet::port::board::now_us();
        let interval = now.wrapping_sub(last_us) as u32;
        last_us = now;

        if interval > PERIOD_US + DEADLINE_SLACK_US {
            MISSES.fetch_add(1, Ordering::Relaxed);
            let late = interval - PERIOD_US;
            WORST_LATE_US.fetch_max(late, Ordering::Relaxed);
        }
    }

    STOP.store(1, Ordering::Release);

    let misses = MISSES.load(Ordering::Acquire);
    rivet::console::write_str("=== deadline_miss_bench ===\n");
    rivet::console::write_str("periods=");
    print_u64(PERIODS as u64);
    rivet::console::write_str(" period_us=");
    print_u64(PERIOD_US as u64);
    rivet::console::write_str(" slack_us=");
    print_u64(DEADLINE_SLACK_US as u64);
    rivet::console::write_str("\nmisses=");
    print_u64(misses as u64);
    rivet::console::write_str(" worst_late_us=");
    print_u64(WORST_LATE_US.load(Ordering::Acquire) as u64);
    rivet::console::write_str("\n");

    rivet::report();

    if misses == 0 {
        rivet::console::write_str("DEADLINE_MISS_BENCH_OK\n");
        rivet::exit_success();
    } else {
        rivet::console::write_str("DEADLINE_MISS_BENCH_MISSES_OBSERVED\n");
        rivet::exit_failure(1);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet deadline_miss_bench\n");

    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let _ = CHAN_TX.set(tx);
    let _ = CHAN_RX.set(rx);

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = low_busy, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = channel_producer, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = channel_consumer, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = mutex_contender, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = mutex_contender, arg = UNIT);

    let periodic =
        rivet::spawn_ptask!(stack = 2048, priority = 9, entry = periodic_deadline_task, arg = UNIT)
            .unwrap_or_else(|_| rivet::exit_failure(9));
    periodic.set_period_us(PERIOD_US);

    rivet::run();
}
