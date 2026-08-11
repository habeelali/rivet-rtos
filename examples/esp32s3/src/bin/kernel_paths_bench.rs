//! Real-time characterization: remaining kernel paths not covered by
//! `latency_bench.rs` (semaphore/mutex/yield only) — task spawn/despawn
//! and channel send/recv, rounding out kernel-path coverage for the
//! testing methodology's "characterize all relevant kernel paths" ask.
//! `Sleep`/timer create-cancel is deliberately not added here: it's an
//! async-tier-only API (`.await`-based), not callable from a preemptive
//! task's synchronous context the way this bench's other operations are
//! — its real-world cost is already reflected in `stress_load_bench`'s/
//! `deadline_miss_bench`'s SchedulingWake histogram (wake-from-`Sleep`
//! is exactly what that measures), just not isolated as its own number.

#![no_std]
#![no_main]

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

use rivet::sync::Channel;

const N: u32 = 2_000;

static CHAN: Channel<u32, 8> = Channel::new();

struct Unit;
static UNIT: Unit = Unit;

struct Stats {
    min: u64,
    max: u64,
    sum: u64,
    n: u64,
}
impl Stats {
    const fn new() -> Self {
        Self { min: u64::MAX, max: 0, sum: 0, n: 0 }
    }
    fn record(&mut self, v: u64) {
        if v < self.min {
            self.min = v;
        }
        if v > self.max {
            self.max = v;
        }
        self.sum += v;
        self.n += 1;
    }
}

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

fn report_stat(name: &str, s: &Stats) {
    rivet::console::write_str(name);
    rivet::console::write_str(" min=");
    print_u64(s.min);
    rivet::console::write_str(" max=");
    print_u64(s.max);
    rivet::console::write_str(" avg=");
    print_u64(if s.n > 0 { s.sum / s.n } else { 0 });
    rivet::console::write_str(" n=");
    print_u64(s.n);
    rivet::console::write_str(" cycles\n");
}

fn dummy_task(_: &'static Unit) -> ! {
    rivet::preempt::park_forever();
}

fn bench_task(_: &'static Unit) -> ! {
    let mut spawn_stat = Stats::new();
    let mut despawn_stat = Stats::new();
    for _ in 0..N {
        let t0 = rivet::port::arch::cycle_count();
        let h = rivet::spawn_ptask!(stack = 512, priority = 2, entry = dummy_task, arg = UNIT);
        let t1 = rivet::port::arch::cycle_count();
        spawn_stat.record(t1 - t0);
        if let Ok(handle) = h {
            // Xtensa-specific (rivet-arch-xtensa): a spawned task's
            // bootstrap-table slot isn't freed until it's actually
            // dispatched at least once — despawning 2000 tasks in a tight
            // loop without ever letting any of them run exhausts that
            // table (`more tasks spawned (21) than the bootstrap table's
            // capacity (20) will ever fit`), a real per-arch limitation
            // this bench's own spawn/despawn stress hit. One `yield_now`
            // gives the newly-spawned task its one dispatch before this
            // despawns it, keeping the table's bootstrap entries
            // recycling instead of monotonically filling up.
            rivet::yield_now();
            let t2 = rivet::port::arch::cycle_count();
            handle.despawn();
            let t3 = rivet::port::arch::cycle_count();
            despawn_stat.record(t3 - t2);
        }
    }

    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let mut send_stat = Stats::new();
    let mut recv_stat = Stats::new();
    for i in 0..N {
        let t0 = rivet::port::arch::cycle_count();
        let ok = tx.try_send(i).is_ok();
        let t1 = rivet::port::arch::cycle_count();
        send_stat.record(t1 - t0);
        if ok {
            let t2 = rivet::port::arch::cycle_count();
            let _ = rx.try_recv();
            let t3 = rivet::port::arch::cycle_count();
            recv_stat.record(t3 - t2);
        }
    }

    rivet::console::write_str("=== kernel_paths_bench results (cycles) ===\n");
    report_stat("spawn_ptask", &spawn_stat);
    report_stat("despawn", &despawn_stat);
    report_stat("channel_try_send", &send_stat);
    report_stat("channel_try_recv", &recv_stat);
    rivet::console::write_str("=== end kernel_paths_bench results ===\n");

    rivet::report();
    rivet::console::write_str("KERNEL_PATHS_BENCH_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet kernel_paths_bench\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = bench_task, arg = UNIT);
    rivet::run();
}
