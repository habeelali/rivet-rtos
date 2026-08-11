//! Real-time characterization: uncontended per-operation latency bench.
//!
//! Measures, via direct `cycle_count()` bracketing (not the
//! `latency-histograms` feature, which this binary also enables and dumps
//! via `rivet::report()` for the operations it *does* cover):
//!
//! - semaphore `try_acquire`/`release` (uncontended fast path)
//! - mutex `try_lock`/unlock (uncontended fast path)
//! - `yield_now()` round trip (a lower bound on raw context-switch cost:
//!   two switches — out to the other ready task and back — not one, see
//!   the printed caveat)
//!
//! A low-priority background task spins the whole time so the periodic
//! tick has real preemption decisions to make, giving the
//! `latency-histograms` dump (IrqEntry/DispatchDecision/CriticalSection/
//! SchedulingWake) genuine samples instead of an empty report. Build with
//! `--features latency-histograms` to get that section; without it, only
//! the direct measurements below print.
//!
//! This is empirical black-box measurement under a controlled, minimal
//! load, not a formally proven WCET bound — see docs/realtime.md for the
//! methodology caveats this number set is meant to be read with.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;
use rivet::sync::Semaphore;

const N: u32 = 20_000;

static SEM: Semaphore<1> = Semaphore::new(1);
static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static STOP_BG: AtomicU32 = AtomicU32::new(0);

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

fn bg_task(_: &'static Unit) -> ! {
    // Keeps the tick handler making real preemption decisions the whole
    // benchmark run, instead of an idle system where DispatchDecision/
    // SchedulingWake histograms would stay empty.
    while STOP_BG.load(Ordering::Acquire) == 0 {
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

/// Iterations for the clock cross-check below — chosen so the loop runs
/// a few real seconds at any plausible clock for this class of MCU
/// (16-240MHz), giving the fixed flash/boot/UART overhead around it a
/// small share of the total when timed externally against wall clock.
const CLOCK_CHECK_ITERS: u32 = 20_000_000;

/// Empirical CPU-clock cross-check (plan.md real-time characterization):
/// a large, fixed-iteration busy loop (`black_box`-guarded so LTO can't
/// fold it away), cycle-counted internally and printed — externally timed
/// against host wall-clock (`time` around the capture window) to convert
/// "N cycles elapsed" into "N cycles took T seconds", i.e. a real,
/// independent Hz estimate rather than trusting the board crate's
/// documented `CPU_HZ` assumption blind. STM32's 16MHz HSI is already
/// corroborated by USART2 decoding cleanly at a baud rate computed from
/// it; this section exists mainly so the *same* binary/methodology also
/// runs on ESP32-S3/C6, where `CPU_HZ` is explicitly undocumented/
/// unmeasured in their own BSP source.
fn clock_check() {
    rivet::console::write_str("CLOCK_CHECK_START\n");
    let t0 = rivet::port::arch::cycle_count();
    // A closed-form-summable loop (e.g. `x += i`) can legally be
    // collapsed by LLVM into constant-time arithmetic even behind
    // `black_box` on the *result* — black_box only blocks "unused value"
    // elimination, not loop-strength-reduction on a provably-summable
    // recurrence. An LCG step (`x = x*A + C`) has no such closed form
    // LLVM can discover, forcing a genuine per-iteration dependency
    // chain — the standard fix for this exact benchmarking pitfall.
    let mut x: u32 = 1;
    for _ in 0..CLOCK_CHECK_ITERS {
        x = core::hint::black_box(x.wrapping_mul(1_103_515_245).wrapping_add(12_345));
    }
    let t1 = rivet::port::arch::cycle_count();
    core::hint::black_box(x);
    rivet::console::write_str("CLOCK_CHECK_CYCLES=");
    print_u64(t1 - t0);
    rivet::console::write_str(" ITERS=");
    print_u64(CLOCK_CHECK_ITERS as u64);
    rivet::console::write_str("\nCLOCK_CHECK_DONE\n");
}

// All measurement happens from a spawned ptask, not `main()` itself:
// `main()` runs before `rivet::run()` starts the preemptive scheduler,
// so there is no "current task" yet for `yield_now()`/mutex bookkeeping
// to operate on — calling them directly from `main()` hard-faults.
fn bench_task(_: &'static Unit) -> ! {
    clock_check();

    let mut sem_take = Stats::new();
    let mut sem_give = Stats::new();
    for _ in 0..N {
        let t0 = rivet::port::arch::cycle_count();
        let ok = SEM.try_acquire();
        let t1 = rivet::port::arch::cycle_count();
        sem_take.record(t1 - t0);
        if ok {
            let t2 = rivet::port::arch::cycle_count();
            SEM.release();
            let t3 = rivet::port::arch::cycle_count();
            sem_give.record(t3 - t2);
        }
    }

    let mut mtx_lock = Stats::new();
    let mut mtx_unlock = Stats::new();
    for _ in 0..N {
        let t0 = rivet::port::arch::cycle_count();
        let g = MTX.try_lock();
        let t1 = rivet::port::arch::cycle_count();
        mtx_lock.record(t1 - t0);
        if let Some(guard) = g {
            let t2 = rivet::port::arch::cycle_count();
            drop(guard);
            let t3 = rivet::port::arch::cycle_count();
            mtx_unlock.record(t3 - t2);
        }
    }

    let mut yield_rt = Stats::new();
    for _ in 0..N {
        let t0 = rivet::port::arch::cycle_count();
        rivet::yield_now();
        let t1 = rivet::port::arch::cycle_count();
        yield_rt.record(t1 - t0);
    }

    rivet::console::write_str("=== latency_bench results (cycles, uncontended) ===\n");
    report_stat("sem_try_acquire", &sem_take);
    report_stat("sem_release", &sem_give);
    report_stat("mutex_try_lock", &mtx_lock);
    report_stat("mutex_unlock", &mtx_unlock);
    report_stat("yield_now_roundtrip(~2x ctx switch)", &yield_rt);
    rivet::console::write_str("=== end latency_bench results ===\n");

    STOP_BG.store(1, Ordering::Release);
    rivet::report();

    rivet::console::write_str("LATENCY_BENCH_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet latency_bench\n");

    let _ = rivet::spawn_ptask!(stack = 256, priority = 1, entry = bg_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = bench_task, arg = UNIT);

    rivet::run();
}
