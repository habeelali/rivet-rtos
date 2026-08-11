//! Real-time characterization: isolate the source of long critical
//! sections seen in `stress_load_bench`'s histograms (all three boards
//! showed samples in the 2^15 bucket — 32768-65535 cycles, ~2-4ms on
//! STM32 at 16MHz — worth identifying, not just noting).
//!
//! Each of these binaries reuses `stress_load_bench`'s load shape but
//! with only ONE interference source live at a time, so whichever one's
//! histogram still shows the 2^15+ outlier is implicated by elimination
//! — no kernel instrumentation added, matching this whole session's
//! black-box-benchmarking approach.
//!
//! This variant: mutex contention only (two same-priority contenders
//! hammering a shared `PriorityMutex`), no channel traffic, no low-prio
//! filler — isolates `PriorityMutexGuard::drop`'s unlock path (owner
//! swap, held-list recompute, `wake_all_waiters` — an O(MAX_PTASKS) scan
//! regardless of actual waiter count) as the suspect.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};
use rivet::preempt::PriorityMutex;

const DURATION_ITERS: u32 = 5_000;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);
static STOP: AtomicU32 = AtomicU32::new(0);
static MTX_ITERS: AtomicU32 = AtomicU32::new(0);

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
        MTX_ITERS.fetch_add(1, Ordering::Relaxed);
    }
    rivet::preempt::park_forever();
}

fn high_periodic(_: &'static Unit) -> ! {
    for _ in 0..DURATION_ITERS {
        rivet::preempt::sleep_ms(1);
    }
    STOP.store(1, Ordering::Release);
    rivet::preempt::park_forever();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet critsec_isolate_bench (mutex-only)\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = mutex_contender, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = mutex_contender, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 8, entry = high_periodic, arg = UNIT);

    rivet::run();
}

#[rivet::task(priority = 0, stack = 512)]
async fn finisher() {
    use rivet::time::Sleep;
    loop {
        if STOP.load(Ordering::Acquire) != 0 {
            for _ in 0..1000 {
                core::hint::spin_loop();
            }
            rivet::console::write_str("mutex_contender_iters=");
            print_u64(MTX_ITERS.load(Ordering::Relaxed) as u64);
            rivet::console::write_str("\n");
            rivet::report();
            rivet::console::write_str("CRITSEC_ISOLATE_BENCH_OK\n");
            rivet::exit_success();
        }
        Sleep::<50_000>::new().await;
    }
}
