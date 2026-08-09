//! Proves genuine concurrent multi-hart execution (plan.md Phase 19),
//! replacing/supplementing `xtask`'s older "safety-only" `-smp N == -smp 1`
//! structural check — that check only proves other harts aren't corrupting
//! shared state (they could all still just be parking); this one proves
//! they're doing real, useful, concurrent scheduling work.
//!
//! Spawns `2 * RIVET_MAX_HARTS` equal-priority preemptive tasks, each
//! incrementing its own dedicated counter a fixed number of times (with a
//! small spin between increments so the run takes enough ticks for the
//! scheduler to actually distribute tasks across harts, not finish inside
//! one quantum on hart 0 alone) and recording every hart id it ever ran on
//! into a shared bitmask. A monitor task waits for every worker to finish,
//! then asserts:
//!
//! - **(a)** more than one distinct hart id was observed (skipped when
//!   built with `RIVET_MAX_HARTS=1`, where this is trivially just hart 0 —
//!   the single-hart-degenerate case plan.md Phase 19 explicitly requires
//!   to still work) — proving real concurrency, not just safety.
//! - **(b)** the sum of every counter exactly equals `N * ITERS` — proving
//!   no dispatch was ever lost or duplicated across harts.
//!
//! Built with `RIVET_MAX_HARTS` set via env (`xtask` passes 1/2/4) and run
//! under matching `-smp N`.

#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

use core::sync::atomic::{AtomicU32, Ordering};

const MAX_HARTS: usize = rivet::config::MAX_HARTS;
const N: usize = 2 * MAX_HARTS;
const ITERS: u32 = 20_000;

static COUNTERS: [AtomicU32; N] = [const { AtomicU32::new(0) }; N];
static DONE: AtomicU32 = AtomicU32::new(0);
static OBSERVED_HARTS: AtomicU32 = AtomicU32::new(0);

fn worker(counter: &'static AtomicU32) -> ! {
    for _ in 0..ITERS {
        counter.fetch_add(1, Ordering::Relaxed);
        OBSERVED_HARTS.fetch_or(1 << rivet::port::arch::hart_id(), Ordering::Relaxed);
        // A little wall-clock weight per iteration: enough that this
        // worker's total run spans many tick periods (giving the
        // scheduler real opportunities to distribute the ready pool
        // across harts), without making the smoke suite slow.
        for _ in 0..200u32 {
            core::hint::spin_loop();
        }
    }
    DONE.fetch_add(1, Ordering::AcqRel);
    rivet::preempt::park_forever();
}

fn print_dec(mut n: u32) {
    if n == 0 {
        rivet::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut buf = [0u8; 10];
    for j in 0..i {
        buf[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&buf[..i]) {
        rivet::console::write_str(s);
    }
}

#[rivet::task(priority = 0, stack = 512)]
async fn monitor() {
    loop {
        if DONE.load(Ordering::Acquire) == N as u32 {
            let sum: u32 = COUNTERS.iter().map(|c| c.load(Ordering::Acquire)).sum();
            let expected = N as u32 * ITERS;
            let distinct = OBSERVED_HARTS.load(Ordering::Acquire).count_ones();

            rivet::console::write_str("SUM=");
            print_dec(sum);
            rivet::console::write_str(" EXPECTED=");
            print_dec(expected);
            rivet::console::write_str(" DISTINCT_HARTS=");
            print_dec(distinct);
            rivet::console::write_str("\n");

            let sum_ok = sum == expected;
            // Single-hart-degenerate case (MAX_HARTS == 1, plan.md Phase
            // 19's explicit requirement): only one hart can ever be
            // observed, so this assertion would be unfalsifiable there —
            // skip it rather than require something structurally
            // impossible.
            let hart_ok = MAX_HARTS <= 1 || distinct > 1;

            if sum_ok && hart_ok {
                rivet::console::write_str("SMP_TEST_OK\n");
                rivet::exit_success();
            } else {
                rivet::console::write_str("SMP_TEST_FAIL\n");
                rivet::exit_failure(1);
            }
        }
        rivet::time::Sleep::<5_000>::new().await;
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet smp_test\n");
    // Each task needs its own dedicated counter slot; `COUNTERS[i]` (not
    // an iterator item) is exactly what `spawn_ptask!`'s `arg` needs to
    // borrow as `&'static AtomicU32`.
    #[allow(clippy::needless_range_loop)]
    for i in 0..N {
        let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker, arg = COUNTERS[i]);
    }
    rivet::run();
}
