//! `rivet::log!`/`rivet::report()` end-to-end test (plan.md Phase 8).
//!
//! Two preemptive tasks each log a few frames (including one from a
//! context switch-heavy loop, to exercise the critical-section-guarded
//! multi-producer path against real concurrency, not just a single
//! caller), a drain task written by hand (not the built-in
//! `log::drain_forever`, to prove `drain_one` works standalone too)
//! flushes them to the console, then `rivet::report()` dumps kernel
//! state before exiting.

#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

use rivet::log::Level;

fn logger_a(_: &'static ()) -> ! {
    for i in 0..5u32 {
        // plan.md Phase 16: exercises `log!`'s interpolated-argument path
        // (not just the plain-message form logger_b still uses below).
        rivet::log!(Level::Info, "hello from A, i={}", i);
        for _ in 0..50_000u32 {
            core::hint::spin_loop();
        }
    }
    rivet::preempt::park_forever();
}

fn logger_b(_: &'static ()) -> ! {
    for _ in 0..5 {
        rivet::log!(Level::Warn, "hello from B");
        for _ in 0..50_000u32 {
            core::hint::spin_loop();
        }
    }
    rivet::preempt::park_forever();
}

#[rivet::task(priority = 0, stack = 512)]
async fn drain_and_report() {
    // Give the two loggers a chance to actually produce frames before we
    // start draining (they're higher-priority preemptive tasks, so they
    // run first anyway, but this keeps the ordering obvious).
    rivet::time::Sleep::<50_000>::new().await;

    let mut drained = 0u32;
    while rivet::log::drain_one() {
        drained += 1;
    }
    rivet::console::write_str("DRAINED ");
    print_dec(drained as usize);
    rivet::console::write_str("\n");

    rivet::report();
    rivet::console::write_str("REPORT_TEST_OK\n");
    rivet::exit_success();
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

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet report_test\n");
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = logger_a, arg = ());
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = logger_b, arg = ());
    rivet::run();
}
