//! RISC-V tick-drift test (plan.md §2.2 / [B6] acceptance).
//!
//! Sleeps 100 ms, 10 times (10k ticks), then asserts the wall-clock
//! elapsed time (measured from the CLINT `mtime`) is within 30 ms of 1 s.
//!
//! Under `-icount shift=10`, each guest instruction advances virtual time
//! by ~1 µs, so the trap handler's interrupt-entry latency is *hundreds of
//! µs per tick*. With [B6] (tick re-armed from the *previous* mtimecmp)
//! the cadence stays exactly 1 ms and elapsed ≈ 1 s ± a few ticks. With
//! the old re-arm-from-`mtime` behavior, each of the 10k ticks drifts by
//! that latency and the test fails with a distinguishable exit code.
//!
//! Run via `cargo run -p xtask -- test --target riscv --suite smoke --icount 10`.

#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

use rivet::time::Sleep;

const SLEEPS: u64 = 10;
const EXPECTED_US: u64 = SLEEPS * 100_000; // 1 s
                                           // ±30 ms tolerance: covers ≤1-tick deadline granularity per sleep and
                                           // read jitter, while being far below the hundreds-of-ms drift the old
                                           // re-arm accumulates over 10k ticks under -icount.
const TOLERANCE_US: u64 = 30_000;

#[rivet::task(priority = 0, stack = 256)]
async fn drifter() {
    let t0 = rivet::port::board::now_us();
    for _ in 0..SLEEPS {
        Sleep::<100_000>::new().await;
    }
    let t1 = rivet::port::board::now_us();
    let elapsed = t1 - t0;

    rivet::console::write_str("\nDRIFT elapsed_us=");
    print_u64(elapsed);
    rivet::console::write_str(" expected_us=");
    print_u64(EXPECTED_US);

    if elapsed.abs_diff(EXPECTED_US) <= TOLERANCE_US {
        rivet::console::write_str(" DRIFT_OK\n");
        rivet::exit_success();
    } else {
        rivet::console::write_str(" DRIFT_FAIL\n");
        rivet::exit_failure(1);
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

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet RISC-V drift_test: 10 x 100ms sleeps\n");
    rivet::run();
}
