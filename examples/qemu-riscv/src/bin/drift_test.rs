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

use core::panic::PanicInfo;
use rivet::time::Sleep;

const SLEEPS: u64 = 10;
const EXPECTED_US: u64 = SLEEPS * 100_000; // 1 s
                                           // ±30 ms tolerance: covers ≤1-tick deadline granularity per sleep and
                                           // read jitter, while being far below the hundreds-of-ms drift the old
                                           // re-arm accumulates over 10k ticks under -icount.
const TOLERANCE_US: u64 = 30_000;

#[rivet::task(priority = 0, stack = 256)]
async fn drifter() {
    let t0 = rivet::arch::now_micros();
    for _ in 0..SLEEPS {
        Sleep::<100_000>::new().await;
    }
    let t1 = rivet::arch::now_micros();
    let elapsed = t1 - t0;

    rivet::arch::debug_print("\nDRIFT elapsed_us=");
    print_u64(elapsed);
    rivet::arch::debug_print(" expected_us=");
    print_u64(EXPECTED_US);

    if elapsed.abs_diff(EXPECTED_US) <= TOLERANCE_US {
        rivet::arch::debug_print(" DRIFT_OK\n");
        rivet::arch::exit_success();
    } else {
        rivet::arch::debug_print(" DRIFT_FAIL\n");
        rivet::arch::exit_failure(1);
    }
}

fn print_u64(mut n: u64) {
    if n == 0 {
        rivet::arch::debug_print("0");
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
        rivet::arch::debug_print(s);
    }
}

// ── Startup ───────────────────────────────────────────────────────

extern "C" {
    static __stack_top: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

core::arch::global_asm!(
    ".section .text._start",
    ".global _start",
    "_start:",
    "  la    sp, __stack_top",
    "  la    t0, __bss_start",
    "  la    t1, __bss_end",
    "1:",
    "  bgeu  t0, t1, 2f",
    "  sw    zero, 0(t0)",
    "  addi  t0, t0, 4",
    "  j     1b",
    "2:",
    "  call  rust_main",
    "  ebreak",
);

#[no_mangle]
fn rust_main() -> ! {
    rivet::arch::early_init();
    rivet::init();
    rivet::arch::debug_print("Rivet RISC-V drift_test: 10 x 100ms sleeps\n");
    rivet::run();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        print_u64(loc.line() as u64);
    }
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}
