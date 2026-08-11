//! CM3 time-wrap soak (plan.md §2.2 / [B5] acceptance).
//!
//! Runs `2^32` microseconds of simulated time — past the point where the
//! old `u32` microsecond counter wrapped (71.6 minutes) — and asserts that
//! `Sleep::<100_000>` still fires. Under the old code `now_micros()` wraps
//! and the deadline can never be reached (silent hang → xtask timeout);
//! with the tick-counter fix it fires and the binary exits 0.
//!
//! Run via `cargo run -p xtask -- test --target cm3 --suite smoke --icount 20`
//! (the harness injects `-icount`).

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use rivet::time::Sleep;

#[rivet::task(priority = 0, stack = 256)]
async fn wrapper() {
    let mut fires: u32 = 0;
    loop {
        Sleep::<100_000>::new().await; // 100 ms
        fires = fires.wrapping_add(1);
        let now = rivet::port::board::now_us();
        if now > (1u64 << 32) {
            rivet::console::write_str("AFTER_WRAP now=");
            print_u64(now);
            rivet::console::write_str(" fires=");
            print_u32(fires);
            rivet::console::write_str("\n");
            rivet::exit_success();
        }
    }
}

fn print_u32(mut n: u32) {
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
    // Seed the tick counter just below the 2^32-µs boundary (4_290_000
    // ticks × 1000 µs = 4.29e9 µs), so the next few ticks cross the old
    // u32-µs wrap point in ~2 s instead of ~71 minutes (plan.md §2.2 [B5]
    // acceptance). The counter then keeps counting as normal. Safe to do
    // any time before `run()`: SysTick isn't actually enabled (and so
    // can't increment away from the seed) until `run()` starts the first
    // task.
    // 2^32 µs = 4_294_967_296 µs → tick seed = 4_294_967 ticks; back off
    // 10 so the crossing happens a few ticks into the run.
    rivet_arch_cortex_m::systick::seed_ticks(4_294_957);
    rivet::console::write_str("Rivet CM3 soak_time_wrap: crossing 2^32 µs\n");
    rivet::run();
}
