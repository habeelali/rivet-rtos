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

use core::panic::PanicInfo;
use rivet::time::Sleep;

#[rivet::task(priority = 0, stack = 256)]
async fn wrapper() {
    let mut fires: u32 = 0;
    loop {
        Sleep::<100_000>::new().await; // 100 ms
        fires = fires.wrapping_add(1);
        let now = rivet::arch::now_micros();
        if now > (1u64 << 32) {
            rivet::arch::debug_print("AFTER_WRAP now=");
            print_u64(now);
            rivet::arch::debug_print(" fires=");
            print_u32(fires);
            rivet::arch::debug_print("\n");
            rivet::arch::exit_success();
        }
    }
}

fn print_u32(mut n: u32) {
    if n == 0 {
        rivet::arch::debug_print("0");
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
        rivet::arch::debug_print(s);
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

// ── Startup (same boilerplate as the main demo) ───────────────────

extern "C" {
    static __data_load: u8;
    static __data_start: u8;
    static __data_end: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

/// # Safety
/// Runs at power-on reset as the vector-table Reset entry; performs the
/// .data copy and .bss zeroing, then starts the kernel.
#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    let data_load = core::ptr::addr_of!(__data_load);
    let data_start = core::ptr::addr_of!(__data_start);
    let data_end = core::ptr::addr_of!(__data_end);
    let count = data_end as usize - data_start as usize;
    for i in 0..count {
        core::ptr::write(
            (data_start as *mut u8).add(i),
            core::ptr::read(data_load.add(i)),
        );
    }

    let bss_start = core::ptr::addr_of!(__bss_start);
    let bss_end = core::ptr::addr_of!(__bss_end);
    let bss_count = bss_end as usize - bss_start as usize;
    for i in 0..bss_count {
        core::ptr::write((bss_start as *mut u8).add(i), 0);
    }

    rivet::arch::early_init();
    // Seed the tick counter just below the 2^32-µs boundary (4_290_000
    // ticks × 1000 µs = 4.29e9 µs), so the next few ticks cross the old
    // u32-µs wrap point in ~2 s instead of ~71 minutes (plan.md §2.2 [B5]
    // acceptance). The counter then keeps counting as normal.
    // 2^32 µs = 4_294_967_296 µs → tick seed = 4_294_967 ticks; back off 10
    // so the crossing happens a few ticks into the run.
    rivet::arch::cortex_m::systick_seed_ticks(4_294_957);
    rivet::init();
    rivet::arch::debug_print("Rivet CM3 soak_time_wrap: crossing 2^32 µs\n");
    rivet::run();
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn SysTick() {
    rivet::arch::cortex_m::systick_handler();
}

/// # Safety
/// Exception entry point installed in the vector table (via
/// `PROVIDE(... = DefaultHandler)` in the linker script); never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn DefaultHandler() {
    rivet::arch::debug_print("DEFAULT_HANDLER/FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

fn print_hex32(mut n: u32) {
    let mut buf = [0u8; 8];
    for i in (0..8).rev() {
        let d = (n & 0xF) as u8;
        buf[i] = if d < 10 { b'0' + d } else { b'a' + d - 10 };
        n >>= 4;
    }
    if let Ok(s) = core::str::from_utf8(&buf) {
        rivet::arch::debug_print(s);
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn HardFault() {
    let cfsr = core::ptr::read_volatile(0xE000ED28 as *const u32);
    rivet::arch::debug_print("HARD_FAULT cfsr=0x");
    print_hex32(cfsr);
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn UsageFault() {
    let cfsr = core::ptr::read_volatile(0xE000ED28 as *const u32);
    rivet::arch::debug_print("USAGE_FAULT cfsr=0x");
    print_hex32(cfsr);
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn BusFault() {
    rivet::arch::debug_print("BUS_FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        print_u32(loc.line());
    }
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}
