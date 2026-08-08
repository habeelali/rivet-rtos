//! Watchdog test (plan.md §3.5 / §3.6).
//!
//! The watchdog is armed with a short period; a task feeds it for a few
//! periods then stops. The watchdog fires: on Cortex-M the real
//! luminary-watchdog hardware resets the system (QEMU models reset-on-
//! expiry); on RISC-V the software watchdog resets via `riscv.sifive.test`
//! (0x7777). Either way "RIVET WATCHDOG TIMEOUT" is printed first — the
//! harness asserts the marker via golden-on-timeout (the reset reboots the
//! guest rather than exiting).

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::time::Duration;

static FEEDS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn feeder(_: &'static ()) -> ! {
    // Feed 3 watchdog periods, then go silent.
    loop {
        let n = FEEDS.load(core::sync::atomic::Ordering::Acquire);
        if n >= 3 {
            // Stop feeding: the watchdog must fire.
            loop {
                core::hint::spin_loop();
            }
        }
        rivet::watchdog::feed();
        FEEDS.store(n + 1, core::sync::atomic::Ordering::Release);
        // Wait ~half a watchdog period between feeds.
        for _ in 0..200_000 {
            core::hint::spin_loop();
        }
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
    rivet::init();
    rivet::arch::debug_print("Rivet watchdog_test: feeding then going silent\n");

    // 250 ms watchdog period (RISC-V software watchdog; CM3 hardware WDT).
    rivet::watchdog::init(Duration::from_millis(250));

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = feeder, arg = ());

    rivet::run();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        let mut n = loc.line();
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
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}
