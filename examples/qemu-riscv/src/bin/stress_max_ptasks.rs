//! Capacity stress: fill the task registry exactly (plan.md §4.4).
//!
//! Spawns every remaining slot; every worker must run; one more spawn
//! returns `Err(SpawnError::RegistryFull)` — a typed error, not a panic or
//! silent drop. Runs with MPU/PMP enabled so any stack corruption becomes
//! a fault instead of a mystery.

#![no_std]
#![no_main]

use core::panic::PanicInfo;

static RAN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static SPAWNER_DONE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
static FULL_OK: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[rivet::task(priority = 0, stack = 256)]
async fn finisher() {
    let expected = (rivet::preempt::tcb::MAX_PTASKS - 2) as u32;
    loop {
        if FULL_OK.load(core::sync::atomic::Ordering::Acquire)
            && RAN.load(core::sync::atomic::Ordering::Acquire) == expected
        {
            rivet::arch::debug_print("STRESS_MAX_OK ran=");
            print_dec(expected as usize);
            rivet::arch::debug_print("\n");
            rivet::arch::exit_success();
        }
        rivet::time::Sleep::<10_000>::new().await;
    }
}

fn print_dec(mut n: usize) {
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
    let mut out = [0u8; 10];
    for j in 0..i {
        out[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&out[..i]) {
        rivet::arch::debug_print(s);
    }
}

fn worker(_: &'static ()) -> ! {
    RAN.fetch_add(1, core::sync::atomic::Ordering::AcqRel);
    rivet::preempt::park_forever();
}

fn spawner(_: &'static ()) -> ! {
    // Idle (from init) + this spawner occupy 2 slots; fill the rest.
    let slots_left = rivet::preempt::tcb::MAX_PTASKS - 2;
    for _ in 0..slots_left {
        let r = rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker, arg = ());
        if r.is_err() {
            rivet::arch::debug_print("EARLY_FULL\n");
            rivet::arch::exit_failure(4);
        }
    }
    // One more must be rejected with the typed error.
    match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker, arg = ()) {
        Err(rivet::preempt::SpawnError::RegistryFull) => {
            FULL_OK.store(true, core::sync::atomic::Ordering::Release);
        }
        other => {
            rivet::arch::debug_print("FULL_FAIL\n");
            let _ = other;
            rivet::arch::exit_failure(5);
        }
    }
    SPAWNER_DONE.store(true, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
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
    rivet::arch::debug_print("Rivet stress_max_ptasks\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spawner, arg = ());

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
