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

// ── Startup (Cortex-M3 boilerplate) ────────────────────────────────

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

    rivet::init();
    rivet::arch::debug_print("Rivet CM3 stress_max_ptasks\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spawner, arg = ());

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

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn HardFault() {
    rivet::arch::debug_print("HARD_FAULT\n");
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly.
#[no_mangle]
pub unsafe extern "C" fn UsageFault() {
    rivet::arch::debug_print("USAGE_FAULT\n");
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
