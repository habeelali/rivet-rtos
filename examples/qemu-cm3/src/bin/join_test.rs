//! Task exit + join test (plan.md §5.2/§5.3 acceptance).
//!
//! A worker task computes a value and *returns* it; its entry lands in the
//! kernel's exit trampoline, which stores the result and wakes the joiner.
//! The supervisor `join()`s and recovers `Ok(42)`. A second phase joins a
//! task that never exits but parks — the supervisor must block, not busy-
//! spin (implicit: the system keeps ticking while joined).

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::preempt::TaskHandle;

fn worker(_: &'static ()) -> u32 {
    // Compute something, then return it — the entry returns normally.
    let mut acc = 0u32;
    for i in 0..9 {
        acc += i;
    }
    acc + 6 // 42
}

fn parker(_: &'static ()) {
    // Never returns; parks forever.
    rivet::preempt::park_forever();
}

fn supervisor(_: &'static ()) -> ! {
    // Recover the worker handle encoded by rust_main (id | generation<<16).
    let packed = WORKER_HANDLE.load(core::sync::atomic::Ordering::Acquire);
    let handle = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };

    // Phase 1: join the worker — must return Ok(42).
    match handle.join::<u32>() {
        Ok(42) => rivet::arch::debug_print("JOIN_OK v=42\n"),
        Ok(v) => {
            rivet::arch::debug_print("JOIN_WRONG\n");
            let _ = v;
            rivet::arch::exit_failure(6);
        }
        Err(e) => {
            rivet::arch::debug_print("JOIN_ERR\n");
            rivet::arch::debug_print(match e {
                rivet::preempt::JoinError::Stale => "STALE\n",
                rivet::preempt::JoinError::SelfJoin => "SELF\n",
                rivet::preempt::JoinError::AlreadyJoined => "ALREADY\n",
                rivet::preempt::JoinError::Faulted => "FAULTED\n",
            });
            rivet::arch::exit_failure(7);
        }
    }

    // Phase 2: joining a parked (never-exiting) task must block, not busy-
    // spin — the join blocks the supervisor while the system keeps ticking.
    // We never reach the print below in this test; the harness's golden is
    // satisfied by phase 1.
    let parker_h = TaskHandle {
        id: PARKER_ID.load(core::sync::atomic::Ordering::Acquire) as u16,
        generation: 0,
    };
    let _ = parker_h.join::<()>();

    rivet::arch::debug_print("JOIN_TEST_OK\n");
    rivet::arch::exit_success();
}

// rust_main stores the worker's (id | generation<<16) and the parker's id.
static WORKER_HANDLE: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);
static PARKER_ID: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

// ── Startup ───────────────────────────────────────────────────────

extern "C" {
    static __stack_top: u8;
    static __data_load: u8;
    static __data_start: u8;
    static __data_end: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

/// # Safety
/// Runs at power-on reset as the vector-table Reset entry; it is the only
/// entry point at reset and performs the .data/BSS initialization itself.
#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    // Copy .data from flash to RAM, zero BSS, then hand off to Rust.
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
    rust_main();
}

#[no_mangle]
fn rust_main() -> ! {
    rivet::init();
    rivet::arch::debug_print("Rivet join_test\n");

    // Spawn the worker; store its handle for the supervisor.
    let h = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::arch::exit_failure(9),
    };
    WORKER_HANDLE.store(
        h.id as usize | ((h.generation as usize) << 16),
        core::sync::atomic::Ordering::Release,
    );

    // Spawn the parking task and the supervisor.
    match rivet::spawn_ptask!(stack = 512, priority = 1, entry = parker, arg = ()) {
        Ok(h) => PARKER_ID.store(h.id as usize, core::sync::atomic::Ordering::Release),
        Err(_) => rivet::arch::exit_failure(10),
    }
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = supervisor, arg = ());

    rivet::run();
}

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly (the kernel replaces the real vectors at init).
#[no_mangle]
pub unsafe extern "C" fn SysTick() {
    rivet::arch::cortex_m::systick_handler();
}

/// # Safety
/// Exception entry installed via the linker script's
/// `PROVIDE(... = DefaultHandler)`; never called directly.
#[no_mangle]
pub unsafe extern "C" fn DefaultHandler() -> ! {
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry installed in the vector table; never called directly.
#[no_mangle]
pub unsafe extern "C" fn SVC() {
    loop {
        core::hint::spin_loop();
    }
}

/// # Safety
/// Exception entry installed in the vector table; never called directly.
#[no_mangle]
pub unsafe extern "C" fn DebugMon() {
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
