//! Spawn-while-running stress (plan.md §2.4 / [B2] acceptance).
//!
//! A running task spawns `MAX_PTASKS` worker tasks in a tight loop (ticks
//! landing mid-registration), then one more which must return `None`
//! (registry full). Under the old `tcb::register`, a tick between the
//! `used` CAS and the `sp` store could context-switch into a half-
//! initialized task (`sp == 0`) — a use-before-init race. The fix
//! publishes `used = true` last, so the scheduler never sees a partial
//! slot. Every worker must run exactly once.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::preempt::Stack;
use rivet::time::Sleep;

const MAX_PTASKS: usize = rivet::preempt::tcb::MAX_PTASKS;

static mut STACKS: [Stack<512>; MAX_PTASKS] = [const { Stack::new() }; MAX_PTASKS];
static RAN: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static SPAWNER_DONE: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

struct Unit;
static UNIT: Unit = Unit;

fn worker(_: &'static Unit) -> ! {
    RAN.fetch_add(1, core::sync::atomic::Ordering::AcqRel);
    rivet::preempt::park_forever();
}

fn spawner(_: &'static Unit) -> ! {
    // `rivet::init()` registered the async idle task and this spawner
    // itself occupies a slot, so exactly MAX_PTASKS - 2 slots remain.
    // Spawn them all from a live task (ticks interleave mid-registration).
    // SAFETY: STACKS is only ever touched from this single spawner task;
    // each stack is handed to exactly one worker for the worker's lifetime.
    // addr_of_mut! avoids creating a reference to the static itself
    // (static_mut_refs lint).
    unsafe {
        // Indexing is required: addr_of_mut! per element (the iterator
        // form would create a reference to the mutable static).
        #[allow(clippy::needless_range_loop)]
        for i in 0..(MAX_PTASKS - 2) {
            let stack = &mut (*core::ptr::addr_of_mut!(STACKS[i])).0;
            let id = rivet::preempt::spawn(stack, 2, worker, &UNIT);
            assert!(id.is_ok(), "spawn failed");
        }
    }
    // One more must be rejected — registry full.
    let extra = unsafe { rivet::preempt::spawn(&mut STACKS[0].0, 2, worker, &UNIT) };
    assert_eq!(
        extra,
        Err(rivet::preempt::SpawnError::RegistryFull),
        "spawn past MAX_PTASKS must fail"
    );
    rivet::arch::debug_print("SPAWNER_FULL_OK\n");
    SPAWNER_DONE.store(true, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
}

#[rivet::task(priority = 0, stack = 256)]
async fn finisher() {
    loop {
        if SPAWNER_DONE.load(core::sync::atomic::Ordering::Acquire)
            && RAN.load(core::sync::atomic::Ordering::Acquire) == (MAX_PTASKS - 2) as u32
        {
            rivet::arch::debug_print("SPAWN_STRESS_OK\n");
            rivet::arch::exit_success();
        }
        Sleep::<10_000>::new().await;
    }
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

    rivet::init(); // (arch::early_init happens inside init)
    rivet::arch::debug_print("Rivet stress_spawn (B2 publish ordering)\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spawner, arg = UNIT);

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
