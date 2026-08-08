//! Fault-isolation test (plan.md §3.4 / §3.6).
//!
//! Under [`rivet::fault::FaultPolicy::IsolateTask`]: a task overflows its
//! stack while holding a mutex. The fault is attributed, the task is
//! marked Faulted, its mutex is poisoned, the user hook runs, and the
//! scheduler switches to a supervisor task — which must observe the
//! poisoned mutex and continue running. The system survives.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::preempt::PriorityMutex;
use rivet::time::Duration;

static MUTEX_M: PriorityMutex<u32> = PriorityMutex::new(0);
static FAULTED_TASK: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

fn overflowing(_: &'static ()) -> ! {
    let mut g = MUTEX_M.lock();
    *g = 42;
    core::hint::black_box(&*g);
    // Stack overflow → guarded fault → IsolateTask.
    let mut buf = [0u8; 2048];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    core::hint::black_box(&buf);
    loop {
        core::hint::spin_loop();
    }
}

fn supervisor(_: &'static ()) -> ! {
    // We run only after the faulting task was isolated.
    let faulted = FAULTED_TASK.load(core::sync::atomic::Ordering::Acquire);
    rivet::arch::debug_print("HOOK_SAW_TASK=");
    print_dec(faulted);
    rivet::arch::debug_print("\n");

    // The poisoned mutex must refuse acquisition.
    match MUTEX_M.lock_timeout(Some(Duration::from_millis(100))) {
        Err(rivet::preempt::mutex::LockError::Poisoned) => {
            rivet::arch::debug_print("POISONED_OK\n");
        }
        other => {
            rivet::arch::debug_print("POISON_FAIL: ");
            match other {
                Err(e) => {
                    rivet::arch::debug_print("err=");
                    print_dec(match e {
                        rivet::preempt::mutex::LockError::Recursive => 1,
                        rivet::preempt::mutex::LockError::Timeout => 2,
                        rivet::preempt::mutex::LockError::TooManyHeldMutexes => 3,
                        rivet::preempt::mutex::LockError::NotInTask => 4,
                        rivet::preempt::mutex::LockError::Poisoned => 5,
                    });
                }
                Ok(_) => rivet::arch::debug_print("ok"),
            }
            rivet::arch::exit_failure(3);
        }
    }

    rivet::arch::debug_print("ISOLATION_OK\n");
    rivet::arch::exit_success();
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
    rivet::arch::debug_print("Rivet CM3 fault_isolate\n");

    rivet::fault::set_policy(rivet::fault::FaultPolicy::IsolateTask);
    rivet::fault::set_on_task_fault(|id, _| {
        FAULTED_TASK.store(id, core::sync::atomic::Ordering::Release);
    });
    rivet::arch::debug_print("faulting task will be isolated\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = overflowing, arg = ());
    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = supervisor, arg = ());

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
