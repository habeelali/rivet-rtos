//! Preemptive-tier mutex QEMU test (plan.md §2.3 acceptance).
//!
//! Phases (priority-ordered so they run sequentially):
//!  1. **Nested priority inheritance** ([B11]): `holder` (prio 2) locks A
//!     then B; a prio-6 waiter blocks on B (boost 6), a prio-8 waiter on A
//!     (boost 8). Unlocking B must NOT drop the boost still held for A —
//!     the trace prints the effective priority after each unlock.
//!  2. **lock_timeout / try_lock**: `t` (prio 4) spawns a prio-5 task that
//!     locks T and parks forever, then `lock_timeout(50ms)` must return
//!     `Err(Timeout)`.
//!  3. **Contention stress** ([B1]): two same-priority tasks (prio 3)
//!     hammer one mutex for 1M cycles each — the exact shape in which a
//!     lost-wakeup deadlock used to live.
//!
//! A cooperative finisher polls a phase-completion mask and exits 0 when
//! all three phases are done.

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::preempt::{sched, tcb, PriorityMutex};
use rivet::time::{Duration, Sleep};

static MUTEX_A: PriorityMutex<u32> = PriorityMutex::new(0);
static MUTEX_B: PriorityMutex<u32> = PriorityMutex::new(0);
static MUTEX_T: PriorityMutex<u32> = PriorityMutex::new(0);
static MUTEX_M: PriorityMutex<u32> = PriorityMutex::new(0);

static PHASES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static STRESS_DONE: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

struct Unit;
static UNIT: Unit = Unit;

fn eff_prio() -> u8 {
    let id = sched::current().expect("task context");
    tcb::get(id)
        .unwrap()
        .effective_priority
        .load(core::sync::atomic::Ordering::Acquire)
}

// ── Phase 1: nested inheritance ([B11]) ────────────────────────────

fn holder(_: &'static Unit) -> ! {
    let ga = MUTEX_A.lock();
    let gb = MUTEX_B.lock();
    rivet::arch::debug_print("HOLDS_AB\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 6, entry = waiter_b, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 8, entry = waiter_a, arg = UNIT);
    // Yield so both waiters run and block on the held mutexes (boosting
    // us) before we read our effective priority.
    rivet::arch::yield_now();

    // Both waiters have run and blocked (they preempt us), boosting us to 8.
    rivet::arch::debug_print("EFF_WHILE_HOLDING=");
    print_u32(eff_prio() as u32);
    rivet::arch::debug_print("\n");

    drop(gb); // unlock B — must NOT drop the boost held for A
    rivet::arch::debug_print("EFF_AFTER_UNLOCK_B=");
    print_u32(eff_prio() as u32);
    rivet::arch::debug_print("\n");

    drop(ga); // unlock A — waiter_a wakes and preempts
    rivet::arch::debug_print("EFF_AFTER_UNLOCK_A=");
    print_u32(eff_prio() as u32);
    rivet::arch::debug_print("\n");

    PHASES.fetch_or(1, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
}

fn waiter_b(_: &'static Unit) -> ! {
    let _g = MUTEX_B.lock();
    rivet::arch::debug_print("WB_GOT_B\n");
    rivet::preempt::park_forever();
}

fn waiter_a(_: &'static Unit) -> ! {
    let _g = MUTEX_A.lock();
    rivet::arch::debug_print("WA_GOT_A\n");
    rivet::preempt::park_forever();
}

// ── Phase 2: lock_timeout / try_lock ───────────────────────────────

fn t_holder(_: &'static Unit) -> ! {
    let _g = MUTEX_T.lock();
    rivet::preempt::park_forever();
}

fn timeout_task(_: &'static Unit) -> ! {
    // Spawn a higher-priority holder that locks T and parks forever, then
    // yield so it actually runs and takes the mutex before we try.
    let _ = rivet::spawn_ptask!(stack = 512, priority = 5, entry = t_holder, arg = UNIT);
    rivet::arch::yield_now();

    let started = rivet::arch::now_micros();
    match MUTEX_T.lock_timeout(Some(Duration::from_millis(50))) {
        Err(rivet::preempt::mutex::LockError::Timeout) => {
            let elapsed = rivet::arch::now_micros() - started;
            rivet::arch::debug_print("TIMEOUT_OK elapsed_us=");
            print_u64(elapsed);
            rivet::arch::debug_print("\n");
        }
        other => {
            rivet::arch::debug_print("TIMEOUT_FAIL: ");
            let _ = other;
            rivet::arch::exit_failure(2);
        }
    }

    // try_lock on a free mutex must succeed; on the held one must fail.
    let free_ok = MUTEX_A.try_lock().is_some();
    let held_ok = MUTEX_T.try_lock().is_none();
    if free_ok && held_ok {
        rivet::arch::debug_print("TRYLOCK_OK\n");
    } else {
        rivet::arch::debug_print("TRYLOCK_FAIL\n");
        rivet::arch::exit_failure(3);
    }

    PHASES.fetch_or(2, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
}

// ── Phase 3: contention stress ([B1]) ──────────────────────────────

const CYCLES: u32 = 1_000_000;

fn stress_task(_: &'static Unit) -> ! {
    for _ in 0..CYCLES {
        let mut g = MUTEX_M.lock();
        *g = (*g).wrapping_add(1);
        core::hint::black_box(&*g);
    }
    let done = STRESS_DONE.fetch_add(1, core::sync::atomic::Ordering::AcqRel) + 1;
    if done == 2 {
        PHASES.fetch_or(4, core::sync::atomic::Ordering::Release);
    }
    rivet::preempt::park_forever();
}

// ── Finisher ───────────────────────────────────────────────────────

#[rivet::task(priority = 0, stack = 256)]
async fn finisher() {
    loop {
        if PHASES.load(core::sync::atomic::Ordering::Acquire) == 7 {
            rivet::arch::debug_print("MUTEX_OK\n");
            rivet::arch::exit_success();
        }
        Sleep::<10_000>::new().await;
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
    rivet::arch::debug_print("Rivet mutex_test (B1/B11/lock_timeout)\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 4, entry = timeout_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = stress_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = stress_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = holder, arg = UNIT);

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
