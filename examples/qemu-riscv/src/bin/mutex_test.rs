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

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

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
    rivet::console::write_str("HOLDS_AB\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 6, entry = waiter_b, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 8, entry = waiter_a, arg = UNIT);
    // Yield so both waiters run and block on the held mutexes (boosting
    // us) before we read our effective priority.
    rivet::yield_now();

    // Both waiters have run and blocked (they preempt us), boosting us to 8.
    rivet::console::write_str("EFF_WHILE_HOLDING=");
    print_u32(eff_prio() as u32);
    rivet::console::write_str("\n");

    drop(gb); // unlock B — must NOT drop the boost held for A
    rivet::console::write_str("EFF_AFTER_UNLOCK_B=");
    print_u32(eff_prio() as u32);
    rivet::console::write_str("\n");

    drop(ga); // unlock A — waiter_a wakes and preempts
    rivet::console::write_str("EFF_AFTER_UNLOCK_A=");
    print_u32(eff_prio() as u32);
    rivet::console::write_str("\n");

    PHASES.fetch_or(1, core::sync::atomic::Ordering::Release);
    rivet::preempt::park_forever();
}

fn waiter_b(_: &'static Unit) -> ! {
    let _g = MUTEX_B.lock();
    rivet::console::write_str("WB_GOT_B\n");
    rivet::preempt::park_forever();
}

fn waiter_a(_: &'static Unit) -> ! {
    let _g = MUTEX_A.lock();
    rivet::console::write_str("WA_GOT_A\n");
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
    rivet::yield_now();

    let started = rivet::port::board::now_us();
    match MUTEX_T.lock_timeout(Some(Duration::from_millis(50))) {
        Err(rivet::preempt::mutex::LockError::Timeout) => {
            let elapsed = rivet::port::board::now_us() - started;
            rivet::console::write_str("TIMEOUT_OK elapsed_us=");
            print_u64(elapsed);
            rivet::console::write_str("\n");
        }
        other => {
            rivet::console::write_str("TIMEOUT_FAIL: ");
            let _ = other;
            rivet::exit_failure(2);
        }
    }

    // try_lock on a free mutex must succeed; on the held one must fail.
    let free_ok = MUTEX_A.try_lock().is_some();
    let held_ok = MUTEX_T.try_lock().is_none();
    if free_ok && held_ok {
        rivet::console::write_str("TRYLOCK_OK\n");
    } else {
        rivet::console::write_str("TRYLOCK_FAIL\n");
        rivet::exit_failure(3);
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
            rivet::console::write_str("MUTEX_OK\n");
            rivet::exit_success();
        }
        Sleep::<10_000>::new().await;
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
    rivet::console::write_str("Rivet mutex_test (B1/B11/lock_timeout)\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 4, entry = timeout_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = stress_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = stress_task, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = holder, arg = UNIT);

    rivet::run();
}
