//! Respawn + pause/resume test (plan.md §5.4/§5.5 acceptance).
//!
//! Phase 1: spawn a worker that returns 42; join it; despawn it (slot +
//! stack released to the pool); respawn a new worker into the recycled
//! slot — the OLD handle must be stale (`is_valid()` false, join →
//! Stale), the NEW handle joins to Ok(7).
//! Phase 2: spawn a chattering task, pause it (progress stops), resume it
//! (progress resumes).

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicUsize, Ordering};
use rivet::preempt::{JoinError, TaskHandle};

fn worker42(_: &'static ()) -> u32 {
    42
}

fn worker7(_: &'static ()) -> u32 {
    7
}

fn chatter(_: &'static ()) {
    loop {
        CHATTER_TICKS.fetch_add(1, Ordering::Relaxed);
        rivet::arch::yield_now();
    }
}

static CHATTER_TICKS: AtomicUsize = AtomicUsize::new(0);
static W1: AtomicUsize = AtomicUsize::new(0); // packed worker-1 handle
static CHATTER_H: AtomicUsize = AtomicUsize::new(0);

fn supervisor(_: &'static ()) -> ! {
    // Phase 1: join worker1 → Ok(42).
    let packed = W1.load(Ordering::Acquire);
    let h1 = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };
    match h1.join::<u32>() {
        Ok(42) => rivet::arch::debug_print("JOIN1_OK\n"),
        _ => rivet::arch::exit_failure(11),
    }

    // Despawn worker1: slot + stack released.
    if !h1.despawn() {
        rivet::arch::debug_print("DESPAWN_FAIL\n");
        rivet::arch::exit_failure(12);
    }
    rivet::arch::debug_print("DESPAWN_OK\n");

    // Respawn a new worker — must reuse the freed slot (id equal).
    let h2 = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker7, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::arch::exit_failure(13),
    };
    // The old handle must now be stale (generation bumped on reuse).
    if h1.is_valid() {
        rivet::arch::debug_print("STALE_CHECK_FAIL\n");
        rivet::arch::exit_failure(14);
    }
    rivet::arch::debug_print("STALE_CHECK_OK\n");
    match h1.join::<u32>() {
        Err(JoinError::Stale) => rivet::arch::debug_print("STALE_JOIN_OK\n"),
        _ => rivet::arch::exit_failure(15),
    }
    match h2.join::<u32>() {
        Ok(7) => rivet::arch::debug_print("JOIN2_OK\n"),
        _ => rivet::arch::exit_failure(16),
    }
    if !h2.despawn() {
        rivet::arch::exit_failure(17);
    }

    // Phase 2: pause/resume.
    let packed = CHATTER_H.load(Ordering::Acquire);
    let hc = TaskHandle {
        id: (packed & 0xFFFF) as u16,
        generation: (packed >> 16) as u32,
    };
    // Let the chatter accumulate some ticks.
    rivet::preempt::sleep_ms(10);
    let before = CHATTER_TICKS.load(Ordering::Relaxed);
    if before == 0 {
        rivet::arch::debug_print("CHATTER_NEVER_RAN\n");
        rivet::arch::exit_failure(18);
    }
    if !hc.pause() {
        rivet::arch::debug_print("PAUSE_FAIL\n");
        rivet::arch::exit_failure(19);
    }
    rivet::arch::debug_print("PAUSE_OK\n");
    // While paused, the chatter must not advance (we sleep; it's skipped).
    rivet::preempt::sleep_ms(10);
    let after = CHATTER_TICKS.load(Ordering::Relaxed);
    if after != before {
        rivet::arch::debug_print("PAUSED_STILL_RAN\n");
        rivet::arch::exit_failure(20);
    }
    rivet::arch::debug_print("PAUSED_STILL\n");
    if !hc.resume() {
        rivet::arch::debug_print("RESUME_FAIL\n");
        rivet::arch::exit_failure(21);
    }
    rivet::arch::debug_print("RESUME_OK\n");
    rivet::preempt::sleep_ms(10);
    if CHATTER_TICKS.load(Ordering::Relaxed) <= after {
        rivet::arch::debug_print("RESUMED_NOT_RUNNING\n");
        rivet::arch::exit_failure(22);
    }
    rivet::arch::debug_print("RESPAWN_TEST_OK\n");
    rivet::arch::exit_success();
}

// ── Startup (RISC-V) ──────────────────────────────────────────────

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
    rivet::arch::debug_print("Rivet respawn_test\n");

    let h1 = match rivet::spawn_ptask!(stack = 512, priority = 2, entry = worker42, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::arch::exit_failure(23),
    };
    W1.store(
        h1.id as usize | ((h1.generation as usize) << 16),
        Ordering::Release,
    );

    let hc = match rivet::spawn_ptask!(stack = 512, priority = 1, entry = chatter, arg = ()) {
        Ok(h) => h,
        Err(_) => rivet::arch::exit_failure(24),
    };
    CHATTER_H.store(
        hc.id as usize | ((hc.generation as usize) << 16),
        Ordering::Release,
    );

    let _ = rivet::spawn_ptask!(stack = 512, priority = 3, entry = supervisor, arg = ());

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
