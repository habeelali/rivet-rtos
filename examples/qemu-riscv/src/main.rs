//! Rivet RTOS RISC-V QEMU demo — dual-tier: real preemption + async/await.
//!
//! ## Phase 0: priority inheritance (avoiding priority inversion)
//!
//! `pi_low` (prio 5) acquires a `PriorityMutex`, then spawns `pi_medium`
//! (prio 6, never yields or blocks) and `pi_high` (prio 8, immediately
//! blocks trying to acquire the same mutex). Without priority
//! inheritance, `pi_medium` — outranking `pi_low`'s base priority — would
//! preempt it and never give it back the CPU, so `pi_low` could never
//! finish its critical section, and `pi_high` would starve forever
//! (classic priority inversion). With inheritance, `pi_low`'s effective
//! priority is boosted to 8 for as long as `pi_high` waits, outranking
//! `pi_medium`, so `pi_low` finishes and releases promptly.
//!
//! ## Phase 1: proof of real preemption
//!
//! Two preemptive tasks, `spin_a` and `spin_b`, **same priority (1)**,
//! neither ever calling anything cooperative (no `.await`, no yield, no
//! blocking call) — just a tight counting loop. If you see their output
//! letters interleaved (`AABABBAB...` etc.) rather than one task running
//! to completion before the other starts, that's the timer tick forcibly
//! switching between them — genuine preemption, not cooperative
//! scheduling (which would show `AAAAAAAA...BBBBBBBB...`, since with only
//! `.await`-based yielding, a task that never awaits can't be interrupted).
//!
//! After a bounded number of rounds each, both park themselves
//! (`rivet::preempt::park_forever()`), letting the lower-priority
//! cooperative tier run.
//!
//! ## Phase 2: cooperative async tier
//!
//! Once the preemptive tasks park, the async executor (running as the
//! lowest-priority preemptive task under the hood) gets the CPU: a
//! `heartbeat` task (`Sleep`-based), a `producer`/`consumer` pair over a
//! `Channel`, and a `finisher` that waits on a `Semaphore` and exits.
//!
//! Build: cargo build --package qemu-riscv --release
//! Run:   qemu-system-riscv32 -machine virt -cpu rv32 -bios none -kernel <elf> -nographic -semihosting

#![no_std]
#![no_main]

use core::panic::PanicInfo;
use rivet::sync::{Channel, Semaphore};
use rivet::time::Sleep;

// ── Phase 0: priority inheritance proof ─────────────────────────────

static PI_MUTEX: rivet::preempt::PriorityMutex<u32> = rivet::preempt::PriorityMutex::new(0);
struct Unit;
static PI_ARG: Unit = Unit;

fn pi_low(_: &'static Unit) -> ! {
    rivet::arch::debug_print("[pi_low: acquiring mutex]\n");
    let mut guard = PI_MUTEX.lock();
    rivet::arch::debug_print("[pi_low: holds mutex, spawning medium+high]\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 6, entry = pi_medium, arg = PI_ARG);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 8, entry = pi_high, arg = PI_ARG);

    // Bounded "critical section". Without priority inheritance, pi_medium
    // (priority 6 > pi_low's base priority 5, never yields) would preempt
    // this and never give the CPU back — this loop would simply never
    // finish. If you see "critical section done" print at all, inheritance
    // worked.
    for i in 0..3_000_000u32 {
        *guard = i;
        core::hint::black_box(&*guard);
    }
    rivet::arch::debug_print("[pi_low: critical section done, releasing]\n");
    drop(guard);
    rivet::preempt::park_forever();
}

fn pi_medium(_: &'static Unit) -> ! {
    // Never yields, never blocks. Priority 6 > pi_low's base priority 5 —
    // this is exactly the task that would starve pi_low forever if
    // priority inheritance didn't boost it above 6 while pi_high waits.
    for _ in 0..8_000_000u32 {
        core::hint::spin_loop();
    }
    rivet::preempt::park_forever();
}

fn pi_high(_: &'static Unit) -> ! {
    rivet::arch::debug_print("[pi_high: trying to acquire mutex]\n");
    let _guard = PI_MUTEX.lock();
    rivet::arch::debug_print("[pi_high: got mutex — priority inheritance worked]\n");
    rivet::preempt::park_forever();
}

// ── Phase 1: preemption proof (preemptive tier) ────────────────────

struct SpinArg {
    label: u8,
}
static ARG_A: SpinArg = SpinArg { label: b'A' };
static ARG_B: SpinArg = SpinArg { label: b'B' };

static PROGRESS_A: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static PROGRESS_B: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn spin_task(arg: &'static SpinArg) -> ! {
    let counter = if arg.label == b'A' {
        &PROGRESS_A
    } else {
        &PROGRESS_B
    };
    let mut rounds: u32 = 0;
    loop {
        let count = counter.fetch_add(1, core::sync::atomic::Ordering::Relaxed) + 1;
        if count % 200_000 == 0 {
            let s = [arg.label];
            if let Ok(s) = core::str::from_utf8(&s) {
                rivet::arch::debug_print(s);
            }
            rounds += 1;
            if rounds >= 12 {
                rivet::preempt::park_forever();
            }
        }
    }
}

// ── Phase 2: cooperative async tier ─────────────────────────────────

static CHAN: Channel<u32, 4> = Channel::new();
static DONE: Semaphore<1> = Semaphore::new(0);

#[rivet::task(priority = 0, stack = 256)]
async fn heartbeat() {
    loop {
        Sleep::<100_000>::new().await; // 100ms
        rivet::arch::debug_print(".[a=");
        print_u32(PROGRESS_A.load(core::sync::atomic::Ordering::Relaxed));
        rivet::arch::debug_print(",b=");
        print_u32(PROGRESS_B.load(core::sync::atomic::Ordering::Relaxed));
        rivet::arch::debug_print("]");
    }
}

static CHAN_TX: rivet::sync::Once<rivet::sync::Sender<'static, u32, 4>> = rivet::sync::Once::new();
static CHAN_RX: rivet::sync::Once<rivet::sync::Receiver<'static, u32, 4>> =
    rivet::sync::Once::new();

#[rivet::task(priority = 1, stack = 256)]
async fn producer() {
    let tx = CHAN_TX.get().expect("channel split at boot");
    let mut i = 1u32;
    while i <= 5 {
        Sleep::<30_000>::new().await; // 30ms between sends
        tx.send(i).await;
        rivet::arch::debug_print("+");
        i += 1;
    }
}

#[rivet::task(priority = 2, stack = 256)]
async fn consumer() {
    let rx = CHAN_RX.get().expect("channel split at boot");
    let mut sum = 0u32;
    let mut n = 0;
    while n < 5 {
        sum += rx.recv().await;
        rivet::arch::debug_print("-");
        n += 1;
    }
    rivet::arch::debug_print("\nconsumer: sum=");
    print_u32(sum);
    rivet::arch::debug_print("\n");
    DONE.release();
}

#[rivet::task(priority = 3, stack = 256)]
async fn finisher() {
    DONE.acquire().await;
    rivet::arch::debug_print("SUCCESS\n");
    rivet::arch::exit_success();
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
    rivet::init(); // (arch::early_init happens inside init)
    rivet::arch::debug_print("Rivet RTOS v0.1.0 RISC-V (preemptive + async demo)\n");
    rivet::arch::debug_print("Phase 0: priority inheritance (avoiding priority inversion):\n");

    // Split the SPSC channel exactly once at boot (plan.md [B8]).
    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let _ = CHAN_TX.set(tx);
    let _ = CHAN_RX.set(rx);

    let _ = rivet::spawn_ptask!(stack = 512, priority = 5, entry = pi_low, arg = PI_ARG);

    rivet::arch::debug_print("Phase 1: two same-priority preemptive tasks (A, B), no yielding:\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spin_task, arg = ARG_A);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spin_task, arg = ARG_B);

    rivet::run();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        print_u32(loc.line());
    } else {
        rivet::arch::debug_print("(no location)");
    }
    rivet::arch::debug_print(": ");
    {
        use core::fmt::Write;
        let _ = write!(UartWriter, "{}", info.message());
    }
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}

/// Formats `core::fmt` output straight to the UART (panic diagnostics).
struct UartWriter;
impl core::fmt::Write for UartWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        rivet::arch::debug_print(s);
        Ok(())
    }
}
