//! Rivet RTOS Cortex-M3 QEMU demo — dual-tier: real preemption + async/await.
//!
//! ## Phase 0: priority inheritance (avoiding priority inversion)
//!
//! `pi_low` (prio 5) acquires a `PriorityMutex`, then spawns `pi_medium`
//! (prio 6, never yields or blocks) and `pi_high` (prio 8, immediately
//! blocks trying to acquire the same mutex). Without priority
//! inheritance, `pi_medium` would preempt `pi_low` and never give the CPU
//! back, so `pi_low` could never finish its critical section and
//! `pi_high` would starve forever (classic priority inversion). With
//! inheritance, `pi_low`'s effective priority is boosted to 8 for as long
//! as `pi_high` waits, outranking `pi_medium`.
//!
//! ## Phase 1: proof of real preemption
//!
//! Two preemptive tasks, `spin_task(A)` and `spin_task(B)`, **same
//! priority (1)**, neither ever calling anything cooperative — just a
//! tight counting loop. Interleaved output letters (`AABABBAB...`) prove
//! the SysTick-driven PendSV switch is forcibly preempting one to run the
//! other; cooperative-only scheduling could never produce that (a task
//! that never yields can't be interrupted).
//!
//! After a bounded number of rounds each, both park themselves, letting
//! the lower-priority cooperative tier run.
//!
//! ## Phase 2: cooperative async tier
//!
//! `heartbeat` (`Sleep`-based), a `producer`/`consumer` pair over a
//! `Channel`, and a `finisher` that waits on a `Semaphore` and exits.
//!
//! Build: cargo build --package qemu-cm3 --release
//! Run:   qemu-system-arm -machine lm3s6965evb -kernel <elf> -nographic -semihosting

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
    // this and never give the CPU back.
    for i in 0..3_000_000u32 {
        *guard = i;
        core::hint::black_box(&*guard);
    }
    rivet::arch::debug_print("[pi_low: critical section done, releasing]\n");
    drop(guard);
    rivet::preempt::park_forever();
}

fn pi_medium(_: &'static Unit) -> ! {
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
    use rivet::hal::gpio::{Input, Pin, PORT_F};

    // Real typestate GPIO: PF1 starts as Input (peripheral reset state,
    // no hardware write yet), then into_output() is a compile-time state
    // transition that also does the real GPIODIR/GPIODEN writes. Calling
    // `.toggle()` on a still-Input pin would not compile.
    let led: Pin<PORT_F, 1, Input> = unsafe { Pin::new() };
    let mut led = led.into_output();

    loop {
        Sleep::<100_000>::new().await; // 100ms
        led.toggle();
        rivet::arch::debug_print(".");
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
    static __data_load: u8;
    static __data_start: u8;
    static __data_end: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

/// # Safety
/// Runs at power-on reset as the vector-table Reset entry; the linker
/// script has zero-initialized BSS and copied `.data` before any C/C++
/// runtime would, and this function performs that copy itself (it is the
/// only entry point at reset).
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
    rivet::arch::debug_print("Rivet RTOS v0.1.0 Cortex-M3 (preemptive + async demo)\n");
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

// ── Exception handlers ────────────────────────────────────────────
//
// PendSV is NOT defined here — it's hand-written assembly in
// `rivet::arch::cortex_m` (global_asm!, symbol `PendSV`), linked directly
// into the vector table. A normal Rust function can't give the precise
// register control a stack-switching handler needs.

/// # Safety
/// Exception entry point installed in the vector table; never called
/// directly. Handler mode runs on MSP and only requests a PendSV
/// reschedule, so it is safe at any interrupt priority.
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
    // Stacked PC at MSP+24 (r0,r1,r2,r3,r12,lr,pc,xPSR); HFSR at 0xE000ED2C.
    let hfsr = core::ptr::read_volatile(0xE000ED2C as *const u32);
    rivet::arch::debug_print(" hfsr=0x");
    print_hex32(hfsr);
    let mut sp: u32;
    unsafe { core::arch::asm!("mov {0}, sp", out(reg) sp) };
    let pc = core::ptr::read_volatile((sp + 24) as *const u32);
    rivet::arch::debug_print(" pc=0x");
    print_hex32(pc);
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

// ── Panic handler ─────────────────────────────────────────────────

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC\n");
    loop {}
}
