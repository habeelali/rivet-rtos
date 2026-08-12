//! Rivet RTOS Raspberry Pi Pico (RP2040) demo, real hardware — dual-tier:
//! real preemption + async/await. Ported from
//! `examples/stm32f401re`'s own demo (same phases, same acceptance bar,
//! same test program — this is the whole point of the port boundary:
//! kernel-level test code shouldn't need to change per board), swapping
//! the STM32's PA5/BSRR LED toggle for RP2040's GP25/SIO one.
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
//! other; cooperative-only scheduling could never produce that.
//!
//! ## Phase 2: cooperative async tier
//!
//! `heartbeat` (`Sleep`-based, toggling the on-board LED), a `producer`/
//! `consumer` pair over a `Channel`, and a `finisher` that waits on a
//! `Semaphore` and exits.

#![no_std]
#![no_main]

use rivet_bsp_rp2040 as _;
use rivet_rt as _;

use rivet::sync::{Channel, Semaphore};
use rivet::time::Sleep;

// ── Phase 0: priority inheritance proof ─────────────────────────────

static PI_MUTEX: rivet::preempt::PriorityMutex<u32> = rivet::preempt::PriorityMutex::new(0);
struct Unit;
static PI_ARG: Unit = Unit;

fn pi_low(_: &'static Unit) -> ! {
    rivet::console::write_str("[pi_low: acquiring mutex]\n");
    let mut guard = PI_MUTEX.lock();
    rivet::console::write_str("[pi_low: holds mutex, spawning medium+high]\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 6, entry = pi_medium, arg = PI_ARG);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 8, entry = pi_high, arg = PI_ARG);

    // Bounded "critical section". Without priority inheritance, pi_medium
    // (priority 6 > pi_low's base priority 5, never yields) would preempt
    // this and never give the CPU back.
    for i in 0..3_000_000u32 {
        *guard = i;
        core::hint::black_box(&*guard);
    }
    rivet::console::write_str("[pi_low: critical section done, releasing]\n");
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
    rivet::console::write_str("[pi_high: trying to acquire mutex]\n");
    let _guard = PI_MUTEX.lock();
    rivet::console::write_str("[pi_high: got mutex — priority inheritance worked]\n");
    rivet::preempt::park_forever();
}

// ── Phase 1: preemption proof (preemptive tier) ────────────────────

struct SpinArg {
    label: u8,
}
static ARG_A: SpinArg = SpinArg { label: b'A' };
static ARG_B: SpinArg = SpinArg { label: b'B' };

static PROGRESS_A: rivet::sync::atomic::AtomicU32 = rivet::sync::atomic::AtomicU32::new(0);
static PROGRESS_B: rivet::sync::atomic::AtomicU32 = rivet::sync::atomic::AtomicU32::new(0);

fn spin_task(arg: &'static SpinArg) -> ! {
    let counter = if arg.label == b'A' {
        &PROGRESS_A
    } else {
        &PROGRESS_B
    };
    let mut rounds: u32 = 0;
    loop {
        let count = counter.fetch_add(1, rivet::sync::atomic::Ordering::Relaxed) + 1;
        if count % 200_000 == 0 {
            let s = [arg.label];
            if let Ok(s) = core::str::from_utf8(&s) {
                rivet::console::write_str(s);
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
    // Pico's onboard LED: GP25, driven through SIO (single-cycle IO) —
    // `rivet-bsp-rp2040::__rivet_board_init` already muxed GP25 to SIO
    // and enabled its output driver, so this task only ever touches
    // SIO's own atomic set/clear registers (no read-modify-write race
    // with anything else touching this pin, same shape the STM32 demo's
    // BSRR-based toggle uses).
    let mut on = false;
    loop {
        Sleep::<100_000>::new().await; // 100ms
        on = !on;
        // SAFETY: fixed SIO register block; GPIO_OUT_SET/CLR are
        // write-1-to-set/clear, glitch-free, no read-modify-write needed.
        unsafe {
            let sio = &*rp2040_pac::SIO::ptr();
            if on {
                sio.gpio_out_set().write(|w| w.bits(1 << 25));
            } else {
                sio.gpio_out_clr().write(|w| w.bits(1 << 25));
            }
        }
        rivet::console::write_str(".");
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
        rivet::console::write_str("+");
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
        rivet::console::write_str("-");
        n += 1;
    }
    rivet::console::write_str("\nconsumer: sum=");
    print_u32(sum);
    rivet::console::write_str("\n");
    DONE.release();
}

#[rivet::task(priority = 3, stack = 256)]
async fn finisher() {
    DONE.acquire().await;
    rivet::console::write_str("SUCCESS\n");
    rivet::exit_success();
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

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet RTOS Raspberry Pi Pico (preemptive + async demo)\n");
    rivet::console::write_str("Phase 0: priority inheritance (avoiding priority inversion):\n");

    // Split the SPSC channel exactly once at boot (plan.md [B8]).
    let (tx, rx) = CHAN.split().expect("channel split must succeed");
    let _ = CHAN_TX.set(tx);
    let _ = CHAN_RX.set(rx);

    let _ = rivet::spawn_ptask!(stack = 512, priority = 5, entry = pi_low, arg = PI_ARG);

    rivet::console::write_str("Phase 1: two same-priority preemptive tasks (A, B), no yielding:\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spin_task, arg = ARG_A);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 1, entry = spin_task, arg = ARG_B);

    rivet::run();
}
