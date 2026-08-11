//! Real-time characterization: nested interrupt latency.
//!
//! No existing test in this workspace drives two different-priority
//! interrupt sources against each other to measure genuine preemption
//! between them — `irq_test.rs` proves one IRQ's dispatch chain end to
//! end, nothing more. This binary does: two NVIC-software-triggered IRQ
//! lines (`NVIC::pend`, the same standard, documented ARMv7-M
//! self-test technique `rivet-arch-cortex-m::nvic::pend` already
//! implements — no real hardware peripheral needed, no side effects on
//! the two borrowed IRQ numbers' nominal peripherals since their
//! registers are never touched) at two different NVIC priorities:
//! `IRQ_LOW` (mid priority) triggers `IRQ_HIGH` (numerically lower —
//! higher urgency) from *inside* its own handler, and the time from
//! that trigger to `IRQ_HIGH`'s handler actually running is genuine
//! nested-interrupt latency: `IRQ_HIGH` preempting `IRQ_LOW`'s handler
//! mid-execution, not just two independent dispatches.
//!
//! Cortex-M-only: this is the one board in this workspace with a real
//! priority-based nesting interrupt controller (NVIC) actually exercised
//! by this port — Xtensa's dispatch in this port collapses every async
//! source onto a single CPU interrupt level (no nesting possible by
//! construction), and this RISC-V port's boards don't assign distinct
//! priorities to different IRQ sources either.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

// Borrowed IRQ numbers (RCC=5, EXTI0=6 in the real vector table) —
// software-pended only, their real peripherals are never touched.
const IRQ_LOW: u32 = 5;
const IRQ_HIGH: u32 = 6;
const PRIO_LOW: u8 = 0x80;
const PRIO_HIGH: u8 = 0x00;

const N: u32 = 500;

static TRIGGERED_AT: AtomicU32 = AtomicU32::new(0);
static NESTED_ENTRY_AT: AtomicU32 = AtomicU32::new(0);
static LOW_RESUMED_AT: AtomicU32 = AtomicU32::new(0);
static NEST_COUNT: AtomicU32 = AtomicU32::new(0);
static ROUND_DONE: AtomicU32 = AtomicU32::new(0);

struct Stats {
    min: u64,
    max: u64,
    sum: u64,
    n: u64,
}
impl Stats {
    const fn new() -> Self {
        Self { min: u64::MAX, max: 0, sum: 0, n: 0 }
    }
    fn record(&mut self, v: u64) {
        if v < self.min {
            self.min = v;
        }
        if v > self.max {
            self.max = v;
        }
        self.sum += v;
        self.n += 1;
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

fn irq_high_handler() {
    // The nested preemption itself: this runs *while* irq_low_handler is
    // still executing (mid-way through its own spin below), at NVIC's
    // hardware-arbitrated higher priority.
    NESTED_ENTRY_AT.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    NEST_COUNT.fetch_add(1, Ordering::Relaxed);
}

fn irq_low_handler() {
    let t0 = rivet::port::arch::cycle_count() as u32;
    TRIGGERED_AT.store(t0, Ordering::Release);
    // Trigger the higher-priority IRQ from inside this handler — NVIC's
    // hardware priority arbitration preempts this handler immediately
    // (before the next instruction, per ARMv7-M's tail-chaining/
    // late-arrival rules) to run `irq_high_handler` first.
    rivet_arch_cortex_m::nvic::pend(IRQ_HIGH);
    // A short spin so there's real "mid-handler" time for the nested
    // preemption to land in, and so resuming here (after IRQ_HIGH
    // returns) is itself proof the nesting unwound correctly, not just
    // that IRQ_HIGH ran at some point.
    for _ in 0..50u32 {
        core::hint::spin_loop();
    }
    LOW_RESUMED_AT.store(rivet::port::arch::cycle_count() as u32, Ordering::Release);
    ROUND_DONE.store(1, Ordering::Release);
}

fn bench_task(_: &'static ()) -> ! {
    rivet::irq::register(IRQ_LOW, irq_low_handler).unwrap();
    rivet::irq::register(IRQ_HIGH, irq_high_handler).unwrap();
    rivet::irq::set_priority(IRQ_LOW, PRIO_LOW);
    rivet::irq::set_priority(IRQ_HIGH, PRIO_HIGH);
    rivet::irq::enable(IRQ_LOW);
    rivet::irq::enable(IRQ_HIGH);

    let mut nest_latency = Stats::new();
    let mut round_trip = Stats::new();

    for _ in 0..N {
        ROUND_DONE.store(0, Ordering::Release);
        rivet_arch_cortex_m::nvic::pend(IRQ_LOW);
        // Poll for completion — both handlers run to completion inside
        // the pend() call's own interrupt-return path, well before this
        // loop would spin many times at any realistic tick rate.
        let mut spins = 0u32;
        while ROUND_DONE.load(Ordering::Acquire) == 0 {
            core::hint::spin_loop();
            spins += 1;
            if spins > 10_000_000 {
                rivet::console::write_str("NESTED_IRQ_BENCH_TIMEOUT\n");
                rivet::exit_failure(1);
            }
        }

        let triggered = TRIGGERED_AT.load(Ordering::Acquire);
        let nested_entry = NESTED_ENTRY_AT.load(Ordering::Acquire);
        let resumed = LOW_RESUMED_AT.load(Ordering::Acquire);

        nest_latency.record(nested_entry.wrapping_sub(triggered) as u64);
        round_trip.record(resumed.wrapping_sub(triggered) as u64);
    }

    rivet::console::write_str("=== nested_irq_bench (cycles) ===\n");
    rivet::console::write_str("nest_count=");
    print_u64(NEST_COUNT.load(Ordering::Relaxed) as u64);
    rivet::console::write_str(" (expected=");
    print_u64(N as u64);
    rivet::console::write_str(")\n");
    rivet::console::write_str("trigger_to_nested_entry: min=");
    print_u64(nest_latency.min);
    rivet::console::write_str(" max=");
    print_u64(nest_latency.max);
    rivet::console::write_str(" avg=");
    print_u64(nest_latency.sum / nest_latency.n);
    rivet::console::write_str("\nlow_handler_round_trip(incl. nested IRQ): min=");
    print_u64(round_trip.min);
    rivet::console::write_str(" max=");
    print_u64(round_trip.max);
    rivet::console::write_str(" avg=");
    print_u64(round_trip.sum / round_trip.n);
    rivet::console::write_str("\n=== end nested_irq_bench ===\n");

    if NEST_COUNT.load(Ordering::Relaxed) != N {
        rivet::console::write_str("NESTED_IRQ_BENCH_COUNT_MISMATCH\n");
        rivet::exit_failure(2);
    }
    rivet::console::write_str("NESTED_IRQ_BENCH_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet nested_irq_bench\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = bench_task, arg = ());
    rivet::run();
}
