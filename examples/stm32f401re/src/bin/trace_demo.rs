//! Live Rivet Debugger capture demo: a realistic multi-task workload,
//! traced end to end over the same UART the ST-LINK exposes as a USB
//! serial port, decoded live by `rivet-debugger-app` (a separate,
//! sibling project — `../rivet-debugger`).
//!
//! Six tasks at five distinct priorities, tuned so *none* of them are
//! permanently starved by another (every priority level gets real,
//! observable time on the trace, not just the busiest one):
//! - `low_holder` / `high_waiter` (prio 6 / 9) — a real, live priority-
//!   inheritance scenario: `low_holder` takes `MTX`, `high_waiter` blocks
//!   on the same mutex shortly after — the trace carries a real
//!   `PriorityInherit` event the instant the boost happens.
//! - `worker_a` / `worker_b` (prio 4) — periodic bursty work, giving the
//!   trace realistic multi-task interleaving.
//! - `heartbeat` (prio 5) — toggles the Nucleo's onboard LED (PA5) every
//!   500ms, `Sleep`-driven; sits *above* the workers so it's never
//!   starved out despite its long sleeps.
//! - `tim3_tick`/`tim4_tick` aren't tasks — they're two independent real
//!   hardware interrupts (TIM3 NVIC IRQ 29 ~10 Hz, TIM4 NVIC IRQ 30
//!   ~23 Hz), registered through `rivet::irq` like any peripheral driver
//!   would be, so `IrqEnter`/`IrqExit` on the trace are genuine ISR
//!   entries, not synthesized.
//!
//! Build/flash with `--features trace`; without it this is a completely
//! ordinary Rivet binary (every trace call compiles to nothing).

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use rivet::preempt::PriorityMutex;

struct Unit;
static UNIT: Unit = Unit;

static MTX: PriorityMutex<u32> = PriorityMutex::new(0);

/// TIM3 global interrupt, STM32F401's fixed NVIC position (RM0368).
const TIM3_IRQ: u32 = 29;
/// TIM4 global interrupt — a second, independent hardware timer at a
/// different rate, so the trace carries more than one real IRQ source
/// (the debugger's Interrupts panel is otherwise stuck showing just one
/// row forever, which understates what `rivet::irq` actually supports).
const TIM4_IRQ: u32 = 30;

/// Assumed board clock (see `rivet-bsp-stm32f401re`'s own module docs:
/// the reset-state HSI default, not independently re-measured) — sent
/// once as the stream's `StreamHeader` event so the host UI shows a real
/// number instead of "unknown."
const CPU_HZ: u32 = 16_000_000;

fn tim3_handler() {
    // SAFETY: fixed TIM3 register block; this ISR is the sole owner of
    // it (nothing else in this binary touches TIM3).
    let tim3 = unsafe { &*stm32f4::stm32f401::TIM3::ptr() };
    // Clear UIF (write-0-to-clear, like every STM32 timer status
    // register) — without this the update interrupt condition is still
    // asserted the instant this handler returns, and it fires forever.
    tim3.sr().modify(|_, w| w.uif().clear_bit());
}

fn tim4_handler() {
    // SAFETY: fixed TIM4 register block; this ISR is the sole owner of it.
    let tim4 = unsafe { &*stm32f4::stm32f401::TIM4::ptr() };
    tim4.sr().modify(|_, w| w.uif().clear_bit());
}

fn heartbeat(_: &'static Unit) -> ! {
    // Nucleo-64 onboard LD2 LED: PA5, push-pull output — same raw
    // register pattern `main.rs`'s own LED toggle uses.
    // SAFETY: fixed GPIOA register block; MODER5=01 (output) touches only
    // PA5's two mode bits, leaving USART2's own AF-mode pins untouched.
    unsafe {
        (&*stm32f4::stm32f401::GPIOA::ptr())
            .moder()
            .modify(|_, w| w.moder5().bits(0b01));
    }
    loop {
        // SAFETY: BSRR is a write-only set/reset register; bit 5 sets PA5
        // high, bit 21 (5+16) resets it — no read-modify-write race
        // possible.
        unsafe {
            (&*stm32f4::stm32f401::GPIOA::ptr()).bsrr().write(|w| w.bs5().set_bit());
        }
        rivet::preempt::sleep_ms(500);
        unsafe {
            (&*stm32f4::stm32f401::GPIOA::ptr()).bsrr().write(|w| w.br5().set_bit());
        }
        rivet::preempt::sleep_ms(500);
    }
}

fn low_holder(_: &'static Unit) -> ! {
    loop {
        let g = MTX.lock();
        let t0 = rivet::port::arch::cycle_count();
        // A real, non-trivial critical section — long enough for the
        // priority boost to be visible in the trace, short enough to
        // keep the demo lively.
        while (rivet::port::arch::cycle_count().wrapping_sub(t0) as u32) < 200_000 {
            core::hint::spin_loop();
        }
        drop(g);
        rivet::preempt::sleep_ms(150);
    }
}

fn high_waiter(_: &'static Unit) -> ! {
    loop {
        let _g = MTX.lock();
        drop(_g);
        rivet::preempt::sleep_ms(80);
    }
}

fn worker(period_ms: &'static u32) -> ! {
    loop {
        let t0 = rivet::port::arch::cycle_count();
        while (rivet::port::arch::cycle_count().wrapping_sub(t0) as u32) < 60_000 {
            core::hint::spin_loop();
        }
        rivet::preempt::sleep_ms(*period_ms as u64);
    }
}

#[rivet::main]
fn main() -> ! {
    // No console::write_str here, deliberately: the interrupt-driven
    // console TX path and this binary's raw-polling trace_write share
    // the same USART2 peripheral, and mixing them lets the console ISR
    // fire constantly — not what a clean trace capture wants. Pure
    // binary trace stream from boot.

    #[cfg(feature = "trace")]
    rivet::trace::stream_header(CPU_HZ, rivet::config::MAX_HARTS as u8);

    // Real hardware timer interrupt, ~10 Hz: RCC enable, then PSC/ARR so
    // the 16 MHz APB1 timer clock counts down to a 100ms period
    // (16MHz / 1600 = 10kHz tick; 10kHz / 1000 = 10Hz update), update-
    // interrupt enabled, registered through the same `rivet::irq` path
    // any real peripheral driver uses.
    // SAFETY: fixed RCC/TIM3 register blocks, configured once here
    // before anything else touches them.
    unsafe {
        let rcc = &*stm32f4::stm32f401::RCC::ptr();
        rcc.apb1enr().modify(|_, w| w.tim3en().set_bit());
        let tim3 = &*stm32f4::stm32f401::TIM3::ptr();
        tim3.psc().write(|w| w.psc().bits(1599));
        tim3.arr().write(|w| w.arr().bits(999));
        tim3.dier().modify(|_, w| w.uie().set_bit());
        tim3.cr1().modify(|_, w| w.cen().set_bit());

        rivet::irq::register(TIM3_IRQ, tim3_handler).unwrap();
        rivet::irq::set_priority(TIM3_IRQ, 0xFF);
        rivet::irq::enable(TIM3_IRQ);

        // TIM4: same setup, a faster ~23 Hz period (16MHz / 100 / 7000 —
        // deliberately not a multiple of TIM3's rate, so the two IRQs
        // don't land in lockstep) — a second, genuinely independent
        // interrupt source on the trace.
        rcc.apb1enr().modify(|_, w| w.tim4en().set_bit());
        let tim4 = &*stm32f4::stm32f401::TIM4::ptr();
        tim4.psc().write(|w| w.psc().bits(99));
        tim4.arr().write(|w| w.arr().bits(6999));
        tim4.dier().modify(|_, w| w.uie().set_bit());
        tim4.cr1().modify(|_, w| w.cen().set_bit());

        rivet::irq::register(TIM4_IRQ, tim4_handler).unwrap();
        rivet::irq::set_priority(TIM4_IRQ, 0xFF);
        rivet::irq::enable(TIM4_IRQ);
    }

    static WORKER_A_PERIOD: u32 = 220;
    static WORKER_B_PERIOD: u32 = 340;

    // Priorities matter here: `low_holder` must outrank the always-ready
    // `worker_a`/`worker_b` (4) or it's starved before it ever takes the
    // mutex for the first time, and `high_waiter`'s lock() then always
    // succeeds trivially with nothing to wait on — no real inheritance
    // ever fires. `heartbeat` sits above the workers too, for the same
    // starvation reason, despite doing far less work overall.
    let _ = rivet::spawn_ptask!(stack = 512, priority = 6, entry = low_holder, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 9, entry = high_waiter, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 5, entry = heartbeat, arg = UNIT);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 4, entry = worker, arg = WORKER_A_PERIOD);
    let _ = rivet::spawn_ptask!(stack = 512, priority = 4, entry = worker, arg = WORKER_B_PERIOD);

    rivet::run();
}
