//! End-to-end IRQ dispatch test (plan.md Phase 13/27), STM32 port.
//!
//! Registers a *test-owned* handler for USART2's IRQ (the same
//! peripheral/line the console's own TX-IRQ path uses — see the board
//! crate's own `uart_irq_handler`; `rivet::irq::register` overwrites
//! that registration for the duration of this test, which is fine: this
//! test's exit path ends in `rivet::exit_success()`, which flushes the
//! console via `flush_sync()` — a direct polling drain that doesn't
//! care whether the hijacked ISR ever ran again, so nothing printed
//! after the hijack is lost). Proves the whole chain is real: vector
//! table → `SCB.ICSR.VECTACTIVE` dispatch (`rivet_irq_handler`) →
//! `rivet::irq::dispatch` → the registered handler, driven by a genuine
//! USART2 TXE condition, not a software-only stand-in.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

static FIRED: AtomicBool = AtomicBool::new(false);

fn uart_tx_handler() {
    // SAFETY: fixed USART2 register block; disabling TXEIE here (rather
    // than in the test task) avoids a level-triggered interrupt storm —
    // TXE is true continuously once nothing else is queued to send.
    unsafe {
        (&*stm32f4::stm32f401::USART2::ptr())
            .cr1()
            .modify(|_, w| w.txeie().clear_bit());
    }
    FIRED.store(true, Ordering::Release);
}

fn test_task(_: &'static ()) -> ! {
    rivet::irq::register(rivet_bsp_stm32f401re::irq::USART2, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_stm32f401re::irq::USART2);

    // SAFETY: kicking a real UART transmit to generate the genuine
    // TXE condition the handler above is waiting for.
    unsafe {
        let usart2 = &*stm32f4::stm32f401::USART2::ptr();
        usart2.dr().write(|w| unsafe { w.dr().bits(b'X' as u16) });
        usart2.cr1().modify(|_, w| w.txeie().set_bit());
    }

    for _ in 0..50 {
        if FIRED.load(Ordering::Acquire) {
            rivet::console::write_str("IRQ_FIRED\n");
            rivet::console::write_str("IRQ_TEST_OK\n");
            rivet::exit_success();
        }
        rivet::preempt::sleep_ms(10);
    }
    rivet::console::write_str("IRQ_TEST_TIMEOUT\n");
    rivet::exit_failure(1);
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet irq_test\n");
    let _ = rivet::spawn_ptask!(stack = 1024, priority = 1, entry = test_task, arg = ());
    rivet::run();
}
