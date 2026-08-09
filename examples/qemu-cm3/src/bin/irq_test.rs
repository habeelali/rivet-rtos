//! End-to-end IRQ dispatch test (plan.md Phase 13).
//!
//! Registers a real handler for UART0's interrupt (NVIC IRQ 5 on
//! lm3s6965evb — verified empirically via a probe binary, not assumed;
//! see `rivet-bsp-lm3s6965::irq::UART0`), enables it at the NVIC, and
//! kicks a UART transmit. Proves the whole chain is real: vector table
//! → `SCB.ICSR.VECTACTIVE` dispatch (`rivet_irq_handler`) →
//! `rivet::irq::dispatch` → the registered handler.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use rivet_bsp_lm3s6965 as _;
use rivet_rt as _;

static FIRED: AtomicBool = AtomicBool::new(false);

const UART0_BASE: u32 = 0x4000_C000;
const UART_DR: *mut u32 = UART0_BASE as *mut u32;
const UART_IMSC: *mut u32 = (UART0_BASE + 0x38) as *mut u32;
const UART_ICR: *mut u32 = (UART0_BASE + 0x44) as *mut u32;
const TXIM: u32 = 1 << 5;

fn uart_tx_handler() {
    // SAFETY: fixed PL011 registers; masking + clearing here (rather than
    // in the test task) avoids a level-triggered interrupt storm.
    unsafe {
        core::ptr::write_volatile(UART_IMSC, 0);
        core::ptr::write_volatile(UART_ICR, 0x7FF);
    }
    FIRED.store(true, Ordering::Release);
}

fn test_task(_: &'static ()) -> ! {
    rivet::irq::register(rivet_bsp_lm3s6965::irq::UART0, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_lm3s6965::irq::UART0);

    // SAFETY: kicking a real UART transmit to generate the genuine
    // TX-empty condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_IMSC, TXIM);
        core::ptr::write_volatile(UART_DR, b'X' as u32);
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
