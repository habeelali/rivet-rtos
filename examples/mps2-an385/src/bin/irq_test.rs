//! End-to-end IRQ dispatch test (plan.md Phase 13).
//!
//! Registers a real handler for UART0's TX interrupt (NVIC IRQ 1 on
//! mps2-an385 — verified empirically via a probe binary, not assumed;
//! see `rivet-bsp-mps2-an385::irq::UART0_TX`), enables it at the NVIC,
//! and kicks a UART transmit. Proves the whole chain is real: vector
//! table → `SCB.ICSR.VECTACTIVE` dispatch (`rivet_irq_handler`) →
//! `rivet::irq::dispatch` → the registered handler.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use rivet_bsp_mps2_an385 as _;
use rivet_rt as _;

static FIRED: AtomicBool = AtomicBool::new(false);

const UART0_BASE: u32 = 0x4000_4000;
const UART_DATA: *mut u32 = UART0_BASE as *mut u32;
const UART_CTRL: *mut u32 = (UART0_BASE + 0x08) as *mut u32;
const UART_INTSTATUS: *mut u32 = (UART0_BASE + 0x0C) as *mut u32;
const UART_CTRL_TX_EN_ONLY: u32 = 0b0000_0001;

fn uart_tx_handler() {
    // SAFETY: fixed CMSDK UART registers; disabling interrupt-enable +
    // clearing INTSTATUS here (rather than in the test task) avoids a
    // level-triggered interrupt storm — TX-ready is true almost
    // continuously once nothing else is queued to send.
    unsafe {
        core::ptr::write_volatile(UART_CTRL, UART_CTRL_TX_EN_ONLY);
        core::ptr::write_volatile(UART_INTSTATUS, 0b1111);
    }
    FIRED.store(true, Ordering::Release);
}

fn test_task(_: &'static ()) -> ! {
    rivet::irq::register(rivet_bsp_mps2_an385::irq::UART0_TX, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_mps2_an385::irq::UART0_TX);

    // SAFETY: kicking a real UART transmit to generate the genuine
    // TX-ready condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_CTRL, 0b0000_0101); // TX_EN | TX_INT_EN
        core::ptr::write_volatile(UART_DATA, b'X' as u32);
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
