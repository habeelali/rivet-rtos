//! End-to-end proof that `rivet::sync::Signal` completes from a real
//! hardware interrupt through the cooperative executor — not a manually
//! polled future in a preemptive task, and not `rivet::irq::dispatch`
//! alone (already proven by `irq_test.rs`). This is the first thing in
//! the tree that combines both: a real ISR (registered through
//! `rivet::irq`, the same UART0 TX interrupt `irq_test.rs` uses) calls
//! `Signal::signal()`, and a genuine `#[rivet::task]` async fn `.await`s
//! it through `rivet::run()`'s real executor loop.

#![no_std]
#![no_main]

use rivet_bsp_mps2_an385 as _;
use rivet_rt as _;

static SIG: rivet::sync::Signal = rivet::sync::Signal::new();

const UART0_BASE: u32 = 0x4000_4000;
const UART_DATA: *mut u32 = UART0_BASE as *mut u32;
const UART_CTRL: *mut u32 = (UART0_BASE + 0x08) as *mut u32;
const UART_INTSTATUS: *mut u32 = (UART0_BASE + 0x0C) as *mut u32;
const UART_CTRL_TX_EN_ONLY: u32 = 0b0000_0001;

fn uart_tx_handler() {
    // SAFETY: fixed CMSDK UART registers; disabling interrupt-enable +
    // clearing INTSTATUS here (rather than in the awaiting task) avoids a
    // level-triggered interrupt storm.
    unsafe {
        core::ptr::write_volatile(UART_CTRL, UART_CTRL_TX_EN_ONLY);
        core::ptr::write_volatile(UART_INTSTATUS, 0b1111);
    }
    SIG.signal();
}

#[rivet::task(priority = 1, stack = 1024)]
async fn signal_task() {
    rivet::irq::register(rivet_bsp_mps2_an385::irq::UART0_TX, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_mps2_an385::irq::UART0_TX);
    SIG.reset();

    // SAFETY: kicking a real UART transmit to generate the genuine
    // TX-ready condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_CTRL, 0b0000_0101); // TX_EN | TX_INT_EN
        core::ptr::write_volatile(UART_DATA, b'X' as u32);
    }

    SIG.wait().await;

    rivet::console::write_str("SIGNAL_FIRED\n");
    rivet::console::write_str("SIGNAL_IRQ_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet signal_irq_test\n");
    rivet::run();
}
