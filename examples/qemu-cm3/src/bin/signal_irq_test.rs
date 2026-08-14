//! End-to-end proof that `rivet::sync::Signal` completes from a real
//! hardware interrupt through the cooperative executor — not a manually
//! polled future in a preemptive task, and not `rivet::irq::dispatch`
//! alone (already proven by `irq_test.rs`). This is the first thing in
//! the tree that combines both: a real ISR (registered through
//! `rivet::irq`, the same UART0 TX-empty interrupt `irq_test.rs` uses)
//! calls `Signal::signal()`, and a genuine `#[rivet::task]` async fn
//! `.await`s it through `rivet::run()`'s real executor loop.

#![no_std]
#![no_main]

use rivet_bsp_lm3s6965 as _;
use rivet_rt as _;

static SIG: rivet::sync::Signal = rivet::sync::Signal::new();

const UART0_BASE: u32 = 0x4000_C000;
const UART_DR: *mut u32 = UART0_BASE as *mut u32;
const UART_IMSC: *mut u32 = (UART0_BASE + 0x38) as *mut u32;
const UART_ICR: *mut u32 = (UART0_BASE + 0x44) as *mut u32;
const TXIM: u32 = 1 << 5;

fn uart_tx_handler() {
    // SAFETY: fixed PL011 registers; masking + clearing here (rather than
    // in the awaiting task) avoids a level-triggered interrupt storm.
    unsafe {
        core::ptr::write_volatile(UART_IMSC, 0);
        core::ptr::write_volatile(UART_ICR, 0x7FF);
    }
    SIG.signal();
}

#[rivet::task(priority = 1, stack = 1024)]
async fn signal_task() {
    rivet::irq::register(rivet_bsp_lm3s6965::irq::UART0, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_lm3s6965::irq::UART0);
    SIG.reset();

    // SAFETY: kicking a real UART transmit to generate the genuine
    // TX-empty condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_IMSC, TXIM);
        core::ptr::write_volatile(UART_DR, b'X' as u32);
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
