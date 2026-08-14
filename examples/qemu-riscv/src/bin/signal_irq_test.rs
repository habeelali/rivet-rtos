//! End-to-end proof that `rivet::sync::Signal` completes from a real
//! hardware interrupt through the cooperative executor — not a manually
//! polled future in a preemptive task, and not `rivet::irq::dispatch`
//! alone (already proven by `irq_test.rs`). This is the first thing in
//! the tree that combines both: a real ISR (registered through
//! `rivet::irq`, the same UART0 THRE interrupt `irq_test.rs` uses) calls
//! `Signal::signal()`, and a genuine `#[rivet::task]` async fn `.await`s
//! it through `rivet::run()`'s real executor loop.

#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

static SIG: rivet::sync::Signal = rivet::sync::Signal::new();

const UART0_BASE: usize = 0x1000_0000;
const UART_IER: *mut u8 = (UART0_BASE + 1) as *mut u8;
const UART_IIR: *const u8 = (UART0_BASE + 2) as *const u8;
const UART_DATA: *mut u8 = UART0_BASE as *mut u8;

fn uart_tx_handler() {
    // SAFETY: fixed NS16550 registers; disabling IER + acking IIR here
    // (rather than in the awaiting task) avoids a level-triggered
    // interrupt storm — THRE stays true almost continuously once nothing
    // else is being transmitted.
    unsafe {
        core::ptr::write_volatile(UART_IER, 0);
        let _ = core::ptr::read_volatile(UART_IIR);
    }
    SIG.signal();
}

#[rivet::task(priority = 1, stack = 1024)]
async fn signal_task() {
    rivet::irq::register(rivet_bsp_qemu_virt::irq::UART0, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_qemu_virt::irq::UART0);
    SIG.reset();

    // SAFETY: kicking a real UART transmit to generate the genuine THRE
    // condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_IER, 0b0000_0010);
        core::ptr::write_volatile(UART_DATA, b'X');
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
