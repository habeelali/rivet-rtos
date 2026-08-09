//! End-to-end IRQ dispatch test (plan.md Phase 13).
//!
//! Registers a real handler for the UART0 TX-empty interrupt (source 10
//! on QEMU virt's PLIC — verified empirically via a probe binary, not
//! assumed; see `rivet-bsp-qemu-virt::irq::UART0`), enables it at both
//! the PLIC and `mie.MEIE`, and kicks a UART transmit. Proves the whole
//! chain is real: vector table → `mcause` external-interrupt dispatch →
//! `plic::claim_dispatch_complete` → `rivet::irq::dispatch` → the
//! registered handler — not a software-only stand-in.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use rivet_bsp_qemu_virt as _;
use rivet_rt as _;

static FIRED: AtomicBool = AtomicBool::new(false);

const UART0_BASE: usize = 0x1000_0000;
const UART_IER: *mut u8 = (UART0_BASE + 1) as *mut u8;
const UART_IIR: *const u8 = (UART0_BASE + 2) as *const u8;
const UART_DATA: *mut u8 = UART0_BASE as *mut u8;

fn uart_tx_handler() {
    // SAFETY: fixed NS16550 registers; disabling IER + acking IIR here
    // (rather than in the test task) avoids a level-triggered interrupt
    // storm — THRE stays true almost continuously once nothing else is
    // being transmitted.
    unsafe {
        core::ptr::write_volatile(UART_IER, 0);
        let _ = core::ptr::read_volatile(UART_IIR);
    }
    FIRED.store(true, Ordering::Release);
}

fn test_task(_: &'static ()) -> ! {
    rivet::irq::register(rivet_bsp_qemu_virt::irq::UART0, uart_tx_handler).unwrap();
    rivet::irq::enable(rivet_bsp_qemu_virt::irq::UART0);

    // SAFETY: kicking a real UART transmit to generate the genuine
    // THRE condition the handler above is waiting for.
    unsafe {
        core::ptr::write_volatile(UART_IER, 0b0000_0010);
        core::ptr::write_volatile(UART_DATA, b'X');
    }

    // Bounded wait for the real hardware interrupt to actually land —
    // not a fixed sleep, so the test fails fast if it never fires.
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
