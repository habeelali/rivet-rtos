//! End-to-end IRQ dispatch test (plan.md Phase 25), the Xtensa/S3 port of
//! `qemu-riscv`'s `irq_test.rs` — same acceptance bar (vector table →
//! hardware dispatch → `rivet::irq::dispatch` → the registered handler,
//! not a software-only stand-in), different peripheral: UART0's `tx_done`
//! interrupt (fires once a real transmission actually completes) routed
//! through the interrupt matrix to CPU line 27 (see
//! `rivet_arch_xtensa`'s `periph_irq` module) instead of an NS16550's
//! THRE condition.

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use rivet_bsp_esp32s3 as _;
use rivet_rt as _;

static FIRED: AtomicBool = AtomicBool::new(false);

fn uart_tx_done_handler() {
    // SAFETY: fixed UART0 register block; acking here (rather than in the
    // test task) matters because `tx_done` is level-triggered — leaving
    // it set would re-fire the instant this handler returns.
    unsafe {
        let uart0 = &*esp32s3::UART0::ptr();
        uart0.int_clr().write(|w| w.tx_done().clear_bit_by_one());
        uart0.int_ena().write(|w| w.tx_done().clear_bit());
    }
    FIRED.store(true, Ordering::Release);
}

fn test_task(_: &'static ()) -> ! {
    rivet::irq::register(rivet_bsp_esp32s3::irq::UART0, uart_tx_done_handler).unwrap();
    rivet::irq::enable(rivet_bsp_esp32s3::irq::UART0);

    // SAFETY: kicking a real UART transmit to generate the genuine
    // tx_done condition the handler above is waiting for.
    unsafe {
        let uart0 = &*esp32s3::UART0::ptr();
        uart0.int_ena().write(|w| w.tx_done().set_bit());
        uart0.fifo().write(|w| w.rxfifo_rd_byte().bits(b'X'));
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
