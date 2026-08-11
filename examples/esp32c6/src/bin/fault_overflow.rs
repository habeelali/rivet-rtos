//! Stack-overflow fault test (plan.md §3.6).
//!
//! A task overflows its stack (a local array larger than the stack). The
//! CM3 MPU pool-deny / RISC-V PMP guard trips, the fault policy (default:
//! Panic) dumps a diagnosis — "RIVET FAULT ... addr=... task=..." — and
//! the kernel exits with a distinguishable failure code. The harness
//! asserts the dump appeared and the exit code matches.
//!
//! Under the pre-fault-isolation kernel, this silently corrupted memory or
//! double-faulted with no output.

#![no_std]
#![no_main]

use rivet_bsp_esp32c6 as _;
use rivet_rt as _;

fn overflow_task(_: &'static ()) -> ! {
    // A local array much larger than the stack: the stack pointer runs
    // below the stack base and hits the guard.
    let mut buf = [0u8; 2048];
    for (i, b) in buf.iter_mut().enumerate() {
        *b = (i & 0xFF) as u8;
    }
    // Never reached.
    core::hint::black_box(&buf);
    loop {
        core::hint::spin_loop();
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet fault_overflow: about to overflow a stack\n");

    let _ = rivet::spawn_ptask!(stack = 1024, priority = 2, entry = overflow_task, arg = ());

    rivet::run();
}
