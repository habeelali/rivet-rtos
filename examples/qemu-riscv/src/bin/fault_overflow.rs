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

use core::panic::PanicInfo;

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

// ── Startup ───────────────────────────────────────────────────────

extern "C" {
    static __stack_top: u8;
    static __bss_start: u8;
    static __bss_end: u8;
}

core::arch::global_asm!(
    ".section .text._start",
    ".global _start",
    "_start:",
    "  la    sp, __stack_top",
    "  la    t0, __bss_start",
    "  la    t1, __bss_end",
    "1:",
    "  bgeu  t0, t1, 2f",
    "  sw    zero, 0(t0)",
    "  addi  t0, t0, 4",
    "  j     1b",
    "2:",
    "  call  rust_main",
    "  ebreak",
);

#[no_mangle]
fn rust_main() -> ! {
    rivet::init();
    rivet::arch::debug_print("Rivet fault_overflow: about to overflow a stack\n");

    let _ = rivet::spawn_ptask!(stack = 512, priority = 2, entry = overflow_task, arg = ());

    rivet::run();
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rivet::arch::debug_print("PANIC: ");
    if let Some(loc) = info.location() {
        rivet::arch::debug_print(loc.file());
        rivet::arch::debug_print(":");
        let mut n = loc.line();
        let mut digits = [0u8; 10];
        let mut i = 0;
        while n > 0 {
            digits[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        let mut buf = [0u8; 10];
        for j in 0..i {
            buf[j] = digits[i - 1 - j];
        }
        if let Ok(s) = core::str::from_utf8(&buf[..i]) {
            rivet::arch::debug_print(s);
        }
    }
    rivet::arch::debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}
