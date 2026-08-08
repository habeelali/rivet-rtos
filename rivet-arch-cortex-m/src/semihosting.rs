//! ARM semihosting debug I/O. Architectural (the `bkpt 0xAB` sequence is
//! recognized by any semihosting-aware debug host, not tied to any
//! particular board's memory map) — a utility a BSP *may* use for its
//! console/exit implementation instead of a real UART.

core::arch::global_asm!(
    ".section .text",
    ".global rivet_semihosting",
    ".thumb_func",
    "rivet_semihosting:",
    "  bkpt 0xAB",
    "  bx   lr",
    ".global rivet_exit_success",
    ".thumb_func",
    "rivet_exit_success:",
    "  movs r0, #0x18",
    "  ldr  r1, =0x20026",
    "  bkpt 0xAB",
    "1:",
    "  b    1b",
);

/// Exit via semihosting `SYS_EXIT` (`ADP_Stopped_ApplicationExit`). Never
/// returns.
pub fn exit_success() -> ! {
    extern "C" {
        fn rivet_exit_success() -> !;
    }
    // SAFETY: `rivet_exit_success` is the semihosting exit sequence
    // defined in the global_asm! block above; it never returns.
    unsafe { rivet_exit_success() }
}
