//! RISC-V port: context switch, interrupt control.

#[cfg(target_arch = "riscv32")]
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_context_switch_asm",
    "rivet_context_switch_asm:",
    "   .option push",
    "   .option norvc",
    "   addi  sp, sp, -64",
    "   sw    ra, 0(sp)",
    "   sw    s0, 4(sp)",
    "   sw    s1, 8(sp)",
    "   sw    s2, 12(sp)",
    "   sw    s3, 16(sp)",
    "   sw    s4, 20(sp)",
    "   sw    s5, 24(sp)",
    "   sw    s6, 28(sp)",
    "   sw    s7, 32(sp)",
    "   sw    s8, 36(sp)",
    "   sw    s9, 40(sp)",
    "   sw    s10, 44(sp)",
    "   sw    s11, 48(sp)",
    "   sw    sp, 0(a0)",
    "   mv    sp, a1",
    "   lw    ra, 0(sp)",
    "   lw    s0, 4(sp)",
    "   lw    s1, 8(sp)",
    "   lw    s2, 12(sp)",
    "   lw    s3, 16(sp)",
    "   lw    s4, 20(sp)",
    "   lw    s5, 24(sp)",
    "   lw    s6, 28(sp)",
    "   lw    s7, 32(sp)",
    "   lw    s8, 36(sp)",
    "   lw    s9, 40(sp)",
    "   lw    s10, 44(sp)",
    "   lw    s11, 48(sp)",
    "   addi  sp, sp, 64",
    "   ret",
    "   .option pop",
);

/// Run a closure with interrupts disabled (critical section).
/// On RISC-V: clears MIE in mstatus, runs closure, restores mstatus.
/// On host: runs closure (no-op for single-threaded tests).
pub fn critical_section<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    #[cfg(target_arch = "riscv32")]
    {
        use riscv::register::mstatus;
        unsafe {
            let prev = mstatus::read();
            mstatus::clear_mie();
            let r = f();
            mstatus::write(prev);
            r
        }
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        f()
    }
}

/// Context switch: save current context to current stack, store sp in *prev_sp,
/// load next_sp into sp, restore context from that stack, return (to new task).
/// Implemented in global asm (no prologue) so saved sp matches exactly on restore.
#[inline(never)]
pub unsafe fn context_switch(prev_sp: *mut usize, next_sp: usize) {
    #[cfg(target_arch = "riscv32")]
    {
        extern "C" {
            fn rivet_context_switch_asm(prev_sp: *mut usize, next_sp: usize);
        }
        rivet_context_switch_asm(prev_sp, next_sp);
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        // Stub for host (tests): just store a dummy so caller doesn't read garbage.
        let _ = next_sp;
        if !prev_sp.is_null() {
            *prev_sp = 0;
        }
    }
}

/// Switch to the first task (no save). Call from bootstrap to enter the scheduler.
/// Loads next_sp into sp, restores ra and s0-s11 from that stack, then ret.
#[inline(never)]
pub unsafe fn switch_to_first(next_sp: usize) {
    #[cfg(target_arch = "riscv32")]
    {
        core::arch::asm!(
            "   mv    sp, a0",
            "   lw    ra, 0(sp)",
            "   lw    s0, 4(sp)",
            "   lw    s1, 8(sp)",
            "   lw    s2, 12(sp)",
            "   lw    s3, 16(sp)",
            "   lw    s4, 20(sp)",
            "   lw    s5, 24(sp)",
            "   lw    s6, 28(sp)",
            "   lw    s7, 32(sp)",
            "   lw    s8, 36(sp)",
            "   lw    s9, 40(sp)",
            "   lw    s10, 44(sp)",
            "   lw    s11, 48(sp)",
            "   addi  sp, sp, 64",
            "   ret",
            in("a0") next_sp,
            options(nostack, preserves_flags)
        );
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        let _ = next_sp;
    }
}

/// QEMU semihosting: print a null-terminated string to the host console.
/// Requires QEMU -semihosting. Use [`qemu_print_str`] for Rust &str.
#[cfg(target_arch = "riscv32")]
fn qemu_semihosting_write0(ptr: *const u8) {
    const SYS_WRITE0: usize = 0x04;
    unsafe {
        core::arch::asm!(
            "   .align 4",
            "   slli x0, x0, 0x1f",
            "   ebreak",
            "   srai x0, x0, 7",
            in("a0") SYS_WRITE0,
            in("a1") ptr,
            options(nostack, preserves_flags)
        );
    }
}

/// Print a string to QEMU console (semihosting SYS_WRITE0). Max 127 bytes.
#[cfg(target_arch = "riscv32")]
pub fn qemu_print_str(s: &str) {
    let bytes = s.as_bytes();
    let len = bytes.len().min(127);
    let mut buf = [0u8; 128];
    buf[..len].copy_from_slice(&bytes[..len]);
    buf[len] = b'\0';
    qemu_semihosting_write0(buf.as_ptr());
}

/// QEMU semihosting exit: exit with status 0 (success).
/// Call with -semihosting; QEMU will exit with code 0.
#[cfg(target_arch = "riscv32")]
pub fn qemu_exit_success() -> ! {
    const SYS_EXIT: usize = 0x18;
    const ADP_STOPPED_APPLICATIONEXIT: u32 = 0x20026;
    unsafe {
        let code = ADP_STOPPED_APPLICATIONEXIT;
        core::arch::asm!(
            "   .align 4",
            "   slli x0, x0, 0x1f",
            "   ebreak",
            "   srai x0, x0, 7",
            in("a0") SYS_EXIT,
            in("a1") &code as *const u32,
            options(nostack, noreturn)
        );
    }
}
