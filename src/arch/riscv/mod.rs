//! RISC-V port: context switch, interrupt control.

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
/// Callee-saved: ra, s0-s11 (13 words). Stack 16-byte aligned (64 bytes).
#[inline(never)]
pub unsafe fn context_switch(prev_sp: *mut usize, next_sp: usize) {
    #[cfg(target_arch = "riscv32")]
    {
        core::arch::asm!(
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
            in("a0") prev_sp,
            in("a1") next_sp,
            options(nostack, preserves_flags)
        );
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
