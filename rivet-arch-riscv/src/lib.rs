//! Rivet RTOS — RV32 ISA port.
//!
//! Implements the Group A (`rivet::port::arch`) symbol contract for RV32
//! targets: context switch, trap entry/dispatch, PMP-based stack guards.
//! Contains **no board/MMIO knowledge** beyond what's genuinely part of the
//! RISC-V privileged architecture (mcause codes, PMP CSRs, mscratch). A
//! board wires this crate in via its `Cargo.toml` and supplies the rest
//! (console, board-level timer source, exit/reset) as `rivet-bsp-*`.
//!
//! # The one platform-dependent piece: rescheduling and the tick
//!
//! RISC-V has no ISA-mandated "pend a reschedule interrupt to self" or
//! "periodic tick" mechanism (unlike Cortex-M's SysTick/PendSV, which are
//! architected). The near-universal convention on RV32 "virt"-like
//! platforms is the SiFive CLINT (`mtime`/`mtimecmp` for the tick, `MSIP`
//! for the reschedule IPI) — this crate ships that as the [`clint`] module,
//! gated behind the `clint` feature. A board without a CLINT (e.g. an
//! ESP32-C3, which has SYSTIMER instead) leaves the feature off and
//! supplies its own `#[no_mangle] extern "Rust" fn
//! __rivet_arch_request_reschedule()` / tick wiring directly — the symbol
//! contract doesn't care which crate defines a given symbol, only that
//! exactly one does.

#![no_std]

pub mod pmp;

#[cfg(feature = "clint")]
pub mod clint;

/// Minimum task stack: the 128-byte trap frame plus slack for the entry
/// trampoline.
pub const MIN_TASK_STACK: usize = 128 + 128;

// ── Dedicated ISR stack ─────────────────────────────────────────────
//
// The trap entry keeps the 128-byte register frame on the interrupted
// task's own stack (so `Tcb.sp` semantics are unchanged and per-task stack
// sizing stays analyzable: "user code + 128 bytes", full stop) but runs the
// *Rust* handler — schedule(), poll_timers(), panic! formatting, fault
// handling — on this dedicated stack, reached via `mscratch`. Without it,
// the handler's own unbounded call chain would compete for space with the
// task's stack, and a stack-overflow fault would re-enter the handler on
// the already-overflowed stack (double-fault loop). Provided by the
// linker (every board's linker script includes `rivet-rt`'s common
// fragment, which defines this section).
extern "C" {
    static __isr_stack_top: u8;
}

/// Group A entry point: install the trap vector, arm the boot-time-locked
/// PMP catch-all, and point `mscratch` at the ISR stack. Does **not** touch
/// any timer/IPI hardware — that's wired by whichever crate implements
/// [`clint`] or an equivalent for this board, from `__rivet_board_init`.
#[no_mangle]
extern "Rust" fn __rivet_arch_init() {
    use riscv::register::mtvec;

    // SAFETY: `__isr_stack_top` is the linker-provided top of the
    // `.isr_stack` section (rivet-rt's common linker fragment); mscratch
    // must hold it before any trap can fire.
    riscv::register::mscratch::write(core::ptr::addr_of!(__isr_stack_top) as usize);

    // SAFETY: `rivet_trap_entry` is the hand-written trap entry defined
    // below; installing it as the direct-mode mtvec handler is required
    // for every interrupt/fault to reach the kernel's dispatcher.
    unsafe {
        mtvec::write(
            rivet_trap_entry as *const () as usize,
            mtvec::TrapMode::Direct,
        );
    }

    pmp::init_catch_all();
}

#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    riscv::asm::wfi();
}

/// Stock CLINT-backed reschedule IPI. Only defined when the `clint`
/// feature is on; a board without a CLINT must supply its own
/// `#[no_mangle] extern "Rust" fn __rivet_arch_request_reschedule()`
/// (the symbol contract doesn't care which crate provides a given symbol,
/// only that exactly one does — see the module docs).
#[cfg(feature = "clint")]
#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    clint::request_reschedule();
}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    MIN_TASK_STACK
}

/// Save/restore only `mstatus.MIE` (not the whole `mstatus`): restoring all
/// of it would also restore MPP/MPIE as captured at entry, undoing any
/// privilege-mode changes made inside the guarded closure. Nested
/// save/restore composes correctly: an inner call observes MIE=0 (already
/// disabled) and its restore is a no-op, leaving the outermost call to
/// actually re-enable.
#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    use riscv::register::mstatus;
    let was_enabled = mstatus::read().mie();
    // SAFETY: disabling the global interrupt-enable bit is always safe;
    // the caller is responsible for pairing with `__rivet_arch_irq_restore`.
    unsafe { mstatus::clear_mie() };
    was_enabled as usize
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(token: usize) {
    // SAFETY: re-enabling MIE only if it was set at the matching
    // `__rivet_arch_irq_save` call (see that function's doc for why only
    // MIE, never the rest of mstatus, is touched).
    if token != 0 {
        unsafe { riscv::register::mstatus::set_mie() };
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(_stack_base: usize, _stack_size: usize) {
    // RISC-V PMP entries that affect M-mode must be locked at boot and are
    // immutable until reset, so there is nothing to reprogram per switch —
    // isolation comes entirely from the boot-time guard bands (see `pmp`).
}

#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(guard_base: usize, slot: usize) {
    pmp::register_guard(guard_base, slot);
}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(_base: usize, _size: usize) {
    // No MPU-equivalent toggle needed: M-mode PMP guards only ever deny a
    // 64-byte guard band, never the task-stack pool itself, so the kernel
    // always has access to the pool for initialization.
}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {}

// ── Preemptive tier: stack bootstrap ──────────────────────────────

// Trampoline entered (via `mepc`) the first time a freshly-spawned
// preemptive task runs. `s0`/`s1` are ordinary GPRs restored by the trap
// epilogue like any other — used here to smuggle the type-erased
// `(arg, entry_fn)` pair out of the initial fake stack frame and into the
// real call.
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_ptask_trampoline",
    "rivet_ptask_trampoline:",
    "  mv   a0, s0", // arg
    "  jalr s1",     // entry(arg) — may return with the result in a0/a1
    "  j    rivet_task_exit",
    // Task exit: the entry returned. a0/a1 carry the return value (or, for
    // >8-byte results, a0 is a pointer into this task's own stack). The
    // core stores it, marks the task exited, wakes joiners, and parks.
    ".global rivet_task_exit",
    "rivet_task_exit:",
    "  call rivet_task_exit_core",
    "1:",
    "  j 1b",
);

extern "C" {
    fn rivet_ptask_trampoline();
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
) -> usize {
    const FRAME_WORDS: usize = 32; // 128 bytes, matches rivet_trap_entry's frame
    const STACK_ALIGN: usize = 16;

    let base = stack_ptr as usize;
    let top = base + stack_len;
    let frame_start = (top - FRAME_WORDS * 4) & !(STACK_ALIGN - 1);
    let frame = frame_start as *mut u32;

    // SAFETY: `frame_start` lies within [stack_ptr, stack_ptr+stack_len)
    // (FRAME_WORDS*4 bytes reserved above), 4-byte aligned; the caller
    // guarantees the stack is at least `MIN_TASK_STACK` bytes.
    unsafe {
        for i in 0..FRAME_WORDS {
            core::ptr::write(frame.add(i), 0);
        }
        // s0 (offset 16 bytes = word 4) = arg
        core::ptr::write(frame.add(4), arg as u32);
        // s1 (offset 20 bytes = word 5) = entry_fn
        core::ptr::write(frame.add(5), entry_fn as u32);
        // mepc (offset 112 bytes = word 28) = trampoline address
        core::ptr::write(frame.add(28), rivet_ptask_trampoline as *const () as u32);
    }

    frame_start
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(sp: usize) -> ! {
    // SAFETY: `sp` was previously produced by `__rivet_arch_init_task_stack`
    // or is a live task frame in the same shape; `rivet_trap_resume` is the
    // shared restore epilogue defined below.
    unsafe {
        core::arch::asm!(
            "mv sp, {sp}",
            "j  rivet_trap_resume",
            sp = in(reg) sp,
            options(noreturn)
        );
    }
}

// ── Trap entry / dispatch ──────────────────────────────────────────

/// Trap dispatch logic, called from `rivet_trap_entry` with the interrupted
/// context (28 GPRs + mepc) already saved to the trap stack frame at
/// `interrupted_sp`. Returns the stack pointer to actually resume from —
/// this is what makes preemption real: if `rivet::preempt::on_tick`
/// decides a different, higher-priority task should run, this returns
/// *that task's* saved sp instead of `interrupted_sp`.
#[no_mangle]
unsafe extern "C" fn rivet_trap_handler_rust(interrupted_sp: usize) -> usize {
    use riscv::register::mcause;

    let cause = mcause::read();
    #[cfg_attr(not(feature = "clint"), allow(unused_mut))]
    let mut resume_sp = interrupted_sp;

    if cause.is_interrupt() {
        let code = cause.code();
        #[cfg(feature = "clint")]
        if code == 7 {
            // Machine timer interrupt: advance time, wake expired Sleep
            // futures, then give the preemptive scheduler a chance to
            // switch to a higher-priority ready task.
            clint::on_timer_irq();
            resume_sp = rivet::preempt::on_tick(interrupted_sp);
        } else if code == 3 {
            // Machine software interrupt: triggered by a voluntary yield or
            // a mutex unlock waking a waiter. Clear the pending bit, then
            // run the same reschedule decision.
            clint::ack_soft_irq();
            resume_sp = rivet::preempt::on_tick(interrupted_sp);
        }
        #[cfg(not(feature = "clint"))]
        let _ = code;
    } else {
        // Synchronous exception. Access faults (mcause 1/5/7) are routed
        // through the fault policy: a faulting address inside the
        // task-stack pool means a stack-overflow PMP guard trip; anything
        // else is an ordinary wild pointer. The policy either resets
        // (Panic) or returns the next task's sp (Isolate).
        let code = cause.code();
        if matches!(code, 1 | 5 | 7) {
            let mepc = riscv::register::mepc::read();
            let mtval = riscv::register::mtval::read();
            let kind = match code {
                1 => rivet::fault::FaultKind::InstructionAccess,
                5 => rivet::fault::FaultKind::LoadAccess,
                _ => rivet::fault::FaultKind::StoreAccess,
            };
            // Attribute to the running task (the trap handler runs on the
            // ISR stack; `sched::current()` is the interrupted task).
            let task_id = rivet::preempt::sched::current();
            let info = rivet::fault::FaultInfo {
                task_id,
                kind,
                address: mtval,
                pc: mepc,
            };
            return rivet::fault::on_fault(&info);
        }

        // Deliberately NOT silently ignored: doing so turns any fault into
        // an infinite loop (mepc unchanged, resume_sp unchanged -> we
        // `mret` right back into the same faulting instruction forever).
        panic!(
            "rivet: unhandled trap, mcause={:#x} mepc={:#x} mtval={:#x}",
            riscv::register::mcause::read().bits(),
            riscv::register::mepc::read(),
            riscv::register::mtval::read(),
        );
    }

    resume_sp
}

// Hand-written trap entry/exit: saves and restores the general-purpose
// register file (x1, x5-x31 — x0 is hardwired zero, x2/sp is the frame
// itself, x3/gp and x4/tp are fixed at boot and never touched by our code
// so are intentionally not saved, matching common RTOS RISC-V port
// convention) plus `mepc`, around the call into Rust. This is required
// because RISC-V `mret` does not auto-stack any registers — unlike
// Cortex-M, where hardware exception entry saves r0-r3/r12/lr/pc/xPSR
// for you.
//
// The frame (128 bytes) lives on the interrupted task's own stack; only
// the Rust call itself runs on the dedicated ISR stack: `csrrw sp,
// mscratch, sp` atomically swaps sp and mscratch, so after the swap sp =
// ISR stack top and mscratch = the task frame pointer. The handler's
// *unbounded* call chain (schedule, poll_timers, panic! formatting, fault
// handling) can therefore never overflow a task stack.
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_trap_entry",
    "rivet_trap_entry:",
    "  addi sp, sp, -128",
    "  sw   ra,   0(sp)",
    "  sw   t0,   4(sp)",
    "  sw   t1,   8(sp)",
    "  sw   t2,  12(sp)",
    "  sw   s0,  16(sp)",
    "  sw   s1,  20(sp)",
    "  sw   a0,  24(sp)",
    "  sw   a1,  28(sp)",
    "  sw   a2,  32(sp)",
    "  sw   a3,  36(sp)",
    "  sw   a4,  40(sp)",
    "  sw   a5,  44(sp)",
    "  sw   a6,  48(sp)",
    "  sw   a7,  52(sp)",
    "  sw   s2,  56(sp)",
    "  sw   s3,  60(sp)",
    "  sw   s4,  64(sp)",
    "  sw   s5,  68(sp)",
    "  sw   s6,  72(sp)",
    "  sw   s7,  76(sp)",
    "  sw   s8,  80(sp)",
    "  sw   s9,  84(sp)",
    "  sw   s10, 88(sp)",
    "  sw   s11, 92(sp)",
    "  sw   t3,  96(sp)",
    "  sw   t4, 100(sp)",
    "  sw   t5, 104(sp)",
    "  sw   t6, 108(sp)",
    "  csrr t0, mepc",
    "  sw   t0, 112(sp)",
    "  mv   a0, sp",                  // arg: interrupted frame ptr
    "  csrrw sp, mscratch, sp",       // sp <- ISR stack; mscratch <- frame ptr
    "  call rivet_trap_handler_rust", // returns: resume_sp (in a0)
    // Re-arm mscratch for the next trap: after `csrrw sp, mscratch, sp`
    // above, mscratch holds the interrupted frame pointer, NOT the ISR
    // stack top — leaving it would make the next trap swap sp to a task
    // frame (or zero) and fault.
    "  la   t1, __isr_stack_top",
    "  csrw mscratch, t1",
    "  mv   sp, a0",            // switch to the resume stack
    "  j    rivet_trap_resume", // shared restore epilogue
);

extern "C" {
    fn rivet_trap_entry();
}

core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_trap_resume",
    "rivet_trap_resume:",
    "  lw   t0, 112(sp)",
    "  csrw mepc, t0",
    // mstatus.MPIE must be 1 so mret enables interrupts (MIE <- MPIE), and
    // mstatus.MPP must be M-mode (0b11) so mret returns to machine mode
    // instead of user mode. Both matter on EVERY resume, not just the
    // first: `mret` unconditionally resets MPP to the least-privileged
    // supported mode after each return.
    // 0x1880 = MPP (bits 12:11, both set = 0b11) | MPIE (bit 7).
    "  li   t0, 0x1880",
    "  csrs mstatus, t0",
    "  lw   ra,   0(sp)",
    "  lw   t0,   4(sp)",
    "  lw   t1,   8(sp)",
    "  lw   t2,  12(sp)",
    "  lw   s0,  16(sp)",
    "  lw   s1,  20(sp)",
    "  lw   a0,  24(sp)",
    "  lw   a1,  28(sp)",
    "  lw   a2,  32(sp)",
    "  lw   a3,  36(sp)",
    "  lw   a4,  40(sp)",
    "  lw   a5,  44(sp)",
    "  lw   a6,  48(sp)",
    "  lw   a7,  52(sp)",
    "  lw   s2,  56(sp)",
    "  lw   s3,  60(sp)",
    "  lw   s4,  64(sp)",
    "  lw   s5,  68(sp)",
    "  lw   s6,  72(sp)",
    "  lw   s7,  76(sp)",
    "  lw   s8,  80(sp)",
    "  lw   s9,  84(sp)",
    "  lw   s10, 88(sp)",
    "  lw   s11, 92(sp)",
    "  lw   t3,  96(sp)",
    "  lw   t4, 100(sp)",
    "  lw   t5, 104(sp)",
    "  lw   t6, 108(sp)",
    "  addi sp, sp, 128",
    // Symbol for the GDB context-switch verification script (tests/gdb):
    // at this point every saved register has been loaded and sp points just
    // past the frame, so the script can compare the live register file
    // against the frame contents (frame base = sp - 128).
    ".global rivet_trap_mret",
    "rivet_trap_mret:",
    "  mret",
);

/// ARM-style "semihosting" debug I/O, RISC-V flavor (QEMU's RISC-V
/// semihosting matcher). Not board-specific (the magic instruction
/// sequence is architectural/QEMU-defined, not tied to any particular
/// memory map) — a utility a BSP *may* use for its console/exit
/// implementation instead of a real UART, kept here rather than
/// duplicated per board.
pub mod semihosting {
    /// # Safety
    /// `ptr` must point at a NUL-terminated string valid for the call.
    unsafe fn write0(ptr: *const u8) {
        const SYS_WRITE0: usize = 0x04;
        // SAFETY: this is the standard ARM/RISC-V semihosting "SYS_WRITE0"
        // sequence (the fixed 3-instruction magic pattern QEMU's
        // semihosting matcher recognizes); the caller guarantees `ptr`
        // points at a NUL-terminated string valid for the call.
        unsafe {
            core::arch::asm!(
                "   .option push",
                "   .option norvc", // force 32-bit ebreak; QEMU's semihosting
                "   .align 4",      // magic-sequence matcher requires the exact
                "   slli x0, x0, 0x1f", // 3x 32-bit instruction pattern below —
                "   ebreak",             // the compressed `c.ebreak` (2 bytes)
                "   srai x0, x0, 7",     // that riscv32imac's C extension would
                "   .option pop",        // otherwise emit breaks the match.
                in("a0") SYS_WRITE0,
                in("a1") ptr,
                options(nostack, preserves_flags)
            );
        }
    }

    /// Print a short string (truncated to 127 bytes) via semihosting.
    pub fn print(s: &str) {
        let bytes = s.as_bytes();
        let len = bytes.len().min(127);
        let mut buf = [0u8; 128];
        buf[..len].copy_from_slice(&bytes[..len]);
        buf[len] = b'\0';
        // SAFETY: `buf` is NUL-terminated at `len` and lives for the call.
        unsafe { write0(buf.as_ptr()) };
    }

    /// Exit via semihosting `SYS_EXIT` (`ADP_Stopped_ApplicationExit`).
    /// Never returns. Secondary exit path — most boards prefer a hardware
    /// exit device (e.g. QEMU virt's `riscv.sifive.test`) when available.
    pub fn exit_success() -> ! {
        const SYS_EXIT: usize = 0x18;
        // a1 holds the ADP stop reason directly (QEMU's RISC-V semihosting
        // uses the "simple" convention here, not a pointer to a [reason,
        // subcode] block).
        const ADP_STOPPED_APPLICATIONEXIT: usize = 0x20026;
        // SAFETY: standard ARM/RISC-V semihosting "SYS_EXIT" sequence;
        // `noreturn` — QEMU terminates the guest on it.
        unsafe {
            core::arch::asm!(
                "   .option push",
                "   .option norvc",
                "   .align 4",
                "   slli x0, x0, 0x1f",
                "   ebreak",
                "   srai x0, x0, 7",
                "   .option pop",
                in("a0") SYS_EXIT,
                in("a1") ADP_STOPPED_APPLICATIONEXIT,
                options(nostack, noreturn)
            );
        }
    }
}
