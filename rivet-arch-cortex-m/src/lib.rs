//! Rivet RTOS — ARM Cortex-M ISA port.
//!
//! Implements the Group A (`rivet::port::arch`) symbol contract for
//! Cortex-M targets: PendSV context switch, MemManage fault handling, MPU
//! programming, SysTick tick source. Contains **no board/MMIO knowledge**
//! beyond what's genuinely part of the Cortex-M architecture (SCB, MPU,
//! SysTick, and their fixed System Control Space addresses — identical on
//! every Cortex-M3/4/7/33). A board's clock rate, console, and exit/reset
//! path are supplied separately by a `rivet-bsp-*` crate.
//!
//! # Preemptive context switch
//!
//! Tasks run in Thread mode using PSP (Process Stack Pointer); exceptions
//! (SysTick, PendSV, everything else) always run in Handler mode using MSP
//! (Main Stack Pointer) — automatic Cortex-M behavior. That split matters:
//! a PendSV handler's own nested Rust calls (the scheduler, atomics, etc.)
//! run on MSP, never touching a task's PSP-based stack — no RISC-V-style
//! risk of the scheduler's own call chain competing for space with
//! whatever a task had reserved for itself.
//!
//! Following ARM's recommended pattern: SysTick only *requests* a
//! reschedule (`SCB.ICSR.PENDSVSET`); the actual register save/restore and
//! scheduling decision happen in PendSV, which — being the lowest-priority
//! exception — never preempts a higher-priority ISR mid-flight.

#![no_std]

pub mod dwt;
pub mod mpu;
pub mod semihosting;
#[cfg(feature = "systick")]
pub mod systick;

/// Minimum task stack: the PendSV frame (32 bytes r4-r11 + 32 bytes
/// hardware-stacked r0-r3/r12/lr/pc/xPSR) plus slack for the entry
/// trampoline.
pub const MIN_TASK_STACK: usize = 64 + 64;

#[no_mangle]
extern "Rust" fn __rivet_arch_init() {
    mpu::init();
    dwt::init();

    // PendSV must run at the lowest possible priority so it never preempts
    // a higher-priority ISR mid-flight — it only runs once everything else
    // has finished, which is what makes it safe to do the actual stack
    // switch there. Set SHPR3.PRI_14 (PendSV) and SHPR3.PRI_15 (SysTick)
    // to the lowest priority (0xFF, all implemented priority bits set).
    //
    // SAFETY: `SCB::PTR` is the statically-known System Control Block
    // base, valid on every Cortex-M; these SHPR/SHCSR writes are volatile
    // MMIO accesses and the SCB is exclusively owned by this module.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.shpr[10].write(0xFF); // PendSV priority (SHPR3 byte 2)
        scb.shpr[11].write(0xFF); // SysTick priority (SHPR3 byte 3)
                                  // Enable the dedicated Bus/Usage/MemManage fault handlers; without
                                  // this they escalate straight to HardFault, hiding the real cause.
        scb.shcsr.write(
            (1 << 16) // MEMFAULTENA
            | (1 << 17) // BUSFAULTENA
            | (1 << 18), // USGFAULTENA
        );
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    cortex_m::asm::wfi();
}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    MIN_TASK_STACK
}

#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    dwt::cycle_count()
}

/// plan.md Phase 12: cycle stamp at the moment a reschedule was
/// requested, consumed by `rivet_pendsv_rust` to record `IrqEntry`
/// latency — the single trigger point below covers both the tick-driven
/// and voluntary-yield paths uniformly (unlike RISC-V, Cortex-M has no
/// separate "just entered the handler" asm hook that's safe to touch
/// without risking the hand-tuned PendSV register-save sequence).
#[cfg(feature = "latency-histograms")]
static RESCHEDULE_REQUESTED_AT: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Set PendSV pending. Single trigger for every context switch, whether
/// tick-driven or a voluntary yield.
#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    #[cfg(feature = "latency-histograms")]
    RESCHEDULE_REQUESTED_AT.store(dwt::cycle_count() as u32, core::sync::atomic::Ordering::Relaxed);
    // SAFETY: `SCB::PTR` is the statically-known System Control Block
    // base, valid on every Cortex-M; `ICSR` write is a volatile MMIO
    // access.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.icsr.write(1 << 28); // PENDSVSET
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    // `Primask::is_active()` means "exceptions are active", i.e.
    // interrupts are currently *enabled* (PRIMASK bit clear) — no
    // negation here, unlike a naive reading of the name might suggest.
    let was_enabled = cortex_m::register::primask::read().is_active();
    cortex_m::interrupt::disable();
    was_enabled as usize
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(token: usize) {
    if token != 0 {
        // SAFETY: re-enabling interrupts only if they were enabled at the
        // matching `__rivet_arch_irq_save` call.
        unsafe { cortex_m::interrupt::enable() };
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(stack_base: usize, stack_size: usize) {
    mpu::set_current_stack(stack_base, stack_size);
}

#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(_guard_base: usize, _slot: usize) {
    // No per-task locked guard on Cortex-M: the two-region MPU design
    // (whole-pool deny + current-stack allow) already gives full mutual
    // stack isolation without per-task PMP-style entries.
}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(base: usize, size: usize) {
    mpu::allow_scratch(base, size);
}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {
    mpu::clear_scratch();
}

// ── Preemptive tier: PendSV context switch ────────────────────────

/// Rust-side PendSV logic. Called from the asm handler with `interrupted_sp`
/// (the interrupted task's PSP, pointing at its saved r4-r11 frame). Saves
/// the interrupted task's registers (already on the stack), asks the
/// scheduler what to run next, and returns the stack pointer to resume.
#[no_mangle]
unsafe extern "C" fn rivet_pendsv_rust(interrupted_sp: usize) -> usize {
    #[cfg(feature = "latency-histograms")]
    {
        let requested_at = RESCHEDULE_REQUESTED_AT.load(core::sync::atomic::Ordering::Relaxed);
        let now = dwt::cycle_count() as u32;
        rivet::latency::record(
            rivet::latency::Kind::IrqEntry,
            now.wrapping_sub(requested_at) as u64,
        );
    }
    rivet::preempt::on_tick(interrupted_sp)
}

core::arch::global_asm!(
    ".section .text.rivet_task_exit",
    ".global rivet_task_exit",
    ".thumb_func",
    "rivet_task_exit:",
    "  bl   rivet_task_exit_core", // r0/r1 carry the return value
    "1:",
    "  b    1b",
);

core::arch::global_asm!(
    ".section .text.PendSV",
    ".global PendSV",
    ".thumb_func",
    "PendSV:",
    "  push {{lr}}",
    "  mrs  r0, psp",
    "  subs r0, r0, #32",
    "  stmia r0, {{r4-r11}}",
    "  bl   rivet_pendsv_rust",
    "  ldmia r0, {{r4-r11}}",
    "  adds r0, r0, #32",
    "  msr  psp, r0",
    "  pop  {{lr}}",
    // Symbol for the GDB context-switch verification script (tests/gdb):
    // r4-r11 have been restored from the frame; frame base = psp - 32.
    ".global rivet_pendsv_resume",
    "rivet_pendsv_resume:",
    "  bx   lr",
);

// ── First task start / initial stack frame ────────────────────────

/// Set up the initial stack frame for a new task, then start the first
/// task's execution. Called once, from `preempt::start`, with the first
/// task's already-built stack frame.
#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(sp: usize) -> ! {
    // SAFETY: `sp` is the freshly-built initial frame of the first task;
    // PSP is set exactly once here, before any interrupt can fire.
    let frame = sp as *const u32;
    let arg = unsafe { core::ptr::read(frame.add(8)) };
    let entry_fn = unsafe { core::ptr::read(frame.add(14)) };

    unsafe {
        core::arch::asm!(
            "msr psp, {sp}",
            "movs r2, #2",
            "msr control, r2", // SPSEL=1 (use PSP in Thread mode), stay privileged
            "isb",
            sp = in(reg) sp,
            out("r2") _,
        );
    }

    // PSP is valid now — safe to let SysTick/PendSV start firing.
    #[cfg(feature = "systick")]
    systick::enable();

    unsafe {
        core::arch::asm!(
            "mov r0, {arg}",
            "bx {entry}",
            arg = in(reg) arg,
            entry = in(reg) entry_fn,
            options(noreturn)
        );
    }
}

/// Frame layout (aligned to 8 bytes, 64 bytes total):
/// ```text
/// [sp+0]  r4
/// [sp+4]  r5
/// [sp+8]  r6
/// [sp+12] r7
/// [sp+16] r8
/// [sp+20] r9
/// [sp+24] r10
/// [sp+28] r11
/// [sp+32] r0   <- arg
/// [sp+36] r1
/// [sp+40] r2
/// [sp+44] r3
/// [sp+48] r12
/// [sp+52] lr   <- entry_fn (with Thumb bit set)
/// [sp+56] pc   <- entry_fn (with Thumb bit set)
/// [sp+60] xPSR <- 0x01000000 (Thumb mode)
/// ```
/// The PendSV handler restores r4-r11 from the first 32 bytes; the
/// hardware un-stacks the remaining 32 bytes on exception return, resuming
/// at `entry_fn` with `r0 = arg`.
unsafe fn init_task_stack_impl(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    const FRAME_WORDS: usize = 16; // 8 (r4-r11) + 8 (hw frame)
    const STACK_ALIGN: usize = 16;

    // SAFETY: `stack` is a valid mutable slice of at least MIN_TASK_STACK
    // bytes (the caller guarantees this); the writes below initialize the
    // frame INSIDE the slice (at the top, aligned down).
    unsafe {
        let base = stack.as_mut_ptr() as usize;
        let top = base + stack.len();
        let frame_start = (top - FRAME_WORDS * 4) & !(STACK_ALIGN - 1);
        let frame = frame_start as *mut u32;

        for i in 0..FRAME_WORDS {
            core::ptr::write(frame.add(i), 0);
        }
        core::ptr::write(frame.add(8), arg as u32); // r0
                                                    // r1,r2,r3,r12 (words 9-12) stay 0
        extern "C" {
            fn rivet_task_exit();
        }
        core::ptr::write(frame.add(13), rivet_task_exit as *const () as usize as u32); // lr
        core::ptr::write(frame.add(14), entry_fn as u32); // pc
        core::ptr::write(frame.add(15), 0x0100_0000); // xPSR: Thumb bit (T=1) set

        frame_start
    }
}

/// SVC-vectored kernel call: builds a new task's initial stack frame from
/// *Handler* mode, where the MPU does not apply the way it does in Thread
/// mode. Thread-mode code cannot write another task's stack: MPU region 6
/// denies the whole `.task_stacks` pool and region 7 only permits the
/// *current* task's stack — a spawner faulting on the new task's stack is
/// exactly what an unprivileged `init_task_stack` would hit.
///
/// Naked (no prologue): the exception frame base must be read from `sp`
/// *before* the compiler pushes anything, and the exception return value
/// in `lr` must be preserved across the call so the handler returns with
/// `bx lr` (EXC_RETURN), not a normal branch.
///
/// # Safety
/// Exception entry point; installed via the board's vector table
/// (`rivet-rt`); never called directly.
#[unsafe(naked)]
#[no_mangle]
unsafe extern "C" fn rivet_svc_handler() {
    // SAFETY: naked handler with no stack frame; the register-level
    // protocol with `rivet_svc_core` is documented in the doc comment.
    core::arch::naked_asm!(
        "mov r4, lr",     // preserve EXC_RETURN (r4 is callee-saved)
        "uxtb r1, r4",    // EXC_RETURN 0xFFFFFFFD = taken from thread
        "cmp  r1, #0xfd", // mode with PSP (spawn from a running task);
        "bne  1f",        // 0xF9 = thread mode with MSP (boot context)
        "mrs  r0, psp",   // frame on PSP
        "b    2f",
        "1:",
        "mov  r0, sp", // frame on MSP
        "2:",
        "bl  rivet_svc_core",
        "mov lr, r4", // restore EXC_RETURN
        "bx  lr",     // exception return
    );
}

/// Rust half of [`rivet_svc_handler`]: `frame` is the exception stack
/// frame ({r0,r1,r2,r3,r12,lr,pc,xPSR}) pushed by the `svc 0` issued from
/// `__rivet_arch_init_task_stack`.
#[no_mangle]
fn rivet_svc_core(frame: *mut u32) {
    // SAFETY: the caller guarantees `frame` points at the live exception
    // stack frame ({r0,r1,r2,r3,...}) pushed by the `svc 0`; all four
    // slots are valid, word-aligned reads.
    let (stack_ptr, stack_len, entry, arg) = unsafe {
        (
            *frame.add(0) as *mut u8,
            *frame.add(1) as usize,
            *frame.add(2) as usize,
            *frame.add(3) as usize,
        )
    };

    // SAFETY: the caller passed a valid `&mut [u8]` slice split across
    // r0/r1 (as_mut_ptr / len).
    let sp = unsafe {
        // The ARMv7-M MPU applies in Handler mode too, so the write into
        // the denied `.task_stacks` pool would fault even here. Disable
        // the MPU for the duration of the frame write (real RTOSes do the
        // same); the SVC handler runs at the highest configurable priority
        // so nothing can preempt us mid-window.
        let saved = mpu::disable_for_scope();
        let sp = init_task_stack_impl(
            core::slice::from_raw_parts_mut(stack_ptr, stack_len),
            entry,
            arg,
        );
        mpu::restore_after_scope(saved);
        sp
    };
    // Deliver the result via the exception frame's saved r0.
    // SAFETY: `frame` points at the live exception stack frame on MSP.
    unsafe {
        core::ptr::write_volatile(frame, sp as u32);
    }
}

/// Issue `init_task_stack_impl` from Handler mode via SVC (see
/// [`rivet_svc_handler`] for why the MPU requires it).
///
/// The caller holds a critical section (PRIMASK=1). An `svc` issued with
/// PRIMASK set runs at execution priority 0 — equal to the SVC's own
/// default priority — which the architecture escalates to HardFault
/// (QEMU's NVIC does exactly this). So PRIMASK is briefly cleared around
/// the `svc`. This is safe: the SVC handler runs at priority 0, the
/// highest configurable priority, so nothing (SysTick/PendSV at 0xFF) can
/// preempt the frame write; the critical section's purpose — no task runs
/// mid-initialization — is preserved.
#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
) -> usize {
    let ptr = stack_ptr as usize;
    let mut sp = 0usize;
    // SAFETY: the SVC handler reads r0-r3 from the exception frame, builds
    // the frame, and writes the new sp back into r0.
    unsafe {
        let mut primask: u32;
        core::arch::asm!(
            "mrs {0}, primask",
            out(reg) primask,
            options(nomem, nostack, preserves_flags),
        );
        if primask & 1 != 0 {
            core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
        }
        core::arch::asm!(
            "svc 0",
            inout("r0") ptr => sp,
            in("r1") stack_len,
            in("r2") entry_fn,
            in("r3") arg,
            options(nomem, nostack, preserves_flags),
        );
        if primask & 1 != 0 {
            core::arch::asm!("cpsid i", options(nomem, nostack, preserves_flags));
        }
    }
    sp
}

/// Cortex-M system reset via SCB AIRCR SYSRESETREQ. A utility for BSPs'
/// `__rivet_board_reset` implementation — architecturally universal, not
/// board-specific.
pub fn system_reset() -> ! {
    // SAFETY: `0xE000ED0C` is the fixed SCB AIRCR register; writing
    // VECTKEY=0x05FA | SYSRESETREQ=1 requests a system reset.
    unsafe {
        core::ptr::write_volatile(0xE000_ED0C as *mut u32, 0x05FA_0004);
    }
    loop {
        core::hint::spin_loop();
    }
}
