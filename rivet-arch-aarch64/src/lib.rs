#![no_std]
//! Rivet's AArch64 (ARMv8-A) architecture port, running the kernel at EL1.
//!
//! Provides Group A of the port contract (see `rivet::port::arch`): the
//! exception vectors, the context switch, interrupt masking and the cycle
//! counter. Board specifics stay in the `rivet-bsp-*` crate.
//!
//! # How a context switch happens here
//!
//! Cortex-M gets a dedicated `PendSV` exception whose whole purpose is
//! deferred context switching, and the hardware stacks half the frame for
//! it. AArch64 has neither, so both halves are explicit:
//!
//! - Every exception saves the full register file onto the *interrupted
//!   stack*, which at EL1h is the running task's own stack. That mirrors
//!   the Cortex-M port's use of the process stack, and it is what makes
//!   the frame addressable as "this task's saved context".
//! - The handler passes that stack pointer to `rivet::preempt::on_tick`,
//!   which returns the stack pointer to actually resume from: the same
//!   one if no switch is due, a different task's frame if one is. The
//!   restore path is identical either way, so there is exactly one
//!   context-switch code path, as the port contract requires.
//! - A task asking to reschedule outside interrupt context issues `SVC`,
//!   which lands in the same save/decide/restore sequence. Inside
//!   interrupt context it is a no-op, because the interrupt's own exit
//!   already runs that sequence and nesting a second one would switch
//!   stacks in the middle of the first.
//!
//! # Interrupt controller
//!
//! Deliberately absent. The obvious choice would be a GICv2 driver, but
//! the first board this port targets (BCM2837) has no usable GIC: it has
//! Broadcom's own controller plus a per-core local block. So
//! `__rivet_arch_irq_*` forwards to board-provided symbols, the same
//! escape hatch `rivet-arch-riscv` offers through its `board-irq-hook`
//! feature for the ESP32-C6's non-standard interrupt matrix. A GICv2
//! module can be added behind a feature when a board that has one turns
//! up, without disturbing this.

use core::sync::atomic::{AtomicBool, Ordering};

extern "Rust" {
    /// Handle and acknowledge whatever raised the current interrupt.
    ///
    /// The board owns its interrupt controller (see the module docs), so
    /// it decides what fired and clears it. Scheduling is not its
    /// concern: this crate calls `rivet::preempt::on_tick` on the way out
    /// regardless.
    fn __rivet_board_on_irq();

    /// Board-owned peripheral IRQ enable/disable/priority, mirroring what
    /// a GIC distributor would do on a board that had one.
    fn __rivet_board_irq_enable(irq_num: u32);
    fn __rivet_board_irq_disable(irq_num: u32);
    fn __rivet_board_irq_set_priority(irq_num: u32, priority: u8);
}

/// Set while an interrupt is being serviced, so that a reschedule request
/// raised from inside one does not issue a nested `SVC`.
static IN_IRQ: AtomicBool = AtomicBool::new(false);

/// Bytes in a saved context. See the layout comment in the assembly.
const FRAME_SIZE: usize = 272;

// Byte offsets into that frame, for the fabricated initial frame below.
const OFF_X19: usize = 0x098;
const OFF_X20: usize = 0x0A0;
const OFF_X30: usize = 0x0F0;
const OFF_ELR: usize = 0x0F8;
const OFF_SPSR: usize = 0x100;

/// `SPSR_EL1` for a task: EL1h, with IRQs unmasked so preemption works,
/// and debug/SError/FIQ masked.
const SPSR_TASK: u64 = 0x345;

core::arch::global_asm!(
    r#"
// Saved context, 272 bytes, 16-byte aligned throughout:
//
//   0x000  x0  x1        0x080  x16 x17       0x0F0  x30 elr_el1
//   0x010  x2  x3        0x090  x18 x19       0x100  spsr_el1 (pad)
//   ...                  ...
//   0x070  x14 x15       0x0E0  x28 x29
//
// Every caller-saved register is in there too, not just the callee-saved
// set: an interrupt lands on arbitrary code, and the handler calls into
// Rust, which is free to clobber x0-x18.
.macro SAVE_FRAME
    sub sp, sp, #272
    stp x0,  x1,  [sp, #0x000]
    stp x2,  x3,  [sp, #0x010]
    stp x4,  x5,  [sp, #0x020]
    stp x6,  x7,  [sp, #0x030]
    stp x8,  x9,  [sp, #0x040]
    stp x10, x11, [sp, #0x050]
    stp x12, x13, [sp, #0x060]
    stp x14, x15, [sp, #0x070]
    stp x16, x17, [sp, #0x080]
    stp x18, x19, [sp, #0x090]
    stp x20, x21, [sp, #0x0A0]
    stp x22, x23, [sp, #0x0B0]
    stp x24, x25, [sp, #0x0C0]
    stp x26, x27, [sp, #0x0D0]
    stp x28, x29, [sp, #0x0E0]
    // x9/x10 are only used as scratch after both have been saved.
    mrs x9,  elr_el1
    stp x30, x9,  [sp, #0x0F0]
    mrs x10, spsr_el1
    str x10, [sp, #0x100]
.endm

.macro RESTORE_FRAME
    ldr x10, [sp, #0x100]
    msr spsr_el1, x10
    ldp x30, x9,  [sp, #0x0F0]
    msr elr_el1, x9
    ldp x28, x29, [sp, #0x0E0]
    ldp x26, x27, [sp, #0x0D0]
    ldp x24, x25, [sp, #0x0C0]
    ldp x22, x23, [sp, #0x0B0]
    ldp x20, x21, [sp, #0x0A0]
    ldp x18, x19, [sp, #0x090]
    ldp x16, x17, [sp, #0x080]
    ldp x14, x15, [sp, #0x070]
    ldp x12, x13, [sp, #0x060]
    // x9/x10 get their real values back here, after their scratch use.
    ldp x10, x11, [sp, #0x050]
    ldp x8,  x9,  [sp, #0x040]
    ldp x6,  x7,  [sp, #0x030]
    ldp x4,  x5,  [sp, #0x020]
    ldp x2,  x3,  [sp, #0x010]
    ldp x0,  x1,  [sp, #0x000]
    add sp, sp, #272
.endm

.section .text, "ax"

// Interrupt: save, let the board acknowledge its controller, ask the
// scheduler what to run, resume onto whatever it picked.
.global rivet_aarch64_irq_entry
rivet_aarch64_irq_entry:
    SAVE_FRAME
    mov x0, sp
    bl  rivet_aarch64_irq_dispatch
    mov sp, x0
    RESTORE_FRAME
    eret

// Synchronous exception. SVC is a reschedule request and goes through the
// same path; everything else is a fault.
.global rivet_aarch64_sync_entry
rivet_aarch64_sync_entry:
    SAVE_FRAME
    mrs x1, esr_el1
    lsr x2, x1, #26
    cmp x2, #0x15                   // EC 0x15 = SVC from AArch64
    b.ne .Lsync_fault
    mov x0, sp
    bl  rivet_aarch64_svc_dispatch
    mov sp, x0
    RESTORE_FRAME
    eret
.Lsync_fault:
    mov x0, sp
    bl  rivet_aarch64_fault
.Lsync_halt:
    wfe
    b   .Lsync_halt

// Anything else that reaches the vectors is unexpected at this stage.
.global rivet_aarch64_unexpected
rivet_aarch64_unexpected:
    SAVE_FRAME
    mov x0, sp
    bl  rivet_aarch64_fault
.Lunexp_halt:
    wfe
    b   .Lunexp_halt

// Resume a saved context: x0 is the frame to restore from. Used both for
// the very first task and, indirectly, by every exception return above.
.global rivet_aarch64_resume
rivet_aarch64_resume:
    mov sp, x0
    RESTORE_FRAME
    eret

// First instruction a freshly spawned task executes. The fabricated frame
// carries its argument in x19 and its entry point in x20, which the
// restore path has already loaded by the time control arrives here.
.global rivet_ptask_trampoline
rivet_ptask_trampoline:
    mov x0, x19
    blr x20
    b   rivet_task_exit

// The entry returned. x0/x1 carry the return value; the kernel stores it,
// marks the task exited, wakes joiners and parks.
.global rivet_task_exit
rivet_task_exit:
    bl  rivet_task_exit_core
.Lexit_halt:
    b   .Lexit_halt

// Sixteen entries of 128 bytes, 2 KiB aligned: four exception kinds for
// each of four origins. Only the EL1h group is reachable while the kernel
// runs, since there are no EL0 tasks yet.
.section .text.rivet_vectors, "ax"
.balign 2048
.global rivet_aarch64_vectors
rivet_aarch64_vectors:
    // Current EL, SP0. Not used: the kernel runs on SPx.
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    // Current EL, SPx. This is the live group.
    .balign 128
    b rivet_aarch64_sync_entry
    .balign 128
    b rivet_aarch64_irq_entry
    .balign 128
    b rivet_aarch64_unexpected      // FIQ, unused
    .balign 128
    b rivet_aarch64_unexpected      // SError
    // Lower EL, AArch64.
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    // Lower EL, AArch32.
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
    b rivet_aarch64_unexpected
    .balign 128
"#
);

extern "C" {
    fn rivet_aarch64_vectors();
    fn rivet_aarch64_resume(frame: usize) -> !;
    fn rivet_ptask_trampoline();
}

/// Interrupt path: the board clears its own source, then the scheduler
/// gets a say. Returns the frame to resume from.
#[no_mangle]
unsafe extern "C" fn rivet_aarch64_irq_dispatch(interrupted_sp: usize) -> usize {
    IN_IRQ.store(true, Ordering::Release);
    // SAFETY: provided by the board crate; acknowledging its own hardware.
    unsafe { __rivet_board_on_irq() };
    IN_IRQ.store(false, Ordering::Release);
    rivet::preempt::on_tick(interrupted_sp)
}

/// `SVC` path: an explicit reschedule request from task context.
#[no_mangle]
extern "C" fn rivet_aarch64_svc_dispatch(interrupted_sp: usize) -> usize {
    rivet::preempt::on_tick(interrupted_sp)
}

/// Unrecoverable exception. Reports through the kernel console and
/// returns; the caller halts.
#[no_mangle]
extern "C" fn rivet_aarch64_fault(frame: usize) {
    let esr = read_sysreg_esr();
    let elr = read_sysreg_elr();
    let far = read_sysreg_far();

    rivet::console::write_str("\n*** AArch64 EXCEPTION ***\n  ESR ");
    write_hex(esr);
    rivet::console::write_str("\n  ELR ");
    write_hex(elr);
    rivet::console::write_str("\n  FAR ");
    write_hex(far);
    rivet::console::write_str("\n  frame ");
    write_hex(frame as u64);
    rivet::console::write_str("\n");
    rivet::console::flush_sync();
}

fn write_hex(v: u64) {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut buf = [0u8; 18];
    buf[0] = b'0';
    buf[1] = b'x';
    for i in 0..16 {
        buf[2 + i] = DIGITS[((v >> (60 - 4 * i)) & 0xf) as usize];
    }
    // SAFETY: every byte written above is ASCII.
    rivet::console::write_str(unsafe { core::str::from_utf8_unchecked(&buf) });
}

macro_rules! read_sysreg {
    ($name:literal) => {{
        let v: u64;
        // SAFETY: reading a system register has no side effects.
        unsafe {
            core::arch::asm!(concat!("mrs {}, ", $name), out(reg) v,
                             options(nomem, nostack, preserves_flags))
        };
        v
    }};
}

fn read_sysreg_esr() -> u64 {
    read_sysreg!("esr_el1")
}
fn read_sysreg_elr() -> u64 {
    read_sysreg!("elr_el1")
}
fn read_sysreg_far() -> u64 {
    read_sysreg!("far_el1")
}

// ── Group A: the port contract ────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_arch_init() {
    IN_IRQ.store(false, Ordering::Release);
    // Take over the vector table from whatever the board installed during
    // boot. That earlier table only reports faults; this one also routes
    // interrupts and SVC into the scheduler.
    // SAFETY: `rivet_aarch64_vectors` is the 2 KiB-aligned table above.
    unsafe {
        core::arch::asm!(
            "msr vbar_el1, {v}",
            "isb",
            v = in(reg) rivet_aarch64_vectors as *const () as u64,
            options(nostack, preserves_flags),
        );
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    // SAFETY: WFI parks the core until an interrupt is pending.
    unsafe { core::arch::asm!("wfi", options(nomem, nostack, preserves_flags)) };
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    // Inside an interrupt the exit path already runs the scheduler, and an
    // SVC here would nest a second switch inside the first.
    if IN_IRQ.load(Ordering::Acquire) {
        return;
    }
    // SAFETY: traps to the synchronous vector, which switches context and
    // returns here (or to another task) via ERET.
    unsafe { core::arch::asm!("svc #1", options(nostack, preserves_flags)) };
}

/// Single core for now. Releasing the other three is a later milestone,
/// and it needs cache maintenance on the spin table: they are parked with
/// caches off, so a plain store from a core running with the D-cache on
/// never reaches them.
#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule_on(hart: usize) {
    if hart == 0 {
        __rivet_arch_request_reschedule();
    }
}

/// Always zero, deliberately, and not the physical core number.
///
/// The kernel uses this to index per-hart scheduler state sized by
/// `RIVET_MAX_HARTS`, so it has to be a dense index into the set of cores
/// rivet is actually scheduling on, not an identifier for the silicon.
/// This port runs the kernel on exactly one core, which may well be core
/// 3 with the others parked or owned by another OS entirely; from the
/// scheduler's point of view that core is hart 0. Returning `MPIDR` here
/// would index a one-element array out of bounds the moment the kernel
/// ran anywhere but core 0.
///
/// `rivet_bsp_rpi3b::smp::current_core` reports the physical number for
/// diagnostics.
#[no_mangle]
extern "Rust" fn __rivet_arch_hart_id() -> usize {
    0
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    let daif: u64;
    // SAFETY: reads the mask state, then masks IRQs. The caller pairs this
    // with `__rivet_arch_irq_restore`.
    unsafe {
        core::arch::asm!(
            "mrs {d}, daif",
            "msr daifset, #2",
            d = out(reg) daif,
            options(nomem, nostack, preserves_flags),
        );
    }
    daif as usize
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(token: usize) {
    // Bit 7 of DAIF is the I mask. Only re-enable if the matching save saw
    // interrupts enabled, so nested critical sections compose.
    if (token as u64) & (1 << 7) == 0 {
        // SAFETY: paired with a preceding `__rivet_arch_irq_save`.
        unsafe { core::arch::asm!("msr daifclr, #2", options(nomem, nostack, preserves_flags)) };
    }
}

/// The architected counter, running at `CNTFRQ_EL0` (19.2 MHz on the
/// BCM2837, not the CPU clock). Monotonic, which is all the contract
/// requires.
#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    read_sysreg!("cntpct_el0")
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_enable(irq_num: u32) {
    // SAFETY: board-provided, see the module docs on why the controller
    // is not this crate's business on this architecture yet.
    unsafe { __rivet_board_irq_enable(irq_num) };
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_disable(irq_num: u32) {
    // SAFETY: as above.
    unsafe { __rivet_board_irq_disable(irq_num) };
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_set_priority(irq_num: u32, priority: u8) {
    // SAFETY: as above.
    unsafe { __rivet_board_irq_set_priority(irq_num, priority) };
}

/// Frame plus room for the trampoline and a few nested calls. Larger than
/// the 32-bit ports need: the saved context alone is 272 bytes here.
#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    1024
}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_guard_size() -> usize {
    64
}

/// No per-task memory protection yet. The MMU tables the board installs
/// are static and cover the whole address space uniformly, so there is
/// nothing to reprogram per switch. Per-task stack guards would mean
/// splitting the flat 2 MiB block mappings into pages, which is worth
/// doing once the scheduler itself is proven here.
#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(_stack_base: usize, _stack_size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(_guard_base: usize, _slot: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(_base: usize, _size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {}

/// Fabricate the context a never-yet-run task needs, such that restoring
/// it lands in the trampoline with the task's argument and entry point in
/// hand.
#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
) -> usize {
    let base = stack_ptr as usize;
    let top = base + stack_len;
    // AArch64 requires SP to be 16-byte aligned whenever it is used.
    let frame_start = (top - FRAME_SIZE) & !0xF;

    // SAFETY: the caller guarantees at least `min_task_stack` bytes, so
    // the whole frame lies inside the slice.
    unsafe {
        core::ptr::write_bytes(frame_start as *mut u8, 0, FRAME_SIZE);
        let at = |off: usize| (frame_start + off) as *mut u64;
        core::ptr::write(at(OFF_X19), arg as u64);
        core::ptr::write(at(OFF_X20), entry_fn as u64);
        core::ptr::write(at(OFF_X30), rivet_task_exit_addr());
        core::ptr::write(at(OFF_ELR), rivet_ptask_trampoline as *const () as u64);
        core::ptr::write(at(OFF_SPSR), SPSR_TASK);
    }

    frame_start
}

fn rivet_task_exit_addr() -> u64 {
    extern "C" {
        fn rivet_task_exit();
    }
    rivet_task_exit as *const () as u64
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(sp: usize) -> ! {
    // The frame carries `SPSR_TASK`, which unmasks interrupts as part of
    // the ERET. That matters: `preempt::start` wraps this call in a
    // critical section whose restore never runs, because this diverges,
    // so every port has to re-enable interrupts as part of dispatch.
    // SAFETY: `sp` is a frame built by `__rivet_arch_init_task_stack`.
    unsafe { rivet_aarch64_resume(sp) }
}
