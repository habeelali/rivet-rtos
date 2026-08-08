//! Memory Protection Unit: two-region stack isolation.
//!
//! Verified against QEMU's lm3s6965evb (MPU_TYPE = 0x0800, 8 data
//! regions), but the register layout is architectural — identical on any
//! Cortex-M3/4/7/33 with an MPU. Design:
//!
//!   Region 6 — the whole `.task_stacks` pool, AP=no-access, XN=1: denies
//!              everything by default (tasks cannot touch each other's
//!              stacks; an overflow past a stack's low end faults).
//!   Region 7 — the *currently running* task's stack, AP=RW, XN=1,
//!              reprogrammed on every context switch (three register
//!              writes). On overlap the highest-numbered enabled region
//!              wins, so region 7 re-enables just the running task's stack
//!              inside the denied pool.
//!
//! PRIVDEFENA lets the kernel (flash, peripherals, kernel data) use the
//! background map untouched. The pool's bounds come from the *kernel's*
//! linker layout (`rivet::preempt::stack_pool::pool_bounds`), not from any
//! board knowledge.

const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

const RASR_XN: u32 = 1 << 28;
const RASR_AP_NO_ACCESS: u32 = 0b000 << 24;
const RASR_AP_RW: u32 = 0b011 << 24;
const RASR_ENABLE: u32 = 1 << 0;
const RBAR_VALID: u32 = 1 << 4;

/// Region number for the whole-pool deny.
const POOL_DENY_REGION: u32 = 6;
/// Region number for the running task's stack.
const CURRENT_STACK_REGION: u32 = 7;

fn write_region(region: u32, base: usize, size: usize, ap: u32, xn: bool) {
    // SAFETY: these are the fixed, memory-mapped MPU registers (volatile);
    // the MPU is exclusively owned by this module.
    unsafe {
        let size_field = size.trailing_zeros() - 1; // region = 2^(SIZE+1)
        core::ptr::write_volatile(MPU_RNR, region);
        core::ptr::write_volatile(MPU_RBAR, (base as u32) | RBAR_VALID | region);
        core::ptr::write_volatile(
            MPU_RASR,
            (if xn { RASR_XN } else { 0 }) | ap | (size_field << 1) | RASR_ENABLE,
        );
    }
}

/// Enable the MPU (PRIVDEFENA + ENABLE) and program the pool-deny region.
/// Region 7 is programmed on the first context switch. Must be called
/// before any task can run.
pub(crate) fn init() {
    let (base, len) = rivet::preempt::stack_pool::pool_bounds();
    // The pool is aligned (linker script); its size is a power of two.
    debug_assert!(len.is_power_of_two());
    write_region(POOL_DENY_REGION, base, len, RASR_AP_NO_ACCESS, true);
    // SAFETY: fixed MPU control register; enable + PRIVDEFENA.
    unsafe {
        core::ptr::write_volatile(MPU_CTRL, 0b101);
    }
}

/// Reprogram region 7 to cover the currently-running task's stack. Called
/// on every actual context switch.
pub(crate) fn set_current_stack(base: usize, size: usize) {
    if base == 0 {
        return;
    }
    debug_assert!(size.is_power_of_two());
    write_region(CURRENT_STACK_REGION, base, size, RASR_AP_RW, true);
}

/// Disable the MPU for a scoped window (used while the SVC handler
/// initializes a newly allocated stack, which lives inside the
/// otherwise-denied pool). Returns the previous `MPU_CTRL` value to
/// restore via [`restore_after_scope`].
pub(crate) fn disable_for_scope() -> u32 {
    // SAFETY: fixed, memory-mapped MPU control register.
    unsafe {
        let saved = core::ptr::read_volatile(MPU_CTRL);
        core::ptr::write_volatile(MPU_CTRL, saved & !1);
        saved
    }
}

pub(crate) fn restore_after_scope(saved: u32) {
    // SAFETY: as above.
    unsafe { core::ptr::write_volatile(MPU_CTRL, saved) };
}

/// Temporarily disable the whole-pool deny region (used while
/// `preempt::spawn` fills and initializes a new stack inside the denied
/// pool). The pool-deny region is the highest-numbered region below the
/// current-stack region, so any other enabled region covering a stack
/// would lose the overlap to it — the deny region itself must be disabled
/// for the window. The caller holds interrupts off (critical section), so
/// no other task can run during the window.
pub(crate) fn allow_scratch(_base: usize, _size: usize) {
    // SAFETY: fixed MPU registers; disable region 6.
    unsafe {
        core::ptr::write_volatile(MPU_RNR, POOL_DENY_REGION);
        core::ptr::write_volatile(MPU_RASR, 0);
    }
}

/// Re-enable the pool-deny region (closes the [`allow_scratch`] window).
pub(crate) fn clear_scratch() {
    let (base, len) = rivet::preempt::stack_pool::pool_bounds();
    write_region(POOL_DENY_REGION, base, len, RASR_AP_NO_ACCESS, true);
}

// ── MemManage fault handling ────────────────────────────────────────

const CFSR: *const u32 = 0xE000_ED28 as *const u32;
const MMFAR: *const u32 = 0xE000_ED34 as *const u32;

/// Handle a MemManage fault: attribute to the running task and dispatch to
/// the fault policy. Under `FaultPolicy::Panic` this resets; under
/// `FaultPolicy::IsolateTask` it abandons the faulted task's frame and
/// returns the next ready task's *full* frame sp (manual r4-r11 save +
/// hardware frame). The asm `MemManage` entry (below) restores r4-r11 and
/// switches PSP to that frame, so the exception return un-stacks the *new*
/// task — a real context switch out of the fault handler, mirroring the
/// PendSV path.
#[no_mangle]
unsafe extern "C" fn rivet_memmanage_rust(faulted_sp: usize) -> usize {
    // SAFETY: fixed, memory-mapped system registers (volatile).
    let cfsr = unsafe { core::ptr::read_volatile(CFSR) };
    // SAFETY: as above — the MMFAR register (valid when CFSR.MMARVALID set).
    let mmfar = unsafe { core::ptr::read_volatile(MMFAR) };

    let fault_pc = if faulted_sp != 0 {
        // SAFETY: the faulted task's stacked frame is at faulted_sp; its
        // PC is at offset 24 (r0,r1,r2,r3,r12,lr,pc,xPSR).
        unsafe { core::ptr::read_volatile((faulted_sp + 24) as *const u32) as usize }
    } else {
        0
    };

    let task_id = rivet::preempt::sched::current();
    if let Some(id) = task_id {
        if let Some(t) = rivet::preempt::tcb::get(id) {
            // Persist the abandoned frame so the resume path would work if
            // this task were ever restarted.
            t.sp.store(faulted_sp, rivet::sync::atomic::Ordering::Release);
        }
    }

    let address = if cfsr & (1 << 7) != 0 {
        mmfar as usize
    } else {
        0
    };
    let info = rivet::fault::FaultInfo {
        task_id,
        kind: rivet::fault::FaultKind::MemManage(cfsr),
        address,
        pc: fault_pc,
    };
    rivet::fault::on_fault(&info)
}

core::arch::global_asm!(
    ".section .text.MemManage",
    ".global MemManage",
    ".thumb_func",
    "MemManage:",
    "  push {{lr}}",               // save EXC_RETURN (bl clobbers lr)
    "  mrs  r0, psp",              // faulted task's stacked frame
    "  bl   rivet_memmanage_rust", // returns next task's full frame sp
    "  ldmia r0, {{r4-r11}}",      // restore the new task's callee-saved
    "  adds r0, r0, #32",          // advance past the manual frame
    "  msr  psp, r0",              // PSP = hw frame; the exception return
    "  pop  {{lr}}",               // restore EXC_RETURN
    "  bx   lr",                   // un-stacks the new task's frame
);
