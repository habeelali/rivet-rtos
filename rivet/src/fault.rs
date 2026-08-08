//! Fault policy (plan.md §3.4).
//!
//! When the arch layer catches a hardware fault — a MemManage fault on
//! Cortex-M, an access fault on RISC-V (mcause 1/5/7, typically from a PMP
//! guard), or a detected stack overflow — it builds a [`FaultInfo`] and
//! dispatches it here. Two explicit policies:
//!
//! - [`FaultPolicy::Panic`] (default): dump a diagnosis (kind, address,
//!   PC, faulting task) and reset the system.
//! - [`FaultPolicy::IsolateTask`]: mark the faulting task `Faulted`,
//!   poison every [`PriorityMutex`](crate::preempt::PriorityMutex) it
//!   holds, invoke the user `on_task_fault` hook, and context-switch to
//!   the next ready task — genuine per-task fault containment, using only
//!   the scheduler's existing "resume an arbitrary sp" primitive.

use core::sync::atomic::{AtomicU8, Ordering};

/// What kind of fault occurred.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultKind {
    /// RISC-V instruction access fault (mcause 1).
    InstructionAccess,
    /// RISC-V load access fault (mcause 5).
    LoadAccess,
    /// RISC-V store access fault (mcause 7).
    StoreAccess,
    /// Cortex-M MemManage fault; payload is the CFSR.
    MemManage(u32),
    /// Detected by stack watermarking at a context switch.
    StackOverflow,
}

/// Everything needed to attribute and report a fault.
#[derive(Debug, Clone, Copy)]
pub struct FaultInfo {
    /// The preemptive task that was running, if any.
    pub task_id: Option<usize>,
    pub kind: FaultKind,
    /// Faulting address (mtval / MMFAR); 0 when not applicable.
    pub address: usize,
    /// PC at the fault (mepc / stacked PC); 0 when not applicable.
    pub pc: usize,
}

/// Fault handling policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultPolicy {
    /// Dump and reset (default).
    Panic,
    /// Mark faulted, poison held mutexes, hook, and switch to another task.
    IsolateTask,
}

const POLICY_PANIC: u8 = 0;
const POLICY_ISOLATE: u8 = 1;
static POLICY: AtomicU8 = AtomicU8::new(POLICY_PANIC);

/// User hook invoked (under `IsolateTask`) with the faulting task id and
/// the fault info, before the scheduler switches away.
pub type OnTaskFault = fn(usize, &FaultInfo);
static HOOK: core::sync::atomic::AtomicUsize = core::sync::atomic::AtomicUsize::new(0);

/// Set the fault policy.
pub fn set_policy(policy: FaultPolicy) {
    POLICY.store(
        match policy {
            FaultPolicy::Panic => POLICY_PANIC,
            FaultPolicy::IsolateTask => POLICY_ISOLATE,
        },
        Ordering::Release,
    );
}

/// Register the `IsolateTask` hook (called with the faulting task id).
pub fn set_on_task_fault(hook: OnTaskFault) {
    HOOK.store(hook as usize, Ordering::Release);
}

/// Handle a fault. Under [`FaultPolicy::Panic`] this diverges (dumps and
/// resets). Under [`FaultPolicy::IsolateTask`] it returns the stack pointer
/// the trap handler should resume — the next ready task's.
pub fn on_fault(info: &FaultInfo) -> usize {
    if POLICY.load(Ordering::Acquire) == POLICY_ISOLATE {
        isolate(info)
    } else {
        panic_policy(info)
    }
}

fn dump(info: &FaultInfo) {
    crate::arch::debug_print("\nRIVET FAULT: ");
    match info.kind {
        FaultKind::InstructionAccess => crate::arch::debug_print("instruction-access"),
        FaultKind::LoadAccess => crate::arch::debug_print("load-access"),
        FaultKind::StoreAccess => crate::arch::debug_print("store-access"),
        FaultKind::MemManage(_) => crate::arch::debug_print("memmanage"),
        FaultKind::StackOverflow => crate::arch::debug_print("stack-overflow"),
    }
    crate::arch::debug_print(" addr=0x");
    print_hex(info.address);
    crate::arch::debug_print(" pc=0x");
    print_hex(info.pc);
    if let Some(id) = info.task_id {
        crate::arch::debug_print(" task=");
        print_dec(id);
    }
    crate::arch::debug_print("\n");
}

fn panic_policy(info: &FaultInfo) -> ! {
    dump(info);
    // Dump stack watermarks for all tasks (helps right-size stacks).
    for (id, t) in crate::preempt::tcb::TASKS.iter().enumerate() {
        if t.used.load(Ordering::Acquire) {
            let base = t.stack_base.load(Ordering::Acquire);
            let size = t.stack_size.load(Ordering::Acquire);
            if base != 0 && size != 0 {
                // SAFETY: reading a static task stack is safe (kernel
                // context, the faulting task is frozen).
                let used = crate::preempt::stack_usage(unsafe {
                    core::slice::from_raw_parts(base as *const u8, size)
                });
                crate::arch::debug_print("  task ");
                print_dec(id);
                crate::arch::debug_print(" stack ");
                print_dec(used);
                crate::arch::debug_print("/");
                print_dec(size);
                crate::arch::debug_print("\n");
            }
        }
    }
    // Halt with a distinguishable code (reset is reserved for the
    // watchdog path, which the fault tests would otherwise loop on).
    crate::arch::exit_failure(0xFA)
}

fn isolate(info: &FaultInfo) -> usize {
    dump(info);

    // 1. Mark the faulting task Faulted (Blocked so the scheduler skips it)
    //    and free its slot's scheduling participation.
    // No task context to isolate — fall back to panic semantics.
    let faulting = info.task_id.unwrap_or_else(|| panic_policy(info));

    // 2. Poison every mutex the task held (plan.md §3.4: held list from
    //    Phase 2.3) and wake its waiters so they observe the poison.
    if let Some(t) = crate::preempt::tcb::get(faulting) {
        for slot in &t.held {
            let ptr = slot.ptr.load(Ordering::Acquire);
            if !ptr.is_null() {
                // SAFETY: the held list only ever contains live
                // `PriorityMutex` addresses registered by push_held.
                unsafe {
                    crate::preempt::mutex::poison_mutex(ptr);
                }
            }
        }
        t.set_state(faulting, crate::preempt::tcb::TaskState::Blocked);
    }

    // 3. User hook.
    let hook = HOOK.load(Ordering::Acquire);
    if hook != 0 {
        // SAFETY: the hook is a function pointer set via set_on_task_fault.
        unsafe {
            let f: OnTaskFault = core::mem::transmute(hook);
            f(faulting, info);
        }
    }

    // 4. Switch to the next ready task.
    match crate::preempt::sched::schedule() {
        Some(next) => {
            if let Some(nt) = crate::preempt::tcb::get(next) {
                nt.set_state(next, crate::preempt::tcb::TaskState::Running);
                crate::preempt::sched::set_current(next);
                crate::arch::on_switch_to(
                    nt.stack_base.load(Ordering::Acquire),
                    nt.stack_size.load(Ordering::Acquire),
                );
            }
            nt_sp(next)
        }
        None => panic_policy(info),
    }
}

fn nt_sp(id: usize) -> usize {
    crate::preempt::tcb::get(id)
        .map(|t| t.sp.load(Ordering::Acquire))
        .unwrap_or(0)
}

fn print_hex(mut n: usize) {
    let mut buf = [0u8; 8];
    for i in (0..8).rev() {
        let d = (n & 0xF) as u8;
        buf[i] = if d < 10 { b'0' + d } else { b'a' + d - 10 };
        n >>= 4;
    }
    if let Ok(s) = core::str::from_utf8(&buf) {
        crate::arch::debug_print(s);
    }
}

fn print_dec(mut n: usize) {
    if n == 0 {
        crate::arch::debug_print("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut out = [0u8; 10];
    for j in 0..i {
        out[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&out[..i]) {
        crate::arch::debug_print(s);
    }
}
