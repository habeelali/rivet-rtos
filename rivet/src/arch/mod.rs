//! Architecture abstraction layer. Uses target arch for code selection,
//! falling back to a dummy implementation for host-side testing.

#[cfg(target_arch = "riscv32")]
#[path = "riscv.rs"]
mod imp;

#[cfg(target_arch = "arm")]
#[path = "cortex_m.rs"]
mod imp;

#[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
#[path = "dummy.rs"]
mod imp;

/// Minimum byte size a preemptive task stack must have (plan.md §2.7):
/// the context-switch frame plus slack for the entry trampoline. Set
/// per-arch; `preempt::spawn` debug-asserts against it.
pub const MIN_TASK_STACK: usize = imp::MIN_TASK_STACK;

pub fn sleep() {
    imp::sleep();
}
pub fn pend_executor() {
    imp::pend_executor();
}
pub fn early_init() {
    imp::early_init();
}
pub fn now_micros() -> u64 {
    imp::now_micros()
}
pub fn debug_print(s: &str) {
    imp::debug_print(s);
}

/// Print a 32-bit value as 8 lowercase hex digits (diagnostics).
pub fn debug_print_hex32(n: u32) {
    imp::debug_print_hex32(n);
}
pub fn exit_success() -> ! {
    imp::exit_success();
}

/// Exit with a distinguishable failure code. Target-specific: RISC-V uses
/// `riscv.sifive.test` (`0x3333 | code << 16` → QEMU exit(code)); Cortex-M
/// prints a marker and halts (QEMU ARM semihosting has no simple failure
/// code path — the harness asserts on UART output instead).
pub fn exit_failure(code: u32) -> ! {
    imp::exit_failure(code);
}

/// Trigger a system reset (watchdog / fault-policy recovery). RISC-V:
/// `riscv.sifive.test` 0x7777; Cortex-M: SCB AIRCR SYSRESETREQ.
pub fn system_reset() -> ! {
    imp::system_reset();
}

/// Called on every actual context switch with the newly-dispatched task's
/// stack range. Cortex-M reprograms MPU region 7 to cover exactly the
/// running task's stack (plan.md §3.1); RISC-V PMP is boot-time-static so
/// this is a no-op there; the dummy backend is a no-op.
pub fn on_switch_to(stack_base: usize, stack_size: usize) {
    imp::on_switch_to(stack_base, stack_size);
}

/// Temporarily allow kernel access to a stack range (used while filling /
/// initializing a newly allocated stack inside the guarded pool, plan.md
/// §3.1). No-op on arches without per-range memory guards.
pub fn mpu_allow_scratch(base: usize, size: usize) {
    imp::mpu_allow_scratch(base, size);
}

/// Close the scratch window opened by [`mpu_allow_scratch`].
pub fn mpu_clear_scratch() {
    imp::mpu_clear_scratch();
}

/// Register a locked PMP guard band for a task stack (RISC-V only, plan.md
/// §3.2); no-op on other arches. `entry` is the PMP entry index (0-14).
pub fn pmp_register_guard(guard_base: usize, entry: usize) {
    imp::pmp_register_guard(guard_base, entry);
}

/// Test-only (dummy arch): reset the fake host clock. Part of the global
/// reset done by [`crate::kernel_test!`].
#[cfg(all(
    feature = "test-support",
    not(any(target_arch = "riscv32", target_arch = "arm"))
))]
pub fn reset_test_clock() {
    imp::reset_test_clock();
}

// ── Preemptive tier support ─────────────────────────────────────────

/// Build the initial stack frame for a new preemptive task so that the
/// first context switch into it starts execution at `entry_fn(arg)`.
///
/// `entry_fn` and `arg` are type-erased to `usize` here (the caller,
/// [`crate::preempt::spawn`], is the generic/typed layer); the arch-specific
/// implementation only needs to place them where its context-switch
/// convention expects an entry point and first argument.
///
/// # Safety
/// `stack` must be suitably aligned (16 bytes) and large enough for at
/// least one context-switch frame; `entry_fn` must be a valid function
/// pointer taking one `usize`-sized argument and never returning.
pub unsafe fn init_task_stack(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    imp::init_task_stack(stack, entry_fn, arg)
}

/// Transfer control to the first preemptive task. Never returns.
///
/// # Safety
/// `sp` must be a stack pointer previously produced by [`init_task_stack`].
pub unsafe fn start_first_task(sp: usize) -> ! {
    imp::start_first_task(sp)
}

/// Request an immediate reschedule opportunity — e.g. a task voluntarily
/// giving up the CPU, or a mutex unlock waking a higher-priority waiter
/// that should preempt right away rather than waiting for the next tick.
///
/// Triggers the *same* interrupt/trap path used by tick-driven preemption
/// (software interrupt on RISC-V, PendSV on Cortex-M) — there is exactly
/// one context-switch code path, not two. Safe to call from task or ISR
/// context.
pub fn yield_now() {
    imp::yield_now();
}

/// Arch-specific sub-modules for ISR handlers.
///
/// Neither arch exposes a `pendsv_handler`/equivalent here: on Cortex-M,
/// `PendSV` is defined directly in hand-written assembly (`global_asm!` in
/// `arch::cortex_m`) matching the vector table entry — it must precisely
/// control register save/restore around the stack switch, which a normal
/// Rust function's compiler-generated prologue can't guarantee. RISC-V's
/// timer tick is similarly handled entirely inside its own hand-written
/// trap entry/dispatch, for the same reason. `systick_handler` remains a
/// normal callable function since SysTick only *requests* a reschedule
/// (via PendSV) rather than performing one itself.
#[cfg(target_arch = "arm")]
pub mod cortex_m {
    pub fn systick_handler() {
        super::imp::systick_handler();
    }
    pub fn systick_init(sysclk_hz: u32) {
        super::imp::systick_init(sysclk_hz);
    }
    pub fn systick_enable() {
        super::imp::systick_enable();
    }
    pub fn systick_reload(ticks: u32) {
        super::imp::systick_reload(ticks);
    }
    pub fn systick_seed_ticks(v: u32) {
        super::imp::systick_seed_ticks(v);
    }
}
