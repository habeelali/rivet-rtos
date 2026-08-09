//! Group A of the port contract: the CPU port.
//!
//! Declares the symbols a `rivet-arch-*` crate must provide (as
//! `#[no_mangle] extern "Rust" fn`s) — context switch, trap/exception
//! entry, per-arch memory-protection programming, interrupt masking. The
//! kernel calls only the safe wrappers below; nothing here reaches into
//! any specific arch crate by name; linkage is by symbol name only; a
//! missing implementation is a link error naming the exact symbol, not a
//! type error.
//!
//! Signatures are restricted to primitives (`usize`, `u32`, `u64`, raw
//! pointers, `!`) specifically so the `extern "Rust"` ABI — otherwise
//! unspecified in general — is unambiguous between the crate that
//! declares a symbol and the crate that defines it (the same convention
//! the `critical-section` crate uses for its provider mechanism).

extern "Rust" {
    /// One-time arch bring-up: install the trap/exception vector, arm any
    /// boot-time-static memory guards (e.g. RISC-V's locked PMP
    /// catch-all), point the ISR stack register at its linker-provided
    /// top. Must not touch board-specific hardware (clocks, timers,
    /// console) — that is [`crate::port::board::init`]'s job, called
    /// separately.
    fn __rivet_arch_init();

    /// Enter a low-power wait for the next interrupt (`wfi` or
    /// equivalent). Called by the executor when every task is pending.
    fn __rivet_arch_idle();

    /// Request an immediate reschedule opportunity: the same trap/
    /// exception path used by tick-driven preemption (software interrupt
    /// on RISC-V, PendSV on Cortex-M) — there is exactly one context-
    /// switch code path, not two. Safe to call from task or ISR context.
    fn __rivet_arch_request_reschedule();

    /// Disable interrupts, returning an opaque token that
    /// [`__rivet_arch_irq_restore`] uses to decide whether to re-enable
    /// them (nested critical sections must compose: an inner disable is a
    /// no-op if interrupts were already off, and only the outermost
    /// restore actually re-enables).
    fn __rivet_arch_irq_save() -> usize;
    fn __rivet_arch_irq_restore(token: usize);

    /// Build the initial stack frame for a new preemptive task so that the
    /// first context switch into it starts execution at `entry_fn(arg)`.
    /// Returns the initial stack pointer.
    ///
    /// # Safety
    /// `stack_ptr`/`stack_len` must describe a suitably aligned region at
    /// least [`__rivet_arch_min_task_stack`] bytes long; `entry_fn` must be
    /// a valid function pointer taking one `usize` argument and never
    /// returning.
    fn __rivet_arch_init_task_stack(
        stack_ptr: *mut u8,
        stack_len: usize,
        entry_fn: usize,
        arg: usize,
    ) -> usize;

    /// Transfer control to the first preemptive task. Never returns.
    ///
    /// # Safety
    /// `sp` must be a stack pointer previously produced by
    /// [`__rivet_arch_init_task_stack`].
    fn __rivet_arch_start_first_task(sp: usize) -> !;

    /// Called on every actual context switch with the newly-dispatched
    /// task's stack range, so an arch with a reprogrammable MPU (Cortex-M)
    /// can grant exactly that range. No-op on arches whose memory guards
    /// are boot-time-static (RISC-V PMP).
    fn __rivet_arch_on_switch_to(stack_base: usize, stack_size: usize);

    /// Register a locked stack-overflow guard band for allocation `slot`
    /// (RISC-V PMP; no-op on arches — Cortex-M — whose two-region MPU
    /// design already gives full isolation without per-task entries).
    fn __rivet_arch_guard_register(guard_base: usize, slot: usize);

    /// Temporarily grant kernel access to a stack range inside an
    /// otherwise memory-guard-denied pool (used while filling/
    /// initializing a newly allocated stack). No-op on arches without a
    /// whole-pool deny region.
    fn __rivet_arch_scratch_open(base: usize, size: usize);
    /// Close the window opened by [`__rivet_arch_scratch_open`].
    fn __rivet_arch_scratch_close();

    /// Minimum byte size a preemptive task stack must have: the
    /// context-switch frame plus slack for the entry trampoline.
    fn __rivet_arch_min_task_stack() -> usize;

    /// Free-running cycle counter (plan.md Phase 10), used for
    /// execution-time accounting and latency histograms. Not required to
    /// start at zero, only to be monotonic (mod 2^64) and to advance at a
    /// fixed, arch-documented rate. Implementations without a hardware
    /// cycle counter may derive one from another monotonic source (e.g. a
    /// SysTick-driven tick count) rather than failing — callers only ever
    /// take deltas, so a coarser-than-ideal but still monotonic source is
    /// still correct, just less precise.
    fn __rivet_arch_cycle_count() -> u64;
}

pub fn init() {
    // SAFETY: implemented by exactly one `rivet-arch-*` crate linked into
    // the final binary; called once, before any task can run.
    unsafe { __rivet_arch_init() }
}

pub fn idle() {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_idle() }
}

pub fn request_reschedule() {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_request_reschedule() }
}

/// Run `f` with interrupts disabled. Nested calls compose: an inner call
/// observes interrupts already disabled and its restore is a no-op,
/// leaving the outermost call to actually re-enable.
#[inline]
pub fn critical_section<R>(f: impl FnOnce() -> R) -> R {
    // SAFETY: see `init`; save/restore is paired within this function.
    let token = unsafe { __rivet_arch_irq_save() };
    let r = f();
    // SAFETY: `token` came from the `__rivet_arch_irq_save` call directly
    // above, in this same function — a matched pair.
    unsafe { __rivet_arch_irq_restore(token) };
    r
}

/// # Safety
/// `stack` must be suitably aligned and at least [`min_task_stack`] bytes
/// long; `entry_fn` must be a valid function pointer taking one
/// `usize`-sized argument and never returning.
pub unsafe fn init_task_stack(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    // SAFETY: forwarded to the arch crate under the same contract.
    unsafe { __rivet_arch_init_task_stack(stack.as_mut_ptr(), stack.len(), entry_fn, arg) }
}

/// # Safety
/// `sp` must be a stack pointer previously produced by [`init_task_stack`].
pub unsafe fn start_first_task(sp: usize) -> ! {
    // SAFETY: forwarded to the arch crate under the same contract.
    unsafe { __rivet_arch_start_first_task(sp) }
}

pub fn on_switch_to(stack_base: usize, stack_size: usize) {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_on_switch_to(stack_base, stack_size) }
}

pub fn guard_register(guard_base: usize, slot: usize) {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_guard_register(guard_base, slot) }
}

pub fn scratch_open(base: usize, size: usize) {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_scratch_open(base, size) }
}

pub fn scratch_close() {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_scratch_close() }
}

pub fn min_task_stack() -> usize {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_min_task_stack() }
}

/// Read the free-running cycle counter. See
/// [`__rivet_arch_cycle_count`] for the monotonicity contract.
pub fn cycle_count() -> u64 {
    // SAFETY: see `init`.
    unsafe { __rivet_arch_cycle_count() }
}
