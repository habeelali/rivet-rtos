//! Preemptive tier: tasks with dedicated stacks, real priority preemption.
//!
//! Unlike `#[rivet::task]` (cooperative — only yields at `.await`), a
//! preemptive task can be suspended by the timer tick at *any* point and
//! resumed later from exactly there, because its full execution context
//! (registers + program counter + stack pointer) is saved/restored on
//! every switch. This is what lets a genuinely higher-priority task
//! interrupt a lower-priority one that never calls anything cooperative.
//!
//! All switching — tick-driven preemption *and* voluntary yields (blocking
//! on a mutex, explicit `port::arch::request_reschedule()`) — goes through the same
//! interrupt/trap path (software interrupt on RISC-V, PendSV on Cortex-M).
//! There's no separate "synchronous context switch function call" — the
//! arch trap handler saves the interrupted task's full context, asks
//! [`on_tick`] which task to resume, and returns to that task's saved
//! context. This matches how real embedded RTOS ports (FreeRTOS, etc.)
//! implement it, and keeps there being exactly one code path that has to
//! be correct instead of two.
//!
//! The cooperative async executor still exists — it runs as an ordinary
//! preemptive task at the lowest priority (see [`crate::init`]), so any
//! real preemptive task immediately preempts it, and it fills otherwise-idle
//! CPU time with async work.

pub mod lifecycle;
pub mod mutex;
pub mod sched;
pub mod stack_pool;
pub mod tcb;

pub use mutex::{PriorityMutex, PriorityMutexGuard};
pub use tcb::TaskState;

use crate::sync::atomic::Ordering;

/// Statically-sized, correctly-aligned stack storage for a preemptive task.
#[repr(C, align(16))]
pub struct Stack<const SIZE: usize>(pub [u8; SIZE]);

impl<const SIZE: usize> Stack<SIZE> {
    pub const fn new() -> Self {
        Self([0; SIZE])
    }
}

impl<const SIZE: usize> Default for Stack<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

/// Implementation helper for [`macro@spawn_ptask`]: allocate the stack from
/// the pool (or use the provided fallback on host builds) and spawn.
#[doc(hidden)]
pub fn spawn_ptask_impl<T: 'static + Send, A: 'static, F: Fn() -> &'static mut [u8]>(
    stack_size: usize,
    priority: u8,
    entry: fn(&'static A) -> T,
    arg: &'static A,
    #[allow(unused_variables)] // embedded: the pool is authoritative
    fallback: F,
) -> Result<TaskHandle, SpawnError> {
    let stack = match crate::preempt::stack_pool::alloc_stack(stack_size) {
        Some(s) => s,
        None => {
            // On a real board the pool is authoritative: a fallback stack
            // outside `.task_stacks` would silently bypass the MPU/PMP
            // guards (plan.md §4.3). The host test backend (no
            // linker-provided pool at all) uses the per-invocation static
            // fallback instead.
            #[cfg(not(feature = "host-port"))]
            return Err(SpawnError::StackPoolFull);
            #[cfg(feature = "host-port")]
            fallback()
        }
    };
    // SAFETY: the pool slice is exclusively owned by the new task for its
    // lifetime; the fallback slice has the same contract (see the macro).
    unsafe { spawn(stack, priority, entry, arg) }
}

/// Handle to a spawned preemptive task: the registry slot id plus the
/// slot's generation counter, so stale handles can be detected after the
/// slot was recycled (plan.md §4.3 / §5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskHandle {
    pub id: u16,
    pub generation: u32,
}

pub use lifecycle::JoinError;

impl TaskHandle {
    /// Whether the handle still refers to the same task (slot not recycled).
    pub fn is_valid(&self) -> bool {
        tcb::get(self.id as usize)
            .map(|t| t.generation.load(Ordering::Acquire) == self.generation)
            .unwrap_or(false)
    }

    /// Ask the task to stop cooperatively (plan.md §5.4): sets the
    /// stop-requested flag, which the task polls via
    /// [`lifecycle::should_stop`]. Returns false if the handle is stale.
    pub fn request_stop(&self) -> bool {
        match tcb::get(self.id as usize) {
            Some(t) if t.generation.load(Ordering::Acquire) == self.generation => {
                t.stop_requested.store(true, Ordering::Release);
                true
            }
            _ => false,
        }
    }

    /// Block until the task's entry returns, then recover its result
    /// (plan.md §5.2/§5.3). See [`JoinError`].
    pub fn join<T: 'static + Send>(&self) -> Result<T, JoinError> {
        lifecycle::join_task::<T>(self)
    }

    /// Configure this task's period (plan.md Phase 11): the task itself
    /// calls [`crate::deadlines::wait_period`] once per iteration to block
    /// until the next boundary. `0` disables periodic waiting. No-op on a
    /// stale handle.
    pub fn set_period_us(&self, period_us: u32) {
        if self.is_valid() {
            crate::deadlines::set_period_us(self.id as usize, period_us);
        }
    }

    /// Configure this task's per-period CPU budget in microseconds
    /// (plan.md Phase 11, estimated via
    /// [`crate::exec_time::estimate_us_from_cycles`]). Exceeding it inside
    /// one period raises [`crate::fault::FaultKind::BudgetExceeded`]
    /// through the normal fault policy. `0` disables enforcement. No-op on
    /// a stale handle. Meaningless without also calling
    /// [`Self::set_period_us`] — the budget window resets at each period
    /// boundary.
    pub fn set_budget_us(&self, budget_us: u32) {
        if self.is_valid() {
            crate::deadlines::set_budget_us(self.id as usize, budget_us);
        }
    }

    /// Release the task's slot and stack for reuse (plan.md §5.4). The
    /// task must have exited (`join` returned) or be a task other than the
    /// current one that is blocked/suspended — despawning a *running*
    /// task is rejected. Returns false for a stale handle.
    pub fn despawn(&self) -> bool {
        let Some(t) = tcb::get(self.id as usize) else {
            return false;
        };
        if t.generation.load(Ordering::Acquire) != self.generation {
            return false;
        }
        if sched::current() == Some(self.id as usize) {
            return false; // cannot despawn the running task
        }
        if !t.used.load(Ordering::Acquire) {
            return false;
        }
        crate::preempt::sched::ready_remove(self.id as usize);

        // Drop any stored result, then reset the slot (state READY so the
        // next `register` claim CAS can succeed; `used=false` publishes).
        let drop_fn = t.result_drop.load(Ordering::Acquire);
        if drop_fn != 0 {
            // SAFETY: `drop_fn` was registered by `spawn` as
            // `drop_in_place_erased::<T> as *const () as usize` for the
            // exact T stored in the result buffer.
            let f: fn(*mut u8) = unsafe { core::mem::transmute(drop_fn) };
            f(t.result_buf.get() as *mut u8);
            t.result_drop.store(0, Ordering::Release);
        }
        t.state.store(tcb::READY, Ordering::Release);
        t.exited.store(false, Ordering::Release);
        t.result_size.store(0, Ordering::Release);
        t.stop_requested.store(false, Ordering::Release);
        t.used.store(false, Ordering::Release);

        // Release the stack back to the pool (refilled with 0xAA).
        let stack = t.stack_info();
        if let Some((base, size)) = stack {
            if base != 0 && size != 0 {
                // SAFETY: the pool slice was given to this task at spawn
                // and is now unused; `release_stack` refills and recycles it.
                let slice: &'static mut [u8] =
                    unsafe { core::slice::from_raw_parts_mut(base as *mut u8, size) };
                crate::preempt::stack_pool::release_stack(slice);
            }
        }
        t.stack_base.store(0, Ordering::Release);
        t.stack_size.store(0, Ordering::Release);
        true
    }

    /// Suspend the task (plan.md §5.5): a READY task moves to SUSPENDED and
    /// is skipped by the scheduler until [`TaskHandle::resume`]. Returns
    /// false for a stale handle or a task that isn't currently READY.
    pub fn pause(&self) -> bool {
        let Some(t) = tcb::get(self.id as usize) else {
            return false;
        };
        if t.generation.load(Ordering::Acquire) != self.generation {
            return false;
        }
        let was_ready = t
            .state
            .compare_exchange(
                tcb::READY,
                tcb::SUSPENDED,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok();
        if was_ready {
            crate::preempt::sched::ready_remove(self.id as usize);
        }
        was_ready
    }

    /// Resume a suspended task (plan.md §5.5). Returns false for a stale
    /// handle or a task that isn't SUSPENDED.
    pub fn resume(&self) -> bool {
        let Some(t) = tcb::get(self.id as usize) else {
            return false;
        };
        if t.generation.load(Ordering::Acquire) != self.generation {
            return false;
        }
        let was_suspended = t
            .state
            .compare_exchange(
                tcb::SUSPENDED,
                tcb::READY,
                Ordering::AcqRel,
                Ordering::Acquire,
            )
            .is_ok();
        if was_suspended {
            crate::preempt::sched::ready_add(self.id as usize);
        }
        was_suspended
    }
}

/// Errors from [`spawn`] / [`macro@spawn_ptask`] — fixed-size resources
/// degrade with a typed error, never a silent drop or panic (plan.md §4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpawnError {
    /// The task registry is full ([`tcb::MAX_PTASKS`] slots in use).
    RegistryFull,
    /// The task-stack pool is exhausted.
    StackPoolFull,
}

/// Spawn a preemptive task with its own stack.
///
/// `entry` receives `arg` (a `'static` reference — no heap allocation
/// needed since the argument just needs to outlive the task, and tasks
/// never exit). Returns the assigned task id, or `None` if the task
/// registry ([`tcb::MAX_PTASKS`]) is full.
///
/// # Safety
/// `stack` must not be shared with any other task and must remain valid
/// (i.e. `'static`) for as long as the task runs.
pub unsafe fn spawn<T: 'static + Send, A: 'static>(
    stack: &'static mut [u8],
    priority: u8,
    entry: fn(&'static A) -> T,
    arg: &'static A,
) -> Result<TaskHandle, SpawnError> {
    assert!(
        stack.len() >= crate::port::arch::min_task_stack(),
        "rivet: task stack too small: {} bytes < arch minimum {} (context-switch frame + entry trampoline; plan.md §2.7)",
        stack.len(),
        crate::port::arch::min_task_stack()
    );
    // Fill with a known pattern so Phase 3's `stack_usage()` can measure
    // the high-water mark (the deepest untouched 0xAA byte marks how far
    // the task actually ran down its stack).
    let base = stack.as_ptr() as usize;
    let size = stack.len();
    // The stack lives in the MPU-denied task-stack pool (plan.md §3.1);
    // open a scratch window so the kernel can fill and initialize it
    // before the task ever runs (no-op on arches without per-range
    // guards). Held under a critical section so no other task can run in
    // the window.
    let sp = crate::critical::enter(|| {
        crate::port::arch::scratch_open(base, size);
        stack.fill(0xAA);
        let sp =
            crate::port::arch::init_task_stack(stack, entry as usize, arg as *const A as usize);
        crate::port::arch::scratch_close();
        sp
    });
    // plan.md Phase 17 (found via soak testing, same investigation as the
    // `joiner`-reset fix in `tcb::register_full`): `register_full` calls
    // `sched::ready_add`, making the new task immediately dispatchable —
    // if it's higher priority than the spawning task, a tick can dispatch
    // and run it to completion (`rivet_task_exit_core`) *before* this
    // function ever reaches the `result_size`/`result_drop` stores below.
    // `rivet_task_exit_core` then sees `result_size == 0`, so `size > 0`
    // is false and the return value is silently never written (and a
    // droppable `T` leaks, since `result_drop` is also still 0). Wrapping
    // registration and the metadata stores in one critical section closes
    // the window: nothing can dispatch the new task until both are
    // published together.
    let registered = crate::critical::enter(|| {
        let id = tcb::register_full(sp, priority, base, size)?;
        let t = tcb::get(id).expect("just registered");
        let sz = core::mem::size_of::<T>();
        debug_assert!(
            sz <= 8,
            "rivet: task return values > 8 bytes are not supported (got {sz})"
        );
        t.result_size.store(sz as u8, Ordering::Release);
        t.result_drop.store(
            if core::mem::needs_drop::<T>() {
                // Cast through a function pointer: `fn` → usize (clippy
                // fn_to_numeric_cast).
                drop_in_place_erased::<T> as *const () as usize
            } else {
                0
            },
            Ordering::Release,
        );
        Some((id, t.generation.load(Ordering::Acquire)))
    });
    match registered {
        Some((id, generation)) => Ok(TaskHandle {
            id: id as u16,
            generation,
        }),
        None => Err(SpawnError::RegistryFull),
    }
}

/// Type-erased `drop_in_place` for a stored task result.
fn drop_in_place_erased<T>(ptr: *mut u8) {
    // SAFETY: the caller guarantees `ptr` points at a live `T`.
    unsafe {
        core::ptr::drop_in_place(ptr as *mut T);
    }
}

/// Declare and spawn a preemptive task in one step.
///
/// ```ignore
/// static CONFIG: MyConfig = MyConfig { ... };
///
/// rivet::spawn_ptask!(stack = 2048, priority = 3, entry = my_task, arg = CONFIG);
///
/// fn my_task(cfg: &'static MyConfig) -> ! {
///     loop { /* runs with real preemption, own stack */ }
/// }
/// ```
#[macro_export]
macro_rules! spawn_ptask {
    (stack = $stack_size:expr, priority = $prio:expr, entry = $entry:expr, arg = $arg:expr) => {{
        // The stack comes from the kernel's task-stack pool (plan.md §3) so
        // MPU/PMP guards can isolate it. On host builds (no pool) fall back
        // to a per-invocation static.
        $crate::preempt::spawn_ptask_impl($stack_size, $prio, $entry, &$arg, || {
            static mut __RIVET_PTASK_STACK: $crate::preempt::Stack<$stack_size> =
                $crate::preempt::Stack::new();
            // SAFETY: this fallback stack is used exactly once, for the
            // lifetime of the task (host builds only).
            #[allow(static_mut_refs)]
            unsafe {
                &mut __RIVET_PTASK_STACK.0
            }
        })
    }};
}

/// Measure a task stack's high-water mark (plan.md §2.7/Phase 3): stacks
/// are filled with `0xAA` at spawn; the deepest byte a task wrote
/// (anything else) marks how far it ran down. Returns the number of bytes
/// used from the top.
pub fn stack_usage(stack: &[u8]) -> usize {
    let used = stack.iter().take_while(|&&b| b == 0xAA).count();
    stack.len().saturating_sub(used)
}

/// Start the preemptive scheduler. Never returns — control transfers
/// permanently to whichever task the scheduler selects first (and from
/// there, forever between tasks via interrupt-driven context switches).
///
/// # Panics
/// Panics if no preemptive tasks have been spawned.
pub fn start() -> ! {
    let first = sched::schedule().expect(
        "rivet::preempt::start(): no preemptive tasks spawned (call rivet::init() first, \
         which spawns the async idle task, or spawn at least one via spawn_ptask!)",
    );
    sched::set_current(first);
    if let Some(t) = tcb::get(first) {
        t.set_state(first, TaskState::Running);
    }
    // First dispatch: advance the RR start past this task (plan.md [B14]),
    // and let the arch layer enable memory protection for its stack.
    sched::on_dispatch(first);
    crate::exec_time::on_first_dispatch();
    let first_tcb = tcb::get(first).unwrap();
    crate::port::arch::on_switch_to(
        first_tcb.stack_base.load(Ordering::Acquire),
        first_tcb.stack_size.load(Ordering::Acquire),
    );
    let sp = first_tcb.sp.load(Ordering::Acquire);
    // SAFETY: `sp` is the freshly-initialized first stack frame of the
    // selected task (produced by `init_task_stack`); `start_first_task`
    // consumes it exactly once and never returns.
    unsafe { crate::port::arch::start_first_task(sp) }
}

/// Permanently remove the current preemptive task from scheduling. Useful
/// for a task that does bounded work and then has nothing left to do —
/// parking (rather than spinning forever at its original priority) lets
/// lower-priority tasks actually run.
///
/// # Panics
/// Panics if called outside of a preemptive task context.
/// Block the current preemptive task for `ms` milliseconds (plan.md §5.6):
/// registers a deadline in the per-task queue, blocks, and lets the timer
/// tick wake it. No-op outside a preemptive task.
pub fn sleep_ms(ms: u64) {
    let deadline = crate::port::board::now_us().wrapping_add(ms.saturating_mul(1000));
    sleep_until(deadline);
}

/// Block the calling preemptive task until the absolute time `deadline_us`
/// (plan.md §5.6 / Phase 11). `sleep_ms` is `sleep_until(now + ms*1000)`;
/// [`crate::deadlines::wait_period`] uses this directly with a
/// drift-corrected deadline so periodic jitter doesn't accumulate. No-op
/// outside a preemptive task context. If `deadline_us` has already
/// passed, still yields once (bounded, not a busy spin) rather than
/// returning immediately.
pub fn sleep_until(deadline_us: u64) {
    let Some(me) = sched::current() else {
        return;
    };
    sched::block_current();
    let _ = crate::timer::register_ptask_deadline(deadline_us, me);
    crate::port::arch::request_reschedule();
    crate::timer::cancel_ptask_deadline(me);
}

pub fn park_forever() -> ! {
    sched::current().expect("park_forever() outside preemptive task context");
    sched::block_current();
    loop {
        crate::port::arch::request_reschedule();
    }
}

/// Called from the arch trap/exception handler (timer tick, or a software
/// interrupt triggered by [`crate::port::arch::request_reschedule`]) with the interrupted
/// task's just-saved stack pointer. Consults the scheduler and returns the
/// stack pointer the arch layer should actually resume — either the same
/// one (no reschedule needed) or a different task's (real preemption /
/// voluntary switch).
///
/// If the preemptive tier hasn't started yet ([`start`] not called),
/// returns `interrupted_sp` unchanged.
pub fn on_tick(interrupted_sp: usize) -> usize {
    #[cfg(feature = "latency-histograms")]
    let __latency_start = crate::port::arch::cycle_count();
    let result = on_tick_impl(interrupted_sp);
    #[cfg(feature = "latency-histograms")]
    crate::latency::record(
        crate::latency::Kind::DispatchDecision,
        crate::port::arch::cycle_count().wrapping_sub(__latency_start),
    );
    result
}

fn on_tick_impl(interrupted_sp: usize) -> usize {
    let Some(running) = sched::current() else {
        return interrupted_sp;
    };

    if let Some(t) = tcb::get(running) {
        t.sp.store(interrupted_sp, Ordering::Release);
    }

    let Some(candidate) = sched::schedule() else {
        return interrupted_sp;
    };

    // Watermark overflow check (plan.md §3.3): the outgoing task's lowest
    // stack word must still be the 0xAA fill pattern. Catches overflow
    // that the MPU/PMP guards miss (RISC-V tasks beyond the PMP budget, or
    // a large stack array jumping over a small guard band).
    if let Some(t) = tcb::get(running) {
        let base = t.stack_base.load(Ordering::Acquire);
        let size = t.stack_size.load(Ordering::Acquire);
        if base != 0 && size >= 4 {
            // SAFETY: reading the running task's own stack is always
            // allowed (it is the MPU-enabled current stack).
            let lowest = unsafe { core::ptr::read_volatile(base as *const u32) };
            if lowest != 0xAAAA_AAAA {
                let info = crate::fault::FaultInfo {
                    task_id: Some(running),
                    kind: crate::fault::FaultKind::StackOverflow,
                    address: base,
                    pc: 0,
                };
                return crate::fault::on_fault(&info);
            }
        }
    }

    // CPU-budget check (plan.md Phase 11): only meaningful for a task
    // that's still actually Running (a task that just blocked itself, via
    // `wait_period`'s own `sleep_until`, resets its budget window on the
    // *next* period start, not here).
    if tcb::get(running).map(|t| t.state()) == Some(TaskState::Running)
        && crate::deadlines::check_budget(running)
    {
        let info = crate::fault::FaultInfo {
            task_id: Some(running),
            kind: crate::fault::FaultKind::BudgetExceeded,
            address: 0,
            pc: 0,
        };
        return crate::fault::on_fault(&info);
    }

    // A task that just blocked itself (e.g. park_forever(), or a
    // PriorityMutex wait) must be switched away from unconditionally —
    // should_preempt()'s priority comparison only makes sense between two
    // tasks that could both legitimately keep running. A Blocked task
    // can't "keep running" at all, regardless of whether the candidate's
    // priority is lower (e.g. falling through to the priority-0 async
    // idle task). Without this check, once every task at the blocked
    // task's priority level is also blocked, on_tick keeps returning
    // interrupted_sp forever — spinning inside the blocked task's own
    // park loop instead of ever handing off to the (lower-priority, but
    // only-ready) candidate.
    let running_blocked = tcb::get(running)
        .map(|t| t.state() == TaskState::Blocked)
        .unwrap_or(true);

    if !running_blocked && !sched::should_preempt(candidate, running) {
        return interrupted_sp;
    }
    if running_blocked && candidate == running {
        // Nothing else is ready; stay parked (spurious wake or no other work).
        return interrupted_sp;
    }

    if let Some(t) = tcb::get(running) {
        if t.state() == TaskState::Running {
            t.set_state(running, TaskState::Ready);
        }
    }
    crate::exec_time::on_switch(running);
    let to_tcb = tcb::get(candidate).unwrap();
    to_tcb.set_state(candidate, TaskState::Running);
    sched::set_current(candidate);
    // An actual switch occurred: advance the RR start past the dispatched
    // task (plan.md [B14] — never advance on no-switch ticks), and enable
    // memory protection for the newly-running task's stack (plan.md §3.1).
    sched::on_dispatch(candidate);
    crate::port::arch::on_switch_to(
        to_tcb.stack_base.load(Ordering::Acquire),
        to_tcb.stack_size.load(Ordering::Acquire),
    );
    to_tcb.sp.load(Ordering::Acquire)
}

#[cfg(test)]
mod stack_tests {
    use super::*;

    #[test]
    fn stack_usage_measures_fill_pattern() {
        let mut stack = [0xAAu8; 512];
        assert_eq!(stack_usage(&stack), 0, "untouched stack uses nothing");
        // The stack grows DOWN from the top: a task that ran 256 bytes
        // deep leaves the bottom 256 bytes as untouched 0xAA.
        stack[256..].fill(0x00);
        assert_eq!(stack_usage(&stack), 256);
    }

    #[test]
    #[should_panic(expected = "task stack too small")]
    fn spawn_rejects_too_small_stack() {
        crate::kernel_test! {
            static mut TINY: [u8; 32] = [0; 32];
            fn entry(_: &'static ()) -> ! { loop { crate::port::arch::request_reschedule(); } }
            static UNIT: () = ();
            // SAFETY: TINY is only used here, before the scheduler runs.
            unsafe {
                // SAFETY: TINY is only used here, before the scheduler
                // runs; addr_of_mut! avoids a reference to the static.
                let stack = &mut (*core::ptr::addr_of_mut!(TINY));
                let _ = spawn(stack, 1, entry, &UNIT);
            }
        }
    }
}
