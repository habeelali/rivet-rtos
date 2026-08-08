//! Priority-aware async executor with single-stack task polling.
//!
//! All cooperative tasks share one system stack. The executor polls tasks
//! in priority order. When a task returns `Poll::Pending`, its state lives
//! in a static `TaskCell` referenced by the task's `TaskReg.user_data` —
//! no per-task stack allocation is needed.
//!
//! When all tasks are pending, the executor calls `arch::sleep()` to enter
//! a low-power state.

use core::task::Poll;

use crate::task::{iter_task_regs, TaskReg, TaskRegistry};
use crate::waker;

/// Identity of the task currently being polled, encoded as
/// `(priority << 8) | index`. `u32::MAX` means "not currently polling a task".
/// Set by the executor immediately before calling a task's `poll_fn` and
/// read by sync primitives (`Semaphore`, `Channel`, `Sleep`) so they know
/// which task to register as a waiter, without needing the user to pass
/// priority/index manually.
#[cfg(not(loom))]
static CURRENT_TASK: crate::sync::atomic::AtomicU32 = crate::sync::atomic::AtomicU32::new(u32::MAX);
#[cfg(loom)]
loom::lazy_static! {
    static ref CURRENT_TASK: crate::sync::atomic::AtomicU32 = crate::sync::atomic::AtomicU32::new(u32::MAX);
}

fn set_current(id: crate::task::TaskId) {
    CURRENT_TASK.store(id.as_u16() as u32, crate::sync::atomic::Ordering::Release);
}

fn clear_current() {
    CURRENT_TASK.store(u32::MAX, crate::sync::atomic::Ordering::Release);
}

/// Test-only: simulate being inside a task's poll, so `Semaphore::acquire()`,
/// `Channel::send()/recv()`, and `Sleep` can be exercised directly from host
/// tests without spinning up the full executor loop.
#[doc(hidden)]
pub fn set_current_for_test(priority: u8, index: u8) {
    set_current(crate::task::TaskId::new(priority, index));
}

/// Test-only: clear the simulated task context set by [`set_current_for_test`].
#[doc(hidden)]
pub fn clear_current_for_test() {
    clear_current();
}

/// The identity of the task currently being polled. `None` if called
/// outside of a task's poll (e.g. from an ISR or before the executor starts).
///
/// Used by `Semaphore::acquire()`, `Channel::send()/recv()`, and `Sleep`
/// to register themselves as waiters without requiring the caller to pass
/// `(priority, index)` explicitly.
pub fn current_task() -> Option<crate::task::TaskId> {
    let v = CURRENT_TASK.load(crate::sync::atomic::Ordering::Acquire);
    if v == u32::MAX {
        None
    } else {
        Some(crate::task::TaskId::from_u16(v as u16))
    }
}

/// The global executor singleton.
pub struct Executor {
    registry: TaskRegistry,
    /// Number of tasks not yet completed (plan.md [B10]): decremented the
    /// first time a task's poll returns `Ready`. Used to skip re-polling
    /// completed tasks and to know when the cooperative tier is idle.
    live_tasks: core::sync::atomic::AtomicUsize,
}

impl Executor {
    pub const fn new() -> Self {
        Self {
            registry: TaskRegistry::new(),
            live_tasks: core::sync::atomic::AtomicUsize::new(0),
        }
    }

    /// Discover tasks from the `.rivet_tasks` linker section.
    /// Assigns per-priority indices. Must be called once before `run()`.
    pub fn init(&mut self) {
        let mut counts: [u8; 32] = [0; 32];

        for reg in iter_task_regs() {
            let prio = reg.priority as usize;
            if prio > (crate::task::MAX_PRIORITY as usize) {
                panic!(
                    "rivet: task priority {} exceeds MAX_PRIORITY {} \
                     (check #[rivet::task(priority = ...)])",
                    reg.priority,
                    crate::task::MAX_PRIORITY
                );
            }
            let idx = counts[prio] as usize;
            if idx >= crate::task::MAX_TASKS {
                // plan.md [B12]: overflow must be loud, not a silent drop.
                panic!(
                    "rivet: too many #[rivet::task]s at priority {} \
                     (limit MAX_TASKS = {} per priority)",
                    reg.priority,
                    crate::task::MAX_TASKS
                );
            }

            self.registry.tasks[prio][idx] = Some(reg as *const TaskReg);
            counts[prio] += 1;
            self.registry.total += 1;
            self.live_tasks
                .fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        }

        self.registry.count_per_priority[..=(crate::task::MAX_PRIORITY as usize)]
            .copy_from_slice(&counts[..=(crate::task::MAX_PRIORITY as usize)]);

        // Mark all tasks as initially ready so the executor polls them once
        // to start their async state machines running.
        for (p, &count) in counts[..=(crate::task::MAX_PRIORITY as usize)]
            .iter()
            .enumerate()
        {
            for i in 0..(count as usize) {
                crate::waker::mark_ready(crate::task::TaskId::new(p as u8, i as u8));
            }
        }
    }

    /// Main executor loop. Never returns.
    pub fn run(&self) -> ! {
        loop {
            waker::clear_pend();

            // Poll all ready tasks, highest priority first.
            while let Some(id) = waker::next_ready() {
                let reg = match self.lookup_task(id.priority(), id.index()) {
                    Some(r) => r,
                    None => continue,
                };

                // Skip completed tasks woken by a stale registration
                // (plan.md [B10]).
                // SAFETY: `reg.completed_fn` was paired with this task's
                // `TaskCell` by `#[rivet::task]`/`register_task!`.
                unsafe {
                    if (reg.completed_fn)(reg.user_data) {
                        continue;
                    }
                }

                let task_waker = waker::task_waker(id);

                set_current(id);
                // SAFETY: `reg.poll_fn` is a type-erased poll function
                // paired with `reg.user_data` (a `TaskCell` pointer) at
                // registration time by `#[rivet::task]`/`register_task!`.
                // The executor only ever polls a task that was registered
                // with a matching (poll_fn, user_data) pair, and never
                // re-enters a task while it's being polled.
                let result = unsafe { (reg.poll_fn)(reg.user_data, &task_waker) };
                clear_current();

                if result == Poll::Ready(()) {
                    self.live_tasks
                        .fetch_sub(1, core::sync::atomic::Ordering::Relaxed);
                }
            }

            // All tasks pending — sleep until an interrupt wakes us.
            if !waker::has_pending() {
                crate::arch::sleep();
            }
        }
    }

    fn lookup_task(&self, priority: u8, index: u8) -> Option<&'static TaskReg> {
        let p = priority as usize;
        let i = index as usize;
        if p > (crate::task::MAX_PRIORITY as usize) || i >= crate::task::MAX_TASKS {
            return None;
        }
        // SAFETY: the pointer came from the linker-section walk in `init()`
        // and points at a `static TaskReg` that lives for the program's
        // lifetime; the registry is written once at boot and only read
        // afterwards.
        self.registry.tasks[p][i].map(|ptr| unsafe { &*ptr })
    }
}

/// Test-only: reset the simulated-task-context global. Part of the global
/// reset done by [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    clear_current();
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

/// Global executor singleton.
pub static mut EXECUTOR: Executor = Executor::new();
