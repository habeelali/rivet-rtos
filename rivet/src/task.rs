//! Task types, generic future storage, and the task registry.
//!
//! Tasks are placed in the `.rivet_tasks` linker section for discovery
//! by the executor at startup. Each entry is a `TaskReg` — just the
//! metadata the executor needs to poll a task. The task's actual
//! `Future` state machine lives in a [`TaskCell`], sized generically
//! and monomorphized per concrete future type.
//!
//! # Why not `static TASK: TaskCell<F> = ...`?
//!
//! `F` is the compiler-generated, unnameable type of an `async fn`'s
//! `Future`. Stable Rust cannot name that type in a `static` declaration
//! (that requires `type_alias_impl_trait`, nightly-only). Instead,
//! [`TaskCell`] is generic over a `usize` **byte size**, which *is*
//! nameable (`TaskCell<512>`), and the actual read/write of the future
//! happens inside a generic method (`TaskCell::poll::<F>`) that the
//! compiler monomorphizes separately for each task's concrete `F`. This
//! gets real `async fn` tasks with static, zero-allocation storage on
//! 100% stable Rust.

use core::cell::UnsafeCell;
use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::task::{Context, Poll, Waker};

/// Maximum number of tasks per priority level (RIVET_MAX_COOP_TASKS).
pub const MAX_TASKS: usize = crate::config::MAX_TASKS;

// The waker bitmap is 32-wide per priority; a larger MAX_TASKS would
// silently truncate indices (plan.md [B12] — a capacity mismatch that
// used to be a runtime masking bug). Make it a compile error.
const _: () = assert!(MAX_TASKS <= 32);

/// The unified identity of a cooperative-tier task: `(priority, index)`
/// packed into one u16, used by the waker, executor, timer queue,
/// semaphore, and channel (plan.md [B12] — replaces two ad-hoc encodings
/// `(prio << 24) | index` and `(prio << 8) | index`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub struct TaskId(u16);

impl TaskId {
    pub const fn new(priority: u8, index: u8) -> Self {
        Self(((priority as u16) << 8) | (index as u16))
    }

    pub fn priority(self) -> u8 {
        (self.0 >> 8) as u8
    }

    pub fn index(self) -> u8 {
        (self.0 & 0xFF) as u8
    }

    /// Raw u16 encoding (used by [`crate::executor::current_task`]).
    pub const fn as_u16(self) -> u16 {
        self.0
    }

    pub const fn from_u16(v: u16) -> Self {
        Self(v)
    }
}

/// Maximum priority level (0 = lowest, 31 = highest).
pub const MAX_PRIORITY: u8 = (crate::config::PRIORITY_LEVELS - 1) as u8;

/// Default byte size reserved for a task's future state machine when the
/// user doesn't override it with `#[rivet::task(stack = N)]`.
pub const DEFAULT_TASK_SIZE: usize = 512;

/// Alignment guaranteed for future storage inside a [`TaskCell`].
/// Covers all primitive/usize/u64-aligned types used in typical embedded
/// futures. `TaskCell::poll` asserts this at runtime on first use.
pub const TASK_CELL_ALIGN: usize = 16;

/// Generic, zero-allocation storage for one task's `Future` state machine.
///
/// `SIZE` is a byte count, chosen by `#[rivet::task(stack = SIZE)]`
/// (default [`DEFAULT_TASK_SIZE`]). The concrete future type is supplied
/// only when polling, via a monomorphized generic method — this is what
/// lets the byte size (and therefore the `static` declaration) be nameable
/// without knowing the future's real type.
#[repr(C, align(16))]
pub struct TaskCell<const SIZE: usize> {
    buf: UnsafeCell<MaybeUninit<[u8; SIZE]>>,
    initialized: core::sync::atomic::AtomicBool,
    /// Set once the future has run to completion and been dropped.
    /// Prevents re-polling a completed task's dropped future (plan.md
    /// [B10]: a stale waiter registration must not poll a completed task).
    completed: core::sync::atomic::AtomicBool,
}

impl<const SIZE: usize> Default for TaskCell<SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

// Safety: TaskCell lives in static memory. It is only ever polled from the
// single-threaded executor loop, which never re-enters a task while it's
// already being polled, so &TaskCell access is effectively single-threaded.
unsafe impl<const SIZE: usize> Sync for TaskCell<SIZE> {}

impl<const SIZE: usize> TaskCell<SIZE> {
    /// Create empty task storage.
    pub const fn new() -> Self {
        Self {
            buf: UnsafeCell::new(MaybeUninit::uninit()),
            initialized: core::sync::atomic::AtomicBool::new(false),
            completed: core::sync::atomic::AtomicBool::new(false),
        }
    }

    /// Whether this task's future has completed (and been dropped).
    /// The executor uses this to skip completed tasks and track the
    /// live-task count (plan.md [B10]).
    pub fn is_completed(&self) -> bool {
        self.completed.load(core::sync::atomic::Ordering::Acquire)
    }

    /// Poll the task's future, creating it on first call via `init`.
    ///
    /// `init` is the task's async fn, used as a zero-sized `fn() -> F`
    /// (so tasks currently take no arguments — shared state goes through
    /// `static`s, which is also the idiomatic embedded pattern for
    /// peripherals and shared queues).
    ///
    /// # Panics
    /// Panics if `F` doesn't fit in `SIZE` bytes or needs stricter
    /// alignment than [`TASK_CELL_ALIGN`]. Increase
    /// `#[rivet::task(stack = N)]` if this fires.
    ///
    /// # Safety
    /// Must only ever be called with the *same* `F` on every invocation
    /// for a given `TaskCell` (true by construction: the proc macro
    /// generates one non-generic wrapper per task that always calls this
    /// with the same `init` function).
    pub unsafe fn poll<F: Future<Output = ()> + 'static>(
        &self,
        init: fn() -> F,
        waker: &Waker,
    ) -> Poll<()> {
        assert!(
            core::mem::size_of::<F>() <= SIZE,
            "rivet: task future ({} bytes) exceeds reserved stack size ({} bytes); \
             increase #[rivet::task(stack = N)]",
            core::mem::size_of::<F>(),
            SIZE
        );
        assert!(
            core::mem::align_of::<F>() <= TASK_CELL_ALIGN,
            "rivet: task future requires stricter alignment than supported"
        );

        let ptr = self.buf.get() as *mut u8;

        if !self.initialized.load(core::sync::atomic::Ordering::Acquire) {
            let future = init();
            core::ptr::write(ptr as *mut F, future);
            self.initialized
                .store(true, core::sync::atomic::Ordering::Release);
        }

        // A completed task is never re-polled — its future has been
        // dropped (plan.md [B10]).
        if self.completed.load(core::sync::atomic::Ordering::Acquire) {
            return Poll::Ready(());
        }

        let fut: &mut F = &mut *(ptr as *mut F);
        let pinned = Pin::new_unchecked(fut);
        let mut cx = Context::from_waker(waker);
        let result = pinned.poll(&mut cx);

        if result.is_ready() {
            // Drop the future in place and mark the cell completed so a
            // stale wake can never poll the dropped state machine.
            // SAFETY: the future at `ptr` is initialized and we hold the
            // only reference; dropping it exactly once is sound.
            unsafe {
                core::ptr::drop_in_place(ptr as *mut F);
            }
            self.completed
                .store(true, core::sync::atomic::Ordering::Release);
        }
        result
    }
}

/// Registration entry placed in the `.rivet_tasks` linker section.
/// The executor walks these to discover all statically-declared tasks.
#[repr(C)]
pub struct TaskReg {
    /// Task priority (0-31).
    pub priority: u8,
    /// Index within this priority level (assigned at init time).
    pub index_in_priority: u8,
    /// Reserved padding for alignment.
    pub _reserved: [u8; 2],
    /// Poll function. Type-erased: internally casts `user_data` back to
    /// the concrete `TaskCell<SIZE>` and calls `TaskCell::poll::<F>`.
    pub poll_fn: unsafe fn(user_data: *mut (), waker: &Waker) -> Poll<()>,
    /// Completed probe. Type-erased: internally casts `user_data` back to
    /// the concrete `TaskCell<SIZE>` and reports `is_completed()` — lets
    /// the executor skip completed tasks (plan.md [B10]).
    pub completed_fn: unsafe fn(user_data: *mut ()) -> bool,
    /// Opaque pointer to the task's `TaskCell`. Set at compile time.
    pub user_data: *mut (),
}

// Safety: TaskReg is placed in static memory (linker section), accessed
// only by the executor loop (single-threaded).
unsafe impl Sync for TaskReg {}

/// Convenience macro to declare a task registration by hand (used
/// internally by `#[rivet::task]`, and available for advanced manual use).
///
/// ```ignore
/// rivet::register_task!(MY_TASK, priority = 1, poll_fn = my_poll, buf = MY_BUF);
/// ```
#[macro_export]
macro_rules! register_task {
    ($name:ident, priority = $prio:expr, poll_fn = $poll:expr, completed = $completed:expr, buf = $buf:expr) => {
        #[link_section = ".rivet_tasks"]
        #[used]
        static $name: $crate::task::TaskReg = $crate::task::TaskReg {
            priority: $prio,
            index_in_priority: 0,
            _reserved: [0; 2],
            poll_fn: $poll as unsafe fn(*mut (), &::core::task::Waker) -> ::core::task::Poll<()>,
            completed_fn: $completed as unsafe fn(*mut ()) -> bool,
            user_data: unsafe { &raw const $buf as *mut () },
        };
    };
}

/// Runtime task registry built during `Executor::init()`.
pub(crate) struct TaskRegistry {
    /// Per-priority array of pointers to TaskReg entries.
    pub tasks: [[Option<*const TaskReg>; MAX_TASKS]; (MAX_PRIORITY as usize) + 1],
    /// Number of tasks at each priority level.
    pub count_per_priority: [u8; (MAX_PRIORITY as usize) + 1],
    /// Total number of registered tasks.
    pub total: u8,
}

impl TaskRegistry {
    pub const fn new() -> Self {
        Self {
            tasks: [[None; MAX_TASKS]; (MAX_PRIORITY as usize) + 1],
            count_per_priority: [0; (MAX_PRIORITY as usize) + 1],
            total: 0,
        }
    }
}

// Symbols defined by the linker script.
extern "C" {
    static __rivet_tasks_start: u8;
    static __rivet_tasks_end: u8;
}

/// Iterate over all TaskReg entries in the `.rivet_tasks` section.
pub(crate) fn iter_task_regs() -> impl Iterator<Item = &'static TaskReg> {
    let start = core::ptr::addr_of!(__rivet_tasks_start) as *const TaskReg;
    let end = core::ptr::addr_of!(__rivet_tasks_end) as *const TaskReg;
    let count = unsafe {
        // SAFETY: `__rivet_tasks_start`/`__rivet_tasks_end` are linker
        // symbols bracketing the `.rivet_tasks` section; both point into
        // the same allocation, so `offset_from` is well-defined.
        end.offset_from(start)
    };
    let count = if count < 0 { 0 } else { count as usize };
    (0..count).map(move |i| unsafe {
        // SAFETY: `i` is bounded by `count`, the number of `TaskReg`
        // entries measured between the section symbols; each entry is a
        // `static` that lives for the program's lifetime.
        &*start.add(i)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU32, Ordering};

    static POLL_COUNT: AtomicU32 = AtomicU32::new(0);

    async fn counting_task() {
        loop {
            POLL_COUNT.fetch_add(1, Ordering::Relaxed);
            TestYield { yielded: false }.await;
        }
    }

    struct TestYield {
        yielded: bool,
    }
    impl Future for TestYield {
        type Output = ();
        fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if self.yielded {
                Poll::Ready(())
            } else {
                self.yielded = true;
                cx.waker().wake_by_ref();
                Poll::Pending
            }
        }
    }

    #[test]
    fn task_cell_polls_real_async_fn() {
        crate::kernel_test! {
            POLL_COUNT.store(0, Ordering::Relaxed);
            static CELL: TaskCell<256> = TaskCell::new();

            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            // SAFETY: `CELL` is a fresh static `TaskCell`; both polls use
            // the same `F` (`counting_task`), as `TaskCell::poll`'s safety
            // contract requires, and the executor never re-enters.
            unsafe {
                let _ = CELL.poll(counting_task, &waker);
                let _ = CELL.poll(counting_task, &waker);
            }
            // Each poll() call drives the loop body once (TestYield yields
            // once then completes, so the outer `loop` re-enters and
            // increments again on the *next* poll call after the inner
            // future completes).
            assert!(POLL_COUNT.load(Ordering::Relaxed) >= 1);
        }
    }

    #[test]
    #[should_panic(expected = "exceeds reserved stack size")]
    fn task_cell_panics_when_future_too_large() {
        crate::kernel_test! {
            async fn big_task() {
                // Held live across the await point (read afterward) so the
                // compiler must keep it in the generated state machine
                // instead of optimizing away an unread local.
                let mut buf = [0u8; 1024];
                buf[0] = 1;
                TestYield { yielded: false }.await;
                core::hint::black_box(&buf);
            }
            static CELL: TaskCell<8> = TaskCell::new();
            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            // SAFETY: `CELL` is a fresh static `TaskCell` polled once
            // with `big_task`; same-`F` contract satisfied.
            unsafe {
                let _ = CELL.poll(big_task, &waker);
            }
        }
    }
}
