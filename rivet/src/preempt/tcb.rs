//! Task Control Block and the preemptive task registry.
//!
//! Unlike the cooperative async tier (which stores task state in a
//! compiler-generated `Future`), preemptive tasks each get their own
//! statically-allocated stack. A context switch saves/restores the full
//! callee-saved register set + stack pointer, so a preemptive task can be
//! suspended at *any* point — not just at `.await` boundaries — which is
//! what makes real priority preemption possible.

use crate::sync::atomic::{AtomicU8, AtomicUsize, Ordering};

/// Maximum number of preemptive tasks (RIVET_MAX_PTASKS).
pub const MAX_PTASKS: usize = crate::config::MAX_PTASKS;

/// Maximum number of mutexes a task may hold simultaneously (RIVET_MAX_HELD_MUTEXES).
/// A task that nests deeper deadlocks its own inheritance bookkeeping — a
/// documented, hard limit (plan.md §2.3).
pub const MAX_HELD: usize = crate::config::MAX_HELD;

/// Sentinel priority meaning "no task" in places that need one.
pub const NO_TASK: usize = usize::MAX;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TaskState {
    Ready,
    Running,
    Blocked,
}

/// One entry in a task's held-mutex list. `ptr` is the type-erased
/// `PriorityMutex` address (0 = empty); `hwp` is that mutex's monomorphized
/// "highest waiter base priority" accessor, so the list can stay
/// heterogeneous ([B11]: unlocking one mutex must not clobber the boost
/// held for another).
pub struct HeldMutex {
    /// Type-erased `PriorityMutex` pointer; null = empty slot. `AtomicPtr`
    /// (not an integer atomic) so pointer provenance survives the
    /// store/load round-trip (miri strict-provenance requirement).
    pub ptr: crate::sync::atomic::AtomicPtr<()>,
    /// That mutex's "highest waiter base priority" accessor,
    /// `fn(*const ()) -> u8` stored as a raw pointer. Written before `ptr`
    /// (Release), read after loading `ptr` (Acquire).
    pub hwp: crate::sync::atomic::AtomicPtr<()>,
}

impl HeldMutex {
    #[cfg(not(loom))]
    pub const fn empty() -> Self {
        Self::empty_impl()
    }

    #[cfg(loom)]
    pub fn empty() -> Self {
        Self::empty_impl()
    }

    #[cfg(not(loom))]
    const fn empty_impl() -> Self {
        Self {
            ptr: crate::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
            hwp: crate::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    #[cfg(loom)]
    fn empty_impl() -> Self {
        Self {
            ptr: crate::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
            hwp: crate::sync::atomic::AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

/// Task Control Block. One per preemptive task, held in the static registry.
pub struct Tcb {
    /// Saved stack pointer. Valid only while the task is not running.
    pub sp: AtomicUsize,
    /// Priority as declared by the task (0 = lowest, 31 = highest).
    pub base_priority: AtomicU8,
    /// Priority currently in effect. Normally equals `base_priority`;
    /// temporarily boosted by [`crate::preempt::mutex::PriorityMutex`] to
    /// the priority of whichever higher-priority task is blocked waiting
    /// on a resource this task holds (priority inheritance — prevents
    /// priority inversion).
    pub effective_priority: AtomicU8,
    /// Current scheduling state.
    pub state: crate::sync::atomic::AtomicU8, // encodes TaskState
    /// Whether this slot holds a live task.
    pub used: crate::sync::atomic::AtomicBool,
    /// Stack base address (low end) and size in bytes, recorded at spawn.
    /// Used by the CM3 MPU per-switch stack region, RISC-V PMP guard
    /// attribution, and stack watermarking (plan.md §3).
    pub stack_base: crate::sync::atomic::AtomicUsize,
    pub stack_size: crate::sync::atomic::AtomicUsize,
    /// Intrusive list of mutexes this task currently holds, for correct
    /// nested priority-inheritance recomputation on unlock (plan.md [B11]).
    pub held: [HeldMutex; MAX_HELD],
    pub held_count: crate::sync::atomic::AtomicU8,
    /// Last task-level watchdog checkin time (µs, low 32 bits); 0 = never
    /// checked in (plan.md §3.5).
    pub last_checkin: crate::sync::atomic::AtomicU32,

    /// Slot generation counter, incremented on every register (slot
    /// reuse). Lets `TaskHandle`-based APIs detect a stale handle (ABA on
    /// slot recycling, plan.md §5.1).
    pub generation: crate::sync::atomic::AtomicU32,
    /// Type-erased return value storage (plan.md §5.2): `result_size` bytes
    /// of `result_buf`, written exactly once by `rivet_task_exit` before
    /// `exited` is published. Read via `TaskHandle::join`.
    pub result_buf: core::cell::UnsafeCell<[u8; 32]>,
    pub result_size: crate::sync::atomic::AtomicU8,
    /// Type-erased `drop_in_place` for the stored result (0 = no-op).
    pub result_drop: crate::sync::atomic::AtomicUsize,
    /// Set by `rivet_task_exit` when the task's entry returned.
    pub exited: crate::sync::atomic::AtomicBool,
    /// Id of the task blocked in `join()` on this task (or NO_TASK).
    pub joiner: crate::sync::atomic::AtomicUsize,
    /// Cooperative cancellation flag (plan.md §5.4): set by
    /// `TaskHandle::request_stop`, polled by `should_stop()`.
    pub stop_requested: crate::sync::atomic::AtomicBool,
}

pub(crate) const READY: u8 = 0;
pub(crate) const RUNNING: u8 = 1;
pub(crate) const BLOCKED: u8 = 2;
/// Transient claim state used only during `register()`: a slot whose state
/// is RESERVED has been claimed but not yet published (its `used` flag is
/// still false), so the scheduler can never observe it half-initialized
/// (plan.md [B2]).
pub(crate) const RESERVED: u8 = 3;
/// Task paused by `TaskHandle::pause` (plan.md §5.5) — skipped by the
/// scheduler until resumed. Never runnable on its own.
pub(crate) const SUSPENDED: u8 = 4;

impl Tcb {
    /// The task's stack allocation `(base, size)` from the pool (0,0 if
    /// none — host fallback stacks).
    pub fn stack_info(&self) -> Option<(usize, usize)> {
        let base = self.stack_base.load(Ordering::Acquire);
        let size = self.stack_size.load(Ordering::Acquire);
        if base == 0 || size == 0 {
            None
        } else {
            Some((base, size))
        }
    }

    #[cfg(not(loom))]
    pub const fn new() -> Self {
        Self {
            sp: AtomicUsize::new(0),
            base_priority: AtomicU8::new(0),
            effective_priority: AtomicU8::new(0),
            state: crate::sync::atomic::AtomicU8::new(READY),
            used: crate::sync::atomic::AtomicBool::new(false),
            stack_base: crate::sync::atomic::AtomicUsize::new(0),
            stack_size: crate::sync::atomic::AtomicUsize::new(0),
            held: [const { HeldMutex::empty() }; MAX_HELD],
            held_count: crate::sync::atomic::AtomicU8::new(0),
            last_checkin: crate::sync::atomic::AtomicU32::new(0),
            generation: crate::sync::atomic::AtomicU32::new(0),
            result_buf: core::cell::UnsafeCell::new([0u8; 32]),
            result_size: crate::sync::atomic::AtomicU8::new(0),
            result_drop: crate::sync::atomic::AtomicUsize::new(0),
            exited: crate::sync::atomic::AtomicBool::new(false),
            joiner: crate::sync::atomic::AtomicUsize::new(NO_TASK),
            stop_requested: crate::sync::atomic::AtomicBool::new(false),
        }
    }

    /// Loom's atomics are not const-constructible, so under `--cfg loom`
    /// `new` is a runtime function (used by the loom models).
    #[cfg(loom)]
    pub fn new() -> Self {
        Self {
            sp: AtomicUsize::new(0),
            base_priority: AtomicU8::new(0),
            effective_priority: AtomicU8::new(0),
            state: crate::sync::atomic::AtomicU8::new(READY),
            used: crate::sync::atomic::AtomicBool::new(false),
            stack_base: crate::sync::atomic::AtomicUsize::new(0),
            stack_size: crate::sync::atomic::AtomicUsize::new(0),
            held: core::array::from_fn(|_| HeldMutex::empty()),
            held_count: crate::sync::atomic::AtomicU8::new(0),
            last_checkin: crate::sync::atomic::AtomicU32::new(0),
            generation: crate::sync::atomic::AtomicU32::new(0),
            result_buf: core::cell::UnsafeCell::new([0u8; 32]),
            result_size: crate::sync::atomic::AtomicU8::new(0),
            result_drop: crate::sync::atomic::AtomicUsize::new(0),
            exited: crate::sync::atomic::AtomicBool::new(false),
            joiner: crate::sync::atomic::AtomicUsize::new(NO_TASK),
            stop_requested: crate::sync::atomic::AtomicBool::new(false),
        }
    }

    /// Record a mutex in this task's held list. Returns false if the list
    /// is full ([`MAX_HELD`]).
    pub fn push_held(&self, ptr: *const (), hwp: fn(*const ()) -> u8) -> bool {
        if self.held_count.load(crate::sync::atomic::Ordering::Acquire) as usize >= MAX_HELD {
            return false;
        }
        for slot in &self.held {
            if slot
                .ptr
                .load(crate::sync::atomic::Ordering::Acquire)
                .is_null()
            {
                // hwp is written before the ptr store; readers load ptr
                // with Acquire, then hwp with Acquire, so the fn pointer
                // is visible (task-context single writer per slot).
                slot.hwp.store(
                    hwp as *const () as *mut (),
                    crate::sync::atomic::Ordering::Release,
                );
                slot.ptr
                    .store(ptr as *mut (), crate::sync::atomic::Ordering::Release);
                self.held_count
                    .fetch_add(1, crate::sync::atomic::Ordering::Release);
                return true;
            }
        }
        false
    }

    /// Remove a mutex from this task's held list (no-op if absent).
    pub fn remove_held(&self, ptr: *const ()) {
        for slot in &self.held {
            let loaded = slot.ptr.load(crate::sync::atomic::Ordering::Acquire);
            if core::ptr::eq(loaded, ptr) {
                slot.ptr.store(
                    core::ptr::null_mut(),
                    crate::sync::atomic::Ordering::Release,
                );
                self.held_count
                    .fetch_sub(1, crate::sync::atomic::Ordering::Release);
                return;
            }
        }
    }

    pub fn state(&self) -> TaskState {
        match self.state.load(Ordering::Acquire) {
            RUNNING => TaskState::Running,
            BLOCKED => TaskState::Blocked,
            _ => TaskState::Ready,
        }
    }

    /// Set the scheduling state and keep the O(1) scheduler's ready
    /// queues consistent (plan.md §4.2): Ready tasks are queued at their
    /// effective priority; Running/Blocked tasks are not queued.
    pub fn set_state(&self, id: usize, s: TaskState) {
        let v = match s {
            TaskState::Ready => READY,
            TaskState::Running => RUNNING,
            TaskState::Blocked => BLOCKED,
        };
        self.state.store(v, Ordering::Release);
        match s {
            TaskState::Ready => crate::preempt::sched::ready_add(id),
            TaskState::Running | TaskState::Blocked => crate::preempt::sched::ready_remove(id),
        }
    }

    /// Set the effective priority (priority inheritance) and move the task
    /// between ready queues if it is currently Ready (plan.md §4.2).
    pub fn set_effective_priority(&self, id: usize, new: u8) {
        let old = self.effective_priority.load(Ordering::Acquire);
        self.effective_priority.store(new, Ordering::Release);
        crate::preempt::sched::on_effective_priority_change(id, old, new);
    }
}

// Safety: all fields are atomics; Tcb is placed in a static array accessed
// by the scheduler (task context) and timer ISR (interrupt context).
unsafe impl Sync for Tcb {}

impl Default for Tcb {
    fn default() -> Self {
        Self::new()
    }
}

/// The static task registry. Fixed-size, no allocation.
#[cfg(not(loom))]
pub static TASKS: [Tcb; MAX_PTASKS] = [const { Tcb::new() }; MAX_PTASKS];

#[cfg(loom)]
loom::lazy_static! {
    // Same registry under loom (`Tcb::new` is not const-constructible).
    pub static ref TASKS: [Tcb; MAX_PTASKS] = core::array::from_fn(|_| Tcb::new());
}

/// Register a new preemptive task in the first free slot.
/// `sp` is the pre-built initial stack pointer (see `port::arch::init_task_stack`).
/// Returns the assigned task id, or `None` if the registry is full.
///
/// Publish ordering (plan.md [B2]): the slot is *claimed* by CASing its
/// state READY→RESERVED (the `used` flag stays false, so the scheduler —
/// which only considers `used` slots — cannot observe it), all fields are
/// written, and only then is `used` published `true` (Release). A tick
/// that lands mid-registration sees either a fully-initialized slot or no
/// slot at all — never a `used`, `Ready` TCB with `sp == 0`.
/// Register a preemptive task with its full stack description.
/// `stack_base`/`stack_size` describe the task's stack allocation (used by
/// MPU/PMP guards and watermarking, plan.md §3); pass (0, 0) when unknown.
pub fn register_full(
    sp: usize,
    priority: u8,
    stack_base: usize,
    stack_size: usize,
) -> Option<usize> {
    for (id, tcb) in TASKS.iter().enumerate() {
        if tcb.used.load(Ordering::Acquire) {
            continue;
        }
        // Claim: only a free slot can be READY→RESERVED. A live task is
        // never READY-with-used=false, so this can't steal a live slot.
        if tcb
            .state
            .compare_exchange(READY, RESERVED, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            // Publish the fields in dependency order; `used = true` last
            // (Release) so any reader that sees `used` also sees every
            // field (Release→Acquire chain).
            tcb.sp.store(sp, Ordering::Release);
            tcb.base_priority.store(priority, Ordering::Release);
            tcb.effective_priority.store(priority, Ordering::Release);
            tcb.stack_base.store(stack_base, Ordering::Release);
            tcb.stack_size.store(stack_size, Ordering::Release);
            // Drop any previously-stored result from a recycled slot.
            let drop_fn = tcb.result_drop.load(Ordering::Acquire);
            if drop_fn != 0 {
                // SAFETY: the type-erased drop fn was registered by
                // `spawn` for the exact T stored in the buffer.
                let f: fn(*mut u8) = unsafe { core::mem::transmute(drop_fn) };
                // SAFETY: result_buf holds a live T when result_drop != 0.
                f(tcb.result_buf.get() as *mut u8);
                tcb.result_drop.store(0, Ordering::Release);
            }
            // Slot recycling must be self-sufficient: don't rely on the
            // previous occupant's own exit/fault path having drained
            // `joiner` back to `NO_TASK` (found via soak testing at
            // scale, plan.md Phase 17 — a task that exits at a *higher*
            // priority than a not-yet-registered joiner drains a
            // `joiner` field that's still `NO_TASK`, then the joiner's
            // own CAS lands on a slot with nobody left to ever clear it,
            // and the *next* occupant of the recycled slot inherits a
            // permanently-stuck `joiner`). Resetting every join/exit-
            // lifecycle field here, inside the RESERVED window (`used`
            // is still `false`, so nothing else can observe this slot
            // yet), makes this the single authoritative reset point
            // regardless of how the previous occupant left.
            tcb.joiner.store(NO_TASK, Ordering::Release);
            tcb.exited.store(false, Ordering::Release);
            tcb.stop_requested.store(false, Ordering::Release);
            tcb.result_size.store(0, Ordering::Release);
            tcb.held_count.store(0, Ordering::Release);
            tcb.state.store(READY, Ordering::Release);
            tcb.used.store(true, Ordering::Release);
            tcb.generation.fetch_add(1, Ordering::Release);
            crate::preempt::sched::ready_add(id);
            return Some(id);
        }
    }
    None
}

/// Register a new preemptive task in the first free slot.
/// `sp` is the pre-built initial stack pointer (see `port::arch::init_task_stack`).
/// Returns the assigned task id, or `None` if the registry is full.
pub fn register(sp: usize, priority: u8) -> Option<usize> {
    register_full(sp, priority, 0, 0)
}

pub fn get(id: usize) -> Option<&'static Tcb> {
    TASKS.get(id).filter(|t| t.used.load(Ordering::Acquire))
}

/// Test-only: mark every TCB slot unused. Part of the global reset done by
/// [`crate::kernel_test!`].
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    for tcb in TASKS.iter() {
        tcb.used.store(false, Ordering::Release);
        tcb.state.store(READY, Ordering::Release);
        tcb.sp.store(0, Ordering::Release);
        tcb.base_priority.store(0, Ordering::Release);
        tcb.effective_priority.store(0, Ordering::Release);
        tcb.exited.store(false, Ordering::Release);
        tcb.result_size.store(0, Ordering::Release);
        tcb.result_drop.store(0, Ordering::Release);
        tcb.joiner.store(NO_TASK, Ordering::Release);
        tcb.stop_requested.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_assigns_first_free_slot_and_sets_fields() {
        crate::kernel_test! {
            let a = register(0x1000, 3).unwrap();
            assert_eq!(a, 0, "first free slot is 0");
            let b = register(0x2000, 5).unwrap();
            assert_eq!(b, 1);
            // Fields must be fully published by the time register returns
            // (plan.md [B2]).
            let ta = get(a).unwrap();
            assert_eq!(ta.sp.load(Ordering::Acquire), 0x1000);
            assert_eq!(ta.base_priority.load(Ordering::Acquire), 3);
            assert_eq!(ta.effective_priority.load(Ordering::Acquire), 3);
            assert_eq!(ta.state(), TaskState::Ready);
        }
    }

    #[test]
    fn register_full_returns_none() {
        crate::kernel_test! {
            for i in 0..MAX_PTASKS {
                assert!(register(0x1000 + i, 1).is_some(), "slot {i}");
            }
            assert_eq!(register(0x9000, 1), None, "registry full");
        }
    }
}
