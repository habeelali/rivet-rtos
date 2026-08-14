//! Zero-allocation waker using atomic priority bitmaps.
//!
//! Each task has a `(priority, index_in_priority)` pair. When a waker fires,
//! it atomically sets the corresponding bit in the per-priority queue and
//! the global ready bitmap. The executor finds the next task in O(1) using
//! `leading_zeros` on the bitmap.

// FQN atomics used below.
use core::task::{RawWaker, RawWakerVTable, Waker};

/// Bit i set => priority level i has at least one ready task.
///
/// Under `--cfg loom` these globals live in `loom::lazy_static!` (loom's
/// atomics are not const-constructible); loom resets them between models.
#[cfg(not(loom))]
static READY_BITMAP: crate::sync::atomic::AtomicU32 = crate::sync::atomic::AtomicU32::new(0);
#[cfg(loom)]
loom::lazy_static! {
    static ref READY_BITMAP: crate::sync::atomic::AtomicU32 = crate::sync::atomic::AtomicU32::new(0);
}

/// Per-priority ready task bitmasks.
#[cfg(not(loom))]
static PRIORITY_QUEUES: [crate::sync::atomic::AtomicU32; 32] = [
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
];
#[cfg(loom)]
loom::lazy_static! {
    static ref PRIORITY_QUEUES: [crate::sync::atomic::AtomicU32; 32] = [
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    crate::sync::atomic::AtomicU32::new(0),
    ];
}

/// CPU flag set when waker fires from ISR context.
#[cfg(not(loom))]
pub(crate) static EXECUTOR_PEND_FLAG: crate::sync::atomic::AtomicU32 =
    crate::sync::atomic::AtomicU32::new(0);
#[cfg(loom)]
loom::lazy_static! {
    pub(crate) static ref EXECUTOR_PEND_FLAG: crate::sync::atomic::AtomicU32 = crate::sync::atomic::AtomicU32::new(0);
}

/// Reset the waker state (for testing).
pub fn reset() {
    READY_BITMAP.store(0, crate::sync::atomic::Ordering::Release);
    for q in PRIORITY_QUEUES.iter() {
        q.store(0, crate::sync::atomic::Ordering::Release);
    }
    EXECUTOR_PEND_FLAG.store(0, crate::sync::atomic::Ordering::Release);
}

/// Mark a task as ready.
#[doc(hidden)]
pub fn mark_ready(id: crate::task::TaskId) {
    let mask = 1u32 << (id.index() & 0x1F);
    PRIORITY_QUEUES[id.priority() as usize].fetch_or(mask, crate::sync::atomic::Ordering::Release);
    READY_BITMAP.fetch_or(
        1u32 << id.priority(),
        crate::sync::atomic::Ordering::Release,
    );
    EXECUTOR_PEND_FLAG.store(1, crate::sync::atomic::Ordering::Release);
}

/// Check if any tasks are pending (used before sleep to avoid race).
#[doc(hidden)]
pub fn has_pending() -> bool {
    EXECUTOR_PEND_FLAG.load(crate::sync::atomic::Ordering::Acquire) != 0
        || READY_BITMAP.load(crate::sync::atomic::Ordering::Acquire) != 0
}

/// Clear the pend flag. Called when executor starts a new polling round.
#[doc(hidden)]
pub fn clear_pend() {
    EXECUTOR_PEND_FLAG.store(0, crate::sync::atomic::Ordering::Release);
}

/// Dequeue the highest-priority ready task. Returns its [`TaskId`] or
/// `None`.
#[doc(hidden)]
pub fn next_ready() -> Option<crate::task::TaskId> {
    let bitmap = READY_BITMAP.load(crate::sync::atomic::Ordering::Acquire);
    if bitmap == 0 {
        return None;
    }

    let prio = (31 - bitmap.leading_zeros()) as u8;
    let queue = PRIORITY_QUEUES[prio as usize].load(crate::sync::atomic::Ordering::Acquire);

    if queue == 0 {
        READY_BITMAP.fetch_and(!(1u32 << prio), crate::sync::atomic::Ordering::AcqRel);
        return None;
    }

    let bit = queue & queue.wrapping_neg();
    let index = bit.trailing_zeros() as u8;

    let prev =
        PRIORITY_QUEUES[prio as usize].fetch_and(!bit, crate::sync::atomic::Ordering::AcqRel);

    if prev == bit {
        READY_BITMAP.fetch_and(!(1u32 << prio), crate::sync::atomic::Ordering::AcqRel);
    }

    Some(crate::task::TaskId::new(prio, index))
}

/// Static cells so the waker data pointer has real provenance (miri
/// strict-provenance clean) instead of an integer→pointer cast. 2 KiB of
/// static storage; indexed by `(priority, index)`.
static TASK_ID_CELLS: [[crate::task::TaskId; 32]; 32] = {
    let mut cells = [[crate::task::TaskId::new(0, 0); 32]; 32];
    let mut p = 0;
    while p < 32 {
        let mut i = 0;
        while i < 32 {
            cells[p][i] = crate::task::TaskId::new(p as u8, i as u8);
            i += 1;
        }
        p += 1;
    }
    cells
};

fn encode_waker_data(id: crate::task::TaskId) -> *const () {
    // SAFETY-free: `&TASK_ID_CELLS[...] as *const _` is a genuine pointer
    // into a static; the cells are never mutated.
    core::ptr::addr_of!(TASK_ID_CELLS[id.priority() as usize][id.index() as usize]) as *const ()
}

fn decode_waker_data(data: *const ()) -> crate::task::TaskId {
    // SAFETY: the pointer was produced by `encode_waker_data` and still
    // points into the immutable static cells.
    unsafe { *(data as *const crate::task::TaskId) }
}

// ── RawWaker vtable ──────────────────────────────────────────────

unsafe fn waker_clone(data: *const ()) -> RawWaker {
    RawWaker::new(data, &WAKER_VTABLE)
}

unsafe fn waker_wake(data: *const ()) {
    mark_ready(decode_waker_data(data));
}

unsafe fn waker_wake_by_ref(data: *const ()) {
    mark_ready(decode_waker_data(data));
}

unsafe fn waker_drop(_data: *const ()) {
    // Nothing to drop; data is statically allocated.
}

static WAKER_VTABLE: RawWakerVTable =
    RawWakerVTable::new(waker_clone, waker_wake, waker_wake_by_ref, waker_drop);

/// Kick every other hart so an executor idling in `wfi`/`waiti` on a
/// different core notices new ready work. Exposed separately from
/// [`wake_task`] so a caller waking several tasks in one batch (e.g.
/// `timer::poll_timers` scanning every expired deadline inside one
/// critical section) can broadcast once instead of once per task.
///
/// Safe to call from ISR/Handler-mode context — every board's periodic
/// tick ISR already does, via `timer::poll_timers`.
pub fn broadcast_reschedule() {
    let hart = crate::port::arch::hart_id();
    for other in 0..crate::config::MAX_HARTS {
        if other != hart {
            crate::port::arch::request_reschedule_on(other);
        }
    }
}

/// Mark `id` ready and broadcast a reschedule request to every other hart.
///
/// Use this — not the lower-level [`mark_ready`] — from any context
/// *outside* the executor's own poll loop: an ISR, another hart, or a
/// driver's completion handler. `mark_ready` alone only flips bitmap
/// flags; on a single-hart build the interrupt that called this is enough
/// to break the executor out of `wfi`, but on `RIVET_MAX_HARTS > 1` an
/// executor idling on a *different* hart would never notice (this is the
/// exact bug `timer::poll_timers`'s own broadcast fixed, found on real
/// dual-core ESP32-S3 hardware via `smp_test.rs`). Safe to call from ISR
/// context.
pub fn wake_task(id: crate::task::TaskId) {
    mark_ready(id);
    broadcast_reschedule();
}

/// Create a `Waker` for the task identified by `id`.
pub fn task_waker(id: crate::task::TaskId) -> Waker {
    let data = encode_waker_data(id);
    let raw = RawWaker::new(data, &WAKER_VTABLE);
    // SAFETY: `raw` is built by [`task_waker`] from the static vtable and
    // a pointer into the immutable `TASK_ID_CELLS` static with no drop
    // state; `waker_drop` is a no-op, so transferring ownership into the
    // `Waker` is safe.
    unsafe { Waker::from_raw(raw) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encode_decode_roundtrip() {
        crate::kernel_test! {
        let data = encode_waker_data(crate::task::TaskId::new(5, 13));
        let id = decode_waker_data(data);
        assert_eq!(id.priority(), 5);
        assert_eq!(id.index(), 13);
        }
    }

    #[test]
    fn mark_and_dequeue_single() {
        crate::kernel_test! {
        mark_ready(crate::task::TaskId::new(2, 0));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(2, 0)));
        assert_eq!(next_ready(), None);
        }
    }

    #[test]
    fn priority_ordering() {
        crate::kernel_test! {
        mark_ready(crate::task::TaskId::new(1, 0));
        mark_ready(crate::task::TaskId::new(5, 0));
        mark_ready(crate::task::TaskId::new(3, 0));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(5, 0)));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(3, 0)));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(1, 0)));
        assert_eq!(next_ready(), None);
        }
    }

    #[test]
    fn multiple_tasks_same_priority() {
        crate::kernel_test! {
        mark_ready(crate::task::TaskId::new(3, 0));
        mark_ready(crate::task::TaskId::new(3, 1));
        mark_ready(crate::task::TaskId::new(3, 2));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(3, 0)));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(3, 1)));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(3, 2)));
        assert_eq!(next_ready(), None);
        }
    }

    #[test]
    fn interleaved_wake() {
        crate::kernel_test! {
        mark_ready(crate::task::TaskId::new(2, 0));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(2, 0)));
        assert_eq!(next_ready(), None);
        mark_ready(crate::task::TaskId::new(2, 0));
        mark_ready(crate::task::TaskId::new(4, 1));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(4, 1)));
        assert_eq!(next_ready(), Some(crate::task::TaskId::new(2, 0)));
        assert_eq!(next_ready(), None);
        }
    }

    #[test]
    fn has_pending_detects_work() {
        crate::kernel_test! {
        assert!(!has_pending());
        mark_ready(crate::task::TaskId::new(0, 0));
        assert!(has_pending());
        next_ready();
        clear_pend();
        assert!(!has_pending());
        }
    }
}
