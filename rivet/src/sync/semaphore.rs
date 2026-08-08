//! Async counting semaphore.
//!
//! `acquire()` returns a real [`Future`] — call it with `.await` from an
//! `async fn` task. The waiting task's identity is read from
//! [`crate::executor::current_task`], so no manual priority/index
//! bookkeeping is needed by the caller.
//!
//! Safe to `release()` from either task context or an ISR.

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use crate::waker;

/// An async counting semaphore.
///
/// ```ignore
/// static SEM: rivet::sync::Semaphore<1> = rivet::sync::Semaphore::new(0);
///
/// #[rivet::task(priority = 1)]
/// async fn waiter() {
///     loop {
///         SEM.acquire().await;
///         // got the semaphore
///     }
/// }
///
/// // ISR or another task:
/// fn signaler() {
///     SEM.release();
/// }
/// ```
pub struct Semaphore<const MAX: u8> {
    /// Current count. 0 = taken, >0 = available.
    count: crate::sync::atomic::AtomicU8,
    /// Per-priority waiter bitmap, mirroring `waker::PRIORITY_QUEUES`
    /// (plan.md [B9]): bit i of `waiters[p]` = task `(p, i)` waiting.
    /// Multiple waiters are supported; `release()` wakes the
    /// highest-priority one.
    waiters: [crate::sync::atomic::AtomicU32; 32],
}

impl<const MAX: u8> Semaphore<MAX> {
    /// Create a new semaphore with the given initial count.
    #[cfg(not(loom))]
    pub const fn new(initial: u8) -> Self {
        Self {
            count: crate::sync::atomic::AtomicU8::new(initial),
            // Inline const avoids a named `const` item with interior
            // mutability.
            waiters: [const { crate::sync::atomic::AtomicU32::new(0) }; 32],
        }
    }

    /// Loom's atomics are not const-constructible; runtime constructor used
    /// by the loom models.
    #[cfg(loom)]
    pub fn new(initial: u8) -> Self {
        Self {
            count: crate::sync::atomic::AtomicU8::new(initial),
            waiters: core::array::from_fn(|_| crate::sync::atomic::AtomicU32::new(0)),
        }
    }

    /// Try to acquire without blocking. Returns true if acquired.
    pub fn try_acquire(&self) -> bool {
        loop {
            let c = self.count.load(crate::sync::atomic::Ordering::Acquire);
            if c == 0 {
                return false;
            }
            if self
                .count
                .compare_exchange_weak(
                    c,
                    c - 1,
                    crate::sync::atomic::Ordering::AcqRel,
                    crate::sync::atomic::Ordering::Acquire,
                )
                .is_ok()
            {
                return true;
            }
        }
    }

    /// Acquire the semaphore, yielding the task if it's currently taken.
    ///
    /// # Panics
    /// Panics if polled outside of a task context (i.e. not from within
    /// the executor's poll of a `#[rivet::task]`).
    pub fn acquire(&self) -> Acquire<'_, MAX> {
        Acquire {
            sem: self,
            registered: None,
        }
    }

    /// Release the semaphore. If tasks are waiting, the highest-priority
    /// one is woken and handed the token directly. Safe to call from ISR
    /// context.
    pub fn release(&self) {
        // Hand the token directly to the highest-priority waiter instead
        // of incrementing count, so a concurrent try_acquire() by a third
        // party can't steal it out from under the woken task.
        //
        // The waiter bit is claimed with a CAS loop, not a plain load +
        // clear: two concurrent `release()` calls must not both pick the
        // same waiter (loom found exactly that race in the initial
        // check-then-act version).
        for (prio, queue) in self.waiters.iter().enumerate().rev() {
            loop {
                let q = queue.load(crate::sync::atomic::Ordering::Acquire);
                if q == 0 {
                    break; // no waiter at this priority — try the next
                }
                let bit = q & q.wrapping_neg();
                match queue.compare_exchange_weak(
                    q,
                    q & !bit,
                    crate::sync::atomic::Ordering::AcqRel,
                    crate::sync::atomic::Ordering::Acquire,
                ) {
                    Ok(_) => {
                        self.count.store(1, crate::sync::atomic::Ordering::Release);
                        waker::mark_ready(crate::task::TaskId::new(
                            prio as u8,
                            bit.trailing_zeros() as u8,
                        ));
                        return;
                    }
                    Err(_) => continue, // another release took it — retry
                }
            }
        }

        // No waiter: increment (bounded by MAX).
        let mut c = self.count.load(crate::sync::atomic::Ordering::Acquire);
        loop {
            if c >= MAX {
                return;
            }
            match self.count.compare_exchange_weak(
                c,
                c + 1,
                crate::sync::atomic::Ordering::AcqRel,
                crate::sync::atomic::Ordering::Acquire,
            ) {
                Ok(_) => return,
                Err(actual) => c = actual,
            }
        }
    }

    fn register_waiter(&self, id: crate::task::TaskId) {
        let mask = 1u32 << id.index();
        self.waiters[id.priority() as usize].fetch_or(mask, crate::sync::atomic::Ordering::Release);
    }

    /// Debug snapshot of the waiter bitmaps (test-only).
    #[cfg(any(loom, feature = "test-support"))]
    #[doc(hidden)]
    pub fn debug_waiters(&self) -> [u32; 32] {
        let mut w = [0u32; 32];
        for (i, q) in self.waiters.iter().enumerate() {
            w[i] = q.load(crate::sync::atomic::Ordering::Acquire);
        }
        w
    }

    fn remove_waiter(&self, id: crate::task::TaskId) {
        let mask = 1u32 << id.index();
        self.waiters[id.priority() as usize]
            .fetch_and(!mask, crate::sync::atomic::Ordering::AcqRel);
    }
}

/// Future returned by [`Semaphore::acquire`].
pub struct Acquire<'a, const MAX: u8> {
    sem: &'a Semaphore<MAX>,
    /// `Some(id)` while registered as a waiter; cleared on
    /// completion, and cancelled in [`Drop`] so a dropped acquire never
    /// leaves a stale registration behind (plan.md §2.5).
    registered: Option<crate::task::TaskId>,
}

impl<'a, const MAX: u8> Drop for Acquire<'a, MAX> {
    fn drop(&mut self) {
        if let Some(id) = self.registered.take() {
            self.sem.remove_waiter(id);
        }
    }
}

impl<'a, const MAX: u8> Future for Acquire<'a, MAX> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        // SAFETY: `Acquire` holds only a `&Semaphore` and an `Option`;
        // no `!Unpin` fields, so projecting is sound.
        let this = unsafe { self.get_unchecked_mut() };
        if this.sem.try_acquire() {
            return Poll::Ready(());
        }

        let id = crate::executor::current_task()
            .expect("Semaphore::acquire().await polled outside of a task context");
        this.sem.register_waiter(id);
        this.registered = Some(id);

        // Re-check: release() may have fired between our first try_acquire()
        // and register_waiter() above.
        if this.sem.try_acquire() {
            if let Some(id) = this.registered.take() {
                this.sem.remove_waiter(id);
            }
            return Poll::Ready(());
        }

        Poll::Pending
    }
}

// Safety: Semaphore uses atomics; safe to share across contexts.
unsafe impl<const MAX: u8> Sync for Semaphore<MAX> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semaphore_try_acquire_release() {
        crate::kernel_test! {
            let sem: Semaphore<3> = Semaphore::new(1);
            assert!(sem.try_acquire());
            assert!(!sem.try_acquire());
            sem.release();
            assert!(sem.try_acquire());
        }
    }

    #[test]
    fn semaphore_counting() {
        crate::kernel_test! {
            let sem: Semaphore<3> = Semaphore::new(2);
            assert!(sem.try_acquire());
            assert!(sem.try_acquire());
            assert!(!sem.try_acquire());
            sem.release();
            assert!(sem.try_acquire());
            assert!(!sem.try_acquire());
        }
    }

    #[test]
    fn acquire_future_ready_when_available() {
        crate::kernel_test! {
            let sem: Semaphore<1> = Semaphore::new(1);
            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = sem.acquire();
            // SAFETY: `fut` is a local `Acquire` future; `Unpin`, never
            // moved while pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            assert_eq!(pinned.poll(&mut cx), Poll::Ready(()));
        }
    }

    #[test]
    #[should_panic(expected = "outside of a task context")]
    fn acquire_future_panics_without_task_context() {
        crate::kernel_test! {
            let sem: Semaphore<1> = Semaphore::new(0);
            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = sem.acquire();
            // SAFETY: `fut` is a local `Acquire` future; `Unpin`, never
            // moved while pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            let _ = pinned.poll(&mut cx);
        }
    }
}

#[cfg(test)]
mod b9_tests {
    use super::*;

    #[test]
    fn two_waiters_both_woken() {
        crate::kernel_test! {
            let sem: Semaphore<1> = Semaphore::new(0);

            // Register two waiters directly (simulating two Pending polls).
            sem.register_waiter(crate::task::TaskId::new(1, 0));
            sem.register_waiter(crate::task::TaskId::new(2, 0));
            assert_eq!(sem.debug_waiters()[1], 1, "waiter (1,0)");
            assert_eq!(sem.debug_waiters()[2], 1, "waiter (2,0)");

            sem.release();
            assert_eq!(crate::waker::next_ready(), Some(crate::task::TaskId::new(2, 0)), "highest priority first");

            sem.release();
            assert_eq!(crate::waker::next_ready(), Some(crate::task::TaskId::new(1, 0)), "second waiter woken");
            assert_eq!(crate::waker::next_ready(), None);
            // Both registrations consumed; no leak.
            assert_eq!(sem.debug_waiters()[1], 0);
            assert_eq!(sem.debug_waiters()[2], 0);
        }
    }

    #[test]
    fn remove_waiter_on_drop_clears_registration() {
        crate::kernel_test! {
            let sem: Semaphore<1> = Semaphore::new(0);
            sem.register_waiter(crate::task::TaskId::new(3, 1));
            assert_eq!(sem.debug_waiters()[3], 1 << 1);
            sem.remove_waiter(crate::task::TaskId::new(3, 1));
            assert_eq!(sem.debug_waiters()[3], 0);
        }
    }
}
