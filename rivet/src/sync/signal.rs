//! A one-shot, latching wake-up handoff between an external context (an
//! ISR, another hart, another task) and exactly one waiting task.
//!
//! This is the building block for a peripheral driver's async completion:
//! the driver's interrupt handler calls [`Signal::signal`] (documented
//! ISR-safe — no critical section, no allocator, no `current_task()`
//! requirement) when a hardware transfer finishes, and the cooperative
//! task driving that transfer `.await`s [`Signal::wait`].
//!
//! `Signal` carries no value — the driver reads the peripheral's own
//! status registers to find out what happened; this avoids an allocator
//! and `Cell<T>` variance questions entirely. If a future version needs a
//! value-carrying variant, that's a new type (`Signal<T>`), not a change
//! to this one.
//!
//! Unlike [`crate::sync::Semaphore`] (a 32-wide per-priority waiter
//! bitmap, because several tasks may legitimately contend for one
//! resource) or [`crate::sync::Channel`] (two independently-driven waiter
//! slots), `Signal` has exactly one waiter slot: a peripheral has one
//! owning task by construction. Registering a second concurrent waiter is
//! a caller bug, not a supported use case — see [`Wait::poll`].

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

use crate::waker;

const NO_WAITER: u32 = 0xFFFF_FFFF;

/// See the [module docs](self).
pub struct Signal {
    /// Latched by `signal()`, consumed by a successful [`Signal::try_take`].
    /// Latching (rather than a plain wake) is what makes "the ISR fired
    /// before `wait()`'s first poll" correct instead of a lost wakeup.
    fired: crate::sync::atomic::AtomicBool,
    /// The single registered cooperative waiter, or `NO_WAITER`.
    waiter: crate::sync::atomic::AtomicU32,
}

impl Signal {
    /// Create a new, unfired signal.
    #[cfg(not(loom))]
    pub const fn new() -> Self {
        Self {
            fired: crate::sync::atomic::AtomicBool::new(false),
            waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
        }
    }

    /// Loom's atomics are not const-constructible; runtime constructor used
    /// by the loom models.
    #[cfg(loom)]
    pub fn new() -> Self {
        Self {
            fired: crate::sync::atomic::AtomicBool::new(false),
            waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
        }
    }

    /// External/ISR side: latch the signal and wake the registered waiter,
    /// if any. Safe to call from Handler mode / an interrupt, from either
    /// hart on an SMP build, or from plain task code.
    pub fn signal(&self) {
        // Store the latch *before* consuming the waiter slot: a `wait()`
        // that re-checks `try_take()` immediately after registering (see
        // `Wait::poll`) must see `fired = true` if this call's swap below
        // is about to (or just did) claim its registration — Release here
        // pairs with the Acquire in `try_take`.
        self.fired
            .store(true, crate::sync::atomic::Ordering::Release);
        let w = self
            .waiter
            .swap(NO_WAITER, crate::sync::atomic::Ordering::AcqRel);
        if w != NO_WAITER {
            waker::wake_task(crate::task::TaskId::from_u16(w as u16));
        }
    }

    /// Non-blocking consume: clears and returns whether the signal had
    /// fired. Usable from a preemptive task, or from boot code before the
    /// scheduler starts.
    pub fn try_take(&self) -> bool {
        self.fired
            .compare_exchange(
                true,
                false,
                crate::sync::atomic::Ordering::AcqRel,
                crate::sync::atomic::Ordering::Acquire,
            )
            .is_ok()
    }

    /// Clear a stale latch. Drivers **must** call this before arming
    /// hardware for a new transaction — otherwise a signal left over from
    /// a previous, already-completed (or cancelled) transaction would
    /// make the next `wait()` return immediately with nothing to report.
    pub fn reset(&self) {
        self.fired
            .store(false, crate::sync::atomic::Ordering::Release);
    }

    /// Cooperative-tier wait: yields the task until [`Signal::signal`] is
    /// called (or returns immediately if it already latched).
    ///
    /// # Panics
    /// Panics if polled outside of a task context (i.e. not from within
    /// the executor's poll of a `#[rivet::task]`).
    pub fn wait(&self) -> Wait<'_> {
        Wait {
            sig: self,
            registered: None,
        }
    }
}

impl Default for Signal {
    fn default() -> Self {
        Self::new()
    }
}

/// Future returned by [`Signal::wait`].
pub struct Wait<'a> {
    sig: &'a Signal,
    /// `Some(id)` while registered as the waiter; cleared on completion,
    /// and cancelled in [`Drop`] so a dropped `wait()` never leaves a
    /// stale registration behind. Does **not** clear `sig.fired` — a
    /// signal that fired while this future was being cancelled must stay
    /// observable to whatever calls `try_take()` next.
    registered: Option<crate::task::TaskId>,
}

impl<'a> Drop for Wait<'a> {
    fn drop(&mut self) {
        if self.registered.take().is_some() {
            self.sig
                .waiter
                .store(NO_WAITER, crate::sync::atomic::Ordering::Release);
        }
    }
}

impl<'a> Future for Wait<'a> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        // SAFETY: `Wait` holds only a `&Signal` and an `Option`; no
        // `!Unpin` fields, so projecting is sound.
        let this = unsafe { self.get_unchecked_mut() };
        if this.sig.try_take() {
            return Poll::Ready(());
        }

        let id = crate::executor::current_task()
            .expect("Signal::wait() polled outside of a task context");
        let prev = this
            .sig
            .waiter
            .swap(id.as_u16() as u32, crate::sync::atomic::Ordering::AcqRel);
        debug_assert_eq!(
            prev, NO_WAITER,
            "rivet: two tasks awaiting the same Signal concurrently — \
             a peripheral has exactly one owning task by construction"
        );
        this.registered = Some(id);

        // Re-check: signal() may have fired between our first try_take()
        // and registering above.
        if this.sig.try_take() {
            if this.registered.take().is_some() {
                this.sig
                    .waiter
                    .store(NO_WAITER, crate::sync::atomic::Ordering::Release);
            }
            return Poll::Ready(());
        }

        Poll::Pending
    }
}

// Safety: Signal uses only atomics; safe to share across contexts.
unsafe impl Sync for Signal {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn try_take_false_when_unfired() {
        crate::kernel_test! {
            let sig = Signal::new();
            assert!(!sig.try_take());
        }
    }

    #[test]
    fn signal_then_try_take() {
        crate::kernel_test! {
            let sig = Signal::new();
            sig.signal();
            assert!(sig.try_take());
            assert!(!sig.try_take(), "try_take consumes the latch");
        }
    }

    #[test]
    fn reset_clears_stale_latch() {
        crate::kernel_test! {
            let sig = Signal::new();
            sig.signal();
            sig.reset();
            assert!(!sig.try_take());
        }
    }

    #[test]
    fn wait_ready_when_already_fired() {
        crate::kernel_test! {
            let sig = Signal::new();
            sig.signal();
            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = sig.wait();
            // SAFETY: `fut` is a local `Wait` future; `Unpin`, never moved
            // while pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            assert_eq!(pinned.poll(&mut cx), Poll::Ready(()));
        }
    }

    #[test]
    #[should_panic(expected = "outside of a task context")]
    fn wait_panics_without_task_context() {
        crate::kernel_test! {
            let sig = Signal::new();
            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = sig.wait();
            // SAFETY: `fut` is a local `Wait` future; `Unpin`, never moved
            // while pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            let _ = pinned.poll(&mut cx);
        }
    }

    #[test]
    fn signal_after_registration_wakes() {
        crate::kernel_test! {
            let sig = Signal::new();
            let id = crate::task::TaskId::new(1, 0);
            let waker = crate::waker::task_waker(id);
            let mut cx = Context::from_waker(&waker);
            let mut fut = sig.wait();
            // SAFETY: see above.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            // First poll: nothing fired yet, registers as waiter.
            crate::executor::set_current_for_test(id.priority(), id.index());
            assert_eq!(pinned.poll(&mut cx), Poll::Pending);

            // ISR fires the signal — should mark the task ready.
            sig.signal();
            assert_eq!(crate::waker::next_ready(), Some(id));
        }
    }

    #[test]
    fn drop_clears_registration_not_latch() {
        crate::kernel_test! {
            let sig = Signal::new();
            let id = crate::task::TaskId::new(2, 0);
            {
                let mut fut = sig.wait();
                // SAFETY: see above.
                let pinned = unsafe { Pin::new_unchecked(&mut fut) };
                let waker = crate::waker::task_waker(id);
                let mut cx = Context::from_waker(&waker);
                crate::executor::set_current_for_test(id.priority(), id.index());
                assert_eq!(pinned.poll(&mut cx), Poll::Pending);
                // fut dropped here — registration must be cleared.
            }
            // A signal firing after the drop must not wake the old id.
            sig.signal();
            assert_eq!(crate::waker::next_ready(), None);
        }
    }

    #[test]
    fn signal_during_cancellation_stays_observable() {
        crate::kernel_test! {
            let sig = Signal::new();
            let id = crate::task::TaskId::new(3, 0);
            {
                let mut fut = sig.wait();
                // SAFETY: see above.
                let pinned = unsafe { Pin::new_unchecked(&mut fut) };
                let waker = crate::waker::task_waker(id);
                let mut cx = Context::from_waker(&waker);
                crate::executor::set_current_for_test(id.priority(), id.index());
                assert_eq!(pinned.poll(&mut cx), Poll::Pending);
                sig.signal();
                // fut dropped here without ever observing the Ready value.
            }
            // The latch must still be observable by whatever asks next.
            assert!(sig.try_take());
        }
    }
}
