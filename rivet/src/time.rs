//! Static timing: const-generic durations and async sleep futures.
//!
//! `Sleep` registers its deadline with [`crate::timer`] on first poll and
//! then returns `Pending` without re-arming its own waker — the platform
//! timer ISR wakes it when the deadline passes. This means a sleeping task
//! does not busy-poll: between ticks the executor has nothing ready and
//! genuinely enters `port::arch::idle()` (WFI), which is what makes tickless
//! idle actually save power instead of spinning.
//!
//! ```ignore
//! use rivet::time::Sleep;
//!
//! #[rivet::task(priority = 0)]
//! async fn blink() {
//!     loop {
//!         toggle_led();
//!         Sleep::<500_000>::new().await; // 500ms
//!     }
//! }
//! ```

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// A duration in microseconds, known at compile time.
#[derive(Clone, Copy, Debug)]
pub struct Duration {
    micros: u64,
}

impl Duration {
    pub const fn from_micros(micros: u64) -> Self {
        Self { micros }
    }

    pub const fn from_millis(ms: u64) -> Self {
        Self { micros: ms * 1000 }
    }

    pub const fn as_micros(&self) -> u64 {
        self.micros
    }

    pub const fn as_millis(&self) -> u64 {
        self.micros / 1000
    }
}

/// A future that resolves after a compile-time-known duration has elapsed.
///
/// The const generic `MICROS` encodes the sleep duration. Must be polled
/// from within a `#[rivet::task]` (needs [`crate::executor::current_task`]
/// to register the wake-up).
pub struct Sleep<const MICROS: u64> {
    deadline: u64,
    /// Outstanding timer-slot registration, if any. Cleared on completion;
    /// cancels in [`Drop`] so a dropped sleep never leaks a slot or fires
    /// a spurious wake (plan.md [B7]).
    slot: Option<crate::timer::TimerHandle>,
}

impl<const MICROS: u64> Sleep<MICROS> {
    /// Create a new sleep future. The deadline is computed on first poll.
    pub const fn new() -> Self {
        Self {
            deadline: 0,
            slot: None,
        }
    }
}

impl<const MICROS: u64> Default for Sleep<MICROS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MICROS: u64> Drop for Sleep<MICROS> {
    fn drop(&mut self) {
        if let Some(handle) = self.slot.take() {
            crate::timer::cancel_deadline(handle);
        }
    }
}

impl<const MICROS: u64> Future for Sleep<MICROS> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        // SAFETY: `Sleep` is a plain struct with no `!Unpin` fields;
        // projecting through the pinned reference is sound.
        let this = unsafe { self.get_unchecked_mut() };
        let now = crate::port::board::now_us();

        if this.deadline == 0 {
            let deadline = now.wrapping_add(MICROS).max(1); // avoid the 0 "unset" sentinel
            this.deadline = deadline;

            let id = crate::executor::current_task()
                .expect("Sleep::poll() called outside of a task context");
            // Queue full surfaces as a documented panic at the call site —
            // a sleep that cannot register would silently never fire.
            let handle = crate::timer::register_deadline(deadline, id).unwrap_or_else(|_| {
                panic!(
                    "rivet: Sleep timer queue full ({} concurrent sleeps supported)",
                    crate::timer::MAX_TIMERS
                )
            });
            this.slot = Some(handle);

            if now >= deadline {
                // MICROS == 0 or wrapped: already elapsed.
                return Poll::Ready(());
            }
            return Poll::Pending;
        }

        if now >= this.deadline {
            // The timer ISR already cleared the slot when it fired; the
            // handle is now stale and its eventual cancel is a no-op.
            this.slot = None;
            Poll::Ready(())
        } else {
            // Not yet elapsed. Do NOT re-wake ourselves — the timer ISR
            // (registered above) will call waker::mark_ready when the
            // deadline passes. Busy-waking here would defeat tickless idle.
            Poll::Pending
        }
    }
}
