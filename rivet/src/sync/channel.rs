//! Static SPSC (Single-Producer Single-Consumer) channel.
//!
//! Const-generic ring buffer with atomic head/tail indices, split into
//! `Sender`/`Receiver` halves. `send()`/`recv()` return real [`Future`]s
//! usable with `.await` from `async fn` tasks — no heap allocation.
//!
//! ```ignore
//! static CHAN: rivet::sync::Channel<u32, 8> = rivet::sync::Channel::new();
//!
//! #[rivet::task(priority = 1)]
//! async fn producer() {
//!     let (mut tx, _) = CHAN.split();
//!     loop {
//!         tx.send(42).await;
//!     }
//! }
//!
//! #[rivet::task(priority = 0)]
//! async fn consumer() {
//!     let (_, mut rx) = CHAN.split();
//!     loop {
//!         let val = rx.recv().await;
//!     }
//! }
//! ```

use core::cell::UnsafeCell;
use core::future::Future;
use core::mem::MaybeUninit;
use core::pin::Pin;
use core::task::{Context, Poll};

use crate::waker;

const NO_WAITER: u32 = 0xFFFF_FFFF;

/// A lock-free SPSC ring buffer. Usable capacity is `N - 1`.
pub struct Channel<T, const N: usize> {
    buffer: UnsafeCell<[MaybeUninit<T>; N]>,
    head: crate::sync::atomic::AtomicUsize,
    tail: crate::sync::atomic::AtomicUsize,
    /// One-shot split guard: `split()` succeeds exactly once (plan.md
    /// [B8]) — the SPSC ownership invariant is enforced, not documented.
    taken: crate::sync::atomic::AtomicBool,
    /// Waiter blocked in `recv()` (encoded priority/index), woken by `send()`.
    recv_waiter: crate::sync::atomic::AtomicU32,
    /// Waiter blocked in `send()` (encoded priority/index), woken by `recv()`.
    send_waiter: crate::sync::atomic::AtomicU32,
}

impl<T, const N: usize> Channel<T, N> {
    /// Create a new empty channel.
    #[cfg(not(loom))]
    pub const fn new() -> Self {
        Self {
            buffer: UnsafeCell::new(unsafe {
                // SAFETY: `MaybeUninit::uninit()` is valid to `assume_init`
                // as a *value* of `MaybeUninit<T>` (not as a `T`); the
                // buffer is only read through `assume_init_read` after a
                // matching `write`, so no uninitialized `T` is ever
                // observed.
                MaybeUninit::uninit().assume_init()
            }),
            head: crate::sync::atomic::AtomicUsize::new(0),
            tail: crate::sync::atomic::AtomicUsize::new(0),
            taken: crate::sync::atomic::AtomicBool::new(false),
            recv_waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
            send_waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
        }
    }

    /// Loom's atomics are not const-constructible; runtime constructor used
    /// by the loom models.
    #[cfg(loom)]
    pub fn new() -> Self {
        Self {
            buffer: UnsafeCell::new(unsafe {
                // SAFETY: see the non-loom `new` — the buffer is only read
                // through `assume_init_read` after a matching `write`.
                MaybeUninit::uninit().assume_init()
            }),
            head: crate::sync::atomic::AtomicUsize::new(0),
            tail: crate::sync::atomic::AtomicUsize::new(0),
            taken: crate::sync::atomic::AtomicBool::new(false),
            recv_waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
            send_waiter: crate::sync::atomic::AtomicU32::new(NO_WAITER),
        }
    }

    /// Split into sender and receiver halves — exactly once per channel
    /// (plan.md [B8]). A second call returns `None`, enforcing the SPSC
    /// ownership invariant at runtime instead of merely documenting it.
    pub fn split(&'static self) -> Option<(Sender<'static, T, N>, Receiver<'static, T, N>)> {
        if self.taken.swap(true, crate::sync::atomic::Ordering::AcqRel) {
            return None;
        }
        Some((Sender { chan: self }, Receiver { chan: self }))
    }
}

impl<T, const N: usize> Default for Channel<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

fn wake_waiter(slot: &crate::sync::atomic::AtomicU32) {
    let w = slot.swap(NO_WAITER, crate::sync::atomic::Ordering::AcqRel);
    if w != NO_WAITER {
        waker::mark_ready(crate::task::TaskId::from_u16(w as u16));
    }
}

fn register_waiter(slot: &crate::sync::atomic::AtomicU32, id: crate::task::TaskId) {
    slot.store(id.as_u16() as u32, crate::sync::atomic::Ordering::Release);
}

/// Sending half of an SPSC channel.
pub struct Sender<'a, T, const N: usize> {
    chan: &'a Channel<T, N>,
}

/// Receiving half of an SPSC channel.
pub struct Receiver<'a, T, const N: usize> {
    chan: &'a Channel<T, N>,
}

impl<'a, T, const N: usize> Sender<'a, T, N> {
    /// Try to send without blocking. Returns `Ok(())` if sent, `Err(val)` if full.
    pub fn try_send(&self, val: T) -> Result<(), T> {
        let chan = self.chan;
        let head = chan.head.load(crate::sync::atomic::Ordering::Acquire);
        let tail = chan.tail.load(crate::sync::atomic::Ordering::Relaxed);
        let next_tail = (tail + 1) % N;

        if next_tail == head {
            return Err(val);
        }

        // SAFETY: the slot at `tail` is free (the SPSC capacity check
        // `next_tail != head` guarantees the receiver hasn't consumed past
        // it), and only this sender writes; the value is published by the
        // `tail.store(Release)` below.
        unsafe {
            (*chan.buffer.get())[tail].write(val);
        }
        chan.tail
            .store(next_tail, crate::sync::atomic::Ordering::Release);
        wake_waiter(&chan.recv_waiter);
        Ok(())
    }

    /// Send a value, yielding the task if the channel is full.
    ///
    /// # Panics
    /// Panics if polled outside of a task context while the channel is full.
    pub fn send(&self, val: T) -> SendFut<'_, 'a, T, N> {
        SendFut {
            tx: self,
            val: Some(val),
            registered: false,
        }
    }
}

impl<'a, T, const N: usize> Receiver<'a, T, N> {
    /// Try to receive without blocking.
    pub fn try_recv(&self) -> Option<T> {
        let chan = self.chan;
        let head = chan.head.load(crate::sync::atomic::Ordering::Relaxed);
        let tail = chan.tail.load(crate::sync::atomic::Ordering::Acquire);

        if head == tail {
            return None;
        }

        // SAFETY: the slot at `head` holds a value written by the sender
        // and not yet consumed (SPSC: head < tail after the emptiness
        // check); only this receiver reads, and `head.store(Release)`
        // publishes the consumption.
        let val = unsafe { (*chan.buffer.get())[head].assume_init_read() };
        chan.head
            .store((head + 1) % N, crate::sync::atomic::Ordering::Release);
        wake_waiter(&chan.send_waiter);
        Some(val)
    }

    /// Receive a value, yielding the task if the channel is empty.
    ///
    /// # Panics
    /// Panics if polled outside of a task context while the channel is empty.
    pub fn recv(&self) -> Recv<'_, 'a, T, N> {
        Recv {
            rx: self,
            registered: false,
        }
    }
}

/// Future returned by [`Sender::send`].
pub struct SendFut<'b, 'a, T, const N: usize> {
    tx: &'b Sender<'a, T, N>,
    val: Option<T>,
    /// True while registered as a `send_waiter` (cleared on completion or
    /// drop — plan.md §2.5: a cancelled send must not leave a stale waiter
    /// that a later `recv` would spuriously wake).
    registered: bool,
}

impl<'b, 'a, T, const N: usize> SendFut<'b, 'a, T, N> {
    fn clear_registration(&self) {
        if self.registered {
            self.tx
                .chan
                .send_waiter
                .store(NO_WAITER, crate::sync::atomic::Ordering::Release);
        }
    }
}

impl<'b, 'a, T, const N: usize> Drop for SendFut<'b, 'a, T, N> {
    fn drop(&mut self) {
        self.clear_registration();
    }
}

impl<'b, 'a, T, const N: usize> Future for SendFut<'b, 'a, T, N> {
    type Output = ();

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<()> {
        // SAFETY: `SendFut` is not `Unpin`-sensitive — its fields (a
        // `&Sender` and an `Option<T>`) are safe to move; the future is
        // only ever polled through the pinned executor path, and the
        // projection keeps the struct's own invariants intact.
        let this = unsafe { self.get_unchecked_mut() };
        let val = this
            .val
            .take()
            .expect("Send future polled after completion");

        match this.tx.try_send(val) {
            Ok(()) => {
                this.clear_registration();
                Poll::Ready(())
            }
            Err(v) => {
                let id = crate::executor::current_task()
                    .expect("Sender::send().await polled outside of a task context");
                register_waiter(&this.tx.chan.send_waiter, id);
                this.registered = true;

                // Re-check: recv() may have freed a slot between try_send and
                // register_waiter above.
                match this.tx.try_send(v) {
                    Ok(()) => {
                        this.clear_registration();
                        Poll::Ready(())
                    }
                    Err(v) => {
                        this.val = Some(v);
                        Poll::Pending
                    }
                }
            }
        }
    }
}

/// Future returned by [`Receiver::recv`].
pub struct Recv<'b, 'a, T, const N: usize> {
    rx: &'b Receiver<'a, T, N>,
    /// Waiter registration to clear on drop (plan.md §2.5).
    registered: bool,
}

impl<'b, 'a, T, const N: usize> Recv<'b, 'a, T, N> {
    fn clear_registration(&self) {
        if self.registered {
            self.rx
                .chan
                .recv_waiter
                .store(NO_WAITER, crate::sync::atomic::Ordering::Release);
        }
    }
}

impl<'b, 'a, T, const N: usize> Drop for Recv<'b, 'a, T, N> {
    fn drop(&mut self) {
        self.clear_registration();
    }
}

impl<'b, 'a, T, const N: usize> Future for Recv<'b, 'a, T, N> {
    type Output = T;

    fn poll(self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<T> {
        // SAFETY: `Recv` holds only a `&Receiver`, which is `Unpin`;
        // moving it while pinned is sound.
        let this = unsafe { self.get_unchecked_mut() };

        if let Some(v) = this.rx.try_recv() {
            this.clear_registration();
            return Poll::Ready(v);
        }

        let id = crate::executor::current_task()
            .expect("Receiver::recv().await polled outside of a task context");
        register_waiter(&this.rx.chan.recv_waiter, id);
        this.registered = true;

        // Re-check: send() may have fired between try_recv and register_waiter.
        if let Some(v) = this.rx.try_recv() {
            this.clear_registration();
            return Poll::Ready(v);
        }

        Poll::Pending
    }
}

// Safety: Channel access is split between Sender (one owner) and Receiver
// (one owner). Atomic head/tail/waiter fields give lock-free SPSC semantics.
unsafe impl<T: Send, const N: usize> Sync for Channel<T, N> {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_recv_roundtrip() {
        crate::kernel_test! {
            static CHAN: Channel<u32, 8> = Channel::new();
            let (tx, rx) = CHAN.split().expect("split once");

            assert!(tx.try_send(42).is_ok());
            assert_eq!(rx.try_recv(), Some(42));
            assert_eq!(rx.try_recv(), None);
        }
    }

    #[test]
    fn full_channel_blocks_send() {
        crate::kernel_test! {
            static CHAN: Channel<u32, 3> = Channel::new(); // capacity = N-1 = 2
            let (tx, rx) = CHAN.split().expect("split once");

            assert!(tx.try_send(1).is_ok());
            assert!(tx.try_send(2).is_ok());
            assert_eq!(tx.try_send(3), Err(3));

            assert_eq!(rx.try_recv(), Some(1));
            assert!(tx.try_send(3).is_ok());
            assert_eq!(rx.try_recv(), Some(2));
            assert_eq!(rx.try_recv(), Some(3));
        }
    }

    #[test]
    fn empty_channel_try_recv_none() {
        crate::kernel_test! {
            static CHAN: Channel<u32, 4> = Channel::new();
            let (_tx, rx) = CHAN.split().expect("split once");
            assert_eq!(rx.try_recv(), None);
        }
    }

    #[test]
    fn recv_future_ready_when_data_present() {
        crate::kernel_test! {
            static CHAN: Channel<u32, 4> = Channel::new();
            let (tx, rx) = CHAN.split().expect("split once");
            tx.try_send(7).unwrap();

            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = rx.recv();
            // SAFETY: `fut` is a local `Recv` future; it is `Unpin`
            // (holds only a `&mut Receiver`) and is never moved while
            // pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            assert_eq!(pinned.poll(&mut cx), Poll::Ready(7));
        }
    }

    #[test]
    #[should_panic(expected = "outside of a task context")]
    fn recv_future_panics_without_task_context_when_empty() {
        crate::kernel_test! {
            static CHAN: Channel<u32, 4> = Channel::new();
            let (_tx, rx) = CHAN.split().expect("split once");

            let waker = crate::waker::task_waker(crate::task::TaskId::new(0, 0));
            let mut cx = Context::from_waker(&waker);
            let mut fut = rx.recv();
            // SAFETY: `fut` is a local `Recv` future; `Unpin`, never
            // moved while pinned — sound for this single poll.
            let pinned = unsafe { Pin::new_unchecked(&mut fut) };
            let _ = pinned.poll(&mut cx);
        }
    }
}
