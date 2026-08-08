//! Host-side integration tests for Rivet RTOS.
//!
//! These exercise real `async fn` task bodies end-to-end (TaskCell storage,
//! executor wake/poll cycle, and the async sync primitives), not just the
//! low-level scheduler primitives in isolation.

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use rivet::sync::{Channel, Semaphore};
use rivet::task::TaskCell;
use rivet::waker;

#[test]
fn waker_mark_and_dequeue() {
    rivet::kernel_test! {
    waker::mark_ready(rivet::task::TaskId::new(2, 0));
    waker::mark_ready(rivet::task::TaskId::new(5, 0));
    waker::mark_ready(rivet::task::TaskId::new(1, 0));

    assert_eq!(waker::next_ready(), Some(rivet::task::TaskId::new(5, 0)));
    assert_eq!(waker::next_ready(), Some(rivet::task::TaskId::new(2, 0)));
    assert_eq!(waker::next_ready(), Some(rivet::task::TaskId::new(1, 0)));
    assert_eq!(waker::next_ready(), None);
    }
}

#[test]
fn semaphore_try_acquire_release() {
    rivet::kernel_test! {
    let sem: Semaphore<3> = Semaphore::new(1);
    assert!(sem.try_acquire());
    assert!(!sem.try_acquire());
    sem.release();
    assert!(sem.try_acquire());
    }
}

#[test]
fn semaphore_acquire_future_blocks_then_wakes() {
    rivet::kernel_test! {
    static SEM: Semaphore<1> = Semaphore::new(0);

    rivet::executor::set_current_for_test(1, 0);
    let w = waker::task_waker(rivet::task::TaskId::new(1, 0));
    let mut cx = Context::from_waker(&w);
    let mut fut = SEM.acquire();
    let pinned = unsafe { Pin::new_unchecked(&mut fut) };
    assert_eq!(pinned.poll(&mut cx), Poll::Pending);
    rivet::executor::clear_current_for_test();

    // release() from "ISR"/other-task context wakes task (1, 0).
    SEM.release();
    assert!(waker::has_pending());
    assert_eq!(waker::next_ready(), Some(rivet::task::TaskId::new(1, 0)));

    // Re-polling now succeeds.
    rivet::executor::set_current_for_test(1, 0);
    let mut fut2 = SEM.acquire();
    let pinned2 = unsafe { Pin::new_unchecked(&mut fut2) };
    assert_eq!(pinned2.poll(&mut cx), Poll::Ready(()));
    rivet::executor::clear_current_for_test();
    }
}

#[test]
fn channel_send_recv_roundtrip() {
    rivet::kernel_test! {
    static CHAN: Channel<u32, 8> = Channel::new();
    let (tx, rx) = CHAN.split().expect("split once");

    assert!(tx.try_send(42).is_ok());
    assert_eq!(rx.try_recv(), Some(42));
    assert_eq!(rx.try_recv(), None);
    }
}

#[test]
fn channel_full_then_drain() {
    rivet::kernel_test! {
    static CHAN: Channel<u32, 3> = Channel::new(); // N-1 = 2 usable slots
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

// ── End-to-end: real async fn tasks driven through TaskCell ───────────

static PRODUCER_CHAN: Channel<u32, 4> = Channel::new();
static E2E_LOG: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

// Split exactly once (plan.md [B8]); the tasks borrow the halves.
static E2E_TX: std::sync::OnceLock<rivet::sync::Sender<'static, u32, 4>> =
    std::sync::OnceLock::new();
static E2E_RX: std::sync::OnceLock<rivet::sync::Receiver<'static, u32, 4>> =
    std::sync::OnceLock::new();

async fn e2e_producer() {
    let tx = E2E_TX.get().expect("split before tasks run");
    for i in 1..=3u32 {
        tx.send(i).await;
    }
    E2E_LOG.fetch_or(0b001, core::sync::atomic::Ordering::Relaxed);
}

async fn e2e_consumer() {
    let rx = E2E_RX.get().expect("split before tasks run");
    let mut sum = 0u32;
    for _ in 0..3 {
        sum += rx.recv().await;
    }
    assert_eq!(sum, 6);
    E2E_LOG.fetch_or(0b010, core::sync::atomic::Ordering::Relaxed);
}

/// Drives a `TaskCell`-backed task manually (as the executor would), polling
/// it whenever its waker fires, without requiring the linker-section-based
/// `.rivet_tasks` discovery (host tests don't link with our custom scripts).
fn drive_to_completion<F: Future<Output = ()> + 'static>(
    cell: &'static TaskCell<512>,
    init: fn() -> F,
    priority: u8,
    index: u8,
    max_polls: usize,
) -> bool {
    for _ in 0..max_polls {
        rivet::executor::set_current_for_test(priority, index);
        let w = waker::task_waker(rivet::task::TaskId::new(priority, index));
        let result = unsafe { cell.poll(init, &w) };
        rivet::executor::clear_current_for_test();
        if result == Poll::Ready(()) {
            return true;
        }
    }
    false
}

#[test]
fn end_to_end_producer_consumer_via_real_async_fn() {
    rivet::kernel_test! {
    E2E_LOG.store(0, core::sync::atomic::Ordering::Relaxed);
    let (tx, rx) = PRODUCER_CHAN.split().expect("split once");
    let _ = E2E_TX.set(tx);
    let _ = E2E_RX.set(rx);

    static PRODUCER_CELL: TaskCell<512> = TaskCell::new();
    static CONSUMER_CELL: TaskCell<512> = TaskCell::new();

    // Cooperative round-robin: poll each once per round until both finish.
    // Mimics the executor's "poll ready tasks" loop without needing the
    // full linker-section task registry.
    let mut producer_done = false;
    let mut consumer_done = false;
    for _ in 0..20 {
        if !producer_done {
            producer_done = drive_to_completion(&PRODUCER_CELL, e2e_producer, 1, 0, 1);
        }
        if !consumer_done {
            consumer_done = drive_to_completion(&CONSUMER_CELL, e2e_consumer, 0, 0, 1);
        }
        if producer_done && consumer_done {
            break;
        }
    }

    assert!(producer_done, "producer task did not complete");
    assert!(consumer_done, "consumer task did not complete");
    assert_eq!(E2E_LOG.load(core::sync::atomic::Ordering::Relaxed), 0b011);
    }
}
