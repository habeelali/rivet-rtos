//! Loom permutation models for the lock-free core (plan.md §1.3).
//!
//! Built and run with:
//! ```text
//! RUSTFLAGS='--cfg loom' cargo test -p rivet-rtos --test loom --release
//! ```
//!
//! Models:
//! - **waker**: `mark_ready` (ISR side) vs `next_ready` (executor side) —
//!   no wakeup may be lost, every marked task dequeues exactly once.
//! - **semaphore**: `release()` vs `Acquire::poll` — the double-check in
//!   `Acquire::poll` must be sufficient for a single waiter (no lost
//!   wakeup, no double-acquire). The two-waiter case ([B9], single waiter
//!   slot) is captured as a known-issue `#[ignore]`d test that Phase 2.5
//!   closes.
//! - **channel**: SPSC `try_send`/`try_recv` — every value sent is
//!   received exactly once, in order, nothing lost or duplicated.
//! - **signal**: `signal()` (ISR side) vs `wait().poll` (task side) — the
//!   register-then-recheck idiom must be sufficient: no interleaving may
//!   leave the waiter both unregistered-by-drop and the latch unset.

#![cfg(loom)]

use core::future::Future;
use loom::thread;

use rivet::sync::atomic::Ordering;
use rivet::sync::{Channel, Semaphore, Signal};
use rivet::waker;

/// Run `f` inside a loom model with kernel state reset first.
fn model<F: Fn() + Send + Sync + 'static>(f: F) {
    loom::model(move || {
        rivet::waker::reset();
        f();
    });
}

// Fresh channel per model run (loom's atomics are not const-constructible,
// so `Channel::new` can't initialize a plain `static`).
loom::lazy_static! {
    static ref CHAN: Channel<u32, 4> = Channel::new();
}

// ── (a) waker: mark_ready vs next_ready ────────────────────────────

#[test]
fn waker_no_lost_wakeups_single_producer() {
    model(|| {
        const N: usize = 2;
        let mut handles = Vec::new();
        for i in 0..N {
            let h = thread::spawn(move || {
                waker::mark_ready(rivet::task::TaskId::new(2, i as u8));
            });
            handles.push(h);
        }
        for h in handles {
            h.join().unwrap();
        }

        let mut seen = [false; N];
        while let Some(id) = waker::next_ready() {
            assert_eq!(id.priority(), 2, "unexpected priority");
            let idx = id.index() as usize;
            assert!(!seen[idx], "task {idx} dequeued twice");
            seen[idx] = true;
        }
        // Bitmap drains must not lose any of the marked tasks.
        assert!(seen.iter().all(|&s| s), "lost wakeups: {seen:?}");
    });
}

// ── (b) semaphore: release vs acquire ──────────────────────────────

#[test]
fn semaphore_single_waiter_no_lost_wakeup() {
    model(|| {
        let sem: loom::sync::Arc<Semaphore<1>> = loom::sync::Arc::new(Semaphore::new(0));

        // Waiter thread: register as (1, 0) and poll acquire once.
        let sem_w = sem.clone();
        let w = thread::spawn(move || {
            rivet::executor::set_current_for_test(1, 0);
            let waker = rivet::waker::task_waker(rivet::task::TaskId::new(1, 0));
            let mut cx = core::task::Context::from_waker(&waker);
            let mut fut = sem_w.acquire();
            let pinned = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
            pinned.poll(&mut cx).is_ready()
        });

        // Signaller thread: release the semaphore (ISR-style).
        let sem_s = sem.clone();
        let s = thread::spawn(move || {
            sem_s.release();
        });

        let poll_ready = w.join().unwrap();
        s.join().unwrap();

        // No lost wakeup means exactly one of these must hold:
        //  - the waiter's own poll already acquired (Ready), or
        //  - the waiter is marked ready in the waker bitmap, or
        //  - the token is still available (count > 0) for its next poll.
        assert!(
            poll_ready || waker::has_pending() || sem.try_acquire(),
            "lost wakeup: waiter never woken, semaphore never released"
        );
    });
}

/// [B9] regression: with two waiters registered and two releases, *both*
/// waiters must eventually be woken or have acquired. The old single-slot
/// waiter storage overwrote the first waiter's registration, permanently
/// losing its wakeup. Registration is done sequentially (the executor is
/// single-threaded; the old overwrite happens between any two registrations
/// regardless of concurrency), then releases run from two threads so loom
/// permutes their handoff interleavings.
// Two waiters: previously failed with a lost wakeup ([B9]); now passes
// after the Phase 2.5 rework. Kept un-ignored so any regression fails CI.
#[test]
fn semaphore_two_waiters_no_lost_wakeup() {
    model(|| {
        let sem: loom::sync::Arc<Semaphore<1>> = loom::sync::Arc::new(Semaphore::new(0));

        // Both waiters register (each poll returns Pending and the future
        // is leaked so its registration survives — as the executor keeps
        // it alive).
        let mut poll1 = false;
        let mut poll2 = false;
        {
            rivet::executor::set_current_for_test(1, 0);
            let waker = rivet::waker::task_waker(rivet::task::TaskId::new(1, 0));
            let mut cx = core::task::Context::from_waker(&waker);
            let mut fut = sem.acquire();
            let pinned = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
            poll1 = pinned.poll(&mut cx).is_ready();
            core::mem::forget(fut);
        }
        {
            rivet::executor::set_current_for_test(2, 0);
            let waker = rivet::waker::task_waker(rivet::task::TaskId::new(2, 0));
            let mut cx = core::task::Context::from_waker(&waker);
            let mut fut = sem.acquire();
            let pinned = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
            poll2 = pinned.poll(&mut cx).is_ready();
            core::mem::forget(fut);
        }
        rivet::executor::clear_current_for_test();

        let sem_s1 = sem.clone();
        let s1 = thread::spawn(move || {
            sem_s1.release();
        });
        let sem_s2 = sem.clone();
        let s2 = thread::spawn(move || {
            sem_s2.release();
        });
        s1.join().unwrap();
        s2.join().unwrap();

        // Drain the waker bitmap to see which tasks were explicitly woken.
        let mut marked: Vec<(u8, u8)> = Vec::new();
        while let Some(id) = waker::next_ready() {
            marked.push((id.priority(), id.index()));
        }

        // Every waiter must either have acquired in its own poll or been
        // woken by a release — a lost wakeup (old [B9] single-slot
        // overwrite) leaves one waiter permanently asleep.
        let w1_ok = poll1 || marked.contains(&(1, 0));
        let w2_ok = poll2 || marked.contains(&(2, 0));
        assert!(
            w1_ok && w2_ok,
            "[B9] lost wakeup: poll1={poll1} poll2={poll2} marked={marked:?}"
        );
    });
}

// ── (c) channel: SPSC try_send/try_recv ────────────────────────────

#[test]
fn channel_spsc_every_value_received_exactly_once() {
    model(|| {
        let (tx, rx) = CHAN.split().expect("split once");

        let producer = thread::spawn(move || {
            for v in 1..=3u32 {
                // Loop until the slot frees (try_send may fail on a full
                // ring; the model's point is the atomic handoff, not
                // blocking).
                while tx.try_send(v).is_err() {
                    loom::hint::spin_loop();
                }
            }
        });

        let consumer = thread::spawn(move || {
            let mut got = Vec::new();
            while got.len() < 3 {
                if let Some(v) = rx.try_recv() {
                    got.push(v);
                } else {
                    loom::hint::spin_loop();
                }
            }
            got
        });

        producer.join().unwrap();
        let got = consumer.join().unwrap();
        assert_eq!(
            got,
            vec![1, 2, 3],
            "SPSC values lost, duplicated, or reordered"
        );
    });
}

// ── (d) signal: signal() vs wait().poll ────────────────────────────

#[test]
fn signal_no_lost_wakeup() {
    model(|| {
        let sig: loom::sync::Arc<Signal> = loom::sync::Arc::new(Signal::new());

        // Waiter thread: register as (1, 0) and poll wait() once.
        let sig_w = sig.clone();
        let w = thread::spawn(move || {
            rivet::executor::set_current_for_test(1, 0);
            let waker = rivet::waker::task_waker(rivet::task::TaskId::new(1, 0));
            let mut cx = core::task::Context::from_waker(&waker);
            let mut fut = sig_w.wait();
            let pinned = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
            pinned.poll(&mut cx).is_ready()
            // `fut` (and its registration, if any) drops here — exactly
            // like `Semaphore::acquire()`'s equivalent single-waiter model
            // above, this is deliberate: it exercises the case where
            // `signal()` races against a registration that's about to be
            // (or just was) cancelled.
        });

        // Signaller thread: fire the signal (ISR-style).
        let sig_s = sig.clone();
        let s = thread::spawn(move || {
            sig_s.signal();
        });

        let poll_ready = w.join().unwrap();
        s.join().unwrap();

        // No lost wakeup means exactly one of these must hold:
        //  - the waiter's own poll already observed Ready, or
        //  - the waiter is marked ready in the waker bitmap, or
        //  - the latch is still set (try_take) for the next poll to see —
        //    covers signal() firing after the waiter's Drop already
        //    cleared its registration.
        assert!(
            poll_ready || waker::has_pending() || sig.try_take(),
            "lost wakeup: waiter never woken, signal never observed"
        );
    });
}
