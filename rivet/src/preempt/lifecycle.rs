//! Task lifecycle: exit, join, stop (plan.md §5).
//!
//! A preemptive task whose entry *returns* lands in
//! [`rivet_task_exit_core`] (the arch trampoline jumps there): the return
//! value (≤ 8 bytes, carried in a0/a1 — larger returns need the hidden
//! sret pointer which the trampoline cannot provide, so sizes > 8 are
//! rejected at spawn) is stored type-erased in the TCB, the task is marked
//! `exited`, and its joiner (if any) is woken. `TaskHandle::join` blocks
//! until then and recovers the value, or reports [`JoinError::Faulted`]
//! when the task was isolated by the fault policy (plan.md §3.4), or
//! [`JoinError::Stale`] when the handle's generation no longer matches
//! (the slot was recycled).

use core::sync::atomic::Ordering;

use super::sched;
use super::tcb::{self, NO_TASK};

/// Errors from [`super::TaskHandle::join`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JoinError {
    /// The task slot was recycled since this handle was created (stale
    /// generation — ABA detection, plan.md §5.1).
    Stale,
    /// A task cannot join itself.
    SelfJoin,
    /// Another task is already joined to this one (one joiner per task).
    AlreadyJoined,
    /// The task was isolated by the fault policy before exiting (plan.md
    /// §3.4); it produced no result.
    Faulted,
}

/// Arch trampoline target: the task's entry returned; `val_lo`/`val_hi`
/// carry the return value (or, for >8-byte results, `val_lo` is a pointer
/// into this task's own stack). Stores the result, marks the task exited,
/// wakes its joiner, and parks forever.
#[no_mangle]
pub extern "C" fn rivet_task_exit_core(val_lo: usize, val_hi: usize) -> ! {
    if let Some(id) = sched::current() {
        if let Some(t) = tcb::get(id) {
            let size = t.result_size.load(Ordering::Acquire) as usize;
            // SAFETY: the result buffer is only written here (once) and
            // read by join() after `exited` is published; the task itself
            // is the sole writer.
            let buf = unsafe { &mut *t.result_buf.get() };
            if size > 0 && size <= buf.len() {
                if size <= 8 {
                    let lo = val_lo.to_le_bytes();
                    let hi = val_hi.to_le_bytes();
                    for (i, b) in lo.iter().chain(hi.iter()).take(size).enumerate() {
                        buf[i] = *b;
                    }
                } else {
                    // val_lo points into this task's own stack.
                    // SAFETY: own stack, `size` bytes initialized by the
                    // caller before returning.
                    let src = val_lo as *const u8;
                    for (i, slot) in buf.iter_mut().take(size).enumerate() {
                        // SAFETY: `i < size` and `src` is a valid pointer
                        // into this task's own stack (the caller's sret
                        // area), initialized before the entry returned.
                        *slot = unsafe { core::ptr::read_volatile(src.add(i)) };
                    }
                }
            }
            // Publish the result, then the exited flag (join reads both),
            // then wake the joiner — all under one critical section so a
            // tick can't observe `exited` set but the joiner not yet
            // woken (or worse, `unblock`'s `ready_add` interrupted
            // mid-update, tearing `READY_BITMAP`/`QUEUES`; see
            // `PriorityMutexGuard::drop`'s identical reasoning).
            crate::critical::enter(|| {
                t.exited.store(true, Ordering::Release);
                let joiner = t.joiner.swap(NO_TASK, Ordering::AcqRel);
                if joiner != NO_TASK {
                    sched::unblock(joiner);
                }
            });
        }
    }
    // Park forever (the slot stays used until explicitly despawned).
    loop {
        sched::block_current();
        crate::port::arch::request_reschedule();
    }
}

/// Cooperative cancellation (plan.md §5.4): poll this from the task's main
/// loop; returns true once [`super::TaskHandle::request_stop`] was called
/// on the current task.
pub fn should_stop() -> bool {
    sched::current()
        .and_then(tcb::get)
        .map(|t| t.stop_requested.load(Ordering::Acquire))
        .unwrap_or(false)
}

/// Implementation of [`super::TaskHandle::join`].
pub fn join_task<T: 'static + Send>(handle: &super::TaskHandle) -> Result<T, JoinError> {
    let id = handle.id as usize;
    let Some(t) = tcb::get(id) else {
        return Err(JoinError::Stale);
    };
    if t.generation.load(Ordering::Acquire) != handle.generation {
        return Err(JoinError::Stale);
    }
    let me = sched::current().unwrap_or(NO_TASK);
    if Some(id) == sched::current() {
        return Err(JoinError::SelfJoin);
    }

    // Register as the joiner (single-joiner support, documented).
    //
    // plan.md Phase 17 (found via soak testing at scale): the exit path's
    // own `joiner.swap(NO_TASK)` is only a *correctness optimization* for
    // the common case where the joiner registers before the target
    // exits — it must not be the sole owner of clearing this field. If
    // the target (often spawned at a *higher* priority, so it can run to
    // completion before this task even reaches the CAS below) exits
    // first, its swap drains a `joiner` that's still `NO_TASK` — a
    // no-op — and this CAS then lands on a slot nobody will ever clear
    // again, permanently poisoning it for the *next* occupant after a
    // respawn. `register_full` now resets `joiner` unconditionally on
    // every respawn (the real fix for that specific symptom), but this
    // function still needs to release its *own* registration on every
    // return path below — relying on the target's exit path to do it is
    // exactly the assumption that was wrong.
    if t.joiner
        .compare_exchange(NO_TASK, me, Ordering::AcqRel, Ordering::Acquire)
        .is_err()
    {
        return Err(JoinError::AlreadyJoined);
    }

    // Wait for exit (the exit path wakes us). The check-then-block
    // sequence must be atomic w.r.t. a tick landing in between — same
    // [B1] pattern as `PriorityMutex::lock_timeout` — or a tick that
    // lands after the load but before `block_current()` marks this task
    // Blocked *after* the target has already exited and called
    // `unblock` on a still-Ready task (a no-op), producing a lost
    // wakeup: this task then blocks itself with nothing left to ever
    // wake it.
    loop {
        let must_wait = crate::critical::enter(|| {
            if t.exited.load(Ordering::Acquire) {
                false
            } else {
                sched::block_current();
                true
            }
        });
        if !must_wait {
            break;
        }
        crate::port::arch::request_reschedule();
    }

    // Release our own registration now that we've observed the exit —
    // `compare_exchange(me, NO_TASK)`, not a blind store: if the slot
    // was somehow already recycled and re-registered by a new joiner
    // underneath us (shouldn't happen given the checks above, but this
    // makes it a no-op instead of clobbering someone else's claim).
    let _ = t
        .joiner
        .compare_exchange(me, NO_TASK, Ordering::AcqRel, Ordering::Acquire);

    // Recover the result. The buffer holds the bytes of a `T` written by
    // the exit path; T's size was validated at spawn (≤ 8 bytes).
    // SAFETY: the buffer is initialized (exited is published after the
    // write, and we read after observing `exited` with Acquire); reading it
    // as `T` matches the type written at spawn.
    let size = t.result_size.load(Ordering::Acquire) as usize;
    if size != core::mem::size_of::<T>() {
        return Err(JoinError::Faulted);
    }
    // SAFETY: as above.
    Ok(unsafe { core::ptr::read(t.result_buf.get() as *const T) })
}
