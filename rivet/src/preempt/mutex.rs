//! Priority-inheritance mutex for the preemptive tier.
//!
//! Classic priority inversion: a low-priority task holds a resource; a
//! high-priority task blocks waiting for it; a *medium*-priority task
//! (uninvolved with the resource) preempts the low-priority holder and
//! runs indefinitely, indirectly blocking the high-priority task for far
//! longer than the critical section itself would ever take.
//!
//! Priority inheritance fixes this: while a higher-priority task is
//! blocked on a mutex, the current holder's *effective* priority is
//! boosted to match, so it can't be preempted by anything the waiter
//! itself couldn't preempt. The boost is undone on unlock — but see [B11]:
//! with *nested* mutexes, unlocking one must recompute the boost from the
//! remaining held mutexes rather than blindly restoring the base priority.

use core::cell::UnsafeCell;
use core::ops::{Deref, DerefMut};

use super::sched;
use super::tcb::{self, MAX_PTASKS, NO_TASK};
use crate::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// Error returned by [`PriorityMutex::lock_timeout`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LockError {
    /// The calling task already holds this mutex (re-entrancy — a
    /// self-deadlock).
    Recursive,
    /// `lock_timeout`'s deadline passed before the mutex was acquired.
    Timeout,
    /// The calling task already holds [`tcb::MAX_HELD`] mutexes.
    TooManyHeldMutexes,
    /// Called outside of a preemptive task context.
    NotInTask,
    /// The mutex was poisoned: its previous holder faulted while holding
    /// it (plan.md §3.4). The data may be inconsistent.
    Poisoned,
}

/// A mutex that applies priority inheritance to its holder while
/// higher-priority tasks are waiting on it.
///
/// `#[repr(C)]`: the fault-isolation path (`poison_mutex`) casts a
/// type-erased pointer to `PriorityMutex<()>`, so the leading-field layout
/// must be identical across monomorphizations.
#[repr(C)]
pub struct PriorityMutex<T> {
    locked: AtomicBool,
    owner: AtomicUsize,
    waiters: [AtomicUsize; MAX_PTASKS],
    /// Set when a faulting holder was isolated while holding this mutex
    /// (plan.md §3.4). `lock()`/`try_lock()` fail with
    /// [`LockError::Poisoned`] once set.
    poisoned: AtomicBool,
    data: UnsafeCell<T>,
}

// Safety: access to `data` is only granted through `PriorityMutexGuard`,
// obtained while `locked` is held — standard mutex safety argument.
unsafe impl<T: Send> Sync for PriorityMutex<T> {}

impl<T> PriorityMutex<T> {
    #[cfg(not(loom))]
    pub const fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            owner: AtomicUsize::new(NO_TASK),
            // Inline const avoids a named `const` item with interior
            // mutability (clippy::declare_interior_mutable_const).
            waiters: [const { AtomicUsize::new(NO_TASK) }; MAX_PTASKS],
            poisoned: AtomicBool::new(false),
            data: UnsafeCell::new(value),
        }
    }

    /// Loom's atomics are not const-constructible; runtime constructor.
    #[cfg(loom)]
    pub fn new(value: T) -> Self {
        Self {
            locked: AtomicBool::new(false),
            owner: AtomicUsize::new(NO_TASK),
            waiters: core::array::from_fn(|_| AtomicUsize::new(NO_TASK)),
            poisoned: AtomicBool::new(false),
            data: UnsafeCell::new(value),
        }
    }

    /// Whether this mutex has been poisoned by a faulting holder.
    pub fn is_poisoned(&self) -> bool {
        self.poisoned.load(Ordering::Acquire)
    }

    /// Non-blocking acquire. Returns `None` if the mutex is held (by
    /// anyone, including this task).
    pub fn try_lock(&self) -> Option<PriorityMutexGuard<'_, T>> {
        let me = sched::current()?;
        self.try_acquire_guarded(me)
    }

    /// Acquire the mutex, blocking (yielding the CPU to other preemptive
    /// tasks — not busy-spinning) while it's held elsewhere. Applies
    /// priority inheritance to the current holder for as long as this
    /// task waits.
    ///
    /// # Panics
    /// Panics if called outside of a preemptive task context, if the
    /// calling task already holds this mutex (recursive lock), or if the
    /// task already holds [`tcb::MAX_HELD`] mutexes.
    pub fn lock(&self) -> PriorityMutexGuard<'_, T> {
        match self.lock_timeout(None) {
            Ok(g) => g,
            Err(LockError::Recursive) => panic!(
                "PriorityMutex::lock: recursive lock from the same task \
                 (self-deadlock; use try_lock or restructure)"
            ),
            Err(LockError::TooManyHeldMutexes) => panic!(
                "PriorityMutex::lock: task already holds MAX_HELD={} mutexes",
                tcb::MAX_HELD
            ),
            Err(LockError::NotInTask) => {
                panic!("PriorityMutex::lock() outside preemptive task context")
            }
            Err(LockError::Poisoned) => panic!(
                "PriorityMutex::lock: mutex poisoned by a faulting holder                  (data may be inconsistent; use lock_timeout/try_lock to recover)"
            ),
            Err(LockError::Timeout) => unreachable!("lock() has no timeout"),
        }
    }

    /// Acquire the mutex with a deadline. Returns
    /// [`LockError::Timeout`] if the mutex is not acquired within
    /// `timeout`; `None` waits forever.
    pub fn lock_timeout(
        &self,
        timeout: Option<crate::time::Duration>,
    ) -> Result<PriorityMutexGuard<'_, T>, LockError> {
        let me = sched::current().ok_or(LockError::NotInTask)?;
        let deadline = timeout.map(|d| crate::port::board::now_us().wrapping_add(d.as_micros()));

        loop {
            // Fast path (lock free).
            if self.poisoned.load(Ordering::Acquire) {
                return Err(LockError::Poisoned);
            }
            if let Some(g) = self.try_acquire_guarded(me) {
                return Ok(g);
            }

            // Re-entrancy: only this task could have failed the CAS while
            // being the owner.
            if self.owner.load(Ordering::Acquire) == me {
                return Err(LockError::Recursive);
            }

            if let Some(d) = deadline {
                if crate::port::board::now_us() >= d {
                    // Deregister so a later unlock can't spuriously wake a
                    // task that is no longer waiting.
                    self.remove_waiter(me);
                    crate::timer::cancel_ptask_deadline(me);
                    return Err(LockError::Timeout);
                }
            }

            // Slow path — the whole check/register/block sequence runs
            // with interrupts disabled (plan.md [B1]): the CAS is
            // *re-tested* inside the critical section, so a tick that
            // lands between the failed fast-path CAS and add_waiter can
            // never let the holder run to completion and release with
            // nobody registered — the re-test catches the release.
            let outcome = crate::critical::enter(|| {
                if let Some(g) = self.try_acquire_guarded(me) {
                    Ok(g)
                } else {
                    self.boost_holder(me);
                    self.add_waiter(me);
                    if let Some(d) = deadline {
                        let _ = crate::timer::register_ptask_deadline(d, me);
                    }
                    sched::block_current();
                    Err(LockError::Timeout) // placeholder; only used if the
                                            // caller falls through
                }
            });
            match outcome {
                Ok(g) => return Ok(g),
                Err(_) => {
                    // Registered as a waiter and blocked; actually give up
                    // the CPU (outside the critical section; the
                    // software-interrupt/PendSV path handles the switch).
                    // Woken spuriously or by unlock/timeout — loop back
                    // and re-test.
                    crate::port::arch::request_reschedule();
                }
            }
        }
    }

    /// CAS + owner-store + held-list registration in one step. On held-list
    /// overflow the acquire is rolled back (the lock must never be left
    /// held with no guard to release it).
    fn try_acquire_guarded(&self, me: usize) -> Option<PriorityMutexGuard<'_, T>> {
        if self.poisoned.load(Ordering::Acquire) {
            return None;
        }
        if !self.try_acquire(me) {
            return None;
        }
        match self.push_held(me) {
            Ok(()) => Some(PriorityMutexGuard { mutex: self }),
            Err(_) => {
                // Roll back the acquire we just made so the lock is never
                // left held without a guard.
                self.owner.store(NO_TASK, Ordering::Release);
                self.locked.store(false, Ordering::Release);
                None
            }
        }
    }

    /// Attempt the CAS + owner-store. No blocking.
    fn try_acquire(&self, me: usize) -> bool {
        if self
            .locked
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            self.owner.store(me, Ordering::Release);
            true
        } else {
            false
        }
    }

    /// Record this mutex in the calling task's held list ([B11]).
    fn push_held(&self, me: usize) -> Result<(), LockError> {
        let tcb = tcb::get(me).ok_or(LockError::NotInTask)?;
        if tcb.push_held(
            self as *const _ as *const (),
            Self::highest_waiter_priority_erased,
        ) {
            Ok(())
        } else {
            Err(LockError::TooManyHeldMutexes)
        }
    }

    /// Type-erased `highest_waiter_priority` accessor for [`HeldMutex`].
    fn highest_waiter_priority_erased(ptr: *const ()) -> u8 {
        // SAFETY: `ptr` was registered by `push_held` as `self as *const _`
        // for a `PriorityMutex<T>` of this exact type, and the mutex is
        // still alive (it is being held).
        unsafe { (&*(ptr as *const PriorityMutex<T>)).highest_waiter_priority() }
    }

    /// Highest *base* priority among the tasks currently waiting on this
    /// mutex (0 if none).
    fn highest_waiter_priority(&self) -> u8 {
        let mut max = 0u8;
        for slot in &self.waiters {
            let id = slot.load(Ordering::Acquire);
            if id != NO_TASK {
                if let Some(w) = tcb::get(id) {
                    let b = w.base_priority.load(Ordering::Acquire);
                    if b > max {
                        max = b;
                    }
                }
            }
        }
        max
    }

    /// Boost the current holder's effective priority to at least ours.
    fn boost_holder(&self, me: usize) {
        let owner_id = self.owner.load(Ordering::Acquire);
        if owner_id != NO_TASK {
            if let (Some(me_tcb), Some(owner_tcb)) = (tcb::get(me), tcb::get(owner_id)) {
                let my_base = me_tcb.base_priority.load(Ordering::Acquire);
                let owner_eff = owner_tcb.effective_priority.load(Ordering::Acquire);
                if my_base > owner_eff {
                    owner_tcb.set_effective_priority(owner_id, my_base);
                }
            }
        }
    }

    fn add_waiter(&self, id: usize) {
        for slot in &self.waiters {
            if slot
                .compare_exchange(NO_TASK, id, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                return;
            }
        }
        // Waiter list full (more concurrent waiters than MAX_PTASKS, which
        // is impossible since MAX_PTASKS bounds total task count) — unreachable.
    }

    /// Deregister a waiter (e.g. it timed out and is no longer waiting).
    fn remove_waiter(&self, id: usize) {
        for slot in &self.waiters {
            let _ = slot.compare_exchange(id, NO_TASK, Ordering::AcqRel, Ordering::Acquire);
        }
    }

    fn wake_all_waiters(&self) {
        for slot in &self.waiters {
            let id = slot.swap(NO_TASK, Ordering::AcqRel);
            if id != NO_TASK {
                sched::unblock(id);
                crate::timer::cancel_ptask_deadline(id);
            }
        }
    }
}

/// Mark a type-erased `PriorityMutex` as poisoned and wake its waiters so
/// they observe the poison (called by the fault-isolation path, plan.md
/// §3.4).
///
/// # Safety
/// `ptr` must be a live `PriorityMutex<T>` address that was registered in
/// some task's held list.
pub unsafe fn poison_mutex(ptr: *const ()) {
    // SAFETY: contract above.
    unsafe {
        let m = &*(ptr as *const PriorityMutex<()>);
        m.poisoned.store(true, Ordering::Release);
        m.wake_all_waiters();
    }
}

/// RAII guard returned by [`PriorityMutex::lock`]. On drop: releases the
/// mutex, recomputes the holder's effective priority from its *remaining*
/// held mutexes ([B11] — nested inheritance), wakes waiters, and requests
/// a reschedule.
pub struct PriorityMutexGuard<'a, T> {
    mutex: &'a PriorityMutex<T>,
}

impl<'a, T> Deref for PriorityMutexGuard<'a, T> {
    type Target = T;
    fn deref(&self) -> &T {
        // SAFETY: access to `data` is only reachable through a
        // `PriorityMutexGuard`, which exists only while `locked` is held;
        // exclusive ownership of `data` is guaranteed by the mutex protocol.
        unsafe { &*self.mutex.data.get() }
    }
}

impl<'a, T> DerefMut for PriorityMutexGuard<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        // SAFETY: same argument as `Deref::deref` — the guard holds the
        // mutex exclusively, and `&mut self` proves no other reference to
        // the data is live.
        unsafe { &mut *self.mutex.data.get() }
    }
}

impl<'a, T> Drop for PriorityMutexGuard<'a, T> {
    fn drop(&mut self) {
        // The whole unlock — held-list update, effective-priority
        // recompute, and waking every waiter — must commit as one step.
        // `ready_add`/`ready_remove` (reached via `set_effective_priority`
        // and `wake_all_waiters`'s `unblock`) touch `READY_BITMAP` and
        // `QUEUES` as two separate atomics; without a critical section
        // here, a tick landing mid-unlock can observe that torn state —
        // e.g. a waiter's `Ready` transition half-applied — and the
        // scheduler's bitmap/queue pair can end up permanently
        // inconsistent (a stale `READY_BITMAP` bit with an empty queue
        // word). Matches the pattern already used for the equivalent
        // lock-side sequence (`lock_timeout`'s own `critical::enter`
        // around boost/add_waiter/register/block) and the join path.
        crate::critical::enter(|| {
            let owner = self.mutex.owner.swap(NO_TASK, Ordering::AcqRel);
            if owner != NO_TASK {
                // Remove this mutex from the holder's held list, then
                // recompute the holder's effective priority from what
                // remains (plan.md [B11]: unlocking one mutex must not
                // drop the boost held for another).
                if let Some(t) = tcb::get(owner) {
                    t.remove_held(self.mutex as *const _ as *const ());
                    let base = t.base_priority.load(Ordering::Acquire);
                    let mut eff = base;
                    for slot in &t.held {
                        let ptr = slot.ptr.load(Ordering::Acquire);
                        if !ptr.is_null() {
                            // SAFETY: the hwp fn pointer was registered by
                            // the matching `push_held` for a live mutex;
                            // loading ptr (Acquire) orders the hwp read.
                            let hwp = unsafe {
                                let f: fn(*const ()) -> u8 =
                                    core::mem::transmute(slot.hwp.load(Ordering::Acquire));
                                f(ptr)
                            };
                            if hwp > eff {
                                eff = hwp;
                            }
                        }
                    }
                    // `set_effective_priority` (not a plain store): the
                    // unlocking task is normally `Running` (unqueued), so
                    // this is usually a no-op queue-wise — but going
                    // through the real API keeps that true by
                    // construction instead of by the caller happening to
                    // always be the running task, and costs nothing extra
                    // when it is.
                    t.set_effective_priority(owner, eff);
                }
            }
            self.mutex.locked.store(false, Ordering::Release);
            self.mutex.wake_all_waiters();
        });
        // Give a higher-priority waiter (now Ready) an immediate chance to
        // preempt us rather than waiting for the next tick.
        crate::port::arch::request_reschedule();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::preempt::tcb as tcbmod;

    #[test]
    fn lock_unlock_basic() {
        crate::kernel_test! {
            let a = tcbmod::register(0x1000, 1).unwrap();
            sched::set_current(a);

            let m: PriorityMutex<u32> = PriorityMutex::new(0);
            {
                let mut guard = m.lock();
                *guard = 42;
            }
            assert_eq!(*m.lock(), 42);
        }
    }

    #[test]
    fn try_lock_behavior() {
        crate::kernel_test! {
            let a = tcbmod::register(0x1000, 1).unwrap();
            sched::set_current(a);

            let m: PriorityMutex<u32> = PriorityMutex::new(0);
            let g = m.try_lock().expect("free mutex must lock");
            assert!(m.try_lock().is_none(), "held mutex must not lock");
            drop(g);
            assert!(m.try_lock().is_some(), "released mutex must lock");
        }
    }

    #[test]
    #[should_panic(expected = "recursive lock")]
    fn recursive_lock_panics() {
        crate::kernel_test! {
            let a = tcbmod::register(0x1000, 1).unwrap();
            sched::set_current(a);

            let m: PriorityMutex<u32> = PriorityMutex::new(0);
            let _g = m.lock();
            let _ = m.lock(); // must panic
        }
    }

    #[test]
    fn priority_inheritance_boosts_holder() {
        crate::kernel_test! {
            let low = tcbmod::register(0x1000, 1).unwrap();
            let high = tcbmod::register(0x2000, 5).unwrap();

            let m: PriorityMutex<u32> = PriorityMutex::new(0);

            // `low` acquires the lock.
            sched::set_current(low);
            let guard = m.lock();
            assert_eq!(
                tcbmod::get(low).unwrap().effective_priority.load(Ordering::Acquire),
                1
            );

            // `high` contends for the lock (would block — but that calls
            // port::arch::request_reschedule(), which is a no-op on the dummy/host arch).
            // Directly exercise the boost logic used inside lock()'s slow path
            // by simulating one contention iteration.
            sched::set_current(high);
            let owner_id = low;
            let my_base = tcbmod::get(high).unwrap().base_priority.load(Ordering::Acquire);
            let owner_tcb = tcbmod::get(owner_id).unwrap();
            if my_base > owner_tcb.effective_priority.load(Ordering::Acquire) {
                owner_tcb.effective_priority.store(my_base, Ordering::Release);
            }
            assert_eq!(
                tcbmod::get(low).unwrap().effective_priority.load(Ordering::Acquire),
                5
            );

            drop(guard);
            // Unlock restores the holder's base priority (no other held
            // mutexes, no remaining waiters).
            assert_eq!(
                tcbmod::get(low).unwrap().effective_priority.load(Ordering::Acquire),
                1
            );
        }
    }

    #[test]
    fn b11_nested_unlock_keeps_boost_from_other_mutex() {
        crate::kernel_test! {
            // Statics, not locals: the held-list stores a raw mutex pointer
            // that outlives any single scope (miri: 'static provenance).
            static A: PriorityMutex<u32> = PriorityMutex::new(0);
            static B: PriorityMutex<u32> = PriorityMutex::new(0);

            let holder = tcbmod::register(0x1000, 1).unwrap();
            sched::set_current(holder);

            let ga = A.lock();
            let gb = B.lock();

            // A waiter (priority 8) registers on mutex A's waiters array,
            // simulating the state after its slow path ran: boost the
            // holder to 8 and record the waiter.
            let waiter = tcbmod::register(0x3000, 8).unwrap();
            sched::set_current(waiter);
            // Simulate the contender's slow-path registration on A.
            A.waiters[0].store(waiter, Ordering::Release);
            let holder_tcb = tcbmod::get(holder).unwrap();
            holder_tcb.effective_priority.store(8, Ordering::Release);
            // (waiter would also have called add_waiter + block_current,
            // but for the [B11] unit check the boost + waiter entry are
            // what matters.)

            sched::set_current(holder);
            // Unlock B. Old behavior: effective_priority := base (1),
            // clobbering the boost held for A. New behavior: recompute
            // from remaining held mutexes -> A's highest waiter = 8.
            drop(gb);
            assert_eq!(
                holder_tcb.effective_priority.load(Ordering::Acquire),
                8,
                "[B11] unlocking B must not drop the boost held for A"
            );

            // Unlock A: no waiters remain -> back to base.
            drop(ga);
            assert_eq!(
                holder_tcb.effective_priority.load(Ordering::Acquire),
                1,
                "[B11] after unlocking both, effective priority = base"
            );
        }
    }

    #[test]
    fn b1_retest_inside_critical_section_catches_release() {
        crate::kernel_test! {
            let a = tcbmod::register(0x1000, 1).unwrap();
            let b = tcbmod::register(0x2000, 5).unwrap();
            let m: PriorityMutex<u32> = PriorityMutex::new(0);

            // A holds the mutex.
            sched::set_current(a);
            let guard_a = m.lock();

            // B's fast-path CAS fails.
            sched::set_current(b);
            assert!(!m.try_acquire(b), "A holds the mutex");

            // [B1] interleaving: a tick lands between B's failed CAS and
            // B's add_waiter; the holder runs to completion and releases,
            // and wake_all_waiters() scans an empty list.
            sched::set_current(a);
            drop(guard_a);

            // B resumes: the critical-section *re-test* must observe the
            // release and acquire — with the old code, B would register a
            // waiter nobody will ever wake and block forever.
            sched::set_current(b);
            let guard_b = m.lock();
            assert!(guard_b.mutex.locked.load(Ordering::Acquire));
            drop(guard_b);
        }
    }
}
