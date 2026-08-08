//! Preemptive task stack pool (plan.md §3).
//!
//! All preemptive task stacks live in one contiguous, power-of-two-aligned
//! `.task_stacks` section. Stacks are carved from it at spawn time with
//! size-aligned addresses (sizes must be powers of two), which is exactly
//! what the CM3 MPU (per-switch current-stack region) and the RISC-V PMP
//! (NAPUT guard bands at the low end of each stack) require.
//!
//! On host builds there is no linker-defined pool; [`alloc_stack`] returns
//! `None` and callers fall back to their own static stacks.

#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
extern "C" {
    static __task_stacks_start: u8;
    static __task_stacks_end: u8;
}

/// Next free offset into the pool. Single writer (spawn happens on one
/// context at a time: boot or a running task); readers are the fault
/// handler and watermarking, which only need the base.
#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
static NEXT: AtomicUsize = AtomicUsize::new(0);

/// Number of stacks allocated (RISC-V PMP guard entry index).
#[cfg(target_arch = "riscv32")]
static ALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Guard band size at the low end of every stack (plan.md §3.2): a 64-byte
/// NAPOT-aligned region the RISC-V PMP denies, so a stack overflow faults
/// instead of corrupting silently. The CM3 MPU denies the whole pool so
/// the guard is redundant there (but harmless).
#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
const GUARD_SIZE: usize = 64;

/// Pool base address on embedded targets.
#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
fn pool_base() -> usize {
    // (Linker symbol; addr_of! is not unsafe on an extern static.)
    core::ptr::addr_of!(__task_stacks_start) as usize
}

#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
fn pool_len() -> usize {
    core::ptr::addr_of!(__task_stacks_end) as usize
        - core::ptr::addr_of!(__task_stacks_start) as usize
}

/// Allocate a size-aligned stack of `size` bytes from the pool, with a
/// 64-byte guard band at its low end (plan.md §3.2). `size` must be a
/// power of two (the MPU/PMP region alignment requires it).
///
/// On RISC-V each stack's guard band is registered as a locked PMP entry
/// (up to 15; the 16th+ stacks fall back to watermark detection — a
/// documented budget constraint). Returns `None` when the pool is
/// exhausted or on host builds without a linker pool.
pub fn alloc_stack(size: usize) -> Option<&'static mut [u8]> {
    debug_assert!(
        size.is_power_of_two(),
        "rivet: task stack size {size} must be a power of two (MPU/PMP region alignment)"
    );
    #[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
    {
        let base = pool_base();
        let len = pool_len();
        // Prefer a released stack (LIFO) — the guard band is still
        // registered for it, so reuse is free.
        if let Some((off, sz)) = FREE_LIST.pop() {
            if sz == size {
                // `off` is the *slice's* offset from the pool base (already
                // past the guard band); adding GUARD_SIZE again would
                // double-count it and shift the recycled stack into the
                // next region (plan.md §5.4 — caught by the CM3 MPU as a
                // switch-time MemManage with base off by 64 bytes).
                let stack_base = base + off;
                // SAFETY: this slice was released by `release_stack` (still
                // within the 'static pool) and is not handed out elsewhere.
                return Some(unsafe {
                    core::slice::from_raw_parts_mut(stack_base as *mut u8, size)
                });
            }
            // Size mismatch: put it back; the caller requested a different
            // size than the last released stack.
            FREE_LIST.push(off, sz);
        }
        let next = NEXT.load(Ordering::Relaxed);
        // Stack base size-aligned; guard = the 64 bytes immediately below.
        let stack_base = (base + next + GUARD_SIZE + size - 1) & !(size - 1);
        let guard_base = stack_base - GUARD_SIZE;
        let offset = guard_base - base;
        if offset + GUARD_SIZE + size > len {
            return None;
        }
        NEXT.store(offset + GUARD_SIZE + size, Ordering::Relaxed);
        #[cfg(target_arch = "riscv32")]
        {
            let entry = ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
            if entry < 15 {
                crate::arch::pmp_register_guard(guard_base, entry);
            }
        }
        // SAFETY: the slice is within the 'static pool, never handed out
        // twice (NEXT only moves forward), and aligned to `size`.
        Some(unsafe { core::slice::from_raw_parts_mut(stack_base as *mut u8, size) })
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
    {
        let _ = size;
        None
    }
}

/// Max free-list entries (a released stack per TCB slot).
#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
const FREE_LIST_CAP: usize = 16;

/// LIFO free list of released stacks (plan.md §5.4 respawn): pairs of
/// (offset, size) into the pool. Released stacks are refilled with 0xAA so
/// watermark detection stays meaningful across respawns. The list is only
/// touched by `alloc_stack`/`release_stack`, which run on one context at a
/// time (boot or a running task with interrupts logically disabled around
/// spawn/despawn).
#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
struct FreeList {
    count: AtomicUsize,
    entries: [AtomicUsize; FREE_LIST_CAP * 2],
}

#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
impl FreeList {
    const fn new() -> Self {
        Self {
            count: AtomicUsize::new(0),
            entries: [const { AtomicUsize::new(0) }; FREE_LIST_CAP * 2],
        }
    }
    fn push(&self, offset: usize, size: usize) {
        let n = self.count.load(Ordering::Relaxed);
        if n < FREE_LIST_CAP {
            self.entries[n * 2].store(offset, Ordering::Relaxed);
            self.entries[n * 2 + 1].store(size, Ordering::Relaxed);
            self.count.store(n + 1, Ordering::Relaxed);
        }
    }
    fn pop(&self) -> Option<(usize, usize)> {
        let n = self.count.load(Ordering::Relaxed);
        if n == 0 {
            return None;
        }
        let i = n - 1;
        let off = self.entries[i * 2].load(Ordering::Relaxed);
        let sz = self.entries[i * 2 + 1].load(Ordering::Relaxed);
        self.count.store(i, Ordering::Relaxed);
        Some((off, sz))
    }
}

#[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
static FREE_LIST: FreeList = FreeList::new();

/// Release a stack back to the pool (plan.md §5.4 despawn/respawn). The
/// slice is refilled with `0xAA` so watermarking keeps working after the
/// stack is reused. No-op if the slice did not come from the pool.
pub fn release_stack(stack: &'static mut [u8]) {
    #[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
    {
        let base = pool_base();
        let offset = stack.as_mut_ptr() as usize - base;
        let size = stack.len();
        if offset + size > pool_len() {
            return; // not from the pool; ignore
        }
        // Refill for watermark detection (0xAA = untouched). The pool is
        // MPU/PMP-denied to thread-mode code, so the refill runs inside
        // the same scratch window (and critical section) the spawn path
        // uses; the svc-based frame init is not involved here (plan.md §5.4).
        crate::critical::enter(|| {
            crate::arch::mpu_allow_scratch(stack.as_ptr() as usize, size);
            for b in stack.iter_mut() {
                *b = 0xAA;
            }
            crate::arch::mpu_clear_scratch();
        });
        FREE_LIST.push(offset, size);
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
    {
        let _ = stack;
    }
}

/// Pool bounds `(base, len)` on embedded targets; `(0, 0)` on host.
pub fn pool_bounds() -> (usize, usize) {
    #[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
    {
        (pool_base(), pool_len())
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
    {
        (0, 0)
    }
}

/// Is address `addr` inside the task-stack pool?
pub fn contains(addr: usize) -> bool {
    #[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
    {
        addr >= pool_base() && addr < pool_base() + pool_len()
    }
    #[cfg(not(any(target_arch = "riscv32", target_arch = "arm")))]
    {
        let _ = addr;
        false
    }
}

/// Test-only: reset the allocation cursor (host tests).
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    #[cfg(any(target_arch = "riscv32", target_arch = "arm"))]
    NEXT.store(0, Ordering::Relaxed);
    #[cfg(target_arch = "riscv32")]
    ALLOC_COUNT.store(0, Ordering::Relaxed);
}
