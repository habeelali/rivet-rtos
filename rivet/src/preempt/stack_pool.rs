//! Preemptive task stack pool (plan.md §3).
//!
//! All preemptive task stacks live in one contiguous, power-of-two-aligned
//! `.task_stacks` section. Stacks are carved from it at spawn time with
//! size-aligned addresses (sizes must be powers of two), which is exactly
//! what the CM3 MPU (per-switch current-stack region) and the RISC-V PMP
//! (NAPOT guard bands at the low end of each stack) require.
//!
//! `__task_stacks_start`/`__task_stacks_end` are part of the linker
//! contract every board's linker script provides (documented in
//! `docs/porting.md`) — not an arch/board API call, just fixed symbol
//! names, the same on every real target. The host test backend has no
//! linker-provided pool at all (gated on the `host-port` feature, not
//! `target_arch`: the distinction is "is there a real linker script",
//! which is unrelated to which arch a real target happens to be); there,
//! [`alloc_stack`] always returns `None` and callers fall back to their
//! own static stacks.

#[cfg(not(feature = "host-port"))]
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(not(feature = "host-port"))]
extern "C" {
    static __task_stacks_start: u8;
    static __task_stacks_end: u8;
}

/// Next free offset into the pool. Single writer (spawn happens on one
/// context at a time: boot or a running task); readers are the fault
/// handler and watermarking, which only need the base.
#[cfg(not(feature = "host-port"))]
static NEXT: AtomicUsize = AtomicUsize::new(0);

/// Number of stacks allocated so far (used as the guard-registration
/// index — meaningful on arches with a limited number of hardware guard
/// slots, e.g. RISC-V PMP; `port::arch::guard_register` is a no-op on
/// arches without that limit, e.g. Cortex-M's two-region MPU design).
#[cfg(not(feature = "host-port"))]
static ALLOC_COUNT: AtomicUsize = AtomicUsize::new(0);

/// Guard band size at the low end of every stack (plan.md §3.2): an
/// aligned region the RISC-V PMP denies, so a stack overflow faults
/// instead of corrupting silently. The CM3 MPU denies the whole pool so
/// the guard is redundant there (but harmless). `64` bytes on every arch
/// this has ever been measured on, except the ESP32-C6 (plan.md Phase
/// 26): its PMP grain forces a larger minimum NAPOT region, so this is a
/// runtime query — [`crate::port::arch::min_guard_size`] — not a
/// hardcoded constant, to guarantee the reservation here and what
/// [`crate::port::arch::guard_register`] actually denies always agree.
#[cfg(not(feature = "host-port"))]
fn guard_size() -> usize {
    crate::port::arch::min_guard_size()
}

/// Pool base address on embedded targets.
#[cfg(not(feature = "host-port"))]
fn pool_base() -> usize {
    // (Linker symbol; addr_of! is not unsafe on an extern static.)
    core::ptr::addr_of!(__task_stacks_start) as usize
}

#[cfg(not(feature = "host-port"))]
fn pool_len() -> usize {
    core::ptr::addr_of!(__task_stacks_end) as usize
        - core::ptr::addr_of!(__task_stacks_start) as usize
}

/// Allocate a size-aligned stack of `size` bytes from the pool, with a
/// guard band (usually 64 bytes — see [`guard_size`]) at its low end
/// (plan.md §3.2). `size` must be a power of two (the MPU/PMP region
/// alignment requires it) and at least as large as the guard band.
///
/// Each stack's guard band is registered via
/// [`crate::port::arch::guard_register`] (a no-op on arches whose memory
/// guard doesn't need one, e.g. Cortex-M). Returns `None` when the pool
/// is exhausted or on the host test backend (no linker-provided pool).
pub fn alloc_stack(size: usize) -> Option<&'static mut [u8]> {
    debug_assert!(
        size.is_power_of_two(),
        "rivet: task stack size {size} must be a power of two (MPU/PMP region alignment)"
    );
    #[cfg(not(feature = "host-port"))]
    {
        let guard_size = guard_size();
        debug_assert!(
            size >= guard_size,
            "rivet: task stack size {size} is smaller than this hardware's minimum PMP/MPU \
             guard band ({guard_size} bytes) — the guard alignment math below assumes a stack \
             is always at least as large as its own guard band"
        );
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
        // Stack base size-aligned; guard = `guard_size` bytes immediately
        // below (`size >= guard_size`, asserted above, guarantees a
        // `size`-aligned address is also `guard_size`-aligned, since both
        // are powers of two — required for the NAPOT guard encoding).
        let stack_base = (base + next + guard_size + size - 1) & !(size - 1);
        let guard_base = stack_base - guard_size;
        let offset = guard_base - base;
        if offset + guard_size + size > len {
            return None;
        }
        NEXT.store(offset + guard_size + size, Ordering::Relaxed);
        let entry = ALLOC_COUNT.fetch_add(1, Ordering::Relaxed);
        crate::port::arch::guard_register(guard_base, entry);
        // SAFETY: the slice is within the 'static pool, never handed out
        // twice (NEXT only moves forward), and aligned to `size`.
        Some(unsafe { core::slice::from_raw_parts_mut(stack_base as *mut u8, size) })
    }
    #[cfg(feature = "host-port")]
    {
        let _ = size;
        None
    }
}

/// Max free-list entries (a released stack per TCB slot).
#[cfg(not(feature = "host-port"))]
const FREE_LIST_CAP: usize = 16;

/// LIFO free list of released stacks (plan.md §5.4 respawn): pairs of
/// (offset, size) into the pool. Released stacks are refilled with 0xAA so
/// watermark detection stays meaningful across respawns. The list is only
/// touched by `alloc_stack`/`release_stack`, which run on one context at a
/// time (boot or a running task with interrupts logically disabled around
/// spawn/despawn).
#[cfg(not(feature = "host-port"))]
struct FreeList {
    count: AtomicUsize,
    entries: [AtomicUsize; FREE_LIST_CAP * 2],
}

#[cfg(not(feature = "host-port"))]
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

#[cfg(not(feature = "host-port"))]
static FREE_LIST: FreeList = FreeList::new();

/// Release a stack back to the pool (plan.md §5.4 despawn/respawn). The
/// slice is refilled with `0xAA` so watermarking keeps working after the
/// stack is reused. No-op if the slice did not come from the pool.
pub fn release_stack(stack: &'static mut [u8]) {
    #[cfg(not(feature = "host-port"))]
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
            crate::port::arch::scratch_open(stack.as_ptr() as usize, size);
            for b in stack.iter_mut() {
                *b = 0xAA;
            }
            crate::port::arch::scratch_close();
        });
        FREE_LIST.push(offset, size);
    }
    #[cfg(feature = "host-port")]
    {
        let _ = stack;
    }
}

/// Pool bounds `(base, len)` on embedded targets; `(0, 0)` on the host
/// test backend.
pub fn pool_bounds() -> (usize, usize) {
    #[cfg(not(feature = "host-port"))]
    {
        (pool_base(), pool_len())
    }
    #[cfg(feature = "host-port")]
    {
        (0, 0)
    }
}

/// Is address `addr` inside the task-stack pool?
pub fn contains(addr: usize) -> bool {
    #[cfg(not(feature = "host-port"))]
    {
        addr >= pool_base() && addr < pool_base() + pool_len()
    }
    #[cfg(feature = "host-port")]
    {
        let _ = addr;
        false
    }
}

/// Test-only: reset the allocation cursor (host tests).
#[cfg(feature = "test-support")]
pub(crate) fn reset_for_test() {
    #[cfg(not(feature = "host-port"))]
    {
        NEXT.store(0, Ordering::Relaxed);
        ALLOC_COUNT.store(0, Ordering::Relaxed);
    }
}
