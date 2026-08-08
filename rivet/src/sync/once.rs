//! A minimal no_std `Once` cell: written exactly once (at boot), read-only
//! afterwards. Used for the `Channel::split`-once pattern (plan.md [B8]):
//! the sender/receiver halves of a static channel are stored here once at
//! boot and borrowed by tasks for the lifetime of the program.

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;

/// Single-writer, multi-reader `Once` cell. `set` may be called at most
/// once (a second call returns `Err` with the value back); `get` returns
/// `None` until then and `Some(&T)` forever after.
pub struct Once<T> {
    set: crate::sync::atomic::AtomicBool,
    cell: UnsafeCell<MaybeUninit<T>>,
}

// Safety: `set` is single-threaded (boot); after publication (Release
// store of the flag) the value is immutable, so shared `&T` reads are
// sound. `T` must be `Sync` for `&T` to be shareable.
unsafe impl<T: Sync> Sync for Once<T> {}

impl<T> Once<T> {
    /// Create an empty `Once`.
    #[cfg(not(loom))]
    pub const fn new() -> Self {
        Self::new_impl()
    }

    #[cfg(loom)]
    pub fn new() -> Self {
        Self::new_impl()
    }

    #[cfg(not(loom))]
    const fn new_impl() -> Self {
        Self {
            set: crate::sync::atomic::AtomicBool::new(false),
            cell: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    #[cfg(loom)]
    fn new_impl() -> Self {
        Self {
            set: crate::sync::atomic::AtomicBool::new(false),
            cell: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Store the value (boot time, single writer). Returns `Err(value)`
    /// if already set.
    pub fn set(&self, value: T) -> Result<(), T> {
        if self.set.load(crate::sync::atomic::Ordering::Acquire) {
            return Err(value);
        }
        // SAFETY: single writer (caller contract); the value is written
        // before the flag is published with Release, so readers that see
        // the flag also see the value (Release→Acquire).
        unsafe {
            (*self.cell.get()).write(value);
        }
        self.set.store(true, crate::sync::atomic::Ordering::Release);
        Ok(())
    }

    /// Borrow the stored value, or `None` if not yet set.
    pub fn get(&self) -> Option<&T> {
        if !self.set.load(crate::sync::atomic::Ordering::Acquire) {
            return None;
        }
        // SAFETY: the value was written before the flag was published; a
        // reader that sees the flag (Acquire) observes the fully written
        // value, and it is never mutated again.
        unsafe { Some(&*(*self.cell.get()).as_ptr()) }
    }
}

impl<T> Default for Once<T> {
    fn default() -> Self {
        Self::new()
    }
}
