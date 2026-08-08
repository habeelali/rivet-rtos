//! Critical section abstraction, built on the Group A `port::arch`
//! interrupt-mask primitives. Arch-agnostic: nested `enter` calls compose
//! correctly (an inner call observes interrupts already disabled and its
//! restore is a no-op, leaving the outermost call to actually re-enable) —
//! guaranteed by [`crate::port::arch::critical_section`] itself.

/// Run a closure with interrupts disabled.
#[inline]
pub fn enter<F, R>(f: F) -> R
where
    F: FnOnce() -> R,
{
    crate::port::arch::critical_section(f)
}
