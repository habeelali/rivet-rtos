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
    #[cfg(feature = "latency-histograms")]
    {
        let start = crate::port::arch::cycle_count();
        let r = crate::port::arch::critical_section(f);
        crate::latency::record(
            crate::latency::Kind::CriticalSection,
            crate::port::arch::cycle_count().wrapping_sub(start),
        );
        r
    }
    #[cfg(not(feature = "latency-histograms"))]
    {
        crate::port::arch::critical_section(f)
    }
}
