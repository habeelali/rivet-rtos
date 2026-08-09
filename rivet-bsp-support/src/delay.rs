//! `embedded-hal-async::delay::DelayNs` on top of the preemptive tier's
//! blocking sleep (plan.md Phase 15).
//!
//! This is a blocking implementation of an async trait — deliberately.
//! Rivet's preemptive tier has no "suspend this task, return control to
//! an executor" primitive the way the cooperative tier's `Sleep` does
//! (that one needs a *compile-time* duration, `Sleep<const MICROS: u64>`,
//! which doesn't fit `DelayNs`'s runtime-parameterized signature); what a
//! preemptive task actually does to give up the CPU for a while *is*
//! block (`rivet::preempt::sleep_ms`) — the scheduler runs something else
//! in the meantime, which is the same practical effect `.await`ing a real
//! async delay would have. A driver written against `DelayNs` works
//! correctly called from a preemptive task; it just isn't meaningful to
//! call from the cooperative executor's own task (blocking there would
//! stall every other cooperative task, exactly as blocking always would).

pub struct RivetDelay;

impl embedded_hal_async::delay::DelayNs for RivetDelay {
    async fn delay_ns(&mut self, ns: u32) {
        // Round up to whole milliseconds — `sleep_ms`'s actual resolution
        // is bounded by `RIVET_TICK_HZ` anyway, so a sub-tick request
        // rounding up to one tick is honest, not a fake no-op.
        let ms = (ns as u64).div_ceil(1_000_000).max(1);
        rivet::preempt::sleep_ms(ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Compile-only: proves `RivetDelay` is usable through generic
    // `embedded-hal-async` code, not just directly.
    #[allow(dead_code)]
    async fn generic<D: embedded_hal_async::delay::DelayNs>(d: &mut D) {
        d.delay_ms(1).await;
    }

    #[test]
    fn type_checks() {
        let _ = generic::<RivetDelay>;
    }
}
