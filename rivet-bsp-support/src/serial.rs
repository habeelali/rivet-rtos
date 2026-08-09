//! `embedded-hal-nb::serial::{Read, Write}` over [`rivet::console`]
//! (plan.md Phase 15).
//!
//! Genuinely board-agnostic: `rivet::console`'s RX/TX rings (plan.md
//! Phase 14) are already portable kernel API, so this wrapper works
//! identically on every board that's wired one up — no per-board code
//! needed, unlike GPIO (which is real, per-board register layout).

use embedded_hal_nb::serial::{ErrorType, Read, Write};

/// Zero-sized handle to the board's console UART, for code written
/// against `embedded-hal-nb`'s serial traits rather than
/// `rivet::console` directly.
pub struct Serial;

impl ErrorType for Serial {
    type Error = core::convert::Infallible;
}

impl Read<u8> for Serial {
    fn read(&mut self) -> nb::Result<u8, Self::Error> {
        rivet::console::try_read_byte().ok_or(nb::Error::WouldBlock)
    }
}

impl Write<u8> for Serial {
    fn write(&mut self, word: u8) -> nb::Result<(), Self::Error> {
        rivet::console::write_bytes(&[word]);
        Ok(())
    }

    fn flush(&mut self) -> nb::Result<(), Self::Error> {
        // `write_bytes` already fully handed the byte to the TX ring (or
        // wrote it directly, on the blocking-polling fallback path) —
        // there's no separate "in-flight, not yet queued" state to wait
        // out here.
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Compile-only: proves `Serial` is usable through generic
    // `embedded-hal-nb` code, not just directly.
    #[allow(dead_code)]
    fn generic_read<R: Read<u8>>(r: &mut R) -> nb::Result<u8, R::Error> {
        r.read()
    }

    #[allow(dead_code)]
    fn generic_write<W: Write<u8>>(w: &mut W, b: u8) -> nb::Result<(), W::Error> {
        w.write(b)
    }

    #[test]
    fn type_checks() {
        let _ = generic_read::<Serial>;
        let _ = generic_write::<Serial>;
    }
}
