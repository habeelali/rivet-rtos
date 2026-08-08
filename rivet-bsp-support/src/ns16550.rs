//! Minimal NS16550-compatible UART driver: blind byte writes to the
//! transmit-holding register, no THRE (transmit-ready) check. This
//! matches the original QEMU-virt port's behavior exactly (QEMU's model
//! never actually backpressures), but is a known simplification — a real
//! NS16550 can drop bytes under load without checking LSR.THRE first.
//! Tracked as future BSP driver work, not fixed here.

/// Write a single byte to the UART's data register at `base`.
///
/// # Safety
/// `base` must be the base address of a real, memory-mapped NS16550-
/// compatible UART.
pub unsafe fn write_byte(base: usize, byte: u8) {
    // SAFETY: forwarded from the caller's contract.
    unsafe { core::ptr::write_volatile(base as *mut u8, byte) };
}

/// Write a byte string, one byte at a time.
///
/// # Safety
/// See [`write_byte`].
pub unsafe fn write_bytes(base: usize, bytes: &[u8]) {
    for &b in bytes {
        // SAFETY: see `write_byte`.
        unsafe { write_byte(base, b) };
    }
}
