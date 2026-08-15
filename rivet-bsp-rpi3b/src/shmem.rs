//! A console ring shared with another operating system.
//!
//! When rivet owns one core and Linux owns the rest, both want the same
//! PL011: there is exactly one UART on the header once `disable-bt` has
//! moved the Bluetooth modem aside. Rather than interleave two consoles
//! onto one line, rivet writes into a ring in memory that Linux was told
//! not to manage, and a reader on the Linux side drains it.
//!
//! The window lives at [`SHARED_BASE`] and is mapped Device on this side,
//! matching how Linux hands out `/dev/mem` mappings of non-RAM regions.
//! That agreement is what makes the sharing work without either side
//! doing cache maintenance. See `mmu::SHARED_BASE`.
//!
//! # Layout
//!
//! ```text
//!  0  magic     u32   "RVTC"
//!  4  version   u32   1
//!  8  capacity  u32   bytes in `data`
//! 12  _pad      u32
//! 16  write     u64   total bytes ever written, never wrapped
//! 24  read      u64   total bytes ever consumed, written by the reader
//! 32  data      [u8; capacity]
//! ```
//!
//! Both indices count bytes for all time and are reduced modulo
//! `capacity` only when indexing, so an empty ring and a full one are
//! never confused. One producer, one consumer, and the producer wins:
//! if the reader falls more than `capacity` behind, the oldest bytes are
//! overwritten. Losing old output beats stalling a real-time core on a
//! consumer that went away.

use core::ptr::{read_volatile, write_volatile};

/// Physical base of the shared window. Must be 2 MiB aligned and inside
/// the region Linux was told to leave alone.
pub const SHARED_BASE: usize = 0x3100_0000;

const MAGIC: u32 = 0x5256_5443; // "RVTC"
const VERSION: u32 = 1;

const OFF_MAGIC: usize = 0;
const OFF_VERSION: usize = 4;
const OFF_CAPACITY: usize = 8;
const OFF_WRITE: usize = 16;
const OFF_READ: usize = 24;
const OFF_DATA: usize = 32;

/// Bytes of payload. The window is 2 MiB; this uses a small part of it
/// and leaves the rest for whatever the two sides want to share next.
pub const CAPACITY: usize = 64 * 1024;

/// Set up the ring header. Safe to call more than once.
///
/// # Safety
/// The shared window must be mapped, and no reader may be mid-drain.
pub unsafe fn init() {
    let base = SHARED_BASE;
    // SAFETY: writing the header of a mapped Device window.
    unsafe {
        write_volatile((base + OFF_WRITE) as *mut u64, 0);
        write_volatile((base + OFF_READ) as *mut u64, 0);
        write_volatile((base + OFF_CAPACITY) as *mut u32, CAPACITY as u32);
        write_volatile((base + OFF_VERSION) as *mut u32, VERSION);
        // Magic last: a reader that sees it can trust everything above.
        write_volatile((base + OFF_MAGIC) as *mut u32, MAGIC);
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// Append bytes to the ring.
///
/// # Safety
/// The shared window must be mapped and [`init`] must have run.
pub unsafe fn write_bytes(bytes: &[u8]) {
    let base = SHARED_BASE;
    // SAFETY: the window is mapped Device and the header is initialised.
    unsafe {
        let mut w = read_volatile((base + OFF_WRITE) as *const u64);
        for &b in bytes {
            let slot = (w as usize) % CAPACITY;
            write_volatile((base + OFF_DATA + slot) as *mut u8, b);
            w += 1;
        }
        // Publish the payload before the index that advertises it, or a
        // reader can be pointed at bytes that have not landed.
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        write_volatile((base + OFF_WRITE) as *mut u64, w);
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}

/// How many bytes the reader is behind by.
pub fn pending() -> u64 {
    // SAFETY: plain reads of a mapped Device window.
    unsafe {
        let w = read_volatile((SHARED_BASE + OFF_WRITE) as *const u64);
        let r = read_volatile((SHARED_BASE + OFF_READ) as *const u64);
        w.wrapping_sub(r)
    }
}
