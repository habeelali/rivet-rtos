//! Rings shared with another operating system.
//!
//! When rivet owns one core and Linux owns the rest, both want the same
//! PL011: there is exactly one UART on the header once `disable-bt` has
//! moved the Bluetooth modem aside. Rather than interleave onto one line,
//! rivet writes into memory Linux was told not to manage and a reader on
//! the Linux side drains it.
//!
//! There are two rings, not one, because they carry different things and
//! must not be interleaved. The console is text for a human. The trace
//! stream is PulseTrace's binary wire format, framed and checksummed, and
//! a stray log line in the middle of it corrupts a frame. Separate rings
//! in the same window keeps both readable.
//!
//! ```text
//! SHARED_BASE + 0x000000   console   64 KiB of text
//! SHARED_BASE + 0x100000   trace    256 KiB of PulseTrace frames
//! ```
//!
//! The whole window is mapped Device on this side, matching how Linux
//! hands out `/dev/mem` mappings of non-RAM regions. That agreement is
//! what makes the sharing work without either side doing cache
//! maintenance. See `mmu::SHARED_BASE`.
//!
//! # Layout of one ring
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
//! never confused. One producer, one consumer, and the producer wins: if
//! the reader falls more than `capacity` behind, the oldest bytes are
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

/// A byte ring at a fixed physical address.
pub struct Ring {
    base: usize,
    capacity: usize,
}

/// Text console. Small: a human reads it.
pub const CONSOLE: Ring = Ring::new(SHARED_BASE, 64 * 1024);
/// PulseTrace frames. Larger, because tracing a scheduler produces far
/// more bytes than logging does, and dropping frames loses events rather
/// than lines.
pub const TRACE: Ring = Ring::new(SHARED_BASE + 0x0010_0000, 256 * 1024);

impl Ring {
    pub const fn new(base: usize, capacity: usize) -> Self {
        Ring { base, capacity }
    }

    /// Set up the header. Safe to call more than once.
    ///
    /// # Safety
    /// The shared window must be mapped, and no reader may be mid-drain.
    pub unsafe fn init(&self) {
        // SAFETY: writing the header of a mapped Device window.
        unsafe {
            write_volatile((self.base + OFF_WRITE) as *mut u64, 0);
            write_volatile((self.base + OFF_READ) as *mut u64, 0);
            write_volatile((self.base + OFF_CAPACITY) as *mut u32, self.capacity as u32);
            write_volatile((self.base + OFF_VERSION) as *mut u32, VERSION);
            // Magic last: a reader that sees it can trust everything above.
            write_volatile((self.base + OFF_MAGIC) as *mut u32, MAGIC);
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }

    /// Append bytes.
    ///
    /// # Safety
    /// The window must be mapped and [`init`](Self::init) must have run.
    pub unsafe fn write_bytes(&self, bytes: &[u8]) {
        // SAFETY: the window is mapped Device and the header is set up.
        unsafe {
            let mut w = read_volatile((self.base + OFF_WRITE) as *const u64);
            for &b in bytes {
                let slot = (w as usize) % self.capacity;
                write_volatile((self.base + OFF_DATA + slot) as *mut u8, b);
                w += 1;
            }
            // Publish the payload before the index that advertises it, or
            // a reader can be pointed at bytes that have not landed.
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
            write_volatile((self.base + OFF_WRITE) as *mut u64, w);
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }

    /// How many bytes the reader is behind by.
    pub fn pending(&self) -> u64 {
        // SAFETY: plain reads of a mapped Device window.
        unsafe {
            let w = read_volatile((self.base + OFF_WRITE) as *const u64);
            let r = read_volatile((self.base + OFF_READ) as *const u64);
            w.wrapping_sub(r)
        }
    }
}

/// Bring up every ring in the window.
///
/// # Safety
/// The window must be mapped, and no reader may be mid-drain.
pub unsafe fn init() {
    // SAFETY: forwarded from this function's contract.
    unsafe {
        CONSOLE.init();
        TRACE.init();
    }
}

/// Append to the console ring.
///
/// # Safety
/// The window must be mapped and [`init`] must have run.
pub unsafe fn write_bytes(bytes: &[u8]) {
    // SAFETY: forwarded from this function's contract.
    unsafe { CONSOLE.write_bytes(bytes) }
}
