//! A description of the running system, published where Linux can read it.
//!
//! The two halves of this system are built separately and installed
//! separately, and until now nothing checked that they agreed. The memory
//! map was declared independently in the linker configuration, the board
//! crate, the Linux loader and the provisioning script; a change to one
//! produced silent corruption rather than an error, because every ring
//! magic still matched and every pointer still pointed somewhere.
//!
//! This is the fix: one block at a known offset, written by rivet at
//! start-up, giving the Linux side enough to answer three questions it
//! previously could not.
//!
//! - **Are we talking to an image built against this loader?** The ABI
//!   version and build identity are here to be compared.
//! - **What is actually running?** Image name, versions, tick rate, core,
//!   and the memory window in use, rather than whatever the tooling was
//!   compiled to assume.
//! - **Is it still alive?** See [`heartbeat`]. Before this, a hung core
//!   and an idle one were indistinguishable from Linux: the console ring
//!   simply stopped, which is also what a system with nothing to say
//!   looks like.
//!
//! It lives in the last 4 KiB of the shared window, clear of all three
//! rings, and is Device memory like the rest of that window, so both
//! sides see writes without cache maintenance.

use core::ptr::write_volatile;
use core::sync::atomic::{AtomicU64, Ordering};

/// Offset of the header within the shared window: the last 4 KiB of the
/// 2 MiB region, past the console, trace and command rings.
pub const HEADER_OFFSET: usize = 0x1F_F000;

/// "RVTS", distinct from a ring's "RVTC" so a misaddressed reader fails
/// rather than parsing a ring as a header.
const MAGIC: u32 = 0x5256_5453;

/// Bump on any incompatible change to the field layout below. The Linux
/// tooling refuses to interpret a header whose ABI it does not know,
/// which is the entire point of having one.
pub const ABI_VERSION: u32 = 1;

/// Field offsets. Fixed by the ABI above and mirrored in the Linux tool.
mod off {
    pub const MAGIC: usize = 0x00;
    pub const ABI: usize = 0x04;
    pub const HEARTBEAT: usize = 0x08;
    pub const BOOT_CNTPCT: usize = 0x10;
    pub const TICK_HZ: usize = 0x18;
    pub const CORE: usize = 0x1C;
    pub const STATE: usize = 0x20;
    pub const HEARTBEAT_HZ: usize = 0x24;
    pub const LOAD_BASE: usize = 0x28;
    pub const SHARED_BASE: usize = 0x30;
    pub const OWNED_LEN: usize = 0x38;
    pub const SYSTEM_VERSION: usize = 0x40; // 32 bytes
    pub const IMAGE_NAME: usize = 0x60; // 32 bytes
    pub const BUILD_ID: usize = 0x80; // 48 bytes
    pub const RIVET_VERSION: usize = 0xB0; // 32 bytes
}

/// What the core is doing. Anything other than `Running` means the
/// heartbeat has legitimately stopped and Linux should not report a
/// fault.
pub mod state {
    pub const BOOTING: u32 = 0;
    pub const RUNNING: u32 = 1;
    pub const FAULTED: u32 = 2;
    pub const EXITED: u32 = 3;
}

/// One heartbeat per this many ticks.
///
/// Not every tick. The write goes to Device memory, which costs a trip to
/// the interconnect, and it would land on the tick path that was just
/// measured and tightened. At the default 10 kHz this is 100 Hz, fast
/// enough that a two-second stall is unambiguous and slow enough to
/// disappear into the noise.
pub const TICKS_PER_BEAT: u32 = 100;

static BEATS: AtomicU64 = AtomicU64::new(0);

fn base() -> usize {
    crate::shmem::SHARED_BASE + HEADER_OFFSET
}

/// # Safety
/// The shared window must be mapped.
unsafe fn put32(o: usize, v: u32) {
    // SAFETY: forwarded from the caller.
    unsafe { write_volatile((base() + o) as *mut u32, v) };
}

/// # Safety
/// The shared window must be mapped.
unsafe fn put64(o: usize, v: u64) {
    // SAFETY: forwarded from the caller.
    unsafe { write_volatile((base() + o) as *mut u64, v) };
}

/// Write a NUL-padded string, truncating rather than overflowing.
///
/// # Safety
/// The shared window must be mapped and `[o, o + cap)` must be inside it.
unsafe fn put_str(o: usize, cap: usize, s: &str) {
    let b = s.as_bytes();
    for i in 0..cap {
        let c = if i < b.len() { b[i] } else { 0 };
        // SAFETY: forwarded from the caller; i is bounded by cap.
        unsafe { write_volatile((base() + o + i) as *mut u8, c) };
    }
}

/// Publish everything known at start-up.
///
/// The magic goes last, so a reader either sees a complete header or no
/// header at all. Same discipline as the rings.
///
/// # Safety
/// Call once, after the shared window is mapped and before any reader is
/// told to look.
pub unsafe fn publish() {
    // SAFETY: forwarded from this function's contract.
    unsafe {
        put32(off::MAGIC, 0);
        put64(off::HEARTBEAT, 0);
        put64(off::BOOT_CNTPCT, read_counter());
        put32(off::TICK_HZ, crate::kernel::configured_tick_hz());
        put32(off::CORE, crate::smp::current_core() as u32);
        put32(off::STATE, state::BOOTING);
        put32(off::HEARTBEAT_HZ, 0);
        put64(off::LOAD_BASE, crate::mmu::layout::OWNED_BASE as u64);
        put64(off::SHARED_BASE, crate::shmem::SHARED_BASE as u64);
        put64(off::OWNED_LEN, crate::mmu::layout::OWNED_LEN as u64);
        put_str(off::SYSTEM_VERSION, 32, SYSTEM_VERSION);
        put_str(off::IMAGE_NAME, 32, "unknown");
        put_str(off::BUILD_ID, 48, env!("RIVET_BUILD_ID"));
        put_str(off::RIVET_VERSION, 32, env!("CARGO_PKG_VERSION"));
        put32(off::ABI, ABI_VERSION);
        put32(off::MAGIC, MAGIC);
    }
}

/// Version of the combined Linux-plus-rivet system, as opposed to the
/// version of the rivet crate. Bumped when the pair is released together.
pub const SYSTEM_VERSION: &str = "0.3.0";

/// Name the running image, so `rivet status` reports what is actually on
/// the core rather than what was most recently installed.
///
/// # Safety
/// Call after [`publish`].
pub unsafe fn set_image_name(name: &str) {
    // SAFETY: forwarded from this function's contract.
    unsafe { put_str(off::IMAGE_NAME, 32, name) };
}

/// Record the tick rate once the timer has actually been started.
///
/// # Safety
/// The shared window must be mapped.
pub unsafe fn set_tick_hz(hz: u32) {
    // SAFETY: forwarded from this function's contract.
    unsafe {
        put32(off::TICK_HZ, hz);
        put32(off::HEARTBEAT_HZ, hz / TICKS_PER_BEAT);
    }
}

/// Move to `state::RUNNING` and start beating.
///
/// # Safety
/// Call after [`publish`].
pub unsafe fn set_running() {
    // SAFETY: forwarded from this function's contract.
    unsafe {
        put32(off::STATE, state::RUNNING);
    }
}

/// Record why the core stopped, so a stalled heartbeat is read as an
/// orderly end rather than a failure.
///
/// # Safety
/// The shared window must be mapped.
pub unsafe fn set_state(s: u32) {
    // SAFETY: forwarded from this function's contract.
    unsafe { put32(off::STATE, s) };
}

/// Called from the tick handler. Writes once every [`TICKS_PER_BEAT`].
///
/// This proves the timer interrupt is still being taken, which covers the
/// failure modes that actually happen here: a fault, an abort, a hang
/// with interrupts masked. It does not prove the scheduler is making
/// progress, so a task spinning forever at the top priority would keep
/// this beating. That case belongs to the watchdog, not here.
pub fn heartbeat() {
    let n = BEATS.fetch_add(1, Ordering::Relaxed) + 1;
    if !n.is_multiple_of(TICKS_PER_BEAT as u64) {
        return;
    }
    // SAFETY: the shared window is mapped for the whole life of an AMP
    // image, and this offset is inside it.
    unsafe { put64(off::HEARTBEAT, n / TICKS_PER_BEAT as u64) };
}

fn read_counter() -> u64 {
    let v: u64;
    // SAFETY: reading the architected counter has no side effects.
    unsafe {
        core::arch::asm!("isb", "mrs {}, cntpct_el0", out(reg) v,
                         options(nomem, nostack, preserves_flags))
    };
    v
}
