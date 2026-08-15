//! Per-core mailboxes in the ARM-local block, used as a doorbell.
//!
//! The command ring lets Linux hand rivet work, but a ring on its own
//! only supports polling, and polling a shared window from a real-time
//! core is exactly the wrong shape: it burns the core when idle and still
//! adds latency when busy.
//!
//! These mailboxes fix that. Unlike almost everything else on this SoC
//! they are genuinely per-core, four to a core, and writing one from any
//! core raises an interrupt on the target. So Linux can ring rivet's
//! doorbell with a single store, and rivet can sit in `WFI` until it
//! arrives.
//!
//! This is the one interrupt source on BCM2837 that can be directed at a
//! specific core. Peripheral interrupts go wherever `GPU_INTERRUPTS_ROUTING`
//! points, which is one global choice for the whole system.
//!
//! ```text
//! 0x4000_0050 + core*4          MAILBOX_IRQCNTL, bit n enables mailbox n
//! 0x4000_0080 + core*16 + n*4   MBOXn_SET, write-to-set, raises the IRQ
//! 0x4000_00C0 + core*16 + n*4   MBOXn_RDCLR, write-1-to-clear
//! ```

use core::ptr::{read_volatile, write_volatile};

const ARM_LOCAL_BASE: usize = 0x4000_0000;
const MAILBOX_IRQCNTL: usize = 0x50;
const MBOX_SET: usize = 0x80;
const MBOX_RDCLR: usize = 0xC0;

/// Mailbox 0 is the doorbell; the other three are unused.
pub const DOORBELL: usize = 0;

/// Bit in `CORE{n}_IRQ_SOURCE` for mailbox 0.
pub const IRQ_SOURCE_MBOX0: u32 = 1 << 4;

fn irqcntl(core: usize) -> usize {
    ARM_LOCAL_BASE + MAILBOX_IRQCNTL + core * 4
}

fn set_reg(core: usize, mbox: usize) -> usize {
    ARM_LOCAL_BASE + MBOX_SET + core * 16 + mbox * 4
}

fn rdclr_reg(core: usize, mbox: usize) -> usize {
    ARM_LOCAL_BASE + MBOX_RDCLR + core * 16 + mbox * 4
}

/// Let mailbox `mbox` raise an interrupt on this core.
///
/// # Safety
/// Writes this core's ARM-local interrupt-control register.
pub unsafe fn enable_on_this_core(mbox: usize) {
    let core = crate::smp::current_core();
    // SAFETY: MMIO write to this core's own control register.
    unsafe {
        let cur = read_volatile(irqcntl(core) as *const u32);
        write_volatile(irqcntl(core) as *mut u32, cur | (1 << mbox));
    }
}

/// Read and clear a mailbox, returning what was in it.
///
/// Clearing is what drops the interrupt line, so an unacknowledged
/// mailbox re-enters the handler forever.
///
/// # Safety
/// Writes this core's mailbox registers.
pub unsafe fn take(mbox: usize) -> u32 {
    let core = crate::smp::current_core();
    // SAFETY: MMIO on this core's own mailbox.
    unsafe {
        let v = read_volatile(rdclr_reg(core, mbox) as *const u32);
        // Write-1-to-clear: writing back what was read clears exactly the
        // bits that were set.
        write_volatile(rdclr_reg(core, mbox) as *mut u32, v);
        v
    }
}

/// Ring another core's doorbell.
///
/// Not used by rivet itself here, since Linux is the one doing the
/// ringing, but it is the same register and worth having in one place.
///
/// # Safety
/// Writes another core's mailbox register.
pub unsafe fn ring(core: usize, mbox: usize, value: u32) {
    // SAFETY: MMIO write to the target core's mailbox.
    unsafe { write_volatile(set_reg(core, mbox) as *mut u32, value) };
}
