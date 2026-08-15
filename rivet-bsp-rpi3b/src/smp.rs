//! Releasing the parked cores.
//!
//! The firmware boots core 0 and leaves cores 1-3 spinning on a mailbox
//! apiece, waiting for someone to write a jump address and signal an
//! event. That is the `spin-table` enable method the upstream device tree
//! declares, and it is the same mechanism Linux uses to bring up its own
//! secondaries:
//!
//! ```text
//! 0xd8  core 0 (unused, core 0 goes straight to the kernel)
//! 0xe0  core 1
//! 0xe8  core 2
//! 0xf0  core 3
//! ```
//!
//! Write a 64-bit address into a core's slot, make it visible, `SEV`, and
//! that core branches to it with the MMU off, caches off, `DAIF` masked
//! and no stack.
//!
//! # The cache trap
//!
//! A parked core is running with its caches off, so its polling load
//! reads straight from memory. A core that has enabled its data cache and
//! then writes the mailbox leaves that write sitting in write-back cache
//! where the parked core will never see it, and the release silently does
//! nothing. [`release_core`] therefore cleans the line to the point of
//! coherency before the `SEV`.
//!
//! This is not hypothetical bookkeeping for later: it is the exact shape
//! of "worked with caches off, hung once I turned them on", and it is the
//! path that matters when the releasing core is eventually Linux.

use core::sync::atomic::{AtomicUsize, Ordering};

/// Base of the firmware's spin table. Slot N lives at `0xd8 + N * 8`.
const SPIN_TABLE_BASE: usize = 0xd8;

/// Set by each core that executes the parking loop in `boot.rs`, so the
/// releasing core can report which cores ever ran our code at all.
///
/// This distinguishes two arrangements that otherwise look identical from
/// the console: on real hardware the firmware parks cores 1-3 before they
/// reach us, so only core 0 checks in; QEMU releases all four at the
/// entry point, so they all do.
#[no_mangle]
pub static mut RIVET_SMP_WITNESS: [u64; 4] = [0; 4];

/// Where a released core should go, set before the mailbox write.
///
/// Only ever written with a plain store and read with a plain load. An
/// atomic read-modify-write here would abort: the releasing core may
/// still have translation off, and a core waking from the spin table
/// certainly does.
static SECONDARY_ENTRY: AtomicUsize = AtomicUsize::new(0);

core::arch::global_asm!(
    r#"
.section .text, "ax"

// Where a released core lands. Gives itself a stack before touching
// anything that could need one, then hands over to Rust.
.global rivet_secondary_entry
rivet_secondary_entry:
    mrs     x0, mpidr_el1
    and     x0, x0, #3
    // 64 KiB of stack per core, indexed by core number, growing down from
    // the top of this core's slot.
    ldr     x1, =__core_stacks_bottom
    add     x2, x0, #1
    lsl     x2, x2, #16
    add     x1, x1, x2
    mov     sp, x1
    bl      rivet_secondary_main
.Lsecondary_halt:
    wfe
    b       .Lsecondary_halt
"#
);

extern "C" {
    fn rivet_secondary_entry();
}

/// Trampoline out of assembly and into whatever the caller registered.
///
/// Indirecting through a function pointer rather than a fixed symbol
/// keeps binaries that never release a core from having to define one.
#[no_mangle]
extern "C" fn rivet_secondary_main(core: u64) -> ! {
    let entry = SECONDARY_ENTRY.load(Ordering::Acquire);
    if entry == 0 {
        // Nobody registered anything. Park rather than branch to zero.
        loop {
            // SAFETY: WFE is side-effect free.
            unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
        }
    }
    // SAFETY: `release_core` only ever stores a valid `extern "C" fn(u64) -> !`.
    let f: extern "C" fn(u64) -> ! = unsafe { core::mem::transmute(entry) };
    f(core)
}

/// Read this core's number from `MPIDR_EL1`.
pub fn current_core() -> usize {
    let v: u64;
    // SAFETY: reading MPIDR_EL1 has no side effects.
    unsafe {
        core::arch::asm!("mrs {}, mpidr_el1", out(reg) v,
                         options(nomem, nostack, preserves_flags))
    };
    (v & 0b11) as usize
}

/// Which cores have executed the parking loop. See [`RIVET_SMP_WITNESS`].
pub fn witness() -> [u64; 4] {
    let p = core::ptr::addr_of!(RIVET_SMP_WITNESS) as *const u64;
    let mut out = [0u64; 4];
    for (i, o) in out.iter_mut().enumerate() {
        // SAFETY: reading four in-bounds words of a static array. Plain
        // loads, because a parked core may have written them with its
        // caches off.
        *o = unsafe { core::ptr::read_volatile(p.add(i)) };
    }
    out
}

/// Wake `core` and send it to `entry`.
///
/// `entry` runs with the MMU off, caches off, interrupts masked, at
/// whatever exception level the firmware left the core in (EL2 on this
/// board), on a 64 KiB per-core stack this crate's linker script
/// reserves. It must not return.
///
/// # Safety
/// `core` must be 1, 2 or 3 and must not already have been released.
pub unsafe fn release_core(core: usize, entry: extern "C" fn(u64) -> !) {
    debug_assert!((1..=3).contains(&core));

    SECONDARY_ENTRY.store(entry as usize, Ordering::Release);

    let slot = (SPIN_TABLE_BASE + core * 8) as *mut u64;
    // SAFETY: the spin table is what the firmware parked these cores on,
    // and this slot belongs to the core being released.
    unsafe {
        core::ptr::write_volatile(slot, rivet_secondary_entry as *const () as u64);

        // Push both the entry pointer and the mailbox out to memory. The
        // target core is polling with its caches off and will not see
        // anything still sitting in this core's write-back cache. Harmless
        // when this core also has caches off, which is why it is
        // unconditional rather than guarded.
        core::arch::asm!(
            "dc cvac, {slot}",
            "dc cvac, {entry_ptr}",
            "dsb sy",
            "sev",
            slot = in(reg) slot as u64,
            entry_ptr = in(reg) &SECONDARY_ENTRY as *const _ as u64,
            options(nostack, preserves_flags),
        );
    }
}
