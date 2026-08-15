//! Identity-mapping MMU bring-up for EL1.
//!
//! This exists for one reason: to make atomic read-modify-write work.
//! With the MMU off, AArch64 treats every access as Device-nGnRnE, which
//! has no exclusive monitor, so `LDXR`/`STXR` aborts. Measured on a Pi 3B
//! as ESR `0x96000035` (EC 0x25, data fault status 0x35). The kernel uses
//! atomics throughout, so nothing of it can run here until RAM is
//! described as Normal Inner-Shareable Write-Back memory, which is what
//! the tables below do.
//!
//! The map is deliberately flat and coarse. Virtual equals physical
//! everywhere, so enabling translation does not move the program,
//! the stack or the peripherals out from under the code doing the
//! enabling:
//!
//! ```text
//! 0x0000_0000 .. 0x3F00_0000   RAM         Normal, Inner-Shareable, WB
//! 0x3F00_0000 .. 0x4000_0000   peripherals Device-nGnRnE, XN
//! 0x4000_0000 .. 0x8000_0000   ARM-local   Device-nGnRnE, XN
//! ```
//!
//! Two 4 KiB tables cover that: one level-1 table whose four entries each
//! span 1 GiB, and one level-2 table splitting the first gigabyte into
//! 2 MiB blocks so RAM and the peripheral window can differ. No level-3
//! tables and no 4 KiB pages, because nothing here needs finer grain.

use core::ptr::addr_of_mut;

use crate::mmio::PERIPHERAL_BASE;

/// One translation table. 512 entries of 8 bytes, and the architecture
/// requires the whole thing to be 4 KiB aligned.
#[repr(C, align(4096))]
struct Table([u64; 512]);

static mut L1: Table = Table([0; 512]);
static mut L2: Table = Table([0; 512]);

// Descriptor bits, shared between block and table entries.
const VALID: u64 = 1 << 0;
/// At levels 0-2 this bit distinguishes a table descriptor from a block.
const TABLE: u64 = 1 << 1;
/// Access flag. Leaving it clear makes the first touch of every mapping
/// take an access-flag fault, which is a popular way to lose an evening.
const AF: u64 = 1 << 10;
/// Inner Shareable. This is the part that actually matters for atomics:
/// the global exclusive monitor only works for Normal Inner-Shareable
/// cacheable memory.
const SH_INNER: u64 = 0b11 << 8;
/// Read/write at EL1, no EL0 access.
const AP_RW_EL1: u64 = 0b00 << 6;
/// Privileged and unprivileged execute-never, for peripherals.
const XN: u64 = (1 << 53) | (1 << 54);

/// MAIR attribute slots, referenced by index from each descriptor in
/// bits 4:2. Normal is index 0, so it contributes nothing to OR in.
const ATTR_NORMAL: u64 = 0;
const ATTR_DEVICE: u64 = 1 << 2;

/// Attribute 0: Normal, Inner and Outer Write-Back Read/Write-Allocate,
/// non-transient. Attribute 1: Device-nGnRnE.
const MAIR_VALUE: u64 = 0xFF;

const BLOCK_2M: usize = 2 * 1024 * 1024;
const BLOCK_1G: usize = 1024 * 1024 * 1024;

/// Everything from here up is peripherals rather than RAM.
const RAM_END: usize = PERIPHERAL_BASE;

/// A 2 MiB window of RAM mapped Device rather than Normal, for memory
/// shared with another operating system.
///
/// Mapping it non-cacheable is not an optimisation choice, it is what
/// makes the sharing work. Linux hands out `/dev/mem` mappings of
/// non-RAM regions as uncached, so if this side wrote through a
/// write-back cache the two would simply not see each other. Matching
/// Device on both sides removes the question. It also removes any need
/// for cache maintenance in the console fast path.
///
/// Must stay 2 MiB aligned, since that is the block size the level-2
/// table uses.
pub const SHARED_BASE: usize = crate::shmem::SHARED_BASE;
const SHARED_LEN: usize = BLOCK_2M;

/// Build the translation tables and turn on the MMU, the data cache and
/// the instruction cache at EL1.
///
/// Returns with translation enabled and the program still executing at
/// the same addresses, because the map is an identity map.
///
/// # Safety
/// Must be called exactly once, from EL1, with the MMU off. Reprograms
/// `MAIR_EL1`, `TCR_EL1`, `TTBR0_EL1` and `SCTLR_EL1`, so nothing else
/// may be relying on the current translation regime.
pub unsafe fn enable_el1() {
    // SAFETY: same contract as this function.
    unsafe {
        build_tables();
        enable_el1_prebuilt();
    }
}

/// Turn on translation using tables another core already built.
///
/// Separate from [`enable_el1`] so that a secondary core can join an
/// existing address space rather than rewriting it underneath the core
/// that made it. The values would be identical either way, but writing
/// them from a core with caches off while another has them on is exactly
/// the kind of thing worth not doing.
///
/// # Safety
/// [`build_tables`] must already have run, and the caller must be at EL1
/// with the MMU off.
pub unsafe fn enable_el1_prebuilt() {
    let l1 = addr_of_mut!(L1) as *mut u64;
    // SAFETY: forwarded from this function's own contract.
    unsafe { program_and_enable(l1) }
}

/// Populate the translation tables without enabling anything.
///
/// Idempotent: every entry is a pure function of the constants above, so
/// running it twice writes the same bytes.
///
/// # Safety
/// Must not run while another core is walking these tables.
pub unsafe fn build_tables() {
    let l1 = addr_of_mut!(L1) as *mut u64;
    let l2 = addr_of_mut!(L2) as *mut u64;

    // Level 2: the first gigabyte, in 2 MiB blocks. RAM up to the
    // peripheral base is Normal; the rest of the gigabyte is the
    // peripheral window.
    let mut addr = 0usize;
    for i in 0..512 {
        let shared = (SHARED_BASE..SHARED_BASE + SHARED_LEN).contains(&addr);
        let attrs = if shared {
            // Shared with another OS: Device, so both sides agree on
            // visibility without cache maintenance. Executable never.
            VALID | AF | AP_RW_EL1 | ATTR_DEVICE | XN
        } else if addr < RAM_END {
            VALID | AF | SH_INNER | AP_RW_EL1 | ATTR_NORMAL
        } else {
            VALID | AF | AP_RW_EL1 | ATTR_DEVICE | XN
        };
        l2.add(i).write_volatile(addr as u64 | attrs);
        addr += BLOCK_2M;
    }

    // Level 1, four entries of 1 GiB each.
    //   [0] delegates to the level-2 table above.
    //   [1] is the ARM-local peripheral block at 0x4000_0000 (per-core
    //       timers and mailboxes), mapped Device as a 1 GiB block.
    //   [2] and [3] stay invalid: this board has nothing there, and
    //       leaving them unmapped turns a stray access into a fault
    //       rather than silence.
    l1.add(0).write_volatile(l2 as u64 | VALID | TABLE);
    l1.add(1)
        .write_volatile(BLOCK_1G as u64 | VALID | AF | AP_RW_EL1 | ATTR_DEVICE | XN);
    l1.add(2).write_volatile(0);
    l1.add(3).write_volatile(0);
}

/// Point the translation registers at `l1` and switch translation on.
///
/// # Safety
/// `l1` must be a populated level-1 table; the caller must be at EL1 with
/// the MMU off.
unsafe fn program_and_enable(l1: *mut u64) {
    // 4 KiB granule, 32-bit virtual address space. T0SZ=32 makes the
    // level-1 table the starting level with exactly the four entries
    // written above. TTBR1 walks are disabled: there is no high half.
    // TG0 (bits 15:14) and the top-byte/reserved fields are all left at
    // zero: 0b00 there selects the 4 KiB granule, so there is nothing to
    // OR in for it.
    let tcr: u64 = 32                  // T0SZ: 2^(64-32) = 4 GiB of VA
        | (0b01 << 8)                  // IRGN0: walks are inner WB WA cacheable
        | (0b01 << 10)                 // ORGN0: and outer WB WA cacheable
        | (0b11 << 12)                 // SH0:   walks are inner shareable
        | (1 << 23)                    // EPD1:  no TTBR1 walks
        | (0b010 << 32); // IPS: 40-bit physical addresses, what A53 implements

    core::arch::asm!(
        // The table writes above must be observable to the table walker
        // before it is pointed at them. They are ordinary stores to what
        // is still Device memory at this point, but ordering against a
        // system-register write is not implied, so make it explicit.
        "dsb sy",
        "msr mair_el1, {mair}",
        "msr tcr_el1,  {tcr}",
        "msr ttbr0_el1, {ttbr}",
        "isb",
        // Translation state must be clean before the first walk. Caches
        // have been off since reset, which the A53 leaves invalidated,
        // so no data-cache maintenance is needed here beyond this.
        "tlbi vmalle1",
        "ic iallu",
        "dsb sy",
        "isb",
        mair = in(reg) MAIR_VALUE,
        tcr  = in(reg) tcr,
        ttbr = in(reg) l1 as u64,
        options(nostack, preserves_flags),
    );

    // M enables translation, C the data cache, I the instruction cache.
    // The ISB afterwards is what makes the change take effect for the
    // instructions that follow.
    core::arch::asm!(
        "mrs {t}, sctlr_el1",
        "orr {t}, {t}, #(1 << 0)",     // M
        "orr {t}, {t}, #(1 << 2)",     // C
        "orr {t}, {t}, #(1 << 12)",    // I
        "msr sctlr_el1, {t}",
        "isb",
        t = out(reg) _,
        options(nostack, preserves_flags),
    );
}

/// Whether translation is currently enabled at EL1.
pub fn enabled_el1() -> bool {
    let sctlr: u64;
    // SAFETY: reading SCTLR_EL1 has no side effects.
    unsafe {
        core::arch::asm!("mrs {}, sctlr_el1", out(reg) sctlr,
                         options(nomem, nostack, preserves_flags))
    };
    sctlr & 1 != 0
}
