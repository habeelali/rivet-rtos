//! PMP guard bands (stack-overflow detection on RV32 M-mode).
//!
//! RISC-V PMP entries with `L=1` are enforced against M-mode and immutable
//! until reset — so isolation is boot-time-static: each task stack's
//! 64-byte guard band is denied by a locked entry programmed when the
//! stack is allocated, and entry 15 is a locked TOR catch-all that
//! explicitly allows everything above the last guard. Lower indices win,
//! so guards take precedence over the catch-all. Overflow past a stack's
//! low end faults (mcause 5/7); the kernel's own access to stacks is
//! unaffected (only the 64-byte guard is denied).
//!
//! Pure ISA — no board/MMIO knowledge.

const NAPOT_GUARD_CFG: u8 = 0x98; // L | NAPOT | no RWX
const TOR_ALLOW_CFG: u8 = 0x8F; // L | TOR | RWX

/// Program the guard for stack allocation `entry` (0-14): a locked NAPOT
/// entry denying the 64-byte band below the stack.
pub fn register_guard(guard_base: usize, entry: usize) {
    use riscv::register::pmpaddr0;
    // NAPOT for a 64-byte region: pmpaddr low 3 bits = 0b111, address >> 2.
    let pmpaddr = (guard_base >> 2) | 0b111;
    // Write the ADDRESS first, then the config byte: the config write
    // (with L=1) LOCKS the entry, and QEMU rejects (and logs a guest
    // error for) any pmpaddr write to an already-locked entry.
    match entry {
        0 => pmpaddr0::write(pmpaddr),
        1 => riscv::register::pmpaddr1::write(pmpaddr),
        2 => riscv::register::pmpaddr2::write(pmpaddr),
        3 => riscv::register::pmpaddr3::write(pmpaddr),
        4 => riscv::register::pmpaddr4::write(pmpaddr),
        5 => riscv::register::pmpaddr5::write(pmpaddr),
        6 => riscv::register::pmpaddr6::write(pmpaddr),
        7 => riscv::register::pmpaddr7::write(pmpaddr),
        8 => riscv::register::pmpaddr8::write(pmpaddr),
        9 => riscv::register::pmpaddr9::write(pmpaddr),
        10 => riscv::register::pmpaddr10::write(pmpaddr),
        11 => riscv::register::pmpaddr11::write(pmpaddr),
        12 => riscv::register::pmpaddr12::write(pmpaddr),
        13 => riscv::register::pmpaddr13::write(pmpaddr),
        14 => riscv::register::pmpaddr14::write(pmpaddr),
        _ => return, // beyond the PMP budget — watermark fallback
    }
    // Now lock the entry (L=1 | NAPOT | no access).
    match entry {
        0 => pmpcfg_write_byte(0, NAPOT_GUARD_CFG),
        1 => pmpcfg_write_byte(1, NAPOT_GUARD_CFG),
        2 => pmpcfg_write_byte(2, NAPOT_GUARD_CFG),
        3 => pmpcfg_write_byte(3, NAPOT_GUARD_CFG),
        4 => pmpcfg_write_byte(4, NAPOT_GUARD_CFG),
        5 => pmpcfg_write_byte(5, NAPOT_GUARD_CFG),
        6 => pmpcfg_write_byte(6, NAPOT_GUARD_CFG),
        7 => pmpcfg_write_byte(7, NAPOT_GUARD_CFG),
        8 => pmpcfg_write_byte(8, NAPOT_GUARD_CFG),
        9 => pmpcfg_write_byte(9, NAPOT_GUARD_CFG),
        10 => pmpcfg_write_byte(10, NAPOT_GUARD_CFG),
        11 => pmpcfg_write_byte(11, NAPOT_GUARD_CFG),
        12 => pmpcfg_write_byte(12, NAPOT_GUARD_CFG),
        13 => pmpcfg_write_byte(13, NAPOT_GUARD_CFG),
        14 => pmpcfg_write_byte(14, NAPOT_GUARD_CFG),
        _ => {}
    }
}

/// Set the 8-bit config byte for PMP entry `i` in the right pmpcfg register.
fn pmpcfg_write_byte(i: usize, byte: u8) {
    use riscv::register::pmpcfg0;
    let shift = (i % 4) * 8;
    let mask = 0xFFusize << shift;
    let value = (byte as usize) << shift;
    match i / 4 {
        0 => pmpcfg0::write((pmpcfg0::read().bits & !mask) | value),
        1 => {
            riscv::register::pmpcfg1::write((riscv::register::pmpcfg1::read().bits & !mask) | value)
        }
        2 => {
            riscv::register::pmpcfg2::write((riscv::register::pmpcfg2::read().bits & !mask) | value)
        }
        3 => {
            riscv::register::pmpcfg3::write((riscv::register::pmpcfg3::read().bits & !mask) | value)
        }
        _ => {}
    }
}

/// Locked catch-all allow for M-mode: everything above the last guard is
/// explicitly permitted. Called once at boot.
pub(crate) fn init_catch_all() {
    use riscv::register::pmpaddr15;
    // Address first, then the locking config (writing pmpaddr to a locked
    // entry is rejected by hardware/QEMU).
    // pmpaddr15 = 0xFFFFFFFF makes entry 15's TOR range end at the top of
    // the address space (safe CSR write).
    pmpaddr15::write(0xFFFF_FFFF);
    // L=1 freezes the entry until reset.
    pmpcfg_write_byte(15, TOR_ALLOW_CFG);
}
