//! RISC-V (RV32) architecture port.
//!
//! Targets: QEMU virt machine (rv32), ESP32-C3.

// ── Sleep ─────────────────────────────────────────────────────────

/// Minimum task stack: the 128-byte trap frame plus slack for the entry
/// trampoline (plan.md §2.7).
pub const MIN_TASK_STACK: usize = 128 + 128;

pub fn sleep() {
    riscv::asm::wfi();
}

// ── Executor pend / preemptive-tier yield (machine software interrupt) ─

pub fn pend_executor() {
    trigger_software_interrupt();
}

pub fn yield_now() {
    trigger_software_interrupt();
}

fn trigger_software_interrupt() {
    // Set MSIP (machine software interrupt pending) in the CLINT.
    // On QEMU virt, the CLINT is at 0x02000000; MSIP for hart 0 is at offset 0x0000.
    const CLINT_BASE: *mut u32 = 0x0200_0000 as *mut u32;
    // SAFETY: `CLINT_BASE` is the fixed, memory-mapped CLINT MSIP register
    // on the QEMU virt machine (and other standard RISC-V platforms); a
    // volatile write of 1 sets the machine software interrupt pending bit.
    unsafe {
        core::ptr::write_volatile(CLINT_BASE, 1);
    }
}

// ── Timer ─────────────────────────────────────────────────────────
//
// The CLINT's `mtime`/`mtimecmp` registers are 64-bit, but this is an RV32
// target: a naive `*mut u64` `read_volatile`/`write_volatile` compiles to
// two separate 32-bit bus accesses with no atomicity guarantee. A read can
// be torn (high word read, mtime rolls over the low word, low word read —
// producing a value off by up to 2^32), and — the specific bug this caused
// here — a torn *write* to mtimecmp could leave the high word holding a
// stale/huge value from a previous arm, silently pushing the "next tick"
// far enough into the future that it never fires again in practice.
// Standard fix: read the high word twice around the low word and retry if
// it changed; write the low word to all-1s before updating the high word
// so a torn write can never observably produce an earlier deadline than
// intended, then write the real low word.

const MTIME_LO: *const u32 = 0x0200_BFF8 as *const u32;
const MTIME_HI: *const u32 = 0x0200_BFFC as *const u32;
const MTIMECMP_LO: *mut u32 = 0x0200_4000 as *mut u32;
const MTIMECMP_HI: *mut u32 = 0x0200_4004 as *mut u32;

fn clint_read_mtime() -> u64 {
    // SAFETY: MTIME_HI/MTIME_LO are the fixed CLINT mtime registers
    // (memory-mapped, volatile). The hi/lo/hi-recheck loop keeps the read
    // tear-free on RV32 (see the module docs above the constants).
    unsafe {
        loop {
            let hi = core::ptr::read_volatile(MTIME_HI);
            let lo = core::ptr::read_volatile(MTIME_LO);
            let hi2 = core::ptr::read_volatile(MTIME_HI);
            if hi == hi2 {
                return ((hi as u64) << 32) | (lo as u64);
            }
        }
    }
}

fn clint_write_mtimecmp(val: u64) {
    // SAFETY: MTIMECMP_LO/HI are the fixed CLINT mtimecmp registers
    // (memory-mapped, volatile). The lo-all-ones/low-first write order
    // makes a torn write unobservable (see the module docs above the
    // constants).
    unsafe {
        core::ptr::write_volatile(MTIMECMP_LO, 0xFFFF_FFFF);
        core::ptr::write_volatile(MTIMECMP_HI, (val >> 32) as u32);
        core::ptr::write_volatile(MTIMECMP_LO, val as u32);
    }
}

/// CLINT `mtime` frequency on QEMU virt: 10 MHz → 10 `mtime` ticks per µs.
const MTIME_PER_MICRO: u64 = 10;
/// Tick period in `mtime` ticks (1 ms at 10 MHz).
const TICK_PERIOD: u64 = 10_000;

/// Previous mtimecmp value armed by the tick handler. Single writer (the
/// timer ISR); used to re-arm from the *previous* compare value rather
/// than from `mtime`, so each tick advances exactly `TICK_PERIOD` and
/// interrupt-entry latency can never accumulate as drift (plan.md [B6]).
static mut MTIMECMP_PREV: u64 = 0;

/// Current time in microseconds, derived directly from the CLINT's 64-bit
/// `mtime` (plan.md [B4]): `mtime / 10` on QEMU virt's 10 MHz clock.
///
/// There is deliberately NO software counter here: `mtime` is a hardware
/// 64-bit counter, so the read is tear-free (hi/lo/hi-recheck) and
/// monotonic, and `now_micros()` can never drift from the hardware clock.
pub fn now_micros() -> u64 {
    clint_read_mtime() / MTIME_PER_MICRO
}

/// Increment system time and wake any expired `Sleep` futures.
/// Call from the machine timer trap.
fn timer_tick() {
    let now = clint_read_mtime();
    crate::watchdog::on_tick();
    crate::timer::poll_timers(now / MTIME_PER_MICRO);

    // Re-arm from the previous mtimecmp value (not from `mtime`): each
    // tick advances the compare value by exactly TICK_PERIOD, so the tick
    // cadence never drifts by interrupt-entry latency (plan.md [B6]).
    // SAFETY: timer_tick runs only in machine-timer ISR context (the sole
    // writer of MTIMECMP_PREV); interrupts are disabled throughout.
    let next = unsafe { MTIMECMP_PREV }
        .wrapping_add(TICK_PERIOD)
        // ...but never arm a compare value that is already in the past: if
        // the ISR itself took longer than one tick period (slow host under
        // -icount, debug output, fault handling), re-arming at prev+period
        // would fire the next interrupt *immediately* — an interrupt storm
        // that starves the guest. Coalesce missed ticks by arming from the
        // current time instead. Exact-cadence behavior is preserved whenever
        // the ISR keeps up (prev+period > now, the normal case).
        .max(clint_read_mtime() + TICK_PERIOD);
    clint_write_mtimecmp(next);
    // SAFETY: timer_tick runs only in machine-timer ISR context (the sole
    // writer of MTIMECMP_PREV); interrupts are disabled throughout.
    unsafe {
        MTIMECMP_PREV = next;
    }
}

// ── UART I/O (QEMU virt NS16550 at 0x10000000) ───────────────────

const UART0_DATA: *mut u8 = 0x1000_0000 as *mut u8;

pub fn debug_print_hex32(_n: u32) {}

pub fn debug_print(s: &str) {
    for &b in s.as_bytes() {
        // SAFETY: `UART0_DATA` is the fixed NS16550 data register on the
        // QEMU virt machine (memory-mapped, volatile); single byte writes
        // are the standard way to drive it.
        unsafe { core::ptr::write_volatile(UART0_DATA, b) };
    }
}

// ── Semihosting I/O (QEMU) ────────────────────────────────────────
// Alternative to UART debug_print(); not used by the example (which uses
// UART), kept as public API for users who want semihosting-only output.

#[allow(dead_code)]
fn semihosting_write0(ptr: *const u8) {
    const SYS_WRITE0: usize = 0x04;
    // SAFETY: this is the standard ARM semihosting "SYS_WRITE0" sequence
    // (the fixed 3-instruction magic pattern QEMU's semihosting matcher
    // recognizes); the caller guarantees `ptr` points at a NUL-terminated
    // string valid for the program's lifetime.
    unsafe {
        core::arch::asm!(
            "   .option push",
            "   .option norvc", // force 32-bit ebreak; QEMU's semihosting
            "   .align 4",      // magic-sequence matcher requires the exact
            "   slli x0, x0, 0x1f", // 3x 32-bit instruction pattern below —
            "   ebreak",             // the compressed `c.ebreak` (2 bytes)
            "   srai x0, x0, 7",     // that riscv32imac's C extension would
            "   .option pop",        // otherwise emit breaks the match.
            in("a0") SYS_WRITE0,
            in("a1") ptr,
            options(nostack, preserves_flags)
        );
    }
}

#[allow(dead_code)]
pub fn semihosting_print(s: &str) {
    let bytes = s.as_bytes();
    let len = bytes.len().min(127);
    let mut buf = [0u8; 128];
    buf[..len].copy_from_slice(&bytes[..len]);
    buf[len] = b'\0';
    semihosting_write0(buf.as_ptr());
}

// ── Exit (QEMU) ───────────────────────────────────────────────────
//
// Primary exit path: the `riscv.sifive.test` device on the virt machine.
// Writing 0x5555 to 0x100000 makes QEMU exit(0); `0x3333 | (code << 16)`
// makes QEMU exit(code); `0x7777` triggers a system reset (used by the
// watchdog tests in Phase 3). This is simpler and more robust than
// semihosting (no `.option norvc` magic-sequence constraints) and gives
// distinguishable failure codes — the QEMU test harness (xtask) asserts
// on them.

const SIFIVE_TEST_BASE: *mut u32 = 0x0010_0000 as *mut u32;
const SIFIVE_TEST_PASS: u32 = 0x5555;
const SIFIVE_TEST_FAIL: u32 = 0x3333;
const SIFIVE_TEST_RESET: u32 = 0x7777;

/// Exit QEMU with status 0 (pass). Never returns.
pub fn exit_success() -> ! {
    // SAFETY: `SIFIVE_TEST_BASE` is the fixed `riscv.sifive.test` device
    // on the QEMU virt machine (memory-mapped, volatile); the guest writes
    // the pass pattern to terminate the simulation.
    unsafe { core::ptr::write_volatile(SIFIVE_TEST_BASE, SIFIVE_TEST_PASS) };
    loop {
        core::hint::spin_loop();
    }
}

/// Exit QEMU with a distinguishable failure code. Never returns.
pub fn exit_failure(code: u32) -> ! {
    // SAFETY: same device as `exit_success`; `0x3333 | (code << 16)` makes
    // QEMU terminate with exit status `code`.
    unsafe { core::ptr::write_volatile(SIFIVE_TEST_BASE, SIFIVE_TEST_FAIL | (code << 16)) };
    loop {
        core::hint::spin_loop();
    }
}

/// Trigger a QEMU system reset via `riscv.sifive.test` (0x7777). Never
/// returns. Used by the watchdog / fault-policy tests.
pub fn system_reset() -> ! {
    // SAFETY: same device as `exit_success`; writing 0x7777 requests a
    // system reset.
    unsafe { core::ptr::write_volatile(SIFIVE_TEST_BASE, SIFIVE_TEST_RESET) };
    loop {
        core::hint::spin_loop();
    }
}

pub fn mpu_allow_scratch(_base: usize, _size: usize) {}
pub fn mpu_clear_scratch() {}

// ── PMP guard bands (plan.md §3.2) ─────────────────────────────────
//
// RISC-V PMP entries with L=1 are enforced against M-mode and immutable
// until reset — so isolation is boot-time-static: each task stack's 64-byte
// guard band is denied by a locked entry programmed when the stack is
// allocated, and entry 15 is a locked TOR catch-all that explicitly allows
// everything above the last guard. Lower indices win, so guards take
// precedence over the catch-all. Overflow past a stack's low end faults
// (mcause 5/7); the kernel's own access to stacks is unaffected (only the
// 64-byte guard is denied).

const PMP_NAPOT_GUARD_CFG: u8 = 0x98; // L | NAPOT | no RWX
const PMP_TOR_ALLOW_CFG: u8 = 0x8F; // L | TOR | RWX

/// Program the guard for stack allocation `entry` (0-14): a locked NAPOT
/// entry denying the 64-byte band below the stack.
pub fn pmp_register_guard(guard_base: usize, entry: usize) {
    use riscv::register::pmpaddr0;
    // NAPOT for a 64-byte region: pmpaddr low 3 bits = 0b111, address >> 2.
    let pmpaddr = (guard_base >> 2) | 0b111;
    // Write the ADDRESS first, then the config byte: the config write
    // (with L=1) LOCKS the entry, and QEMU rejects (and logs a guest
    // error for) any pmpaddr write to an already-locked entry.
    // The riscv crate's pmpaddr writes are safe functions; the entries are
    // configured before any task can fault on them.
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
        0 => pmpcfg_write_byte(0, PMP_NAPOT_GUARD_CFG),
        1 => pmpcfg_write_byte(1, PMP_NAPOT_GUARD_CFG),
        2 => pmpcfg_write_byte(2, PMP_NAPOT_GUARD_CFG),
        3 => pmpcfg_write_byte(3, PMP_NAPOT_GUARD_CFG),
        4 => pmpcfg_write_byte(4, PMP_NAPOT_GUARD_CFG),
        5 => pmpcfg_write_byte(5, PMP_NAPOT_GUARD_CFG),
        6 => pmpcfg_write_byte(6, PMP_NAPOT_GUARD_CFG),
        7 => pmpcfg_write_byte(7, PMP_NAPOT_GUARD_CFG),
        8 => pmpcfg_write_byte(8, PMP_NAPOT_GUARD_CFG),
        9 => pmpcfg_write_byte(9, PMP_NAPOT_GUARD_CFG),
        10 => pmpcfg_write_byte(10, PMP_NAPOT_GUARD_CFG),
        11 => pmpcfg_write_byte(11, PMP_NAPOT_GUARD_CFG),
        12 => pmpcfg_write_byte(12, PMP_NAPOT_GUARD_CFG),
        13 => pmpcfg_write_byte(13, PMP_NAPOT_GUARD_CFG),
        14 => pmpcfg_write_byte(14, PMP_NAPOT_GUARD_CFG),
        _ => {}
    }
}

/// Set the 8-bit config byte for PMP entry `i` in the right pmpcfg register.
fn pmpcfg_write_byte(i: usize, byte: u8) {
    use riscv::register::pmpcfg0;
    let shift = (i % 4) * 8;
    // CSR read-modify-write; only the target entry's byte changes (the
    // riscv crate's pmpcfg writes are safe functions).
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
/// explicitly permitted (plan.md §3.2). Called once at boot.
fn pmp_init_catch_all() {
    use riscv::register::pmpaddr15;
    // Address first, then the locking config (writing pmpaddr to a locked
    // entry is rejected by hardware/QEMU).
    // pmpaddr15 = 0xFFFFFFFF makes entry 15's TOR range end at the top of
    // the address space (safe CSR write).
    pmpaddr15::write(0xFFFF_FFFF);
    // L=1 freezes the entry until reset.
    pmpcfg_write_byte(15, PMP_TOR_ALLOW_CFG);
}

pub fn on_switch_to(_stack_base: usize, _stack_size: usize) {
    // RISC-V PMP entries that affect M-mode must be locked at boot and are
    // immutable until reset (plan.md §3.2), so there is nothing to do per
    // switch — isolation comes from the boot-time guard bands.
}

/// Semihosting-based exit, kept as a secondary path (the primary is
/// `riscv.sifive.test` via [`exit_success`]). Never returns.
#[allow(dead_code)]
pub fn exit_success_semihosting() -> ! {
    const SYS_EXIT: usize = 0x18;
    // a1 holds the ADP stop reason directly (QEMU's RISC-V semihosting uses
    // the "simple" convention here, not a pointer to a [reason, subcode]
    // block — passing a pointer made QEMU treat the request as unrecognized
    // and exit(1) instead of exit(0)).
    const ADP_STOPPED_APPLICATIONEXIT: usize = 0x20026;
    // SAFETY: this is the standard ARM semihosting "SYS_EXIT" sequence;
    // the inline asm is `noreturn` and QEMU terminates the guest on it.
    unsafe {
        core::arch::asm!(
            "   .option push",
            "   .option norvc", // see semihosting_write0: must not compress
            "   .align 4",
            "   slli x0, x0, 0x1f",
            "   ebreak",
            "   srai x0, x0, 7",
            "   .option pop",
            in("a0") SYS_EXIT,
            in("a1") ADP_STOPPED_APPLICATIONEXIT,
            options(nostack, noreturn)
        );
    }
}

// ── Early init ────────────────────────────────────────────────────

// Dedicated ISR stack (plan.md §2.1 / [B3]): the trap entry keeps the
// 128-byte register frame on the interrupted task's stack (so `Tcb.sp`
// semantics are unchanged and per-task stack sizing is analyzable) but
// runs the *Rust* handler — `schedule()`, `poll_timers`, `panic!`
// formatting, and (Phase 3) the fault handler — on this dedicated stack,
// switched to via `mscratch`. Without it, the handler's own call chain
// competes for space with the task's stack, and a stack-overflow fault
// would re-enter the handler on the already-overflowed stack (double-fault
// loop).
extern "C" {
    static __isr_stack_top: u8;
}

pub fn early_init() {
    use riscv::register::{mie, mtvec};

    // Point mscratch at the top of the ISR stack. The trap entry swaps
    // `sp <-> mscratch` (`csrrw sp, mscratch, sp`), so after the swap
    // mscratch holds the interrupted task's frame pointer and sp is the
    // ISR stack. Must be set before any interrupt can fire. (The riscv
    // crate's `mscratch::write` is a safe function.)
    riscv::register::mscratch::write(core::ptr::addr_of!(__isr_stack_top) as usize);

    // Set up trap handler for machine-mode interrupts.
    //
    // IMPORTANT: mtvec must point to `rivet_trap_entry` (hand-written asm),
    // NOT a plain `extern "C" fn`. RISC-V hardware does not auto-save any
    // general-purpose registers on trap entry (unlike Cortex-M's automatic
    // exception stacking). A normal Rust function's prologue only preserves
    // *callee-saved* registers — it silently clobbers whatever caller-saved
    // registers (a0-a7, t0-t6) the interrupted code had live, corrupting
    // in-progress computations whenever a timer interrupt lands mid-function.
    // `rivet_trap_entry` saves/restores the full register file before/after
    // calling into Rust.
    // SAFETY: `rivet_trap_entry` is the hand-written trap entry defined in
    // this module's global_asm! block; installing it as the direct-mode
    // mtvec handler is required for every interrupt/fault to reach the
    // kernel's dispatcher.
    unsafe {
        mtvec::write(
            rivet_trap_entry as *const () as usize,
            mtvec::TrapMode::Direct,
        );
    }

    // Locked PMP catch-all: everything above the last guard band is
    // explicitly allowed for M-mode (plan.md §3.2). Guards programmed at
    // spawn time take precedence (lower index wins).
    pmp_init_catch_all();

    // Set up machine timer for periodic interrupts. Arm the first compare
    // value and record it as MTIMECMP_PREV so the tick handler re-arms
    // from it (plan.md [B6]).
    let first = clint_read_mtime() + TICK_PERIOD;
    clint_write_mtimecmp(first);
    // SAFETY: early_init runs before any interrupt is enabled, so this is
    // the sole write to MTIMECMP_PREV before the ISR takes over.
    unsafe {
        MTIMECMP_PREV = first;
    }
    // Unmask timer/software interrupt *sources* (mie), but deliberately do
    // NOT set the global mstatus.MIE enable here. If interrupts were live
    // this early, a tick could land while `preempt::start()` is still
    // executing on the plain boot stack, between `sched::set_current(first)`
    // (which makes task 1 look "current") and `start_first_task` actually
    // transferring control to it — `on_tick` would then treat the boot
    // stack as if it were task 1's real context, corrupting its saved sp
    // before it ever ran a single instruction. Global MIE is instead
    // enabled for the first time by `start_first_task`'s bootstrap `mret`
    // (via mstatus.MPIE), at the exact moment we're safely running as a
    // real registered task.
    // SAFETY: enabling the machine timer and software interrupt *sources*
    // in `mie` is safe because the global mstatus.MIE is still clear (see
    // the comment above); no interrupt can fire until the first `mret`.
    unsafe {
        mie::set_mtimer();
        mie::set_msoft();
    }
}

/// Trap dispatch logic, called from `rivet_trap_entry` with the interrupted
/// context (28 GPRs + mepc) already saved to the trap stack frame at
/// `interrupted_sp`.
///
/// Returns the stack pointer to actually resume from — this is what makes
/// preemption real: if [`crate::preempt::on_tick`] decides a different,
/// higher-priority task should run, this returns *that task's* saved sp
/// instead of `interrupted_sp`, and `rivet_trap_entry`'s epilogue restores
/// from there (including its saved mepc) instead of returning to where we
/// were interrupted.
#[no_mangle]
unsafe extern "C" fn rivet_trap_handler_rust(interrupted_sp: usize) -> usize {
    use riscv::register::mcause;

    let cause = mcause::read();
    let mut resume_sp = interrupted_sp;

    if cause.is_interrupt() {
        let code = cause.code();
        if code == 7 {
            // Machine timer interrupt: advance time, wake expired Sleep
            // futures, then give the preemptive scheduler a chance to
            // switch to a higher-priority ready task.
            timer_tick();
            resume_sp = crate::preempt::on_tick(interrupted_sp);
        } else if code == 3 {
            // Machine software interrupt: triggered by yield_now() (a
            // voluntary yield, or a mutex unlock waking a waiter). Clear
            // the pending bit, then run the same reschedule decision.
            const CLINT_BASE: *mut u32 = 0x0200_0000 as *mut u32;
            core::ptr::write_volatile(CLINT_BASE, 0);
            resume_sp = crate::preempt::on_tick(interrupted_sp);
        }
    } else {
        // Synchronous exception. Access faults (mcause 1/5/7) are routed
        // through the fault policy (plan.md §3.4): a faulting address
        // inside the task-stack pool means a stack-overflow PMP guard
        // trip; anything else is an ordinary wild pointer. The policy
        // either resets (Panic) or returns the next task's sp (Isolate).
        let code = cause.code();
        if matches!(code, 1 | 5 | 7) {
            let mepc = riscv::register::mepc::read();
            let mtval = riscv::register::mtval::read();
            let kind = match code {
                1 => crate::fault::FaultKind::InstructionAccess,
                5 => crate::fault::FaultKind::LoadAccess,
                _ => crate::fault::FaultKind::StoreAccess,
            };
            // Attribute to the running task (the trap handler runs on the
            // ISR stack; `sched::current()` is the interrupted task).
            let task_id = crate::preempt::sched::current();
            let info = crate::fault::FaultInfo {
                task_id,
                kind,
                address: mtval,
                pc: mepc,
            };
            return crate::fault::on_fault(&info);
        }

        // Deliberately NOT silently ignored: doing so turns any fault into
        // an infinite loop (mepc unchanged, resume_sp unchanged -> we
        // `mret` right back into the same faulting instruction forever).
        // That exact failure mode hid a real bug here once already: an
        // `mret` targeting the wrong privilege mode faulted on itself,
        // and a handler that only handled interrupts silently spun on it
        // with every outward symptom of "the resumed task made no
        // progress" instead of an obvious crash.
        panic!(
            "rivet: unhandled trap, mcause={:#x} mepc={:#x} mtval={:#x}",
            riscv::register::mcause::read().bits(),
            riscv::register::mepc::read(),
            riscv::register::mtval::read(),
        );
    }

    resume_sp
}

// Hand-written trap entry/exit: saves and restores the general-purpose
// register file (x1, x5-x31 — x0 is hardwired zero, x2/sp is the frame
// itself, x3/gp and x4/tp are fixed at boot and never touched by our code
// so are intentionally not saved, matching common RTOS RISC-V port
// convention) plus `mepc`, around the call into Rust. This is required
// because RISC-V `mret` does not auto-stack any registers — unlike
// Cortex-M, where hardware exception entry saves r0-r3/r12/lr/pc/xPSR
// for you.
//
// Saving `mepc` (not just GPRs) and letting the Rust handler return a
// *different* stack pointer to resume from is what turns this from "an
// interrupt handler that always returns to where it was called" into
// "a real preemptive context switch": the epilogue below always restores
// from whatever sp `rivet_trap_handler_rust` returned, which may belong
// to an entirely different task with its own previously-saved mepc.
//
// The frame (128 bytes) lives on the interrupted task's own stack; only
// the Rust call itself runs on the dedicated ISR stack (plan.md §2.1 /
// [B3]): `csrrw sp, mscratch, sp` atomically swaps sp and mscratch, so
// after the swap sp = ISR stack top and mscratch = the task frame pointer.
// The handler's *unbounded* call chain (schedule, poll_timers, panic!
// formatting, future fault handling) can therefore never overflow a task
// stack. Nested-trap handling (a fault inside the handler) is added in
// Phase 3 via an mscratch nesting discriminator.
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_trap_entry",
    "rivet_trap_entry:",
    "  addi sp, sp, -128",
    "  sw   ra,   0(sp)",
    "  sw   t0,   4(sp)",
    "  sw   t1,   8(sp)",
    "  sw   t2,  12(sp)",
    "  sw   s0,  16(sp)",
    "  sw   s1,  20(sp)",
    "  sw   a0,  24(sp)",
    "  sw   a1,  28(sp)",
    "  sw   a2,  32(sp)",
    "  sw   a3,  36(sp)",
    "  sw   a4,  40(sp)",
    "  sw   a5,  44(sp)",
    "  sw   a6,  48(sp)",
    "  sw   a7,  52(sp)",
    "  sw   s2,  56(sp)",
    "  sw   s3,  60(sp)",
    "  sw   s4,  64(sp)",
    "  sw   s5,  68(sp)",
    "  sw   s6,  72(sp)",
    "  sw   s7,  76(sp)",
    "  sw   s8,  80(sp)",
    "  sw   s9,  84(sp)",
    "  sw   s10, 88(sp)",
    "  sw   s11, 92(sp)",
    "  sw   t3,  96(sp)",
    "  sw   t4, 100(sp)",
    "  sw   t5, 104(sp)",
    "  sw   t6, 108(sp)",
    "  csrr t0, mepc",
    "  sw   t0, 112(sp)",
    "  mv   a0, sp",                  // arg: interrupted frame ptr
    "  csrrw sp, mscratch, sp",       // sp <- ISR stack; mscratch <- frame ptr
    "  call rivet_trap_handler_rust", // returns: resume_sp (in a0)
    // Re-arm mscratch for the next trap: after `csrrw sp, mscratch, sp`
    // above, mscratch holds the interrupted frame pointer, NOT the ISR
    // stack top — leaving it would make the next trap swap sp to a task
    // frame (or zero) and fault. Nested-trap handling (a fault while
    // already in the handler) is Phase 3's mscratch discriminator work.
    "  la   t1, __isr_stack_top",
    "  csrw mscratch, t1",
    "  mv   sp, a0",            // switch to the resume stack
    "  j    rivet_trap_resume", // shared restore epilogue
);

extern "C" {
    fn rivet_trap_entry();
}

// ── Preemptive tier: stack bootstrap ──────────────────────────────

// Trampoline entered (via `mepc`) the first time a freshly-spawned
// preemptive task runs. `s0`/`s1` are ordinary GPRs restored by the trap
// epilogue like any other — used here to smuggle the type-erased
// `(arg, entry_fn)` pair out of the initial fake stack frame and into the
// real call.
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_ptask_trampoline",
    "rivet_ptask_trampoline:",
    "  mv   a0, s0", // arg
    "  jalr s1",     // entry(arg) — may return with the result in a0/a1
    "  j    rivet_task_exit",
    // Task exit: the entry returned. a0/a1 carry the return value (or, for
    // >8-byte results, a0 is a pointer into this task's own stack). The
    // core stores it, marks the task exited, wakes joiners, and parks.
    ".global rivet_task_exit",
    "rivet_task_exit:",
    "  call rivet_task_exit_core",
    "1:",
    "  j 1b",
);

extern "C" {
    fn rivet_ptask_trampoline();
}

/// Build the initial trap-frame-shaped stack for a new preemptive task,
/// matching `rivet_trap_entry`'s restore layout exactly, so the very first
/// "resume" (via [`start_first_task`] or a tick that schedules it in) goes
/// through the identical epilogue code path as any other switch.
pub unsafe fn init_task_stack(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    const FRAME_WORDS: usize = 32; // 128 bytes, matches rivet_trap_entry's frame
    const STACK_ALIGN: usize = 16;

    let base = stack.as_mut_ptr() as usize;
    let top = base + stack.len();
    let frame_start = (top - FRAME_WORDS * 4) & !(STACK_ALIGN - 1);
    let frame = frame_start as *mut u32;

    // Zero the whole frame, then fill in what the trampoline needs.
    for i in 0..FRAME_WORDS {
        core::ptr::write(frame.add(i), 0);
    }
    // s0 (offset 16 bytes = word 4) = arg
    core::ptr::write(frame.add(4), arg as u32);
    // s1 (offset 20 bytes = word 5) = entry_fn
    core::ptr::write(frame.add(5), entry_fn as u32);
    // mepc (offset 112 bytes = word 28) = trampoline address
    core::ptr::write(frame.add(28), rivet_ptask_trampoline as *const () as u32);

    frame_start
}

/// Enter the very first preemptive task. Reuses `rivet_trap_entry`'s
/// restore epilogue (jumped to directly, skipping the save+dispatch
/// portion) so there's exactly one place that knows how to reconstruct a
/// task's context from its saved frame.
pub unsafe fn start_first_task(sp: usize) -> ! {
    core::arch::asm!(
        "mv sp, {sp}",
        "j  rivet_trap_resume",
        sp = in(reg) sp,
        options(noreturn)
    );
}

core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".global rivet_trap_resume",
    "rivet_trap_resume:",
    "  lw   t0, 112(sp)",
    "  csrw mepc, t0",
    // mstatus.MPIE must be 1 so mret enables interrupts (MIE <- MPIE), and
    // mstatus.MPP must be M-mode (0b11) so mret returns to machine mode
    // instead of user mode. Both matter on EVERY resume, not just the
    // first: `mret` unconditionally resets MPP to the least-privileged
    // supported mode after each return, so if we don't re-assert M-mode
    // here, the very next mret targets U-mode. On QEMU (with zero PMP
    // rules configured) that makes `mret` itself fault — an instruction
    // access fault raised AT the mret, whose mepc then points back at the
    // same mret, so a naive trap handler that ignores non-interrupt causes
    // (ours did) silently loops on it forever: the task never executes a
    // single real instruction, yet every resume "succeeds" (correct saved
    // registers restored) right up until the fatal mret. This was the
    // actual root cause of a bug that looked exactly like a scheduler or
    // register-corruption issue: task A, launched via `start_first_task`
    // (bypassing any real trap that would have set MPP=M as a side
    // effect), never made any progress, while task B — first entered via
    // a genuine timer-tick trap return — worked fine.
    // 0x1880 = MPP (bits 12:11, both set = 0b11) | MPIE (bit 7).
    "  li   t0, 0x1880",
    "  csrs mstatus, t0",
    "  lw   ra,   0(sp)",
    "  lw   t0,   4(sp)",
    "  lw   t1,   8(sp)",
    "  lw   t2,  12(sp)",
    "  lw   s0,  16(sp)",
    "  lw   s1,  20(sp)",
    "  lw   a0,  24(sp)",
    "  lw   a1,  28(sp)",
    "  lw   a2,  32(sp)",
    "  lw   a3,  36(sp)",
    "  lw   a4,  40(sp)",
    "  lw   a5,  44(sp)",
    "  lw   a6,  48(sp)",
    "  lw   a7,  52(sp)",
    "  lw   s2,  56(sp)",
    "  lw   s3,  60(sp)",
    "  lw   s4,  64(sp)",
    "  lw   s5,  68(sp)",
    "  lw   s6,  72(sp)",
    "  lw   s7,  76(sp)",
    "  lw   s8,  80(sp)",
    "  lw   s9,  84(sp)",
    "  lw   s10, 88(sp)",
    "  lw   s11, 92(sp)",
    "  lw   t3,  96(sp)",
    "  lw   t4, 100(sp)",
    "  lw   t5, 104(sp)",
    "  lw   t6, 108(sp)",
    "  addi sp, sp, 128",
    // Symbol for the GDB context-switch verification script (tests/gdb):
    // at this point every saved register has been loaded and sp points just
    // past the frame, so the script can compare the live register file
    // against the frame contents (frame base = sp - 128).
    ".global rivet_trap_mret",
    "rivet_trap_mret:",
    "  mret",
);
