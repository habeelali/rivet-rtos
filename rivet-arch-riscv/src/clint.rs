//! Stock SiFive CLINT driver: `mtime`/`mtimecmp` tick source and `MSIP`
//! reschedule IPI.
//!
//! This is the "Group C" case from the layering plan: the *mechanism*
//! (torn-read-safe 64-bit mtime access, coalescing re-arm so ISR latency
//! never accumulates as drift, MSIP-based self-IPI) is universal across
//! CLINT-equipped RV32 platforms and belongs in the arch crate — but the
//! CLINT's base address and clock rate are per-platform facts that only
//! the board knows. Call [`configure`] once, from the board's
//! `__rivet_board_init`, before calling [`tick_start`].
//!
//! Boards without a CLINT (e.g. an ESP32-C3, which uses SYSTIMER) don't
//! enable the `clint` feature and wire their own tick/IPI source instead.

use core::sync::atomic::{AtomicUsize, Ordering};

static BASE: AtomicUsize = AtomicUsize::new(0);
// RV32 has no native 64-bit atomics. `MTIME_HZ`/`TICK_PERIOD` are written
// once at board init (`configure`/`tick_start`), before interrupts are
// enabled and before any concurrent reader exists; `MTIMECMP_PREV` is
// written only from timer-ISR context (interrupts disabled throughout).
// All three follow the same single-writer-before-any-reader-or-ISR-only
// discipline, matching the original arch/riscv.rs precedent.
//
// plan.md Phase 19 memory-ordering audit: under real SMP this needs one
// more link, not just re-stating "single writer" — the writer (hart 0,
// via `rivet::init()`'s `board::init()`/`tick_start()` calls) and any
// reader on a *different* hart (a secondary hart's `now_micros()`/
// `on_timer_irq()` — though per the module docs above, only hart 0 ever
// actually owns the tick, so in practice only hart 0 reads
// `TICK_PERIOD`/`MTIMECMP_PREV` too) are different threads of execution,
// so "happens before" needs a real synchronizes-with edge, not just
// program order. That edge exists: `rivet::run()` does a `Release` store
// to `KERNEL_READY` strictly after `init()` (and everything `init()`
// wrote, including these statics) completes, and `rivet-rt`'s secondary-
// hart bring-up spins on an `Acquire` load of the same flag before
// touching any kernel state — a standard Release/Acquire pair, so every
// write these functions make is guaranteed visible to a secondary hart by
// the time it starts running, not merely "boot-time-early" by convention.
static mut MTIME_HZ: u64 = 0;
static mut TICK_PERIOD: u64 = 0;

/// Previous mtimecmp value armed by the tick handler. Single writer (the
/// timer ISR); used to re-arm from the *previous* compare value rather
/// than from `mtime`, so each tick advances exactly `TICK_PERIOD` and
/// interrupt-entry latency can never accumulate as drift.
static mut MTIMECMP_PREV: u64 = 0;

/// Register offsets from `base`, per the SiFive CLINT memory map: MSIP at
/// offset 0, `mtimecmp` at `0x4000`, `mtime` at `0xBFF8`.
const MSIP_OFFSET: usize = 0x0000;
const MTIMECMP_OFFSET: usize = 0x4000;
const MTIME_OFFSET: usize = 0xBFF8;

/// Tell the driver where this board's CLINT lives and how fast `mtime`
/// counts. Must be called before any other function in this module.
pub fn configure(base: usize, mtime_hz: u64) {
    BASE.store(base, Ordering::Relaxed);
    // SAFETY: called once from board init, before interrupts are enabled
    // and before any other function in this module can run concurrently.
    unsafe {
        MTIME_HZ = mtime_hz;
    }
}

fn base() -> usize {
    let b = BASE.load(Ordering::Relaxed);
    debug_assert!(
        b != 0,
        "rivet-arch-riscv::clint::configure was never called"
    );
    b
}

// `mtime` (unlike `mtimecmp`/`msip`) is a single, hart-shared CLINT
// register, not one-per-hart — every hart reads the same counter, so
// `mtime_lo`/`mtime_hi` need no hart-awareness at all.
fn mtime_lo() -> *const u32 {
    (base() + MTIME_OFFSET) as *const u32
}
fn mtime_hi() -> *const u32 {
    (base() + MTIME_OFFSET + 4) as *const u32
}
// `mtimecmp` IS one-per-hart hardware (like `msip`), but unlike `msip`
// this crate deliberately keeps hart 0 as the sole tick owner (module
// docs above / plan.md Phase 19) — `tick_start`/`on_timer_irq` are only
// ever called from hart 0, so hardcoding offset 0 here is correct *by
// that design choice*, not the same bug `msip()` had (which was supposed
// to be hart-generic — every hart calls `request_reschedule()` on
// itself — but wasn't).
fn mtimecmp_lo() -> *mut u32 {
    (base() + MTIMECMP_OFFSET) as *mut u32
}
fn mtimecmp_hi() -> *mut u32 {
    (base() + MTIMECMP_OFFSET + 4) as *mut u32
}
/// The **calling hart's own** MSIP register. Before plan.md Phase 19 this
/// was hardwired to offset 0 (hart 0's register) — harmless when hart 0
/// was the only hart that ever ran kernel code, but wrong the moment a
/// secondary hart calls a self-targeting function like
/// [`request_reschedule`]: it would pend hart 0's software interrupt
/// instead of its own, so the calling hart's own trap would never fire
/// (found via a real, 100%-reproducible hang at `-smp 2` — the two-hart
/// case has less redundant cross-hart IPI traffic from
/// [`request_reschedule_on`] to accidentally mask the bug than `-smp 4`
/// did). Always resolves via the live `mhartid` CSR, not a cached value,
/// so it's correct regardless of which hart calls it.
fn msip() -> *mut u32 {
    msip_for(riscv::register::mhartid::read())
}

/// `msip[hart]`: the SiFive CLINT lays out one 4-byte `MSIP` register per
/// hart, contiguous from `MSIP_OFFSET` (plan.md Phase 19) — real,
/// per-hart hardware, not a software construct.
fn msip_for(hart: usize) -> *mut u32 {
    (base() + MSIP_OFFSET + hart * 4) as *mut u32
}

// The CLINT's `mtime`/`mtimecmp` registers are 64-bit, but this is an RV32
// target: a naive `*mut u64` read/write compiles to two separate 32-bit
// bus accesses with no atomicity guarantee. A read can be torn (high word
// read, mtime rolls over the low word, low word read — producing a value
// off by up to 2^32); a torn write to mtimecmp could leave the high word
// holding a stale/huge value from a previous arm, silently pushing "next
// tick" far enough into the future that it never fires again in practice.
// Fix: read the high word twice around the low word and retry if it
// changed; write the low word to all-1s before updating the high word so
// a torn write can never observably produce an earlier deadline than
// intended, then write the real low word.

fn read_mtime() -> u64 {
    // SAFETY: `mtime_hi`/`mtime_lo` are the configured CLINT mtime
    // registers (memory-mapped, volatile); the hi/lo/hi-recheck loop keeps
    // the read tear-free on RV32.
    unsafe {
        loop {
            let hi = core::ptr::read_volatile(mtime_hi());
            let lo = core::ptr::read_volatile(mtime_lo());
            let hi2 = core::ptr::read_volatile(mtime_hi());
            if hi == hi2 {
                return ((hi as u64) << 32) | (lo as u64);
            }
        }
    }
}

fn write_mtimecmp(val: u64) {
    // SAFETY: `mtimecmp_lo`/`mtimecmp_hi` are the configured CLINT
    // mtimecmp registers (memory-mapped, volatile). The lo-all-ones/
    // low-first write order makes a torn write unobservable.
    unsafe {
        core::ptr::write_volatile(mtimecmp_lo(), 0xFFFF_FFFF);
        core::ptr::write_volatile(mtimecmp_hi(), (val >> 32) as u32);
        core::ptr::write_volatile(mtimecmp_lo(), val as u32);
    }
}

/// Current time in microseconds, derived directly from the CLINT's
/// hardware 64-bit `mtime` counter — tear-free (hi/lo/hi-recheck) and
/// monotonic, so this can never drift from the hardware clock (there is
/// deliberately no software counter).
pub fn now_micros() -> u64 {
    // SAFETY: set once by `configure`, before any reader (including this
    // one) can run.
    let hz = unsafe { MTIME_HZ };
    (read_mtime() as u128 * 1_000_000 / hz as u128) as u64
}

/// Arm the periodic tick at `tick_hz` and unmask the machine timer/software
/// interrupt sources (but not the global `mstatus.MIE` enable — that first
/// happens at the bootstrap `mret` in `__rivet_arch_start_first_task`, once
/// a real task is safely current).
pub fn tick_start(tick_hz: u32) {
    // SAFETY: set once by `configure`, before any reader can run.
    let hz = unsafe { MTIME_HZ };
    let period = hz / tick_hz as u64;
    // SAFETY: tick_start runs once from board init, before interrupts are
    // enabled and before the ISR's reader can run.
    unsafe {
        TICK_PERIOD = period;
    }

    let first = read_mtime() + period;
    write_mtimecmp(first);
    // SAFETY: tick_start runs once from board init, before interrupts are
    // enabled, so this is the sole write to MTIMECMP_PREV before the ISR
    // takes over.
    unsafe {
        MTIMECMP_PREV = first;
    }

    // SAFETY: enabling the machine timer/software interrupt *sources* in
    // `mie` is safe because global mstatus.MIE is still clear at this point
    // in boot — no interrupt can fire until the first `mret`.
    unsafe {
        riscv::register::mie::set_mtimer();
        riscv::register::mie::set_msoft();
    }
}

/// Set MSIP: pends a machine software interrupt against this hart,
/// re-entering the trap handler to run the scheduler. This is the RV32
/// backend for `__rivet_arch_request_reschedule`.
pub fn request_reschedule() {
    // SAFETY: `msip()` is the configured CLINT MSIP register for this hart
    // (memory-mapped, volatile); writing 1 sets the pending bit.
    unsafe { core::ptr::write_volatile(msip(), 1) };
}

/// Set `msip[hart]`: pends a machine software interrupt against
/// **another** hart (plan.md Phase 19) — the RV32 backend for
/// `__rivet_arch_request_reschedule_on`. Required, not just an
/// optimization: this crate's SMP design keeps hart 0 as the sole CLINT
/// tick owner (see module docs), so a secondary hart that has run out of
/// ready work and is parked in `wfi` has no timer of its own to
/// eventually notice new work — without an explicit IPI, it would never
/// wake even after another hart makes a task ready for it.
pub fn request_reschedule_on(hart: usize) {
    // SAFETY: `msip_for(hart)` is `hart`'s CLINT MSIP register — real,
    // per-hart hardware (memory-mapped, volatile); writing 1 sets the
    // pending bit on that hart specifically, leaving every other hart's
    // pending bit untouched.
    unsafe { core::ptr::write_volatile(msip_for(hart), 1) };
}

/// Clear the pending software interrupt. Called from the trap dispatcher
/// on mcause=3 (machine software interrupt).
pub(crate) fn ack_soft_irq() {
    // SAFETY: as above; writing 0 clears the pending bit.
    unsafe { core::ptr::write_volatile(msip(), 0) };
}

/// Advance system time, feed the watchdog and timer queue, and re-arm the
/// next tick. Called from the trap dispatcher on mcause=7 (machine timer
/// interrupt).
pub(crate) fn on_timer_irq() {
    rivet::watchdog::on_tick();
    rivet::timer::poll_timers(now_micros());

    // SAFETY: written only by `tick_start`, before interrupts are enabled.
    let period = unsafe { TICK_PERIOD };
    // Re-arm from the previous mtimecmp value (not from `mtime`): each
    // tick advances the compare value by exactly `period`, so the tick
    // cadence never drifts by interrupt-entry latency.
    // SAFETY: on_timer_irq runs only in machine-timer ISR context (the
    // sole writer of MTIMECMP_PREV); interrupts are disabled throughout.
    let next = unsafe { MTIMECMP_PREV }
        .wrapping_add(period)
        // ...but never arm a compare value that is already in the past: if
        // the ISR itself took longer than one tick period (slow host under
        // -icount, debug output, fault handling), re-arming at prev+period
        // would fire the next interrupt *immediately* — an interrupt storm
        // that starves the guest. Coalesce missed ticks by arming from the
        // current time instead. Exact-cadence behavior is preserved
        // whenever the ISR keeps up (prev+period > now, the normal case).
        .max(read_mtime() + period);
    write_mtimecmp(next);
    // SAFETY: as above.
    unsafe {
        MTIMECMP_PREV = next;
    }
}
