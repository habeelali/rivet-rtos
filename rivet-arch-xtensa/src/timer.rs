//! Xtensa core-internal timer + software interrupt (plan.md Phase 21).
//!
//! Unlike RISC-V's CLINT (a board-attached MMIO peripheral, so its base
//! address is a Group C board fact), Xtensa's `CCOUNT`/`CCOMPARE0` and the
//! software-interrupt-set/clear mechanism are genuine CPU special
//! registers — no MMIO, no base address, nothing board-specific about the
//! *mechanism*. Only the CPU clock rate (needed to convert a tick period
//! in Hz into a `CCOMPARE0` delta in cycles) is a board fact, supplied via
//! [`configure`] from `rivet-bsp-esp32s3`'s `__rivet_board_init`, mirroring
//! `rivet-arch-riscv::clint`'s `configure(base, mtime_hz)` shape minus the
//! base address.
//!
//! The tick (`Timer1`, interrupt number 15, `CCOMPARE1`) and the
//! reschedule self-IPI (`Software1`, interrupt number 29) are configured,
//! on the `xtensa-esp32s3-none-elf` target, at CPU interrupt priority
//! level **3** (confirmed against `xtensa-lx-rt`'s own
//! `config/esp32s3.rs`: `XCHAL_INT15_LEVEL = 3`, `XCHAL_INT29_LEVEL = 3`)
//! — deliberately *not* `Timer0`/`Software0` (level 1, `INT6`/`INT7`):
//! `esp-hal` (linked in by `rivet-bsp-esp32s3` for its clock-tree and
//! watchdog-disable bring-up — see that crate's `Cargo.toml`) claims the
//! level-1 slot itself (`__level_1_interrupt`, defined in
//! `esp-hal::interrupt::xtensa`) for its own peripheral-interrupt
//! dispatch. Two crates cannot both provide the same
//! `xtensa-lx-rt`-mandated symbol — found as a real `multiple definition`
//! link error, not assumed — so Rivet's own scheduler interrupts use
//! level 3 instead, which nothing else in this dependency graph claims.
//! `rivet-arch-xtensa::__level_3_interrupt` is the shared dispatcher for
//! both, distinguishing them by reading which bit(s) are pending.

use core::sync::atomic::{AtomicU32, Ordering};

/// `PS.WOE` (Window Overflow Enable) — must be set for any windowed-ABI
/// code (which is everything on this target) to run at all; see
/// `lib.rs`'s module docs for where this is used to fabricate a fresh
/// task's initial `PS`.
pub const PS_WOE: u32 = 0x0004_0000;

/// Interrupt bit 15 (`Timer1`, `CCOMPARE1`), level 3.
pub const TIMER1_MASK: u32 = 1 << 15;
/// Interrupt bit 29 (`Software1`), level 3.
pub const SOFTWARE1_MASK: u32 = 1 << 29;

static CPU_HZ: AtomicU32 = AtomicU32::new(0);
static TICK_PERIOD: AtomicU32 = AtomicU32::new(0);
/// Previous `CCOMPARE0` value armed by the tick handler — re-arming from
/// this (not from a fresh `CCOUNT` read) is what keeps tick cadence from
/// drifting by interrupt-entry latency, the same coalescing technique
/// `rivet-arch-riscv::clint` uses for `mtimecmp`.
static CCOMPARE_PREV: AtomicU32 = AtomicU32::new(0);

/// Tell this module the CPU's real clock rate. Must be called before
/// [`tick_start`].
pub fn configure(cpu_hz: u32) {
    CPU_HZ.store(cpu_hz, Ordering::Relaxed);
}

/// Arm the periodic tick at `tick_hz` and enable both `Timer1` and
/// `Software1` at the interrupt controller. Does **not** touch the global
/// interrupt-enable state — matching every other arch's `tick_start`,
/// that first happens when the dispatched task's context is actually
/// resumed (`PS.INTLEVEL` restored to 0 as part of its `Context`).
pub fn tick_start(tick_hz: u32) {
    let hz = CPU_HZ.load(Ordering::Relaxed);
    debug_assert!(hz != 0, "rivet-arch-xtensa::timer::configure was never called");
    let period = hz / tick_hz;
    TICK_PERIOD.store(period, Ordering::Relaxed);

    let first = xtensa_lx::timer::get_cycle_count().wrapping_add(period);
    xtensa_lx::timer::set_ccompare1(first);
    CCOMPARE_PREV.store(first, Ordering::Relaxed);

    // SAFETY: enabling these two interrupt sources is safe because the
    // global interrupt-enable gate (`INTENABLE`, separate from this
    // per-source enable) stays clear until a task context with
    // `PS.INTLEVEL = 0` is actually resumed — matching every other arch's
    // "arm sources, enable globally only once a real task is current"
    // sequencing.
    unsafe {
        xtensa_lx::interrupt::enable_mask(TIMER1_MASK | SOFTWARE1_MASK);
    }
}

/// Re-arm `CCOMPARE1` for the next tick. Called from the level-3 handler
/// when `Timer1` is pending.
///
/// # Safety
/// Must only be called from within the level-3 interrupt handler.
pub unsafe fn on_timer_irq() {
    rivet::watchdog::on_tick();
    let period = TICK_PERIOD.load(Ordering::Relaxed);
    let now = xtensa_lx::timer::get_cycle_count();
    // Coalesce missed ticks (a slow handler, fault dump, etc.) by arming
    // from the current time instead of letting `prev + period` fire
    // immediately in the past — same reasoning as
    // `rivet-arch-riscv::clint::on_timer_irq`.
    let next = CCOMPARE_PREV
        .load(Ordering::Relaxed)
        .wrapping_add(period)
        .max(now.wrapping_add(period));
    xtensa_lx::timer::set_ccompare1(next);
    CCOMPARE_PREV.store(next, Ordering::Relaxed);

    // plan.md Phase 30 (round-robin fairness): give every *other* hart a
    // periodic "reconsider what you're running" nudge via the existing
    // cross-hart IPI plumbing (the same `request_reschedule_on` machinery
    // `ready_add`'s `wake_other_harts` already uses), without a second
    // hardware timer or touching `critical::enter`'s locking. The only
    // one of three fundamentally different fix designs that passed real-
    // hardware verification — see `rivet_appcpu_rust_entry`'s own comment
    // for the other two (both reverted on hardware evidence: one doubled
    // hart-0-owned tick duties across both cores, the other panicked at
    // boot on an unhandled level-1 interrupt).
    //
    // `BROADCAST_EVERY` was tuned empirically against real ESP32-S3
    // hardware, not derived analytically — a higher rate was tried and
    // rejected on hardware evidence:
    //   - Every tick (1x): measurably slowed the receiving hart's own
    //     useful work (`smp_latency_bench`'s `waiter` dropped from
    //     completing 1000 cross-core samples well inside 5s to ~35).
    //
    // Every 2nd tick used to hit a deterministic real-hardware fault
    // (`InstrProhibited`, `console::write_str`'s `retw.n` computing a
    // garbage return target because `A0` had gone to exactly zero) —
    // root-caused via live JTAG (`xtensa-esp-elf-gdb` + `openocd-esp32`
    // against the S3's native USB-JTAG): `CONTEXTS` (this crate's
    // per-task saved-register array, keyed by task id) is genuinely
    // shared across harts, but each `CONTEXTS[id]` read/write was a
    // plain, non-atomic 136-byte struct copy with *no* synchronization —
    // not even a lock scoped to just the copy. A higher broadcast rate
    // means more concurrent tick/dispatch activity on both harts, which
    // made a torn read of a live `Context` (hart B reading `CONTEXTS[id]`
    // mid-write by hart A) enough likelier to actually land within a
    // realistic test run. Fixed in `__level_3_interrupt` (see its own
    // comment) by wrapping each `CONTEXTS` copy in `critical::enter`. See
    // `ContextCell`'s own `unsafe impl Sync` comment for the corrected
    // safety reasoning.
    //
    // That fix has a real, measured cost: the extra `critical::enter`
    // call sites add stack usage to every dispatch, and dispatches run on
    // whichever task's stack was interrupted — 4096-byte task stacks
    // (this bench's original size) were no longer enough headroom even
    // at the shipped rate of 32, let alone 2; a *different* hardware
    // fault (`StoreProhibited`/`LoadProhibited`, a garbage-pointer write
    // from genuine stack corruption) appeared instead. 8192 bytes (2x)
    // still wasn't enough; 16384 (4x, this bench's current size) was
    // clean, 6/6 runs, at both `BROADCAST_EVERY = 2` and `= 32`. Any
    // application built against this arch with tight task stacks should
    // re-check its own headroom against this cost.
    //
    // Every 32nd tick: 3/3 clean runs of `smp_latency_bench` at each of
    // `BROADCAST_EVERY = 2` and `= 32` on real hardware post-fix (see
    // above), both fully deterministic (identical `min`/`max`/`avg` and
    // iteration counts every run) — no stall, no crash. Not proven safe
    // at *every* possible rate between 2 and 32 — 32 is simply the value
    // this session verified clean, not a value with a proven-safe
    // boundary below it.
    const BROADCAST_EVERY: u32 = 32;
    static BROADCAST_TICK: AtomicU32 = AtomicU32::new(0);
    if BROADCAST_TICK
        .fetch_add(1, Ordering::Relaxed)
        .is_multiple_of(BROADCAST_EVERY)
    {
        let me = rivet::port::arch::hart_id();
        for other in 0..rivet::config::MAX_HARTS {
            if other != me {
                rivet::port::arch::request_reschedule_on(other);
            }
        }
    }
}
