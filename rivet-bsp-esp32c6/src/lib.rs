//! Rivet RTOS board support: ESP32-C6 real hardware (plan.md Phase 26).
//!
//! # Status
//!
//! Confirmed working on real hardware: boot (through `espflash`'s
//! ESP-IDF app image format — see `link-esp32c6.ld`'s module docs),
//! watchdog disable, the monotonic clock (SYSTIMER), the polling
//! console, preemptive task spawning and dispatch, and — the hard part —
//! genuine tick-driven preemption between two same-priority tasks
//! (`preempt_test.rs`: reliable "ABABBABAB..." interleaving, both tasks
//! reaching completion, repeatable across independent flash/boot
//! cycles).
//!
//! Four real, hardware-confirmed bugs found getting there, all
//! root-caused (not guessed) via bisecting print statements or, once
//! prints stopped showing up at all, via bisecting hardware behavior
//! itself against `esp-hal`'s reference source:
//!
//! 1. This core's PMP doesn't behave like the privileged spec's reference
//!    shape `rivet-arch-riscv::pmp` was written against (proven correct
//!    on QEMU virt and MPS2-AN385) — probing its grain via the standard
//!    technique returned an implausible ~256 MiB minimum NAPOT region,
//!    not a real hardware property. Worked around, not fixed: hardware
//!    PMP guards are off entirely for this board (`pmp-guard` disabled on
//!    `rivet-arch-riscv`), falling back to the kernel's own software
//!    stack-watermark check alone — already proven sufficient by itself
//!    on the ESP32-S3, which has no PMP/MPU-equivalent hardware guard at
//!    all.
//! 2. `mcycle` (RV32's cycle-count CSR) hangs forever the instant it's
//!    read on this core — not the standard rollover-safe `mcycleh`
//!    double-read loop failing to converge (tried a 32-bit-only read
//!    too, still hung), the bare single-instruction `csrr` itself never
//!    completes, with no trap/panic ever printed. `rivet-arch-riscv`'s
//!    `no-mcycle` feature skips defining `__rivet_arch_cycle_count`
//!    entirely; this crate provides it instead, derived from SYSTIMER
//!    (already used for `now_us()`).
//! 3. This chip's interrupt matrix gates CPU-level delivery through
//!    `PLIC_MX` (0x2000_1000), a genuinely different peripheral from
//!    `INTPRI` (0x600c_5000) despite similar-looking field names in the
//!    `esp32c6` PAC — writing enable/priority/threshold to `INTPRI` was
//!    silently a no-op for delivery. See the "interrupt matrix" section
//!    below.
//! 4. Enabling one of this chip's custom per-line `mie` bits (12-31)
//!    while `mtvec` is in *direct* mode hard-locks the core immediately
//!    — no trap, no exception, nothing printable, every time. This
//!    chip's "plic" controller flavour requires *vectored* `mtvec`
//!    (confirmed against `esp-hal`'s own `_setup_interrupts`, which
//!    always installs one before touching these lines) —
//!    `rivet-arch-riscv`'s new `vectored-trap` feature. Getting the
//!    resulting image to actually flash uncovered a fifth, tooling-level
//!    issue: see `rivet-arch-riscv`'s own vector-table doc comment for
//!    the `espflash` alignment-padding quirk that cost the most time of
//!    anything in this list.
//!
//! Full S3-equivalent test suite now passing on real hardware:
//! `demo`, `preempt_test`, `sleep_test`, `mutex_test`, `stress_spawn`,
//! `join_test`, `respawn_test`, `stress_max_ptasks`, `soak_smoke`,
//! `report_test`, `deadline_test`, `watchdog_test` (genuine LP_WDT
//! reset — `rst:0x10 LP_WDT_SYS` — after the expected marker),
//! `fault_overflow`/`fault_isolate` (via the software watermark check;
//! PMP guards are off, see bug #1), `irq_test` (a real peripheral
//! interrupt — UART0 `tx_done` — routed through `rivet::irq`, not a
//! software-only stand-in; see the "generic peripheral IRQ" section
//! below), `embedded_hal_test` (`rivet-bsp-support`'s `RivetDelay`/
//! `Serial` are entirely board-agnostic, so this needed no new code at
//! all). `smp_test` doesn't apply — single-core chip.
//!
//! # The interrupt matrix + PLIC_MX (plan.md Phase 26 follow-up)
//!
//! Same two-part design as the ESP32-S3's own interrupt matrix
//! (`rivet-arch-xtensa`'s `periph_irq`/`ipi` modules), just RISC-V-side
//! register names: `INTERRUPT_CORE0.core_0_intr_map(source)` routes a
//! peripheral interrupt *source* (e.g. `SYSTIMER_TARGET0` = 57,
//! `FROM_CPU_INTR0` = 22 — confirmed against this PAC version's own
//! `Interrupt` enum) to one of 32 CPU interrupt *lines*; `PLIC_MX`
//! (0x2000_1000 — a *different* peripheral from `INTPRI`, despite
//! `esp32c6`'s SVD using similar-looking names for both; confirmed by
//! reading `esp-hal`'s own `interrupt/riscv/plic.rs` driver, which
//! esp-metadata's `soc.toml` says is the controller flavour this chip
//! actually uses) enables/prioritizes each line via its
//! `mxint_enable`/`_pri`/`_thresh`/`_clear` registers. `INTPRI` still
//! has one real job: its `cpu_intr_from_cpu` register is the software
//! self-trigger ("write 1 to raise IPI to self") used for the
//! reschedule line — a genuinely separate mechanism from the delivery
//! gate above. `rivet-arch-riscv`'s trap handler doesn't know any of
//! this — dispatch reaches this crate via its `board-irq-hook` feature,
//! an escape hatch exactly for controllers that are real RISC-V
//! ISA-adjacent hardware but not portable across vendors the way
//! CLINT/PLIC are. Both PLIC_MX-enabled lines also need their `mie` bit
//! set directly at the core (raw CSR access — bits 12-31 aren't in the
//! `riscv` crate's typed API) — see bug #4 above for why that requires
//! `rivet-arch-riscv/vectored-trap`.
//!
//! CPU lines 16 (tick) and 17 (reschedule) were picked arbitrarily from
//! the "platform-defined" range (avoiding 3/7/11, which carry standard
//! RISC-V meaning even though this core has no CLINT wired to them) —
//! not verified against any documented reservation, just chosen to be
//! unlikely to collide with anything.
//!
//! # Generic peripheral IRQ (`rivet::irq`)
//!
//! Lines 18-23 are a small pool reserved for whatever peripherals an app
//! registers via `rivet::irq::register`/`enable` — a board-owned
//! `[AtomicU32; 6]` (`IRQ_LINE_SOURCE`) hands out one on first use and
//! remembers the mapping so a later `enable` for the same id reuses it.
//! `rivet::irq::register`/`dispatch` both index a fixed
//! `[T; RIVET_MAX_IRQS]` table in `rivet` itself (default 32) directly
//! by the `u32` the caller passes — this chip's own peripheral source
//! numbers run well past that (`UART0` = 43), so `irq::UART0` etc. are
//! small *logical* ids instead of raw source numbers, translated to the
//! real source only where `INTERRUPT_CORE0` routing actually needs it
//! (`LOGICAL_TO_SOURCE`). `rivet-arch-riscv`'s `board-irq-hook` feature
//! grew three more extern hooks for this (`__rivet_board_irq_{enable,
//! disable,set_priority}`), the same split as the tick/resched dispatch
//! hook: the mechanism (`rivet::irq`'s table, `__rivet_arch_irq_*`'s
//! call-through) is arch-generic, which peripheral is which id and how
//! it's wired to a CPU line is this board's own business.
//!
//! # Why no `esp-hal` dependency
//!
//! Same reasoning as `rivet-bsp-esp32s3`: `esp-hal`'s own interrupt/timer
//! runtime would fight Rivet for ownership of the same hardware. Watchdog
//! disable and the boot-time cache/clock state are hand-rolled directly
//! against the `esp32c6` PAC, cross-checked against esp-hal's own source
//! for correctness (register names, unlock keys, bit meanings) without
//! linking esp-hal itself.

#![no_std]

// Forces `rivet-arch-riscv`'s object code (its `#[no_mangle]` Group A
// symbols — context switch, trap entry, PMP) to actually be linked in.
// Cargo compiles every listed dependency regardless, but the *linker*
// only pulls a crate's code from its `.rlib` when something in the
// dependency graph syntactically references it — with `clint`/`plic`
// both off, nothing in this crate calls into `rivet-arch-riscv` yet
// (confirmed: omitting this line reproduces a full set of undefined-
// symbol errors for every Group A function, from `__rivet_arch_init`
// down, even though the crate is a real `Cargo.toml` dependency).
use rivet_arch_riscv as _;

esp_bootloader_esp_idf::esp_app_desc!();

/// Assumed CPU clock (boot ROM default, not yet measured/configured) —
/// same documented-approximation status as the S3 board's own `CPU_HZ`.
const CPU_HZ: u32 = 80_000_000;

// ── Watchdog disable ─────────────────────────────────────────────────
//
// Espressif chips leave watchdogs armed by default at boot; QEMU boards
// in this workspace need none of this (nothing is enabled until Rivet
// asks). The C6 has three: LP_WDT's own "RWDT" (main RTC watchdog),
// LP_WDT's "SWD" (super watchdog, a fast independent backstop), and
// TIMG0's own MWDT — same three-watchdog shape as the S3, different
// register grouping (the C6's newer "LP" low-power-domain design folds
// RWDT+SWD into one LP_WDT block instead of the S3's RTC_CNTL).

const WDT_WKEY_UNLOCK: u32 = 0x50D8_3AA1;
const WDT_WKEY_LOCK: u32 = 0;
// Super watchdog unlock key: confirmed against esp-hal's own
// `rtc_cntl::Swd::set_write_protection` — this key is shared by every
// chip *except* ESP32-C2/C3/S2/S3 (which use `0x8F1D_312A` instead, per
// that same function; the S3 board's own code uses that other key).
const SWD_WKEY_UNLOCK: u32 = 0x50D8_3AA1;
const SWD_WKEY_LOCK: u32 = 0;

pub fn disable_watchdogs() {
    // SAFETY: `LP_WDT`/`TIMG0` are the real, fixed-address register
    // blocks for this chip (`esp32c6` PAC); plain MMIO register access,
    // nothing else has "taken" these peripherals — runs once, at the
    // very start of board init.
    unsafe {
        let lp_wdt = &*esp32c6::LP_WDT::ptr();

        // RTC main watchdog off.
        lp_wdt.wdtwprotect().write(|w| w.bits(WDT_WKEY_UNLOCK));
        lp_wdt.wdtconfig0().write(|w| w.bits(0));
        lp_wdt.wdtwprotect().write(|w| w.bits(WDT_WKEY_LOCK));

        // Super watchdog off (auto-feed so it never actually fires).
        lp_wdt.swd_wprotect().write(|w| w.swd_wkey().bits(SWD_WKEY_UNLOCK));
        lp_wdt.swd_conf().write(|w| w.swd_auto_feed_en().set_bit());
        lp_wdt.swd_wprotect().write(|w| w.swd_wkey().bits(SWD_WKEY_LOCK));

        // TIMG0's own watchdog off.
        let timg0 = &*esp32c6::TIMG0::ptr();
        timg0.wdtwprotect().write(|w| w.wdt_wkey().bits(WDT_WKEY_UNLOCK));
        timg0.wdtconfig0().write(|w| w.bits(0));
        timg0.wdtwprotect().write(|w| w.wdt_wkey().bits(WDT_WKEY_LOCK));
    }
}

// ── Console (UART0, polling) ─────────────────────────────────────────
//
// Same register layout as the S3 board's own UART0 driver (`fifo`/
// `status().txfifo_cnt()`) — confirmed against the `esp32c6` PAC
// directly (not assumed identical just because both are Espressif
// chips): field names match exactly.

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    disable_watchdogs();
    // SAFETY: real, fixed-address SoC register; `rivet::init()` (the only
    // caller) runs once, before anything else touches the timer.
    unsafe {
        systimer_init();
    }
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port contract.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    rivet::critical::enter(|| {
        // SAFETY: UART0's register block, fixed base; already initialized
        // by the boot ROM (it uses UART0 for its own boot log), so no
        // baud/config setup is needed here, only polling writes.
        let uart0 = unsafe { &*esp32c6::UART0::ptr() };
        for &b in bytes {
            // Real hardware TX FIFO is 128 bytes; spin until there's room
            // rather than dropping, matching every other board's baseline
            // polling console before any interrupt-driven upgrade.
            while uart0.status().read().txfifo_cnt().bits() >= 124 {
                core::hint::spin_loop();
            }
            uart0.fifo().write(|w| unsafe { w.rxfifo_rd_byte().bits(b) });
        }
    });
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {
    // No interrupt-driven console yet (polling-only, matching every other
    // board's own starting point) — nothing to do.
}

// ── Monotonic clock (SYSTIMER unit 0) ────────────────────────────────
//
// Register sequence (enable unit0 counting, then update+wait-valid+read
// lo/hi to sample it) cross-checked against esp-hal's own
// `timer::systimer` driver — not guessed from the PAC's field names
// alone.

unsafe fn systimer_init() {
    // SAFETY: real, fixed-address SoC register; called once from
    // `__rivet_board_init`.
    unsafe {
        let systimer = &*esp32c6::SYSTIMER::ptr();
        systimer.conf().modify(|_, w| w.timer_unit0_work_en().set_bit());
    }
}

/// Raw SYSTIMER unit-0 tick count (16MHz — see `__rivet_board_now_us`'s
/// own doc for the same not-yet-independently-measured caveat as
/// `CPU_HZ`). Shared by [`__rivet_board_now_us`] (divided down to
/// microseconds) and [`__rivet_arch_cycle_count`] (used directly, since
/// `rivet::exec_time`/latency callers only ever take deltas — a
/// documented-monotonic-source substitute for `mcycle`, which hangs
/// forever on this core the instant it's read; see
/// `rivet-arch-riscv`'s `no-mcycle` feature docs).
fn systimer_ticks() -> u64 {
    // SAFETY: real, fixed-address SoC register; read-only sampling.
    unsafe {
        let systimer = &*esp32c6::SYSTIMER::ptr();
        systimer.unit_op(0).write(|w| w.update().set_bit());
        while !systimer.unit_op(0).read().value_valid().bit_is_set() {}
        let unit_value = systimer.unit_value(0);
        let mut lo_prev = unit_value.lo().read().bits();
        loop {
            let lo = lo_prev;
            let hi = unit_value.hi().read().bits();
            lo_prev = unit_value.lo().read().bits();
            if lo == lo_prev {
                break ((hi as u64) << 32) | lo as u64;
            }
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    // SYSTIMER runs at 16MHz (boot ROM default divider) — confirmed
    // against esp-hal's own `SystemTimer::ticks_per_second()` default
    // path, not measured independently on this board yet (same
    // documented-approximation status as `CPU_HZ`).
    systimer_ticks() / 16
}

#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    systimer_ticks()
}

// ── Reset ─────────────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    // No dedicated "software reset" bit found in the C6's LP-domain
    // register set via static analysis (unlike the S3's
    // `RTC_CNTL.options0().sw_sys_rst`) — using the main RTC watchdog
    // with the shortest possible timeout instead: a well-documented,
    // universally-available reset mechanism on every Espressif chip, and
    // one this board already needs a correct register sequence for
    // anyway (`rivet::watchdog`). Sequence (stage-0 timeout + `wdt_stg0`
    // action + `wdt_en`) cross-checked against esp-hal's own
    // `Rwdt::set_enabled`/`set_timeout`, not guessed. Give the console a
    // moment to drain first — the S3 board learned this the hard way
    // (its own `reset` doc comment): a reset that also power-cycles the
    // console peripheral can lose whatever was just printed if triggered
    // immediately after.
    let deadline = __rivet_board_now_us().wrapping_add(20_000);
    while __rivet_board_now_us() < deadline {
        core::hint::spin_loop();
    }
    // SAFETY: real, fixed-address SoC register; arming the shortest
    // watchdog timeout to force a reset is the intended, documented use
    // of this register, not a misuse.
    unsafe {
        let lp_wdt = &*esp32c6::LP_WDT::ptr();
        lp_wdt.wdtwprotect().write(|w| w.bits(WDT_WKEY_UNLOCK));
        // Stage 0's timeout register (`wdtconfig(0)` — distinct from the
        // control register `wdtconfig0`): shortest nonzero hold value.
        lp_wdt.wdtconfig(0).write(|w| w.hold().bits(1));
        lp_wdt.wdtconfig0().write(|w| {
            w.wdt_pause_in_slp().set_bit();
            w.wdt_cpu_reset_length().bits(7);
            w.wdt_sys_reset_length().bits(7);
            w.wdt_stg0().bits(4); // RwdtStageAction::ResetSystem
            w.wdt_en().set_bit()
        });
        lp_wdt.wdtwprotect().write(|w| w.bits(WDT_WKEY_LOCK));
    }
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    // Real hardware has no QEMU exit-device/process-exit-code channel —
    // print a distinguishable marker (the xtask/manual-test harness
    // greps for this) and reset, matching the S3 board's own convention.
    if code == 0 {
        rivet::console::write_str("RIVET_EXIT_OK\n");
    } else {
        rivet::console::write_str("RIVET_EXIT_FAIL code=");
        print_dec(code as usize);
        rivet::console::write_str("\n");
    }
    rivet::console::flush_sync();
    let deadline = __rivet_board_now_us().wrapping_add(20_000);
    while __rivet_board_now_us() < deadline {
        core::hint::spin_loop();
    }
    loop {
        core::hint::spin_loop();
    }
}

fn print_dec(mut n: usize) {
    if n == 0 {
        rivet::console::write_str("0");
        return;
    }
    let mut digits = [0u8; 10];
    let mut i = 0;
    while n > 0 {
        digits[i] = b'0' + (n % 10) as u8;
        n /= 10;
        i += 1;
    }
    let mut buf = [0u8; 10];
    for j in 0..i {
        buf[j] = digits[i - 1 - j];
    }
    if let Ok(s) = core::str::from_utf8(&buf[..i]) {
        rivet::console::write_str(s);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_cpu_hz() -> u32 {
    CPU_HZ
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
    rivet_bsp_support::sw_watchdog::init(period_us, __rivet_board_now_us());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    rivet_bsp_support::sw_watchdog::feed(__rivet_board_now_us());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {
    if rivet_bsp_support::sw_watchdog::expired(__rivet_board_now_us()) {
        rivet::console::write_str("RIVET WATCHDOG TIMEOUT\n");
        rivet::port::board::reset();
    }
}

// ── Tick / reschedule (interrupt matrix + PLIC_MX) ────────────────────
//
// See the module docs' own "interrupt matrix + PLIC_MX" section for the
// full picture. Peripheral *source* numbers, confirmed against this PAC
// version's own `esp32c6::Interrupt` enum:

/// `FROM_CPU_INTR0` — self-triggerable, used for the reschedule IPI.
const SOURCE_FROM_CPU_INTR0: usize = 22;
/// `SYSTIMER_TARGET0` — the periodic tick.
const SOURCE_SYSTIMER_TARGET0: usize = 57;

/// CPU interrupt lines (0-31) these two sources are routed to — see the
/// module docs for why these two specific numbers.
const LINE_TICK: u32 = 16;
const LINE_RESCHED: u32 = 17;

/// Target tick period in SYSTIMER ticks (16MHz — see `systimer_ticks`'s
/// own doc), set by `__rivet_board_tick_start` and consumed once by
/// `configure_interrupt_matrix` (called lazily, from whichever of
/// `tick_start`/`request_reschedule` runs first).
static TICK_PERIOD_TICKS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static MATRIX_CONFIGURED: core::sync::atomic::AtomicBool =
    core::sync::atomic::AtomicBool::new(false);

/// Route both sources to their CPU lines and enable+prioritize those
/// lines at INTPRI. Idempotent — safe to call from both
/// `__rivet_board_tick_start` and `__rivet_arch_request_reschedule[_on]`
/// without caring which runs first (`rivet::init()`'s own sequence calls
/// `tick_start` before `run()` ever could reach a reschedule call, but
/// nothing enforces that at this layer, so this doesn't assume it).
fn configure_interrupt_matrix() {
    if MATRIX_CONFIGURED.swap(true, core::sync::atomic::Ordering::AcqRel) {
        return;
    }
    // SAFETY: real, fixed-address SoC register blocks; this runs at most
    // once (guarded by `MATRIX_CONFIGURED` above), before either line can
    // actually fire (INTPRI's own per-line enable, set last, is what
    // actually arms them).
    unsafe {
        // Defensively ack both sources' pending state before anything
        // downstream can unmask them — the boot ROM/2nd-stage bootloader
        // is not guaranteed to have left either at rest (confirmed on
        // hardware to matter: enabling the CPU-level path before this
        // led to a reboot at an inconsistent point during this very
        // function, symptomatic of a trap being taken on stale pending
        // state rather than a freshly-armed one).
        let systimer = &*esp32c6::SYSTIMER::ptr();
        systimer.int_clr().write(|w| w.target0().clear_bit_by_one());
        let intpri_ack = &*esp32c6::INTPRI::ptr();
        intpri_ack.cpu_intr_from_cpu(0).write(|w| w.cpu_intr().clear_bit());

        let core0 = &*esp32c6::INTERRUPT_CORE0::ptr();
        core0
            .core_0_intr_map(SOURCE_SYSTIMER_TARGET0)
            .write(|w| w.bits(LINE_TICK));
        core0
            .core_0_intr_map(SOURCE_FROM_CPU_INTR0)
            .write(|w| w.bits(LINE_RESCHED));

        // `INTPRI` (0x600c_5000) is NOT the register block that actually
        // gates CPU interrupt delivery on this chip — that's a different
        // peripheral entirely, `PLIC_MX` at 0x2000_1000 (confirmed by
        // reading esp-hal's own `interrupt/riscv/plic.rs` driver, which
        // esp-metadata's `soc.toml` says is the controller flavour this
        // chip actually uses). Writing enable/priority/threshold to
        // INTPRI was silently a no-op as far as the core's actual
        // interrupt-delivery hardware is concerned. `INTPRI.
        // cpu_intr_from_cpu` is a separate, genuinely-INTPRI mechanism
        // (the self-trigger "write" side of the FROM_CPU_INTR0..3
        // sources) and stays as-is above — only the enable/priority/
        // threshold gate moves.
        let plic_mx = &*esp32c6::PLIC_MX::ptr();
        // Level-triggered (type bit clear): both sources are acked at
        // the peripheral (SYSTIMER's own int_clr, INTPRI's own
        // cpu_intr_from_cpu clear), matching every other Espressif
        // peripheral-interrupt source in this workspace (S3's IPI/
        // peripheral IRQ included) — not edge.
        plic_mx.mxint_pri(LINE_TICK as usize).write(|w| w.cpu_mxint_pri().bits(4));
        plic_mx.mxint_pri(LINE_RESCHED as usize).write(|w| w.cpu_mxint_pri().bits(4));
        plic_mx.mxint_thresh().write(|w| w.cpu_mxint_thresh().bits(0));
        plic_mx.mxint_enable().modify(|r, w| {
            w.cpu_mxint_enable().bits(r.cpu_mxint_enable().bits() | (1 << LINE_TICK) | (1 << LINE_RESCHED))
        });

        // Unmask both lines at the core itself. These are this chip's
        // *custom* per-line `mie` bits (12-31), not the standard 3 —
        // `rivet-arch-riscv`'s `vectored-trap` feature (required by this
        // board, see its own docs) is what makes this safe: confirmed on
        // hardware that setting either of these bits while `mtvec` was
        // in direct mode hard-locked the core immediately, every time,
        // with no trap, no exception, nothing printable, matching
        // exactly how esp-hal's own driver for this controller flavour
        // always installs a vectored `mtvec` before touching them. Not
        // reachable through the `riscv` crate's typed API (only the 3
        // standard bits are) — raw CSR access.
        core::arch::asm!(
            "csrs mie, {0}",
            in(reg) (1u32 << LINE_TICK) | (1u32 << LINE_RESCHED),
        );
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    configure_interrupt_matrix();
    let period_ticks = (16_000_000u32 / hz).max(1);
    TICK_PERIOD_TICKS.store(period_ticks, core::sync::atomic::Ordering::Relaxed);
    // SAFETY: real, fixed-address SoC register; periodic-mode target
    // register sequence cross-checked against esp-hal's own
    // `timer::systimer::set_period` — the `comp_load` "commit" write
    // after `target_conf` matters: without it, on real hardware, the
    // target fires exactly once and never re-arms (confirmed directly —
    // `preempt_test.rs`'s task A ran to completion alone, task B never
    // dispatched at all, meaning `on_tick` only ever ran for the single
    // initial firing, not periodically).
    unsafe {
        let systimer = &*esp32c6::SYSTIMER::ptr();
        systimer.target_conf(0).write(|w| {
            w.period().bits(period_ticks);
            w.period_mode().set_bit();
            w.timer_unit_sel().clear_bit()
        });
        systimer.comp_load(0).write(|w| w.load().set_bit());
        systimer.conf().modify(|_, w| w.target0_work_en().set_bit());
        systimer.int_ena().modify(|_, w| w.target0().set_bit());
    }
}

/// Called from `rivet-arch-riscv`'s trap handler (`board-irq-hook`
/// feature) for any async interrupt it doesn't already own (`clint`/
/// `plic`, both off for this board — everything reaches here). `code` is
/// the CPU interrupt line number directly (`mcause`'s own encoding —
/// vectored mode doesn't change this, see `vectored-trap`'s own docs),
/// so no separate pending-bitmap lookup is needed.
#[no_mangle]
extern "Rust" fn __rivet_board_on_async_trap(code: usize, resume_sp: usize) -> usize {
    if code == LINE_TICK as usize {
        // SAFETY: real, fixed-address SoC register; acking at the
        // peripheral (periodic mode re-arms itself — no target-register
        // rewrite needed, unlike a one-shot target).
        unsafe {
            let systimer = &*esp32c6::SYSTIMER::ptr();
            systimer.int_clr().write(|w| w.target0().clear_bit_by_one());
        }
        rivet::timer::poll_timers(__rivet_board_now_us());
        // Matches every other tick source in this workspace (`clint`'s
        // own tick handler, `rivet-arch-xtensa`'s for the S3) — the
        // software watchdog's own deadline check happens here, not as
        // part of `on_tick`'s own scheduling decision.
        rivet::watchdog::on_tick();
        return rivet::preempt::on_tick(resume_sp);
    }
    if code == LINE_RESCHED as usize {
        // SAFETY: real, fixed-address SoC register; clearing the same
        // bit `request_reschedule[_on]` sets acknowledges it.
        unsafe {
            let intpri = &*esp32c6::INTPRI::ptr();
            intpri.cpu_intr_from_cpu(0).write(|w| w.cpu_intr().clear_bit());
        }
        return rivet::preempt::on_tick(resume_sp);
    }
    if (FIRST_IRQ_LINE as usize..FIRST_IRQ_LINE as usize + N_IRQ_LINES).contains(&code) {
        let source = IRQ_LINE_SOURCE[code - FIRST_IRQ_LINE as usize]
            .load(core::sync::atomic::Ordering::Acquire);
        // `!= u32::MAX`: a still-enabled `mie` bit for a line whose
        // source was never actually assigned would mean a genuine
        // bookkeeping bug elsewhere, not something to dispatch on.
        if source != u32::MAX {
            rivet::irq::dispatch(source);
        }
    }
    resume_sp
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    configure_interrupt_matrix();
    // SAFETY: real, fixed-address SoC register; `cpu_intr_from_cpu(0)` is
    // this core's own self-trigger source (single-core board — "the
    // other hart" doesn't exist, so `_on` below is always this same
    // path).
    unsafe {
        let intpri = &*esp32c6::INTPRI::ptr();
        intpri.cpu_intr_from_cpu(0).write(|w| w.cpu_intr().set_bit());
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule_on(_hart: usize) {
    // Single-core board: `_hart` is always this same core (`rivet::config
    // ::MAX_HARTS == 1`), so this is just the self-IPI above.
    __rivet_arch_request_reschedule();
}

// ── Generic peripheral IRQ (`rivet::irq`, plan.md Phase 26 follow-up) ─
//
// `rivet::irq::register`/`enable`/`dispatch` all key on the *peripheral
// source* number (matching `rivet-arch-xtensa`'s own `periph_irq`
// module for the S3) — this board owns the source-to-CPU-line mapping
// entirely itself (`rivet-arch-riscv` only knows to call these three
// functions, via `board-irq-hook`; see their own doc comments there).
//
// A small fixed pool of CPU lines, disjoint from `LINE_TICK`/
// `LINE_RESCHED`: plenty for the peripherals a bare-metal RTOS app
// realistically registers by hand (this isn't a general driver
// framework with dozens of concurrent sources).
const FIRST_IRQ_LINE: u32 = 18;
const N_IRQ_LINES: usize = 6;
/// `IRQ_LINE_SOURCE[i]` holds the peripheral source currently routed to
/// line `FIRST_IRQ_LINE + i`, or `u32::MAX` if that slot is free.
static IRQ_LINE_SOURCE: [core::sync::atomic::AtomicU32; N_IRQ_LINES] =
    [const { core::sync::atomic::AtomicU32::new(u32::MAX) }; N_IRQ_LINES];

/// `rivet::irq::register`/`enable`/`dispatch` all key on a plain `u32`
/// that's also used to index a fixed-size `[T; RIVET_MAX_IRQS]` table in
/// `rivet` itself (default 32) — this chip's own peripheral interrupt
/// *source* numbers run past that (`UART0` = 43), so `irq::UART0` below
/// is a small board-local *logical* id instead, translated to the real
/// source number only at the two points that actually need it
/// (`INTERRUPT_CORE0`'s routing register). Extend this table, in the
/// same order as `irq`'s constants, when a peripheral beyond UART0 is
/// wired up.
const LOGICAL_TO_SOURCE: [usize; 1] = [43 /* UART0 */];

/// Find `irq_id`'s already-assigned line, or claim a free slot for it.
/// `None` only if every slot is already taken by some *other* id — not
/// expected in practice given the pool size above.
fn irq_line_for(irq_id: u32) -> Option<u32> {
    for (i, slot) in IRQ_LINE_SOURCE.iter().enumerate() {
        if slot.load(core::sync::atomic::Ordering::Acquire) == irq_id {
            return Some(FIRST_IRQ_LINE + i as u32);
        }
    }
    for (i, slot) in IRQ_LINE_SOURCE.iter().enumerate() {
        if slot
            .compare_exchange(
                u32::MAX,
                irq_id,
                core::sync::atomic::Ordering::AcqRel,
                core::sync::atomic::Ordering::Acquire,
            )
            .is_ok()
        {
            return Some(FIRST_IRQ_LINE + i as u32);
        }
    }
    None
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_enable(irq_id: u32) {
    configure_interrupt_matrix();
    let Some(line) = irq_line_for(irq_id) else {
        return;
    };
    let Some(&source) = LOGICAL_TO_SOURCE.get(irq_id as usize) else {
        return;
    };
    // SAFETY: real, fixed-address SoC register blocks; same sequence as
    // `configure_interrupt_matrix`'s own tick/resched setup, just for a
    // caller-chosen source/line pair instead of the two fixed ones.
    unsafe {
        (&*esp32c6::INTERRUPT_CORE0::ptr())
            .core_0_intr_map(source)
            .write(|w| w.bits(line));
        let plic_mx = &*esp32c6::PLIC_MX::ptr();
        plic_mx.mxint_pri(line as usize).write(|w| w.cpu_mxint_pri().bits(4));
        plic_mx.mxint_enable().modify(|r, w| {
            w.cpu_mxint_enable().bits(r.cpu_mxint_enable().bits() | (1 << line))
        });
        // See `configure_interrupt_matrix`'s own doc for why this needs
        // `vectored-trap` to be safe on this core.
        core::arch::asm!("csrs mie, {0}", in(reg) 1u32 << line);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_disable(irq_id: u32) {
    let Some(line) = irq_line_for(irq_id) else {
        return;
    };
    // SAFETY: real, fixed-address SoC register; only clearing the gate
    // this same id's `enable` call set — the id-to-line mapping and the
    // slot in `IRQ_LINE_SOURCE` both stay intact, so a later `enable`
    // for the same id reuses the same line without a fresh allocation.
    unsafe {
        (&*esp32c6::PLIC_MX::ptr()).mxint_enable().modify(|r, w| {
            w.cpu_mxint_enable().bits(r.cpu_mxint_enable().bits() & !(1 << line))
        });
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_set_priority(irq_id: u32, priority: u8) {
    let Some(line) = irq_line_for(irq_id) else {
        return;
    };
    // PLIC_MX's priority field is 4 bits (0-15); 0 means "never fires"
    // in standard PLIC semantics, so floor the caller's 0-255 range at 1
    // rather than passing an accidental 0 straight through.
    let pri = ((priority as u32) & 0xF).max(1);
    // SAFETY: real, fixed-address SoC register.
    unsafe {
        (&*esp32c6::PLIC_MX::ptr())
            .mxint_pri(line as usize)
            .write(|w| w.cpu_mxint_pri().bits(pri as u8));
    }
}

/// Board-specific peripheral interrupt ids for `rivet::irq::register`/
/// `enable`/`disable`/`set_priority` — small logical indices, *not* this
/// chip's own raw interrupt-matrix source numbers (see
/// `LOGICAL_TO_SOURCE`'s own doc for why).
pub mod irq {
    pub const UART0: u32 = 0;
}
