//! Rivet RTOS board support: ESP32-S3 real hardware (plan.md Phases 22-24).
//!
//! Implements the Group B (`rivet::port::board`) contract for a genuine
//! ESP32-S3 dev board — no QEMU model exists for Xtensa in this
//! environment, so unlike every other board in this workspace, nothing
//! here has ever run in a simulator; every piece is validated by flashing
//! and reading the real serial transcript.
//!
//! # Why no `esp-hal` dependency
//!
//! An earlier version of this crate depended on `esp-hal` for its
//! well-tested clock-tree and watchdog-disable bring-up (`esp_hal::
//! init()`). That does not coexist with Rivet owning its own preemptive-
//! tick interrupt: `esp-hal::interrupt::xtensa` defines
//! `__level_N_interrupt` for every level 1 through 7 as part of its own
//! generic peripheral-interrupt dispatch, and `xtensa-lx-rt` only allows
//! exactly one definition of each — confirmed as a hard `multiple
//! definition` link error at two different levels tried, not assumed.
//! Watchdog disable is instead hand-rolled directly against the `esp32s3`
//! PAC: the exact register sequence and magic unlock keys below were
//! cross-checked against `esp-hal`'s own source for correctness, without
//! linking `esp-hal` itself.
//!
//! # CPU clock: not yet measured
//!
//! [`CPU_HZ`] is a documented assumption (the boot ROM's XTAL-derived
//! default), not something this crate independently measures or
//! configures — tick *cadence* is therefore approximate for now, not
//! exact. Correctness of scheduling itself does not depend on this being
//! precise (every tick still fires and is handled), only real-world
//! timing accuracy does. Tracked as a follow-up once basic preemption is
//! proven on hardware.
//!
//! # This board's mask ROM needs a JTAG-assisted boot, every boot
//!
//! Confirmed live on the N16R8 devkit this session (chip revision v0.2),
//! reproduces after a full USB power cycle with no debugger ever
//! attached — this is a real, persistent property of this chip, not a
//! debugging artifact: on reset, before any flashed firmware (or even
//! the 2nd-stage bootloader) ever runs, Espressif's own mask ROM
//! `main()` reads the CPU's `PRID` special register, finds it equal to
//! `0xabab` (the value ROM treats as "running under a simulator"), and
//! spins forever in a tight loop waiting for an external debugger to
//! write the real boot entry point into a fixed hardware register
//! (`0x600c0004`) — normal SPI-flash boot is never attempted. Nothing
//! in this crate, `rivet-arch-xtensa`, or `rivet` itself can fix this:
//! it happens entirely inside ROM, before any of our code is reachable.
//! `scripts/esp32s3-jtag-unblock.sh` (repo root) automates the
//! workaround — reset, halt in the spin loop, write the ELF's real
//! entry point into that register, resume — and was verified to let
//! the flashed app run to completion (traced via JTAG all the way to
//! `__rivet_board_exit`). Needed on every boot, not just once.

#![no_std]

pub mod gpio;

// esp-idf's second-stage bootloader looks for this descriptor in the
// flashed image; without it, the bootloader will not recognize the image
// as a valid application and refuses to boot it. Content doesn't matter
// for a Rivet build, only presence.
esp_bootloader_esp_idf::esp_app_desc!();

/// Assumed CPU clock (plan.md Phase 22 — see module docs: not yet
/// measured/configured, the boot ROM's own default is used as-is).
const CPU_HZ: u32 = 40_000_000;

// ── Watchdog disable (hand-rolled against the `esp32s3` PAC) ────────
//
// The boot ROM leaves three watchdogs armed by default — unlike every
// QEMU board in this workspace, where nothing is enabled until Rivet asks
// for it. All three must be disabled (or Rivet must feed them fast enough
// to never matter, which is not attempted here) before `rivet::init()`
// can safely take as long as it likes.

const WDT_WKEY_UNLOCK: u32 = 0x50D8_3AA1;
const WDT_WKEY_LOCK: u32 = 0;
const SWD_WKEY_UNLOCK: u32 = 0x8F1D_312A; // ESP32-S3 specifically (not the C6/H2 key)
const SWD_WKEY_LOCK: u32 = 0;

pub fn disable_watchdogs() {
    // SAFETY: `RTC_CNTL`/`TIMG0` are the real, fixed-address register
    // blocks for this chip (`esp32s3` PAC); `Periph::ptr()` is always
    // valid to dereference — these are plain MMIO register accesses, not
    // exclusive-ownership Rust API misuse (nothing else has "taken" these
    // peripherals; this runs once, at the very start of board init).
    unsafe {
        let rtc_cntl = &*esp32s3::RTC_CNTL::ptr();

        // RTC main watchdog off.
        rtc_cntl
            .wdtwprotect()
            .write(|w| w.bits(WDT_WKEY_UNLOCK));
        rtc_cntl.wdtconfig0().write(|w| w.bits(0));
        rtc_cntl.wdtwprotect().write(|w| w.bits(WDT_WKEY_LOCK));

        // "Super WDT" off.
        rtc_cntl
            .swd_wprotect()
            .write(|w| w.swd_wkey().bits(SWD_WKEY_UNLOCK));
        rtc_cntl.swd_conf().write(|w| w.swd_auto_feed_en().set_bit());
        rtc_cntl
            .swd_wprotect()
            .write(|w| w.swd_wkey().bits(SWD_WKEY_LOCK));

        // TIMG0's own watchdog off.
        let timg0 = &*esp32s3::TIMG0::ptr();
        timg0.wdtwprotect().write(|w| w.wdt_wkey().bits(WDT_WKEY_UNLOCK));
        timg0.wdtconfig0().write(|w| w.bits(0));
        timg0.wdtwprotect().write(|w| w.wdt_wkey().bits(WDT_WKEY_LOCK));
    }
}

// Root cause found (plan.md Phase 22), via bisection on real hardware:
// reads from DROM (flash-cache-mapped `.rodata`) AND reads from UART0's
// own MMIO registers both faulted (`LoadStoreDataError`) immediately
// after the ESP-IDF bootloader hands off — while writes to the exact
// same addresses, and reads from plain SRAM, never did. Bisected by
// swapping `__pre_init` through five variants (naked-asm immediate-only
// writes; `entry`-only; a real fn with an unchecked FIFO write; a real
// fn with a single plain-SRAM read) — every write-only variant survived
// indefinitely, every variant containing a DROM or MMIO *read* faulted
// at the exact same point, and the plain-SRAM-read variant proved reads
// aren't broken in general. This matches `esp-hal`'s own
// `configure_cpu_caches()` (`esp-hal/src/soc/esp32s3/mod.rs`): the
// bootloader's flash cache configuration does not carry over to the
// app, and every real esp-hal app explicitly reconfigures/re-enables it
// via these same ROM helper functions before touching any `.rodata`.
// We were never doing that, so every DROM read was reading through a
// cache the app had never actually turned on.
unsafe extern "C" {
    fn rom_config_instruction_cache_mode(cache_size: u32, ways: u8, line_size: u8);
    fn rom_config_data_cache_mode(cache_size: u32, ways: u8, line_size: u8);
    fn Cache_Suspend_DCache();
    fn Cache_Resume_DCache(param: u32);
}

#[no_mangle]
extern "C" fn __pre_init() {
    // SAFETY: these are the real ESP32-S3 mask-ROM cache-configuration
    // helpers (addresses provided by `esp-rom-sys`'s linker fragments,
    // already a transitive dependency via `esp-bootloader-esp-idf`);
    // called once, before any `.rodata`/DROM access, matching exactly
    // what `esp-hal`'s own `configure_cpu_caches()` does: 32KB/8-way/
    // 32-byte-line for both instruction and data cache.
    unsafe {
        rom_config_instruction_cache_mode(0x8000, 8, 32);
        Cache_Suspend_DCache();
        rom_config_data_cache_mode(0x8000, 8, 32);
        Cache_Resume_DCache(0);
    }

    // The RTC/Super/TIMG0 watchdogs are disabled here too (in addition
    // to `__rivet_board_init`, reached much later via `rivet::init()`):
    // their default timeouts are short enough to fire during `Reset`'s
    // own bss-zero/data-copy and the rest of the boot path before
    // `main` is ever reached.
    disable_watchdogs();
}

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    disable_watchdogs();
    rivet_arch_xtensa::timer::configure(CPU_HZ);
    release_app_cpu();
}

// ── APP_CPU release (plan.md Phase 24) ──────────────────────────────

unsafe extern "C" {
    // `rivet-arch-xtensa`'s naked APP_CPU entry point (see its own docs)
    // — the raw address the boot ROM jumps to once released.
    fn rivet_appcpu_entry();
}

/// Release APP_CPU (core 1) from reset and hand it `rivet_appcpu_entry`
/// as its entry point — a no-op if this build is single-core-configured
/// (`RIVET_MAX_HARTS == 1`, the default), matching the RISC-V SMP
/// precedent's "only release harts within the configured ceiling"
/// discipline (plan.md Phase 19). Sequence cross-checked against
/// `esp-hal`'s own `cpu_control::start_core1` for this exact chip: set
/// the boot ROM's own "where does APP_CPU start" pointer
/// (`ets_set_appcpu_boot_addr`, a real ROM function, not board-invented),
/// enable APP_CPU's clock gate, release its runstall, then pulse its
/// reset bit (set then clear) so it actually restarts and picks up the
/// new boot address rather than continuing wherever it was stalled.
fn release_app_cpu() {
    if rivet::config::MAX_HARTS < 2 {
        return;
    }
    unsafe extern "C" {
        fn ets_set_appcpu_boot_addr(addr: u32);
    }
    // SAFETY: `ets_set_appcpu_boot_addr` is the real ESP32-S3 mask-ROM
    // function at its documented fixed address (via `esp-rom-sys`'s
    // linker fragments); `SYSTEM.core_1_control_0` is a real,
    // fixed-address SoC register. This runs exactly once, from
    // `__rivet_board_init`, before anything else could race it.
    unsafe {
        ets_set_appcpu_boot_addr(rivet_appcpu_entry as *const () as u32);
        let system = &*esp32s3::SYSTEM::ptr();
        system
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_clkgate_en().set_bit());
        system
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_runstall().clear_bit());
        system
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_reseting().set_bit());
        system
            .core_1_control_0()
            .modify(|_, w| w.control_core_1_reseting().clear_bit());
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    now_us()
}

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    rivet_arch_xtensa::timer::tick_start(hz);
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: `ptr`/`len` describe a valid `&[u8]` per the port contract.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    // Root cause (plan.md Phase 24), found on real dual-core hardware:
    // this had no cross-core synchronization at all — two cores writing
    // near-simultaneously produced real, repeatable byte-level
    // interleaving on the wire (confirmed directly, e.g. `"Rivet esp32s3
    // demo"` and `"APPCPU_ALIVE"` merging into
    // `"smApP_PtCePU_ALIVsEt"`). `critical::enter` is this workspace's
    // existing cross-hart lock (reentrant — safe even if a print
    // happens while already inside one, e.g. from a panic handler
    // already holding it) and is cheap to reuse here rather than
    // inventing a second lock just for the console.
    rivet::critical::enter(|| {
        // SAFETY: UART0's register block, fixed base per the `esp32s3`
        // PAC; already initialized by the boot ROM (it uses UART0 for
        // its own boot log, observed directly on this hardware — plan.md
        // Phase 20), so no baud/config setup is needed here, only
        // polling writes.
        let uart0 = unsafe { &*esp32s3::UART0::ptr() };
        for &b in bytes {
            // Real hardware TX FIFO is 128 bytes on the S3; spin until
            // there's room rather than dropping, matching every other
            // board's baseline polling console before any
            // interrupt-driven upgrade.
            while uart0.status().read().txfifo_cnt().bits() >= 124 {
                core::hint::spin_loop();
            }
            uart0.fifo().write(|w| unsafe { w.rxfifo_rd_byte().bits(b) });
        }
    });
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {
    // No interrupt-driven console yet (plan.md Phase 22 is polling-only,
    // matching every other board's own starting point) — nothing to do.
}

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    // `sw_sys_rst` resets the whole digital core, including the USB
    // Serial/JTAG peripheral console goes out over — confirmed on real
    // hardware (`watchdog_test`, plan.md Phase 25): without a drain
    // delay, "RIVET WATCHDOG TIMEOUT" (printed immediately before this
    // call) never reached the host at all, every run, because the USB-CDC
    // connection drops and the chip starts re-enumerating before the
    // still-buffered bytes are ever sent — unlike a real UART's FIFO,
    // there's no shift-register tail to simply "finish" once the
    // peripheral is gone. A short busy-wait here gives the USB stack time
    // to actually flush before the reset it's asked to report on erases
    // its own evidence.
    let deadline = now_us().wrapping_add(20_000);
    while now_us() < deadline {
        core::hint::spin_loop();
    }
    // SAFETY: `RTC_CNTL.options0().sw_sys_rst` is the documented software
    // system-reset trigger.
    unsafe {
        (&*esp32s3::RTC_CNTL::ptr())
            .options0()
            .write(|w| w.sw_sys_rst().set_bit());
    }
    loop {
        core::hint::spin_loop();
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    // Real hardware has no QEMU exit-device/process-exit-code channel —
    // whoever is driving the board reads this marker from the serial
    // transcript instead (see plan.md Phase 23's test harness).
    if code == 0 {
        rivet::console::write_str("RIVET_EXIT_OK\n");
    } else {
        rivet::console::write_str("RIVET_EXIT_FAIL code=");
        print_dec(code);
        rivet::console::write_str("\n");
    }
    loop {
        core::hint::spin_loop();
    }
}

fn print_dec(mut n: u32) {
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
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
    rivet_bsp_support::sw_watchdog::init(period_us, now_us());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    rivet_bsp_support::sw_watchdog::feed(now_us());
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {
    if rivet_bsp_support::sw_watchdog::expired(now_us()) {
        rivet::console::write_str("RIVET WATCHDOG TIMEOUT\n");
        rivet::port::board::reset();
    }
}

// ── Monotonic microseconds from CCOUNT ──────────────────────────────
//
// `CCOUNT` is only 32 bits (no wider hardware pair, unlike RISC-V's
// mtime), rolling over roughly every ~107s at 40MHz — far too soon to
// treat as "the" clock the way CLINT's real 64-bit `mtime` is on RISC-V.
// Widened here with a software-accumulated high half, following the
// established `static mut u64` + `critical::enter` discipline this
// workspace already uses wherever RV32/ARMv7-M lack native 64-bit
// atomics (`rivet-arch-riscv::clint`, `rivet::timer`'s deadline slots) —
// Xtensa has the identical constraint for a different reason (register
// width, not missing atomic instructions), same fix applies.

static mut LAST_CCOUNT: u32 = 0;
static mut CYCLES_HIGH: u64 = 0;

fn now_us() -> u64 {
    rivet::critical::enter(|| {
        // SAFETY: guarded by `critical::enter`, matching every other
        // `static mut` in this crate's position elsewhere in the
        // workspace.
        unsafe {
            let now = xtensa_lx::timer::get_cycle_count();
            if now < LAST_CCOUNT {
                CYCLES_HIGH += 1u64 << 32;
            }
            LAST_CCOUNT = now;
            let cycles = CYCLES_HIGH + now as u64;
            cycles / (CPU_HZ as u64 / 1_000_000)
        }
    })
}

/// Board-defined IRQ source numbers for `rivet::irq` (plan.md Phase 25):
/// the same split as every other board — **the controller is arch**
/// (`rivet-arch-xtensa`'s interrupt-matrix routing), **the number is
/// board** (which peripheral source a given interrupt actually is).
/// Matches `esp32s3::Interrupt`'s own discriminants directly.
pub mod irq {
    /// UART0's peripheral interrupt source (`esp32s3::Interrupt::UART0`).
    pub const UART0: u32 = 27;
}
