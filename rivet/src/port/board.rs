//! Group B of the port contract: the board port.
//!
//! Declares the symbols a `rivet-bsp-*` crate must provide — clock/board
//! bring-up, the monotonic clock, the tick source, the console, exit/
//! reset, and the watchdog. Everything here is a fact about a specific
//! board (memory map, clock rate, which UART); the kernel never embeds any
//! of it directly.

extern "Rust" {
    /// One-time board bring-up: clocks/PLL, console hardware init, any
    /// board state needed before [`__rivet_board_tick_start`] runs. Called
    /// once, after [`crate::port::arch::init`].
    fn __rivet_board_init();

    /// Current time in microseconds since boot. Must be monotonic and
    /// tear-free (the kernel calls this from both task and ISR context on
    /// multi-word-read architectures).
    fn __rivet_board_now_us() -> u64;

    /// Start the periodic tick at `hz` (the kernel's configured
    /// `RIVET_TICK_HZ`) and wire it to call back into the kernel's tick
    /// handlers on every period.
    fn __rivet_board_tick_start(hz: u32);

    /// Write raw bytes to the board's debug console.
    fn __rivet_board_console_write(ptr: *const u8, len: usize);

    /// Re-arm the console's TX-empty interrupt if it isn't already
    /// armed (plan.md Phase 14): called whenever [`crate::console`]
    /// pushes bytes into its TX ring, in case the board's TX ISR had
    /// already disabled itself after previously draining the ring empty.
    /// A no-op on a board that never called
    /// [`crate::console::enable_irq_tx`] (the console is still using the
    /// blocking polling path, so nothing needs kicking).
    fn __rivet_board_console_kick_tx();

    /// Trigger a system reset. Never returns.
    fn __rivet_board_reset() -> !;

    /// Terminate the program with `code` (0 = success). On real hardware
    /// this typically reduces to [`__rivet_board_reset`] or a halt; under
    /// QEMU it uses whatever exit device / semihosting path the board
    /// provides, giving the test harness a distinguishable status.
    fn __rivet_board_exit(code: u32) -> !;

    /// Arm the hardware (or best-effort software) watchdog for
    /// `period_us`. `period_us == 0` means "the watchdog was requested off"
    /// — implementations should treat that as a no-op / disable.
    fn __rivet_board_wdt_init(period_us: u32);

    /// Kick the watchdog, re-arming its deadline.
    fn __rivet_board_wdt_feed();

    /// Called from the kernel's tick handler every tick. Boards with a
    /// real hardware watchdog (it counts down autonomously) implement
    /// this as a no-op; boards with only a software watchdog (checking a
    /// deadline against the clock) do the expiry check and reset here.
    fn __rivet_board_wdt_check();
}

pub fn init() {
    // SAFETY: implemented by exactly one `rivet-bsp-*` crate linked into
    // the final binary; called once, after `port::arch::init`.
    unsafe { __rivet_board_init() }
}

pub fn now_us() -> u64 {
    // SAFETY: see `init`.
    unsafe { __rivet_board_now_us() }
}

pub fn tick_start(hz: u32) {
    // SAFETY: see `init`.
    unsafe { __rivet_board_tick_start(hz) }
}

pub fn console_write(bytes: &[u8]) {
    // SAFETY: forwarded to the board crate under the same contract.
    unsafe { __rivet_board_console_write(bytes.as_ptr(), bytes.len()) }
}

pub fn console_kick_tx() {
    // SAFETY: see `init`.
    unsafe { __rivet_board_console_kick_tx() }
}

pub fn reset() -> ! {
    // Drain anything still queued in the interrupt-driven console's TX
    // ring (plan.md Phase 14) before halting — see
    // `crate::console::flush_sync`'s docs for why this can't wait for the
    // interrupt that would normally do it.
    crate::console::flush_sync();
    // SAFETY: see `init`.
    unsafe { __rivet_board_reset() }
}

pub fn exit(code: u32) -> ! {
    crate::console::flush_sync();
    // SAFETY: see `init`.
    unsafe { __rivet_board_exit(code) }
}

pub fn wdt_init(period_us: u32) {
    // SAFETY: see `init`.
    unsafe { __rivet_board_wdt_init(period_us) }
}

pub fn wdt_feed() {
    // SAFETY: see `init`.
    unsafe { __rivet_board_wdt_feed() }
}

pub fn wdt_check() {
    // SAFETY: see `init`.
    unsafe { __rivet_board_wdt_check() }
}
