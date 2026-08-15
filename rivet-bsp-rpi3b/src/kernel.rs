//! Group B of the port contract: the board port for the Pi 3B.
//!
//! Clock, tick, console, reset and watchdog, plus the interrupt-controller
//! hooks `rivet-arch-aarch64` forwards to. See that crate's module docs
//! for why the controller lives here rather than there: BCM2837 has no
//! usable GIC, so there is nothing architectural to drive.
//!
//! Two independent timers are in play, which is worth keeping straight:
//!
//! - The **System Timer** at `0x3F003000` is a free-running 1 MHz counter,
//!   used for `now_us`. One microsecond per count means no scaling at all.
//! - The **architected generic timer** (`CNTP_*_EL0`, 19.2 MHz here)
//!   drives the periodic tick, routed to the core through the ARM-local
//!   block at `0x4000_0000`. It is per-core, which is what the eventual
//!   multi-core work wants, and it needs no MMIO to read.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use crate::mmio::PERIPHERAL_BASE;
use crate::Pl011;

/// Free-running 1 MHz counter.
const SYSTEM_TIMER_BASE: usize = PERIPHERAL_BASE + 0x0000_3000;
const ST_CLO: usize = 0x04;
const ST_CHI: usize = 0x08;

/// Broadcom's own interrupt controller. Not a GIC.
const IRQ_BASE: usize = PERIPHERAL_BASE + 0x0000_B200;
const ENABLE_IRQS_1: usize = 0x10;
const ENABLE_IRQS_2: usize = 0x14;
const DISABLE_IRQS_1: usize = 0x1C;
const DISABLE_IRQS_2: usize = 0x20;

/// Per-core timers and mailboxes. Outside the peripheral window.
const ARM_LOCAL_BASE: usize = 0x4000_0000;
/// Timer interrupt control and pending-source registers are banked per
/// core, four bytes apart. Reaching for the core 0 instances from
/// whichever core happens to be running is the obvious mistake here, and
/// it presents as a tick that simply never fires.
const CORE_TIMER_IRQCNTL_BASE: usize = 0x40;
const CORE_IRQ_SOURCE_BASE: usize = 0x60;

fn core_timer_irqcntl() -> usize {
    ARM_LOCAL_BASE + CORE_TIMER_IRQCNTL_BASE + crate::smp::current_core() * 4
}

fn core_irq_source() -> usize {
    ARM_LOCAL_BASE + CORE_IRQ_SOURCE_BASE + crate::smp::current_core() * 4
}
/// Non-secure physical timer, routed as IRQ rather than FIQ.
const TIMER_IRQ_CNTPNSIRQ: u32 = 1 << 1;

/// Power management block, which owns the watchdog and the reset path.
const PM_BASE: usize = PERIPHERAL_BASE + 0x0010_0000;
const PM_RSTC: usize = 0x1C;
const PM_WDOG: usize = 0x24;
const PM_PASSWORD: u32 = 0x5A00_0000;
const PM_RSTC_WRCFG_FULL_RESET: u32 = 0x20;
const PM_RSTC_WRCFG_CLR: u32 = 0xFFFF_FFCF;

/// Ticks elapsed since `tick_start`, and the period it was started with.
static TICK_PERIOD_US: AtomicU32 = AtomicU32::new(0);
/// Generic-timer counts between ticks, reloaded on every expiry.
static TICK_INTERVAL: AtomicU64 = AtomicU64::new(0);

macro_rules! read_sysreg {
    ($name:literal) => {{
        let v: u64;
        // SAFETY: reading a system register has no side effects.
        unsafe {
            core::arch::asm!(concat!("mrs {}, ", $name), out(reg) v,
                             options(nomem, nostack, preserves_flags))
        };
        v
    }};
}

macro_rules! write_sysreg {
    ($name:literal, $v:expr) => {{
        // SAFETY: each caller documents why the write is sound.
        unsafe {
            core::arch::asm!(concat!("msr ", $name, ", {}"), in(reg) $v,
                             options(nomem, nostack, preserves_flags))
        };
    }};
}

// ── Clock ─────────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_now_us() -> u64 {
    // The 64-bit counter is two 32-bit registers, so a naive read can tear
    // across a low-word wrap. Re-read the high word and retry if it moved.
    loop {
        // SAFETY: plain MMIO reads of the System Timer.
        unsafe {
            let hi0 = read_volatile((SYSTEM_TIMER_BASE + ST_CHI) as *const u32);
            let lo = read_volatile((SYSTEM_TIMER_BASE + ST_CLO) as *const u32);
            let hi1 = read_volatile((SYSTEM_TIMER_BASE + ST_CHI) as *const u32);
            if hi0 == hi1 {
                // 1 MHz, so counts are already microseconds.
                return ((hi1 as u64) << 32) | lo as u64;
            }
        }
    }
}

// ── Tick ──────────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_tick_start(hz: u32) {
    let hz = hz.max(1);
    let freq = read_sysreg!("cntfrq_el0");
    let interval = freq / hz as u64;
    TICK_INTERVAL.store(interval, Ordering::Release);
    TICK_PERIOD_US.store(1_000_000 / hz, Ordering::Release);

    // Route the non-secure physical timer to *this* core as an IRQ. The
    // kernel may well be running on a core the firmware never booted.
    // SAFETY: MMIO write to this core's ARM-local interrupt-control
    // register.
    unsafe {
        write_volatile(core_timer_irqcntl() as *mut u32, TIMER_IRQ_CNTPNSIRQ);
    }

    // Arm the first expiry against the absolute comparator rather than a
    // countdown, then enable the timer. CNTP_CTL_EL0: bit 0 enable,
    // bit 1 mask.
    let now = read_sysreg!("cntpct_el0");
    write_sysreg!("cntp_cval_el0", now + interval);
    write_sysreg!("cntp_ctl_el0", 1u64);
}

/// Called from the interrupt path when the generic timer has expired.
fn on_timer_tick() {
    // Advance the absolute comparator rather than reloading a countdown.
    //
    // Writing CNTP_TVAL_EL0 sets the deadline relative to *now*, meaning
    // the moment the handler gets around to it, so every tick quietly
    // absorbs the exception entry, the register save and the MMIO read
    // above it. That is a small constant, but it is a constant added
    // every single tick, and it makes the tick grid walk away from real
    // time forever. Advancing CVAL by a fixed interval cannot drift: a
    // late handler produces one late tick, not a permanently skewed
    // grid.
    let interval = TICK_INTERVAL.load(Ordering::Acquire);
    let cval = read_sysreg!("cntp_cval_el0");
    write_sysreg!("cntp_cval_el0", cval + interval);

    rivet::watchdog::on_tick();
    rivet::timer::poll_timers(__rivet_board_now_us());
}

// ── Interrupts ────────────────────────────────────────────────────

/// Acknowledge and service whatever raised the current interrupt.
///
/// Scheduling is not done here: `rivet-arch-aarch64` runs the scheduler on
/// the way out of the interrupt regardless of what fired.
#[no_mangle]
extern "Rust" fn __rivet_board_on_irq() {
    // SAFETY: MMIO read of this core's pending-source register.
    let source = unsafe { read_volatile(core_irq_source() as *const u32) };

    if source & TIMER_IRQ_CNTPNSIRQ != 0 {
        on_timer_tick();
    }

    // Peripheral interrupts arrive through the legacy controller, which
    // this core sees as one aggregated source. Anything registered with
    // `rivet::irq` gets dispatched by number.
    if source & (1 << 8) != 0 {
        dispatch_peripheral_irqs();
    }
}

/// Walk the legacy controller's two pending words and dispatch each set
/// line. There is no claim/complete handshake to perform: the peripheral
/// itself owns the acknowledgement, which its handler does.
fn dispatch_peripheral_irqs() {
    // SAFETY: MMIO reads of the pending registers.
    let (p1, p2) = unsafe {
        (
            read_volatile((IRQ_BASE + 0x04) as *const u32),
            read_volatile((IRQ_BASE + 0x08) as *const u32),
        )
    };
    for bit in 0..32 {
        if p1 & (1 << bit) != 0 {
            rivet::irq::dispatch(bit);
        }
    }
    for bit in 0..32 {
        if p2 & (1 << bit) != 0 {
            rivet::irq::dispatch(32 + bit);
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_enable(irq_num: u32) {
    let (reg, bit) = if irq_num < 32 {
        (ENABLE_IRQS_1, irq_num)
    } else {
        (ENABLE_IRQS_2, irq_num - 32)
    };
    // These registers are write-to-set: writing a zero bit does nothing,
    // so there is no read-modify-write and no race with an interrupt
    // enabling a different line.
    // SAFETY: MMIO write to the legacy interrupt controller.
    unsafe { write_volatile((IRQ_BASE + reg) as *mut u32, 1 << bit) };
}

#[no_mangle]
extern "Rust" fn __rivet_board_irq_disable(irq_num: u32) {
    let (reg, bit) = if irq_num < 32 {
        (DISABLE_IRQS_1, irq_num)
    } else {
        (DISABLE_IRQS_2, irq_num - 32)
    };
    // SAFETY: as above, write-to-clear.
    unsafe { write_volatile((IRQ_BASE + reg) as *mut u32, 1 << bit) };
}

/// No-op: this controller has no priority levels at all. Every enabled
/// line is equal, and the dispatch order above is simply bit order.
#[no_mangle]
extern "Rust" fn __rivet_board_irq_set_priority(_irq_num: u32, _priority: u8) {}

// ── Console ───────────────────────────────────────────────────────

/// `init_uart_clock` and the baud from config.txt.
const UART_CLK_HZ: u32 = 48_000_000;
const BAUD: u32 = 115_200;

#[no_mangle]
extern "Rust" fn __rivet_board_init() {
    // Leave the UART alone when another OS owns it. `Pl011::init`
    // re-muxes GPIO14/15, which would pull the serial console out from
    // under Linux mid-sentence.
    if CONSOLE_TO_SHMEM.load(Ordering::Acquire) != 0 {
        return;
    }
    // SAFETY: called once, before any task runs, and this core is the
    // only thing touching the UART or GPIO14/15.
    unsafe { Pl011.init(UART_CLK_HZ, BAUD) };
}

/// Where kernel console output goes.
///
/// Defaults to the UART. Running alongside another OS that owns the
/// serial line, [`use_shared_console`] redirects it into the ring in
/// shared memory instead, since there is only one UART on the header.
static CONSOLE_TO_SHMEM: AtomicU32 = AtomicU32::new(0);

/// Send kernel console output to the shared-memory ring rather than the
/// UART, for when another OS owns the serial line.
///
/// # Safety
/// The shared window must be mapped, and [`crate::shmem::init`] must
/// have run.
pub unsafe fn use_shared_console() {
    CONSOLE_TO_SHMEM.store(1, Ordering::Release);
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: the kernel passes a valid pointer/length pair from a live
    // byte slice.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    if CONSOLE_TO_SHMEM.load(Ordering::Acquire) != 0 {
        // SAFETY: the redirect is only armed once the window is mapped
        // and the ring initialised.
        unsafe { crate::shmem::write_bytes(bytes) };
        return;
    }
    for &b in bytes {
        // SAFETY: blocking write to a UART brought up in `__rivet_board_init`.
        unsafe { Pl011.put_byte(b) };
    }
}

/// No-op: the console is a blocking polled writer, so there is no TX
/// interrupt that could have disarmed itself and need kicking.
#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {}

// ── Trace transport ───────────────────────────────────────────────

/// PulseTrace frames go to their own ring, never the console.
///
/// The wire format is framed and checksummed binary, so a log line
/// landing in the middle of it corrupts a frame. Sharing one transport
/// between the two would make both unreadable, which is why the shared
/// window carries a second ring rather than one.
///
/// The UART is not an option here even standalone: on this board it is
/// either the console or, alongside Linux, not ours at all.
#[cfg(feature = "trace")]
#[no_mangle]
extern "Rust" fn __rivet_board_trace_write(ptr: *const u8, len: usize) {
    // SAFETY: the kernel passes a valid pointer/length pair from a live
    // byte slice.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    // SAFETY: the trace ring is brought up by `board_bringup` before any
    // task can run and therefore before anything can emit a frame.
    unsafe { crate::shmem::TRACE.write_bytes(bytes) };
}

// ── Reset, exit, watchdog ─────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_board_reset() -> ! {
    // SAFETY: arming the PM watchdog for the shortest possible period and
    // requesting a full reset is the documented reset path on this SoC;
    // every write needs the password in the top byte.
    unsafe {
        write_volatile((PM_BASE + PM_WDOG) as *mut u32, PM_PASSWORD | 1);
        let rstc = read_volatile((PM_BASE + PM_RSTC) as *const u32);
        write_volatile(
            (PM_BASE + PM_RSTC) as *mut u32,
            PM_PASSWORD | (rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET,
        );
    }
    loop {
        core::hint::spin_loop();
    }
}

/// Park rather than reset.
///
/// On the QEMU boards `exit` reaches a semihosting exit device and gives
/// the test harness a status. There is no such device here, and resetting
/// would drop straight back into the same image, so a board watched over
/// a serial line is better served by halting with the exit code visible
/// than by looping forever through its own boot.
#[no_mangle]
extern "Rust" fn __rivet_board_exit(code: u32) -> ! {
    rivet::console::write_str(if code == 0 {
        "\n[rivet] exit(0)\n"
    } else {
        "\n[rivet] exit(nonzero)\n"
    });
    rivet::console::flush_sync();
    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}

/// The PM watchdog counts at 65536 Hz, so one tick is about 15.26 us.
/// Treating microseconds as ticks, which an earlier version did, asks for
/// a timeout 65536 times shorter than intended.
const WDOG_TICKS_PER_SEC: u64 = 65536;
/// The counter is 20 bits, so the longest period it can express is a
/// little over 16 seconds.
const WDOG_MAX_TICKS: u64 = 0x000F_FFFF;

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_init(period_us: u32) {
    if period_us == 0 {
        return;
    }
    arm_watchdog(period_us);
}

#[no_mangle]
extern "Rust" fn __rivet_board_wdt_feed() {
    let period = WDT_PERIOD_US.load(Ordering::Acquire);
    if period != 0 {
        arm_watchdog(period);
    }
}

/// No-op: this is a real hardware watchdog that counts down on its own,
/// so there is no software deadline to check against the clock.
#[no_mangle]
extern "Rust" fn __rivet_board_wdt_check() {}

static WDT_PERIOD_US: AtomicU32 = AtomicU32::new(0);

fn arm_watchdog(period_us: u32) {
    WDT_PERIOD_US.store(period_us, Ordering::Release);
    // Saturate rather than wrap: wrapping a too-long period turns a
    // generous timeout into a very short one, which resets the board
    // instead of protecting it.
    let ticks = ((period_us as u64 * WDOG_TICKS_PER_SEC) / 1_000_000).min(WDOG_MAX_TICKS) as u32;
    // SAFETY: password-protected PM register writes, as in `reset`.
    unsafe {
        write_volatile((PM_BASE + PM_WDOG) as *mut u32, PM_PASSWORD | ticks);
        let rstc = read_volatile((PM_BASE + PM_RSTC) as *const u32);
        write_volatile(
            (PM_BASE + PM_RSTC) as *mut u32,
            PM_PASSWORD | (rstc & PM_RSTC_WRCFG_CLR) | PM_RSTC_WRCFG_FULL_RESET,
        );
    }
}
