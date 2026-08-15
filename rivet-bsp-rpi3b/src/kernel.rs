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

    // Arm the first expiry, then enable the timer with its output
    // unmasked. CNTP_CTL_EL0: bit 0 enable, bit 1 mask.
    write_sysreg!("cntp_tval_el0", interval);
    write_sysreg!("cntp_ctl_el0", 1u64);
}

/// Called from the interrupt path when the generic timer has expired.
fn on_timer_tick() {
    // Re-arm before doing any work, so the period does not stretch by
    // however long the tick handler takes.
    write_sysreg!("cntp_tval_el0", TICK_INTERVAL.load(Ordering::Acquire));

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
    // SAFETY: called once, before any task runs, and this core is the
    // only thing touching the UART or GPIO14/15.
    unsafe { Pl011.init(UART_CLK_HZ, BAUD) };
}

#[no_mangle]
extern "Rust" fn __rivet_board_console_write(ptr: *const u8, len: usize) {
    // SAFETY: the kernel passes a valid pointer/length pair from a live
    // byte slice.
    let bytes = unsafe { core::slice::from_raw_parts(ptr, len) };
    for &b in bytes {
        // SAFETY: blocking write to a UART brought up in `__rivet_board_init`.
        unsafe { Pl011.put_byte(b) };
    }
}

/// No-op: the console is a blocking polled writer, so there is no TX
/// interrupt that could have disarmed itself and need kicking.
#[no_mangle]
extern "Rust" fn __rivet_board_console_kick_tx() {}

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

/// PM watchdog countdown, in units of roughly 16 microseconds.
const WDOG_TICKS_PER_US: u32 = 1;

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
    // The counter is 20 bits wide, so the longest period it can express is
    // about 16 seconds; saturate rather than wrap into a much shorter one.
    let ticks = period_us.saturating_mul(WDOG_TICKS_PER_US).min(0x000F_FFFF);
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
