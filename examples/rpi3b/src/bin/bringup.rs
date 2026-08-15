#![no_std]
#![no_main]
//! Raspberry Pi 3B bring-up: prove the toolchain, the boot path and the
//! serial console on real BCM2837 silicon.
//!
//! One power-on has to answer every open question at once, so this does
//! rather more than print a banner: it reports the state the firmware
//! handed over, brings up both of the board's UARTs in turn (settling
//! which one actually reaches the header pins), drops to EL1, and then
//! heartbeats so "still running" is distinguishable from "printed once,
//! then wedged".
//!
//! The boot sequence, checkpoints and fault reporting live in
//! `rivet_bsp_rpi3b::boot`.

use core::fmt::Write;

use rivet_bsp_rpi3b::{drop_to_el1, fsel, mux_uart_pins, MiniUart, Pl011};

/// `init_uart_clock` from config.txt. The PL011's reference clock.
const UART_CLK_HZ: u32 = 48_000_000;
/// `core_freq` from config.txt. The mini UART divides this instead, which
/// is exactly why it is the fallback and not the primary console.
const CORE_FREQ_HZ: u32 = 250_000_000;
const BAUD: u32 = 115_200;

/// Read a system register into a `u64`.
macro_rules! sysreg {
    ($name:literal) => {{
        let v: u64;
        // SAFETY: reading a system register has no side effects, and
        // every register named here is readable at the EL it is read at.
        unsafe {
            core::arch::asm!(concat!("mrs {}, ", $name), out(reg) v,
                             options(nomem, nostack, preserves_flags))
        };
        v
    }};
}

/// Busy-wait using the architected generic timer.
fn delay_ms(ms: u64) {
    let freq = sysreg!("cntfrq_el0");
    let target = sysreg!("cntpct_el0") + freq.saturating_mul(ms) / 1000;
    while sysreg!("cntpct_el0") < target {
        core::hint::spin_loop();
    }
}

/// The image's true runtime base, read PC-relatively.
fn start_addr() -> u64 {
    let v: u64;
    // SAFETY: ADR only computes a PC-relative address.
    unsafe { core::arch::asm!("adr {}, _start", out(reg) v, options(nomem, nostack)) };
    v
}

#[no_mangle]
pub extern "C" fn rust_main(dtb: u64) -> ! {
    // Sample what the firmware left behind before overwriting it. The
    // divisors are the only way to observe the real UARTCLK from
    // software: if init_uart_clock was not 48 MHz, what the firmware
    // programmed will not match what is computed below, and the ratio
    // between them gives the true clock.
    let mut uart = Pl011;
    // SAFETY: plain register reads against the PL011.
    let (fw_ibrd, fw_fbrd) = unsafe { uart.divisors() };
    // SAFETY: as above.
    let fw_cr = unsafe { uart.control() };
    // SCTLR_EL2 has to be sampled while still at EL2.
    let sctlr_el2 = sysreg!("sctlr_el2");

    // SAFETY: sole owner of the PL011 and of GPIO14/15 at this point.
    unsafe { uart.init(UART_CLK_HZ, BAUD) };
    // SAFETY: as above.
    let (ibrd, fbrd) = unsafe { uart.divisors() };

    let _ = write!(
        uart,
        "\n\
         ==== rivet rpi3b bring-up ====\n\
         CurrentEL      {}\n\
         MPIDR_EL1      {:#018x}\n\
         DTB pointer    {:#018x}\n\
         CNTFRQ_EL0     {} Hz\n\
         SCTLR_EL2      {:#018x}\n\
         _start at      {:#018x}\n\
         firmware PL011 CR={fw_cr:#x} IBRD={fw_ibrd} FBRD={fw_fbrd}\n\
         our PL011      IBRD={ibrd} FBRD={fbrd}\n\
         PL011 DRIVER OK\n",
        sysreg!("CurrentEL") >> 2,
        sysreg!("mpidr_el1"),
        dtb,
        sysreg!("cntfrq_el0"),
        sctlr_el2,
        start_addr(),
    );

    // Prove the other UART in the same boot, so one power-on settles
    // which of the two actually reaches the header pins. Flushing first
    // matters: re-muxing the pins mid-transmission drops whatever is
    // still sitting in the FIFO.
    // SAFETY: draining before the pins move underneath the transmitter.
    unsafe { uart.flush() };
    let mut mini = MiniUart;
    // SAFETY: takes over GPIO14/15 and the AUX block.
    unsafe { mini.init(CORE_FREQ_HZ, BAUD) };
    let _ = writeln!(mini, "MINIUART DRIVER OK (core_freq={CORE_FREQ_HZ})");
    // SAFETY: draining before the pins move back.
    unsafe { mini.flush() };

    // Back to the PL011 for everything that follows.
    // SAFETY: reclaiming GPIO14/15 for the PL011.
    unsafe {
        mux_uart_pins(fsel::ALT0);
        uart.init(UART_CLK_HZ, BAUD);
    }
    let _ = writeln!(uart, "BACK ON PL011");

    // Drop to EL1. Nothing here needs it, but it is the transition the
    // real port will be built on, and proving it now costs one line.
    // SAFETY: called once, from EL2, with a valid stack.
    unsafe { drop_to_el1() };
    let _ = writeln!(uart, "EL1 OK, CurrentEL={}", sysreg!("CurrentEL") >> 2);

    let mut n: u64 = 0;
    loop {
        let _ = writeln!(uart, "TICK n={n}");
        n += 1;
        delay_ms(1000);
    }
}
