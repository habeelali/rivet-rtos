#![no_std]
#![no_main]
//! Turns on the MMU and re-runs the atomic that aborts without it.
//!
//! This is the milestone the whole port is gated on. `faultcheck` shows
//! that `AtomicUsize::fetch_add` takes a data abort on this board with
//! translation off, measured as ESR `0x96000035`. Here the identical
//! operation runs after an identity map is installed with RAM described
//! as Normal Inner-Shareable Write-Back, which is the memory type the
//! global exclusive monitor requires.
//!
//! The pre-MMU half of the comparison lives in `faultcheck` rather than
//! here, because that path halts: an aborting atomic cannot be resumed
//! from, so one image cannot demonstrate both. Run `faultcheck` for the
//! failure and this for the success. Expected output:
//!
//! ```text
//! MMU enabled, SCTLR_EL1.M=1
//! fetch_add -> 0
//! fetch_add -> 1
//! compare_exchange -> Ok(2)
//! ATOMICS OK
//! ```
//!
//! Reaching `ATOMICS OK` means the kernel's synchronisation primitives
//! can run on this board.

use core::fmt::Write;
use core::sync::atomic::{AtomicUsize, Ordering};

use rivet_bsp_rpi3b::{drop_to_el1, mmu, Pl011};

const UART_CLK_HZ: u32 = 48_000_000;
const BAUD: u32 = 115_200;

static COUNTER: AtomicUsize = AtomicUsize::new(0);

macro_rules! sysreg {
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

#[no_mangle]
pub extern "C" fn rust_main(_dtb: u64) -> ! {
    let mut uart = Pl011;
    // SAFETY: sole owner of the PL011 and GPIO14/15.
    unsafe { uart.init(UART_CLK_HZ, BAUD) };

    let _ = write!(
        uart,
        "\n\
         ==== rivet rpi3b MMU check ====\n\
         CurrentEL      {}\n\
         SCTLR_EL1.M    {}\n",
        sysreg!("CurrentEL") >> 2,
        u8::from(mmu::enabled_el1()),
    );

    // The MMU work targets EL1, since that is where the kernel will run
    // and it leaves EL0 available for tasks later.
    // SAFETY: called once, from EL2, with a valid stack.
    unsafe { drop_to_el1() };
    let _ = writeln!(
        uart,
        "dropped to EL{}, SCTLR_EL1 {:#018x}",
        sysreg!("CurrentEL") >> 2,
        sysreg!("sctlr_el1"),
    );

    let _ = writeln!(uart, "enabling MMU...");
    // SAFETY: draining first, so nothing is stranded in the FIFO if the
    // translation tables are wrong and this faults instead of returning.
    unsafe { uart.flush() };

    // SAFETY: called once, at EL1, with translation currently off.
    unsafe { mmu::enable_el1() };

    let _ = write!(
        uart,
        "MMU enabled, SCTLR_EL1.M={} SCTLR_EL1={:#018x}\n\
         TTBR0_EL1 {:#018x}  TCR_EL1 {:#018x}  MAIR_EL1 {:#018x}\n",
        u8::from(mmu::enabled_el1()),
        sysreg!("sctlr_el1"),
        sysreg!("ttbr0_el1"),
        sysreg!("tcr_el1"),
        sysreg!("mair_el1"),
    );

    // The peripheral window has to still work, or this line never
    // appears: the UART is now reached through a Device-nGnRnE mapping
    // rather than by virtue of translation being off.
    let _ = writeln!(uart, "peripherals still reachable through the map");

    // And the actual point of all of it.
    let a = COUNTER.fetch_add(1, Ordering::SeqCst);
    let _ = writeln!(uart, "fetch_add -> {a}");
    let b = COUNTER.fetch_add(1, Ordering::SeqCst);
    let _ = writeln!(uart, "fetch_add -> {b}");
    let c = COUNTER.compare_exchange(2, 42, Ordering::SeqCst, Ordering::SeqCst);
    let _ = writeln!(uart, "compare_exchange -> {c:?}");

    if a == 0 && b == 1 && c == Ok(2) && COUNTER.load(Ordering::SeqCst) == 42 {
        let _ = writeln!(uart, "ATOMICS OK");
    } else {
        let _ = writeln!(uart, "!! ATOMICS WRONG: unexpected values");
    }

    // SAFETY: draining before the idle loop.
    unsafe { uart.flush() };
    let mut n: u64 = 0;
    loop {
        let _ = writeln!(uart, "TICK n={n}");
        n += 1;
        let freq = sysreg!("cntfrq_el0");
        let target = sysreg!("cntpct_el0") + freq;
        while sysreg!("cntpct_el0") < target {
            core::hint::spin_loop();
        }
    }
}
