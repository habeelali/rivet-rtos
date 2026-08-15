#![no_std]
#![no_main]
//! Rivet running on core 3 while cores 0-2 stay parked.
//!
//! This is the arrangement the eventual Linux-alongside setup needs, with
//! the other three cores idle instead of running an OS: one core is
//! handed to rivet and owns its own timer, scheduler and console, while
//! the rest never enter the kernel at all.
//!
//! Core 0 brings the machine up far enough to be representative of that
//! future, rather than taking the easy path. It enables its own MMU and
//! caches *before* releasing core 3, which means the mailbox write has to
//! be cleaned to the point of coherency or the target core, still running
//! uncached, never sees it. Getting that wrong is invisible with caches
//! off and a hang with them on, so it is worth exercising now.
//!
//! Output is deliberately verbose: each stage reports the state it
//! observes, so a failure names the stage rather than going quiet.

use core::fmt::Write;

use rivet_arch_aarch64 as _;
use rivet_bsp_rpi3b::{drop_to_el1, mmu, smp, Pl011};

const UART_CLK_HZ: u32 = 48_000_000;
const BAUD: u32 = 115_200;

/// Which core rivet gets. The others stay where the firmware left them.
const RIVET_CORE: usize = 3;

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

fn uart() -> Pl011 {
    Pl011
}

// ── Core 0: bring up, release core 3, then get out of the way ─────

#[no_mangle]
pub extern "C" fn rust_main(_dtb: u64) -> ! {
    let mut u = uart();
    // SAFETY: core 0 is the only thing touching the UART at this point.
    unsafe { u.init(UART_CLK_HZ, BAUD) };

    let _ = write!(
        u,
        "\n\
         ==== rivet rpi3b: core {RIVET_CORE} handover ====\n\
         [core0] MPIDR_EL1   {:#018x}  (core {})\n\
         [core0] CurrentEL   {}\n\
         [core0] CNTFRQ_EL0  {} Hz\n",
        sysreg!("mpidr_el1"),
        smp::current_core(),
        sysreg!("CurrentEL") >> 2,
        sysreg!("cntfrq_el0"),
    );

    // Build the address space before enabling anything, so core 3 can
    // join it later without rewriting tables underneath a core that is
    // already walking them.
    // SAFETY: no other core is using the tables yet.
    unsafe { mmu::build_tables() };
    let _ = writeln!(u, "[core0] translation tables built");

    // Take the harder path on purpose: caches on before the release.
    // SAFETY: called once, from EL2, on the boot stack.
    unsafe { drop_to_el1() };
    // SAFETY: at EL1 with translation off, tables already built.
    unsafe { mmu::enable_el1_prebuilt() };
    let _ = write!(
        u,
        "[core0] dropped to EL{}, MMU on (SCTLR_EL1 {:#018x})\n\
         [core0] caches are live, so the mailbox write needs cleaning to PoC\n",
        sysreg!("CurrentEL") >> 2,
        sysreg!("sctlr_el1"),
    );

    let before = smp::witness();
    let _ = write!(
        u,
        "[core0] witness before release: [{}, {}, {}, {}]\n\
         [core0] releasing core {RIVET_CORE}...\n",
        before[0], before[1], before[2], before[3],
    );
    // SAFETY: draining first, so nothing is stranded if the handover
    // fails and this core parks with output still in the FIFO.
    unsafe { u.flush() };

    // SAFETY: core 3 has not been released before, and `core3_entry`
    // never returns.
    unsafe { smp::release_core(RIVET_CORE, core3_entry) };

    // Core 0's work is done. Park exactly like cores 1 and 2, so that
    // anything printed from here on came from core 3.
    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}

// ── Core 3: become the rivet core ─────────────────────────────────

/// Entered by core 3 straight off the spin table: MMU off, caches off,
/// interrupts masked, on its own 64 KiB stack.
extern "C" fn core3_entry(core: u64) -> ! {
    let mut u = uart();
    let _ = write!(
        u,
        "\n[core{core}] woke from the spin table\n\
         [core{core}] MPIDR_EL1   {:#018x}\n\
         [core{core}] CurrentEL   {}  (before the drop)\n\
         [core{core}] SCTLR_EL1.M {}  (before the MMU)\n",
        sysreg!("mpidr_el1"),
        sysreg!("CurrentEL") >> 2,
        u8::from(mmu::enabled_el1()),
    );

    if core as usize != RIVET_CORE {
        let _ = writeln!(u, "!! wrong core woke: expected {RIVET_CORE}");
        park();
    }

    // SAFETY: called once on this core, from EL2, with a valid stack.
    unsafe { drop_to_el1() };
    // SAFETY: at EL1 with translation off; core 0 already built the
    // tables, so join that address space rather than rebuilding it.
    unsafe { mmu::enable_el1_prebuilt() };

    let _ = write!(
        u,
        "[core{core}] dropped to EL{}, MMU on (SCTLR_EL1 {:#018x})\n\
         [core{core}] TTBR0_EL1   {:#018x}  (same tables core 0 built)\n",
        sysreg!("CurrentEL") >> 2,
        sysreg!("sctlr_el1"),
        sysreg!("ttbr0_el1"),
    );

    // Atomics are the gate on running any kernel code at all, and this
    // core has its own MMU state, so prove them here rather than assume
    // core 0's result carries over.
    {
        use core::sync::atomic::{AtomicUsize, Ordering};
        static N: AtomicUsize = AtomicUsize::new(0);
        let a = N.fetch_add(1, Ordering::SeqCst);
        let b = N.compare_exchange(1, 7, Ordering::SeqCst, Ordering::SeqCst);
        let ok = a == 0 && b == Ok(1) && N.load(Ordering::SeqCst) == 7;
        let _ = writeln!(
            u,
            "[core{core}] atomics    {}",
            if ok { "OK" } else { "WRONG" }
        );
        if !ok {
            park();
        }
    }

    let w = smp::witness();
    let _ = write!(
        u,
        "[core{core}] witness     [{}, {}, {}, {}]  (1 = core ran our parking loop)\n\
         [core{core}] handing over to the kernel\n",
        w[0], w[1], w[2], w[3],
    );
    // SAFETY: draining before the kernel takes over the console.
    unsafe { u.flush() };

    extern "C" {
        fn rivet_main() -> !;
    }
    // SAFETY: generated by `#[rivet::main]` below.
    unsafe { rivet_main() }
}

fn park() -> ! {
    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}

// ── The kernel, now running on core 3 ─────────────────────────────

use core::sync::atomic::{AtomicU32, Ordering};

static A_COUNT: AtomicU32 = AtomicU32::new(0);
static B_COUNT: AtomicU32 = AtomicU32::new(0);

fn worker_a(_: &'static ()) -> ! {
    loop {
        A_COUNT.fetch_add(1, Ordering::Relaxed);
        rivet::preempt::sleep_ms(10);
    }
}

fn worker_b(_: &'static ()) -> ! {
    loop {
        B_COUNT.fetch_add(1, Ordering::Relaxed);
        rivet::preempt::sleep_ms(10);
    }
}

fn checker(_: &'static ()) -> ! {
    let phys = smp::current_core();
    rivet::console::write_str("[kernel] running on physical core ");
    print_dec(phys);
    rivet::console::write_str(", scheduler hart ");
    print_dec(rivet::port::arch::hart_id());
    rivet::console::write_str("\n");

    // The kernel must be on the core we handed over, not core 0.
    if phys != RIVET_CORE {
        rivet::console::write_str("CORE_FAIL: kernel is not on the released core\n");
        rivet::exit_failure(3);
    }

    let start = rivet::port::board::now_us();
    rivet::preempt::sleep_ms(500);
    let elapsed = rivet::port::board::now_us() - start;
    let a = A_COUNT.load(Ordering::Relaxed);
    let b = B_COUNT.load(Ordering::Relaxed);

    rivet::console::write_str("[kernel] elapsed_us=");
    print_dec(elapsed as usize);
    rivet::console::write_str(" a=");
    print_dec(a as usize);
    rivet::console::write_str(" b=");
    print_dec(b as usize);
    rivet::console::write_str("\n");

    if !(400_000..=700_000).contains(&elapsed) {
        rivet::console::write_str("CLOCK_FAIL\n");
        rivet::exit_failure(1);
    }
    if a < 10 || b < 10 {
        rivet::console::write_str("SCHED_FAIL\n");
        rivet::exit_failure(2);
    }

    rivet::console::write_str("CORE3_DEMO_OK\n");
    rivet::exit_success();
}

fn print_dec(mut v: usize) {
    let mut buf = [0u8; 20];
    let mut i = buf.len();
    loop {
        i -= 1;
        buf[i] = b'0' + (v % 10) as u8;
        v /= 10;
        if v == 0 {
            break;
        }
    }
    // SAFETY: every byte written is an ASCII digit.
    rivet::console::write_str(unsafe { core::str::from_utf8_unchecked(&buf[i..]) });
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("[kernel] rivet started\n");
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 1, entry = worker_a, arg = ());
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 1, entry = worker_b, arg = ());
    let _ = rivet::spawn_ptask!(stack = 4096, priority = 2, entry = checker, arg = ());
    rivet::run();
}
