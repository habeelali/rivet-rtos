#![no_std]
#![no_main]
//! Provokes the fault that gates this whole port, and doubles as a check
//! on the exception vectors.
//!
//! With the MMU off, AArch64 treats all memory as Device-nGnRnE, which
//! has no exclusive monitor. The load/store-exclusive pair that LLVM
//! emits for `AtomicUsize::fetch_add` therefore takes a synchronous data
//! abort rather than working. **Confirmed on a Pi 3B**, exactly as the
//! architecture predicts:
//!
//! ```text
//! *** EXCEPTION ***
//! ESR  0x0000000096000035  (EC=0x25 ISS=0x35)
//!   -> unsupported exclusive/atomic access: ...
//! ```
//!
//! That is why the kernel cannot be linked into this board yet, and why
//! enabling the MMU is the next milestone rather than a later
//! optimisation.
//!
//! QEMU does not reproduce it. Its `raspi3b` model permits the exclusive
//! access and `fetch_add` simply succeeds, which is why this had to be
//! settled on real silicon alongside GPIO muxing and everything else in
//! the firmware path emulation never runs. The binary reports whichever
//! way it goes rather than assuming, and executes a `BRK` afterwards so
//! the vectors and the decoder are proven on the runs where the atomic
//! survives. Under QEMU, expect "NO FAULT" then the `BRK` dump (EC=0x3c).

use core::fmt::Write;
use core::sync::atomic::{AtomicUsize, Ordering};

use rivet_bsp_rpi3b::Pl011;

const UART_CLK_HZ: u32 = 48_000_000;
const BAUD: u32 = 115_200;

/// In `.bss`, so the access is a genuine memory operation the compiler
/// cannot fold away.
static COUNTER: AtomicUsize = AtomicUsize::new(0);

#[no_mangle]
pub extern "C" fn rust_main(_dtb: u64) -> ! {
    let mut uart = Pl011;
    // SAFETY: sole owner of the PL011 and GPIO14/15.
    unsafe { uart.init(UART_CLK_HZ, BAUD) };

    let _ = write!(
        uart,
        "\n\
         ==== rivet rpi3b fault check ====\n\
         A plain atomic load is fine with the MMU off, since it compiles\n\
         to an ordinary LDR.\n"
    );
    let _ = writeln!(uart, "  load  -> {}", COUNTER.load(Ordering::Relaxed));

    let _ = write!(
        uart,
        "Now an atomic read-modify-write, which compiles to LDXR/STXR.\n\
         On hardware this should abort; under QEMU it is expected to\n\
         succeed, because the model does not enforce the Device-memory\n\
         restriction.\n"
    );
    // SAFETY: draining before the possible abort, so the message above
    // is not stranded in the FIFO.
    unsafe { uart.flush() };

    let prev = COUNTER.fetch_add(1, Ordering::SeqCst);
    core::hint::black_box(prev);

    let _ = write!(
        uart,
        "\nNO FAULT: fetch_add returned {prev} and execution continued.\n\
         Expected under QEMU. On hardware it would mean atomics are\n\
         usable with the MMU off, and the MMU milestone can be reordered.\n\
         \n\
         Executing BRK #0 to prove the vectors work regardless...\n"
    );
    // SAFETY: draining before the guaranteed trap.
    unsafe { uart.flush() };

    // A software breakpoint always raises a synchronous exception
    // (EC=0x3c), so reaching the dump below confirms the vector table,
    // the EL demux and the decoder independently of the atomics question.
    // SAFETY: the exception vectors are installed and BRK is trapped.
    unsafe { core::arch::asm!("brk #0", options(nomem, nostack)) };

    let _ = write!(
        uart,
        "\n!! BRK did not trap: exception vectors are broken.\n"
    );
    // SAFETY: draining before halting.
    unsafe { uart.flush() };
    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}
