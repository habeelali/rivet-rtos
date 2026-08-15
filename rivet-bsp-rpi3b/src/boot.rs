//! Entry point, exception vectors and fault reporting for the Pi 3B.
//!
//! This is the board's equivalent of what `rivet-rt` provides elsewhere,
//! kept here rather than added as a fourth architecture to that crate
//! until the port is far enough along to need the kernel at all.
//!
//! The binary supplies the actual program by defining
//! `#[no_mangle] extern "C" fn rust_main(dtb: u64) -> !`.
//!
//! Physical iteration on this board is slow (write an SD card, hand it
//! over, read back a terminal), so the entry sequence emits single
//! character checkpoints from its first instructions onward. A boot that
//! dies halfway still reports exactly how far it got:
//!
//! ```text
//! A<addr>   raw poke into the firmware-initialised PL011, then the
//!           image's true runtime address
//! B         .bss zeroed
//! C         FP/SIMD untrapped at EL2
//! D         exception vectors installed
//! ```

use core::fmt::Write;

#[cfg(not(feature = "amp"))]
use crate::Pl011;

// Entry when rivet owns the machine, loaded by the firmware at
// `kernel_address` with every other core still parked.
#[cfg(not(feature = "amp"))]
core::arch::global_asm!(
    r#"
.section .text.boot, "ax"
.global _start
_start:
    // Entry state, fixed by the firmware's ARM stub: EL2h, non-secure,
    // MMU and caches off, DAIF fully masked, x0 = DTB pointer,
    // x1..x3 = 0, and SP *undefined*.
    mov     x19, x0                     // keep the DTB pointer for the banner

    // Only core 0 runs the bring-up. Both the real firmware and QEMU park
    // cores 1-3 on the spin table before they ever reach this image, so
    // in practice nothing else arrives here: the witness array below
    // reads all zeroes on both. Kept anyway, because a stray core landing
    // on the boot path would otherwise race core 0 through .bss zeroing
    // with no sign of why.
    mrs     x0, mpidr_el1
    and     x0, x0, #3
    cbz     x0, .Lcore0

    // Secondary cores implement the same spin-table protocol the firmware
    // uses, so a release works identically whether this core was parked
    // by the firmware or arrived here under emulation.
.Lpark:
    // Check in, so the releasing core can report which cores ever ran
    // this code. Written on every wake, which keeps it from being lost to
    // core 0's .bss zeroing happening concurrently at boot.
    adrp    x2, RIVET_SMP_WITNESS
    add     x2, x2, :lo12:RIVET_SMP_WITNESS
    mov     x3, #1
    str     x3, [x2, x0, lsl #3]
    dsb     sy
    wfe
    // Spin table at 0xd8, one 64-bit slot per core.
    mov     x2, #0xd8
    ldr     x1, [x2, x0, lsl #3]
    cbz     x1, .Lpark
    br      x1
.Lcore0:

    // The stack comes first, because everything below is a real call and
    // AArch64 hands us no stack at all. __stack_top is a link-time
    // absolute in the first megabyte of RAM, which is backed on this
    // board no matter where the image itself was loaded.
    ldr     x0, =__stack_top
    mov     sp, x0

    // Checkpoint 'A', poked straight into the PL011 data register with
    // no configuration at all. With uart_2ndstage=1 the firmware has
    // already brought that UART up, so a single 'A' proves the card, the
    // firmware, the load address, the branch and the wiring, before any
    // register write of ours enters the picture.
    mov     w0, #0x41                   // 'A'
    bl      boot_putc

    // The image's true runtime address, taken PC-relatively so it is
    // correct even if the firmware ignored kernel_address and loaded us
    // elsewhere. Expect 0000000000080000.
    adr     x0, _start
    bl      boot_puthex
    bl      boot_crlf

    // Zero .bss. Both ends are 16-byte aligned by the linker script, so
    // this can go a register pair at a time.
    ldr     x0, =__bss_start
    ldr     x1, =__bss_end
.Lbss:
    cmp     x0, x1
    b.hs    .Lbss_done
    stp     xzr, xzr, [x0], #16
    b       .Lbss
.Lbss_done:
    mov     w0, #0x42                   // 'B'
    bl      boot_putc

    // Let EL2 use FP/SIMD. CPTR_EL2 already resets with TFP clear on
    // this core, but aarch64-unknown-none enables NEON and LLVM emits
    // vector registers for ordinary struct moves, so this is not worth
    // leaving to a reset value.
    msr     cptr_el2, xzr
    isb
    mov     w0, #0x43                   // 'C'
    bl      boot_putc

    // Exception vectors, installed before anything can fault. This is
    // what turns every later "it just hung" into a decodable dump.
    adr     x0, _vectors
    msr     vbar_el2, x0
    isb
    mov     w0, #0x44                   // 'D'
    bl      boot_putc
    bl      boot_crlf

    mov     x0, x19                     // DTB pointer
    bl      rust_main
.Lhalt:
    wfe
    b       .Lhalt
"#
);

// Entry when another operating system owns the machine.
//
// Reached by whichever core the other OS released, so there is no core
// guard: the core that arrives is the one rivet was handed. There are no
// checkpoint characters either, because the UART belongs to the other
// side and poking it would cut into its console mid-line. Diagnostics go
// to the shared-memory ring instead, once the caller sets it up.
#[cfg(feature = "amp")]
core::arch::global_asm!(
    r#"
.section .text.boot, "ax"
.global _start
_start:
    mov     x19, x0

    // This core's own stack. Cores released by another OS arrive with SP
    // undefined, exactly like one released from the spin table.
    ldr     x0, =__stack_top
    mov     sp, x0

    // Zero .bss. Both ends are 16-byte aligned by the linker script.
    ldr     x0, =__bss_start
    ldr     x1, =__bss_end
.Lamp_bss:
    cmp     x0, x1
    b.hs    .Lamp_bss_done
    stp     xzr, xzr, [x0], #16
    b       .Lamp_bss
.Lamp_bss_done:

    msr     cptr_el2, xzr               // let EL2 use FP/SIMD
    isb
    adr     x0, _vectors                // fault reporting before anything can fault
    msr     vbar_el2, x0
    isb

    mov     x0, x19
    bl      rust_main
.Lamp_halt:
    wfe
    b       .Lamp_halt
"#
);

core::arch::global_asm!(
    r#"
.section .text.boot, "ax"

// Emit one byte on the PL011 using only registers, so this works before
// the stack, .bss or any static data are usable. The literal-pool load
// keeps it position-independent.
.global boot_putc
boot_putc:
    ldr     x1, =0x3F201000             // PL011 base
.Lputc_wait:
    ldr     w2, [x1, #0x18]             // FR
    tbnz    w2, #5, .Lputc_wait         // spin while TXFF
    str     w0, [x1]                    // DR
    ret

// Sixteen hex digits of x0, most significant first.
boot_puthex:
    stp     x29, x30, [sp, #-32]!
    stp     x20, x21, [sp, #16]
    mov     x20, x0
    mov     x21, #60
.Lhex_loop:
    lsr     x0, x20, x21
    and     x0, x0, #0xf
    cmp     x0, #9
    b.hi    .Lhex_alpha
    add     x0, x0, #0x30               // '0'
    b       .Lhex_emit
.Lhex_alpha:
    add     x0, x0, #0x57               // 'a' - 10
.Lhex_emit:
    bl      boot_putc
    subs    x21, x21, #4
    b.ge    .Lhex_loop
    ldp     x20, x21, [sp, #16]
    ldp     x29, x30, [sp], #32
    ret

boot_crlf:
    stp     x29, x30, [sp, #-16]!
    mov     w0, #0x0D
    bl      boot_putc
    mov     w0, #0x0A
    bl      boot_putc
    ldp     x29, x30, [sp], #16
    ret

// Drop from EL2 to EL1, resuming at this function's own return address.
// Setting ELR_EL2 to x30 is what makes an ERET behave like a return.
.global drop_to_el1
drop_to_el1:
    mov     x0, #(3 << 20)
    msr     cpacr_el1, x0               // FPEN=0b11: no FP/SIMD trap at EL1/EL0
    mov     x0, #3
    msr     cnthctl_el2, x0             // EL1PCTEN|EL1PCEN: EL1 may read the timer
    msr     cntvoff_el2, xzr
    mov     x0, #1
    lsl     x0, x0, #31
    msr     hcr_el2, x0                 // RW=1: EL1 is AArch64
    ldr     x0, =0x30d00800
    msr     sctlr_el1, x0               // RES1 bits only; MMU and caches stay off
    adr     x0, _vectors
    msr     vbar_el1, x0                // EL2's VBAR no longer covers EL1 faults
    mov     x0, sp
    msr     sp_el1, x0                  // EL1h banks its own SP; carry ours over
    mov     x0, #0x3c5                  // DAIF masked, M[3:0]=0b0101 = EL1h
    msr     spsr_el2, x0
    msr     elr_el2, x30
    eret

// Every exception funnels here. The table is installed in both VBAR_EL2
// and VBAR_EL1, so read the fault registers of whichever EL we are in.
fault_common:
    mrs     x4, CurrentEL
    lsr     x4, x4, #2
    cmp     x4, #2
    b.eq    .Lfault_el2
    mrs     x0, esr_el1
    mrs     x1, elr_el1
    mrs     x2, far_el1
    mrs     x3, spsr_el1
    b       .Lfault_report
.Lfault_el2:
    mrs     x0, esr_el2
    mrs     x1, elr_el2
    mrs     x2, far_el2
    mrs     x3, spsr_el2
.Lfault_report:
    bl      rust_fault_handler
.Lfault_halt:
    wfe
    b       .Lfault_halt

// Sixteen entries of 128 bytes each, 2 KiB aligned: four exception kinds
// (synchronous, IRQ, FIQ, SError) for each of four origins (current EL on
// SP0, current EL on SPx, lower EL in AArch64, lower EL in AArch32).
.section .text.vectors, "ax"
.balign 2048
.global _vectors
_vectors:
    .balign 128
    b       fault_common                // current EL, SP0, synchronous
    .balign 128
    b       fault_common                // current EL, SP0, IRQ
    .balign 128
    b       fault_common                // current EL, SP0, FIQ
    .balign 128
    b       fault_common                // current EL, SP0, SError
    .balign 128
    b       fault_common                // current EL, SPx, synchronous
    .balign 128
    b       fault_common                // current EL, SPx, IRQ
    .balign 128
    b       fault_common                // current EL, SPx, FIQ
    .balign 128
    b       fault_common                // current EL, SPx, SError
    .balign 128
    b       fault_common                // lower EL, AArch64, synchronous
    .balign 128
    b       fault_common                // lower EL, AArch64, IRQ
    .balign 128
    b       fault_common                // lower EL, AArch64, FIQ
    .balign 128
    b       fault_common                // lower EL, AArch64, SError
    .balign 128
    b       fault_common                // lower EL, AArch32, synchronous
    .balign 128
    b       fault_common                // lower EL, AArch32, IRQ
    .balign 128
    b       fault_common                // lower EL, AArch32, FIQ
    .balign 128
    b       fault_common                // lower EL, AArch32, SError
    .balign 128
"#
);

extern "C" {
    /// Drops from EL2 to EL1 and returns there, on the same stack.
    ///
    /// # Safety
    /// Only callable once, from EL2, and it reprograms `SCTLR_EL1`,
    /// `HCR_EL2` and `VBAR_EL1` on the way through.
    pub fn drop_to_el1();
}

/// Where boot-time diagnostics go.
///
/// The UART when rivet owns the machine. When another OS does, that line
/// is its console, so this writes into the shared-memory ring instead
/// rather than cutting into someone else's output.
pub struct Diag;

#[cfg(not(feature = "amp"))]
impl core::fmt::Write for Diag {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        Pl011.write_str(s)
    }
}

#[cfg(feature = "amp")]
impl core::fmt::Write for Diag {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // SAFETY: the AMP entry maps the shared window before anything
        // can fault or panic.
        unsafe { crate::shmem::write_bytes(s.as_bytes()) };
        Ok(())
    }
}

impl Diag {
    /// Drain whatever the sink needs draining. A no-op for the ring.
    fn flush(&self) {
        #[cfg(not(feature = "amp"))]
        // SAFETY: waiting for the transmitter to go idle.
        unsafe {
            Pl011.flush()
        };
    }
}

/// Called from the exception vectors with the faulting EL's state.
///
/// Reports rather than recovers: the caller halts afterwards. Turning a
/// silent hang into a decoded exception is the whole point.
#[no_mangle]
pub extern "C" fn rust_fault_handler(esr: u64, elr: u64, far: u64, spsr: u64) {
    let ec = (esr >> 26) & 0x3f;
    let mut uart = Diag;
    let _ = write!(
        uart,
        "\n*** EXCEPTION ***\n\
         ESR  {esr:#018x}  (EC={ec:#04x} ISS={:#x})\n\
         ELR  {elr:#018x}\n\
         FAR  {far:#018x}\n\
         SPSR {spsr:#018x}\n",
        esr & 0x1ff_ffff,
    );

    // EC 0x24/0x25 is a data abort; data fault status code 0x35 within
    // it means an exclusive or atomic access the memory type cannot
    // support. With the MMU off everything is Device memory, which has
    // no exclusive monitor, so this is what an atomic read-modify-write
    // looks like on this SoC.
    if (ec == 0x24 || ec == 0x25) && (esr & 0x3f) == 0x35 {
        let _ = write!(
            uart,
            "  -> unsupported exclusive/atomic access: an atomic RMW ran with\n\
             \x20    the MMU off. Device memory has no exclusive monitor, so the\n\
             \x20    kernel cannot run here until an identity map exists.\n"
        );
    }
    uart.flush();
}

/// Default panic handler, reporting over the PL011.
///
/// Disable with `default-features = false` to supply your own, matching
/// how `rivet-rt` gates the same thing.
#[cfg(feature = "panic-handler")]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let mut uart = Diag;
    let _ = write!(uart, "\n*** PANIC: {info}\n");
    uart.flush();
    loop {
        // SAFETY: WFE is side-effect free.
        unsafe { core::arch::asm!("wfe", options(nomem, nostack)) };
    }
}
