//! Rivet RTOS — ARM Cortex-M0/M0+ (ARMv6-M) ISA port.
//!
//! A **separate crate** from `rivet-arch-cortex-m`, not a `#[cfg]` branch
//! inside it — ARMv6-M differs from ARMv7-M (M3/M4/M7) in ways that reach
//! every corner of a context-switch port, not just one or two symbols:
//!
//! - **No MPU.** Cortex-M0+ implementations (RP2040 included) ship
//!   without one. `rivet-arch-cortex-m`'s two-region stack-isolation
//!   design has nothing to program here — every Group A symbol that would
//!   touch it (`on_switch_to`, `scratch_open`/`scratch_close`,
//!   `guard_register`) is a plain no-op in this crate.
//! - **No DWT.** No cycle counter exists to probe; `cycle_count()` always
//!   returns [`systick::now_micros_precise`] directly — no probe-and-
//!   fall-back dance, because there is nothing to probe.
//! - **No `MemManage`/`BusFault`/`UsageFault`, no `SHCSR`.** ARMv6-M has
//!   exactly one fault handler (`HardFault`) for everything that would be
//!   a distinct, enable-able fault on ARMv7-M. `__rivet_arch_init` never
//!   touches `SCB.SHCSR` here — the field doesn't exist.
//! - **No 32-bit Thumb-2 `STM`/`LDM`.** ARMv6-M's 16-bit `STM`/`LDM`
//!   encoding only reaches registers r0-r7; the context-switch frame
//!   needs r4-r11. This crate's `PendSV` shuttles r8-r11 through r4-r7
//!   (`mov` between any register pair *is* available on ARMv6-M) with two
//!   4-register `stmia`/`ldmia` blocks instead of `rivet-arch-cortex-m`'s
//!   single 8-register one — see the asm block's own comments for the
//!   exact sequence.
//! - **No MPU means no reason for the SVC-vectored `init_task_stack`
//!   dance either** — `rivet-arch-cortex-m`'s version routes through an
//!   `svc` specifically so the frame write happens from Handler mode,
//!   where the MPU doesn't block it. With no MPU to work around, this
//!   crate's `__rivet_arch_init_task_stack` is a plain Rust function.
//!
//! Otherwise the same design as `rivet-arch-cortex-m`: tasks run in
//! Thread mode on PSP, exceptions run in Handler mode on MSP, PendSV
//! (lowest priority) does the actual save/restore/reschedule, SysTick
//! only requests one.

#![no_std]

#[cfg(target_feature = "vfp2")]
compile_error!(
    "rivet-arch-cortex-m0 targets ARMv6-M (Cortex-M0/M0+), which has no \
     FPU at all — this cfg firing means the wrong target triple was used"
);

#[cfg(feature = "nvic")]
pub mod nvic;
#[cfg(feature = "systick")]
pub mod systick;

/// Minimum task stack: the PendSV frame (32 bytes r4-r11 + 32 bytes
/// hardware-stacked r0-r3/r12/lr/pc/xPSR) plus slack for the entry
/// trampoline — identical shape to `rivet-arch-cortex-m`'s (the frame
/// layout is the same; only *how* PendSV populates it differs).
pub const MIN_TASK_STACK: usize = 64 + 64;

#[no_mangle]
extern "Rust" fn __rivet_arch_init() {
    // SCB.VTOR's reset value is architecturally 0x00000000 — see
    // `rivet-arch-cortex-m::__rivet_arch_init`'s identical comment for
    // why this must be set explicitly for any board whose vector table
    // isn't at address 0 (RP2040's flash-resident table, past the boot2
    // stage, is not).
    unsafe extern "C" {
        static __vector_table: u32;
    }
    // SAFETY: `SCB::PTR` is the statically-known System Control Block
    // base; `__vector_table` is a linker-defined symbol.
    unsafe {
        (*cortex_m::peripheral::SCB::PTR)
            .vtor
            .write(core::ptr::addr_of!(__vector_table) as u32);
    }

    // PendSV/SysTick to the lowest priority — same reasoning as
    // `rivet-arch-cortex-m`'s identical block, but ARMv6-M's `SCB.shpr`
    // is `[RW<u32>; 2]` (`shpr[0]` = SHPR2, `shpr[1]` = SHPR3 — whole
    // *words*, per the `cortex-m` crate itself), not ARMv7-M's flat
    // `[RW<u8>; 12]`. `SHPR3` still packs PendSV at byte 2 / SysTick at
    // byte 3 within that word, same as ARMv7-M — only the addressing
    // granularity changed, not the layout — so this is a
    // read-modify-write of `shpr[1]`'s top two bytes instead of two
    // independent byte writes. `SHCSR` (fault-enable bits) and the
    // MemManage/BusFault/UsageFault vectors it would enable don't exist
    // on this architecture at all, so — unlike `rivet-arch-cortex-m` —
    // there is no `shcsr` write here.
    //
    // SAFETY: `SCB::PTR` is the statically-known SCB base, valid on every
    // Cortex-M; these are volatile MMIO accesses to registers this
    // module exclusively owns.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        let mut shpr3 = scb.shpr[1].read();
        shpr3 |= 0xFF00_0000 // SysTick priority (SHPR3 byte 3)
            | 0x00FF_0000; // PendSV priority (SHPR3 byte 2)
        scb.shpr[1].write(shpr3);
    }

    // Floor every implemented IRQ to PendSV/SysTick's own 0xFF — same
    // reasoning as `rivet-arch-cortex-m`'s identical loop. Unlike
    // ARMv7-M, ARMv6-M's `IPR` is word-addressed (4 packed priority
    // bytes per word — see `nvic::set_priority`'s own doc), so filling
    // "every byte with 0xFF" means writing `0xFFFF_FFFF` words, not
    // `0xFF` words (which would only floor 1 of every 4 IRQs and leave
    // the other 3 at the *highest* priority, 0x00 — confirmed as a real
    // bug here before this comment, caught by the type checker: `ipr`'s
    // element type is `u32` on this target, not `u8`).
    //
    // SAFETY: `NVIC::PTR` is the statically-known NVIC base; IPR is
    // plain volatile MMIO, and this runs once, before any IRQ is enabled.
    unsafe {
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        for ipr in nvic.ipr.iter() {
            ipr.write(0xFFFF_FFFF);
        }
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    cortex_m::asm::wfi();
}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    MIN_TASK_STACK
}

/// No MPU on this architecture at all — see this crate's own module docs.
/// Still a real power of two: `rivet::preempt::stack_pool`'s layout math
/// needs *some* value even when nothing is actually enforced.
#[no_mangle]
extern "Rust" fn __rivet_arch_min_guard_size() -> usize {
    64
}

/// No DWT on ARMv6-M — always the SysTick-derived, sub-tick-precision
/// microsecond clock. See `systick::now_micros_precise`'s own docs for
/// why this is still meaningfully better than the plain tick-quantized
/// [`systick::now_micros`].
#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    #[cfg(feature = "systick")]
    {
        systick::now_micros_precise()
    }
    #[cfg(not(feature = "systick"))]
    {
        0
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_enable(_irq_num: u32) {
    #[cfg(feature = "nvic")]
    nvic::enable(_irq_num);
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_disable(_irq_num: u32) {
    #[cfg(feature = "nvic")]
    nvic::disable(_irq_num);
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_set_priority(_irq_num: u32, _priority: u8) {
    #[cfg(feature = "nvic")]
    nvic::set_priority(_irq_num, _priority);
}

/// RP2040 is dual-core (two Cortex-M0+), but this port only brings up
/// core 0 for now — same scope decision `rivet-arch-cortex-m` made for
/// every single-core Cortex-M board it supports. `rivet::config::MAX_HARTS`
/// stays at its default (1) for this arch until a real SMP bring-up
/// (separate boot stack + `SIO`-based inter-core FIFO/doorbell for
/// cross-core reschedule) is done — a substantial follow-up, not a
/// same-session addition.
#[no_mangle]
extern "Rust" fn __rivet_arch_hart_id() -> usize {
    0
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule_on(hart: usize) {
    debug_assert_eq!(hart, 0, "rivet-arch-cortex-m0: single-core (for now), hart must be 0");
    __rivet_arch_request_reschedule();
}

#[cfg(feature = "latency-histograms")]
static RESCHEDULE_REQUESTED_AT: rivet::sync::atomic::AtomicU32 = rivet::sync::atomic::AtomicU32::new(0);

/// Set PendSV pending. Single trigger for every context switch, whether
/// tick-driven or a voluntary yield — `SCB.ICSR` is at the same fixed
/// address on every Cortex-M.
#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    #[cfg(feature = "latency-histograms")]
    RESCHEDULE_REQUESTED_AT.store(
        __rivet_arch_cycle_count() as u32,
        rivet::sync::atomic::Ordering::Relaxed,
    );
    // SAFETY: `SCB::PTR` is the statically-known SCB base; `ICSR` write is
    // a volatile MMIO access.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.icsr.write(1 << 28); // PENDSVSET
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    // Same PRIMASK primitive as `rivet-arch-cortex-m` — baseline on every
    // Cortex-M including M0/M0+.
    let was_enabled = cortex_m::register::primask::read().is_active();
    cortex_m::interrupt::disable();
    was_enabled as usize
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(token: usize) {
    if token != 0 {
        // SAFETY: re-enabling interrupts only if they were enabled at the
        // matching `__rivet_arch_irq_save` call.
        unsafe { cortex_m::interrupt::enable() };
    }
}

/// No MPU on this architecture — nothing to reprogram on a context
/// switch.
#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(_stack_base: usize, _stack_size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(_guard_base: usize, _slot: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(_base: usize, _size: usize) {}

#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {}

// ── Preemptive tier: PendSV context switch ────────────────────────

/// Rust-side PendSV logic — identical role to `rivet-arch-cortex-m`'s.
#[no_mangle]
unsafe extern "C" fn rivet_pendsv_rust(interrupted_sp: usize) -> usize {
    #[cfg(feature = "latency-histograms")]
    {
        let requested_at = RESCHEDULE_REQUESTED_AT.load(rivet::sync::atomic::Ordering::Relaxed);
        let now = __rivet_arch_cycle_count() as u32;
        rivet::latency::record(
            rivet::latency::Kind::IrqEntry,
            now.wrapping_sub(requested_at) as u64,
        );
    }
    rivet::preempt::on_tick(interrupted_sp)
}

core::arch::global_asm!(
    ".section .text.rivet_task_exit",
    ".global rivet_task_exit",
    ".thumb_func",
    "rivet_task_exit:",
    "  bl   rivet_task_exit_core", // r0/r1 carry the return value
    "1:",
    "  b    1b",
);

core::arch::global_asm!(
    ".section .text.PendSV",
    ".global PendSV",
    ".thumb_func",
    "PendSV:",
    // Same one-word-push alignment fix as `rivet-arch-cortex-m`'s PendSV
    // (a lone `push {{lr}}` leaves sp 4-mod-8 across the `bl`).
    "  push {{lr}}",
    "  sub  sp, sp, #4",
    "  mrs  r0, psp",
    "  subs r0, r0, #32",
    // ARMv6-M's 16-bit STM/LDM only reaches r0-r7 — r4-r11 (the frame)
    // doesn't fit in one instruction like it does on ARMv7-M. Store the
    // real low half first, then shuttle r8-r11 through r4-r7 (`mov`
    // between any register pair, including r8+, *is* available on
    // ARMv6-M) for a second 4-register store. `stmia Rn!, {{..}}` writes
    // back Rn (Thumb-1 STM always does), which is exactly what's needed
    // to advance to the second block.
    "  stmia r0!, {{r4-r7}}",  // [frame+0,16) <- real r4-r7; r0 -> frame+16
    "  mov  r4, r8",
    "  mov  r5, r9",
    "  mov  r6, r10",
    "  mov  r7, r11",
    "  stmia r0!, {{r4-r7}}",  // [frame+16,32) <- r8-r11; r0 -> frame+32
    "  subs r0, r0, #32",       // r0 = frame base again (== interrupted_sp)
    "  bl   rivet_pendsv_rust", // r0 (arg+return) = frame base, old then new
    // Mirror image of the store: high block first (into scratch r4-r7,
    // then real r8-r11), so the low block can be loaded into real r4-r7
    // last without needing a second pointer register.
    "  adds r0, r0, #16",       // r0 -> new_frame+16 (high block)
    "  ldmia r0!, {{r4-r7}}",  // scratch <- [new_frame+16,32); r0 -> +32
    "  mov  r8, r4",
    "  mov  r9, r5",
    "  mov  r10, r6",
    "  mov  r11, r7",
    "  subs r0, r0, #32",       // r0 -> new_frame+0 (low block)
    "  ldmia r0!, {{r4-r7}}",  // real r4-r7 <- [new_frame+0,16); r0 -> +16
    "  adds r0, r0, #16",       // r0 -> new_frame+32 (hardware frame start)
    "  msr  psp, r0",
    "  add  sp, sp, #4",
    // ARMv6-M's `POP` can only load a saved value into `pc` (branching
    // immediately), never into `lr` — unlike ARMv7-M's Thumb-2 `POP`
    // encoding, which `rivet-arch-cortex-m`'s identical-looking
    // `pop {{lr}}` relies on. Popping into a low register and `mov`-ing
    // it into `lr` instead gets the same EXC_RETURN value into place
    // while leaving a labelable instruction (the `bx lr` below) as the
    // actual return, same as the ARMv7-M port's shape.
    "  pop  {{r3}}",
    "  mov  lr, r3",
    // Symbol for the GDB context-switch verification script (tests/gdb):
    // r4-r11 have been restored; frame base = psp - 32.
    ".global rivet_pendsv_resume",
    "rivet_pendsv_resume:",
    "  bx   lr",
);

// ── First task start / initial stack frame ────────────────────────

/// Set up the initial stack frame for a new task, then start the first
/// task's execution — identical to `rivet-arch-cortex-m`'s (no
/// ARMv6-M-specific instructions needed here: `msr control`/`isb`/`bx`
/// are all baseline).
#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(sp: usize) -> ! {
    // SAFETY: `sp` is the freshly-built initial frame of the first task;
    // PSP is set exactly once here, before any interrupt can fire.
    let frame = sp as *const u32;
    let arg = unsafe { core::ptr::read(frame.add(8)) };
    let entry_fn = unsafe { core::ptr::read(frame.add(14)) };

    unsafe {
        core::arch::asm!(
            "msr psp, {sp}",
            "movs r2, #2",
            "msr control, r2", // SPSEL=1 (use PSP in Thread mode), stay privileged
            "isb",
            sp = in(reg) sp,
            out("r2") _,
        );
    }

    #[cfg(feature = "systick")]
    systick::enable();

    // Same reasoning as `rivet-arch-cortex-m`'s identical call: the
    // caller's `critical_section` wrapper never returns into this
    // `-> !` function, so re-enabling interrupts here is this arch's
    // responsibility, not something that happens for free on unwind.
    unsafe {
        cortex_m::interrupt::enable();
    }

    unsafe {
        core::arch::asm!(
            "mov r0, {arg}",
            "bx {entry}",
            arg = in(reg) arg,
            entry = in(reg) entry_fn,
            options(noreturn)
        );
    }
}

/// Frame layout — identical to `rivet-arch-cortex-m`'s (see its own doc
/// comment for the full byte-offset table); only *which instructions*
/// `PendSV` uses to populate/consume it differs.
unsafe fn init_task_stack_impl(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    const FRAME_WORDS: usize = 16; // 8 (r4-r11) + 8 (hw frame)
    const STACK_ALIGN: usize = 16;

    // SAFETY: `stack` is a valid mutable slice of at least MIN_TASK_STACK
    // bytes (the caller guarantees this); the writes below initialize the
    // frame INSIDE the slice (at the top, aligned down).
    unsafe {
        let base = stack.as_mut_ptr() as usize;
        let top = base + stack.len();
        let frame_start = (top - FRAME_WORDS * 4) & !(STACK_ALIGN - 1);
        let frame = frame_start as *mut u32;

        for i in 0..FRAME_WORDS {
            core::ptr::write(frame.add(i), 0);
        }
        core::ptr::write(frame.add(8), arg as u32); // r0
                                                    // r1,r2,r3,r12 (words 9-12) stay 0
        extern "C" {
            fn rivet_task_exit();
        }
        core::ptr::write(frame.add(13), rivet_task_exit as *const () as usize as u32); // lr
        core::ptr::write(frame.add(14), entry_fn as u32); // pc
        core::ptr::write(frame.add(15), 0x0100_0000); // xPSR: Thumb bit (T=1) set

        frame_start
    }
}

/// Builds a new task's initial stack frame directly, from whatever mode
/// the caller is in — unlike `rivet-arch-cortex-m`, there is no MPU here
/// that would deny a Thread-mode write to another task's stack, so the
/// SVC-vectored Handler-mode dance that works around that isn't needed:
/// this is a plain Rust function, no `naked_asm!`, no `svc`.
#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
) -> usize {
    // SAFETY: caller guarantees `stack_ptr`/`stack_len` describe a valid,
    // exclusively-owned mutable byte slice at least `MIN_TASK_STACK` long.
    unsafe {
        init_task_stack_impl(
            core::slice::from_raw_parts_mut(stack_ptr, stack_len),
            entry_fn,
            arg,
        )
    }
}

/// Cortex-M system reset via SCB AIRCR SYSRESETREQ — architecturally
/// universal, identical to `rivet-arch-cortex-m::system_reset`.
pub fn system_reset() -> ! {
    // SAFETY: `0xE000ED0C` is the fixed SCB AIRCR register; writing
    // VECTKEY=0x05FA | SYSRESETREQ=1 requests a system reset.
    unsafe {
        core::ptr::write_volatile(0xE000_ED0C as *mut u32, 0x05FA_0004);
    }
    loop {
        core::hint::spin_loop();
    }
}
