//! Rivet RTOS — Xtensa LX7 ISA port (ESP32-S3, plan.md Phases 21/24).
//!
//! Implements the Group A (`rivet::port::arch`) symbol contract for the
//! `xtensa-esp32s3-none-elf` target: context switch, interrupt entry/exit,
//! critical section. Contains no board/MMIO knowledge (UART, watchdog,
//! SYSTIMER) — that is `rivet-bsp-esp32s3`'s job.
//!
//! # Why this crate depends on `xtensa-lx`/`xtensa-lx-rt`
//!
//! Unlike `rivet-arch-riscv`/`rivet-arch-cortex-m` (hand-written trap entry
//! assembly, verified against a QEMU machine model), this port targets
//! real hardware only — no Xtensa QEMU model is available in this
//! environment. Xtensa's exception/interrupt architecture (multiple
//! priority levels, hardware-assisted windowed-register spill/fill) is
//! substantially harder to get right blind than RISC-V's single `mtvec`
//! or Cortex-M's fixed vector table, with a real bricking risk and no fast
//! simulator iteration loop. `xtensa-lx`/`xtensa-lx-rt` are the direct
//! Xtensa analogs of the `riscv`/`cortex-m` crates the other two ports
//! depend on for CSR/register access — except here the boot-glue layer
//! (vector table, exception/interrupt entry assembly, `Reset`) is also
//! sourced from there rather than hand-written, because that is exactly
//! the layer where a mistake is most likely and least recoverable. Rivet's
//! own code starts where `xtensa-lx-rt` hands control to a Rust interrupt
//! handler with a full, already-correctly-saved `Context`.
//!
//! # Task switching: mutate the save frame, don't hand-write asm
//!
//! `xtensa_lx_rt::exception::Context` is the CPU's full saved state
//! (PC, PS, A0-A15, SAR, ...). `#[interrupt(3)]`'s handler receives
//! `&mut Context` pointing at a frame `xtensa-lx-rt`'s own assembly
//! allocated 256 bytes below whatever stack was live at interrupt time
//! (confirmed by reading `SAVE_CONTEXT`/`RESTORE_CONTEXT` in
//! `xtensa-lx-rt`'s source: `A1` is saved/restored as the genuine
//! *pre-interrupt* stack pointer). Whatever this handler leaves in that
//! struct is what gets resumed on return — so a context switch is: copy
//! the outgoing task's full state out, copy the incoming task's full
//! state in. No hand-written save/restore assembly needed for the switch
//! itself.
//!
//! Because the CPU's windowed-register overflow/underflow spill handling
//! is defined *relative to the current stack* (`A1`), not any global
//! state, switching `A1` to a different task's stack as part of restoring
//! its `Context` is sufficient on its own — the hardware spills/fills
//! each task's own window state to/from that task's own stack memory on
//! demand, already isolated by construction. `Context` does not persist
//! `WINDOWBASE`/`WINDOWSTART` — confirmed against `xtensa-lx-rt`'s own
//! struct definition, consistent with this being re-derivable hardware
//! state rather than architectural task state.
//!
//! # A task's first-ever dispatch does not go through a real interrupt
//!
//! A brand new task can be dispatched two different ways: via
//! [`__rivet_arch_start_first_task`] (the very first task the scheduler
//! ever runs, reached directly from boot) or, just as often, via the
//! ordinary tick/reschedule interrupt path (every *other* task spawned
//! before `rivet::run()`, since only one task can be "first"). Both cases
//! fabricate the same shape of `Context` — see [`fresh_task_context`] —
//! reached by jumping (never calling) into [`rivet_ptask_trampoline_impl`],
//! whose first instruction is `entry a1, 0`. This is exactly how
//! `xtensa-lx-rt`'s own `Reset` reaches `main` (a plain jump into
//! windowed code with `A1` pre-pointed at a fresh stack), and the one
//! thing a fabricated first dispatch does *not* need to get right — a
//! valid return address / caller frame — is never exercised, because the
//! trampoline never executes `retw`: task exit goes through an explicit
//! call to `rivet_task_exit_core`, never a return.
//!
//! # Distinguishing "never dispatched" from "has a saved `Context`"
//!
//! [`__rivet_arch_init_task_stack`] runs before the task has an id
//! (`rivet::preempt::spawn` calls it before registering the task), so it
//! cannot yet index into [`CONTEXTS`] (which is keyed by task id). Its
//! returned `sp` therefore encodes `(entry_fn, arg, stack_base,
//! stack_len)` directly — a *bootstrap marker*, not a `Context` index.
//! `sp` values of the two kinds are reliably distinguishable at every
//! point that needs to tell them apart: a task id is always
//! `< MAX_PTASKS` (a handful, in practice); a bootstrap marker is always
//! `>= MAX_PTASKS` by construction (see [`encode_bootstrap`]). The first
//! time a task is actually interrupted (by either path), its `Tcb.sp` is
//! permanently rewritten to its task id, and the bootstrap marker for it
//! is never consulted again.

#![no_std]
#![feature(asm_experimental_arch)]

use core::cell::UnsafeCell;

use xtensa_lx_rt::exception::Context;

pub mod timer;

/// Per-task saved CPU state, indexed by task id — arch-private, never
/// exposed to the kernel (which only ever sees `sp: usize` as an opaque,
/// arch-defined cookie, exactly as on the other two arches). Sized to
/// `rivet::preempt::tcb::MAX_PTASKS`; task ids are already bounded and
/// recycled by the kernel's own spawn/despawn lifecycle (Phase 17), so
/// this needs no separate allocation or freeing of its own.
const MAX_PTASKS: usize = rivet::preempt::tcb::MAX_PTASKS;

/// `rivet::preempt::spawn` always calls [`__rivet_arch_init_task_stack`]
/// *before* checking whether the ptask registry actually has room
/// (`rivet/src/preempt/mod.rs`'s `spawn`: `init_task_stack` happens inside
/// the first `critical::enter`, `tcb::register_full`'s capacity check
/// inside the second) — on RISC-V/Cortex-M that ordering is free, since
/// their `init_task_stack` has no capacity of its own to exhaust. Here it
/// does: a spawn that's ultimately rejected with `SpawnError::RegistryFull`
/// still permanently burns one [`BOOTSTRAPS`] slot (never reclaimed except
/// by dispatch). Tests that deliberately fill the registry then attempt
/// one more spawn to prove it's rejected (`stress_spawn`, `stress_max_ptasks`)
/// are exactly this pattern once — this headroom absorbs that without
/// requiring `MAX_PTASKS` itself to grow just to tolerate probing it.
const BOOTSTRAP_HEADROOM: usize = 4;
const BOOTSTRAP_CAPACITY: usize = MAX_PTASKS + BOOTSTRAP_HEADROOM;

struct ContextCell(UnsafeCell<Context>);
// SAFETY: every access happens from inside the level-3 interrupt handler,
// which cannot be re-entered on the *same* hart (same interrupt line,
// masked for its own duration) — but `CONTEXTS` is indexed by task id,
// not by hart, so that alone does not rule out two different harts each
// running their own level-3 handler and touching the same slot at once.
// The real guarantee is cross-hart: every access is wrapped in
// `rivet::critical::enter` (see `__level_3_interrupt`'s dispatch code),
// a genuine cross-hart spinlock, not just local interrupt masking — found
// missing on real hardware (a torn read of a same-task `Context` under
// concurrent save/restore) before that wrap was added.
unsafe impl Sync for ContextCell {}

static CONTEXTS: [ContextCell; MAX_PTASKS] =
    [const { ContextCell(UnsafeCell::new(Context::new())) }; MAX_PTASKS];

/// A never-dispatched task's bootstrap state, keyed by the same bump
/// index [`encode_bootstrap`] embeds in its returned `sp` — a small,
/// fixed-capacity side table (matching this workspace's usual pattern for
/// per-task arch-side bookkeeping), populated once by
/// [`__rivet_arch_init_task_stack`] and consumed at most once (by
/// whichever of [`__rivet_arch_start_first_task`] or the interrupt
/// handler's fabrication path reaches this task first).
#[derive(Clone, Copy)]
struct Bootstrap {
    stack_base: usize,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
}

struct BootstrapCell(UnsafeCell<Bootstrap>);
// SAFETY: each slot is written exactly once (by `init_task_stack`, under
// the kernel's own `critical::enter`, before the task is schedulable) and
// read exactly once (by whichever dispatch path reaches this task first);
// never concurrent.
unsafe impl Sync for BootstrapCell {}

static BOOTSTRAPS: [BootstrapCell; BOOTSTRAP_CAPACITY] = [const {
    BootstrapCell(UnsafeCell::new(Bootstrap {
        stack_base: 0,
        stack_len: 0,
        entry_fn: 0,
        arg: 0,
    }))
}; BOOTSTRAP_CAPACITY];

/// Bootstrap-table allocator state: a high-water mark (`next`, indices never
/// yet touched) plus a LIFO free list (`free`/`free_len`, indices consumed
/// and released back). A pure bump counter can't survive a long-running
/// soak test (`soak_smoke`) that spawns and despawns tasks thousands of
/// times — every consumed bootstrap marker (see [`Bootstrap`]'s docs: "the
/// bootstrap marker for it is never consulted again") is genuinely free to
/// reuse the moment the dispatch path that reads it is done, so this
/// recycles instead of growing without bound.
///
/// Guarded by [`BOOTSTRAP_LOCK`], a **raw** spinlock — not
/// `rivet::critical::enter`. Found the hard way, on real dual-core
/// hardware (plan.md Phase 25): `critical::enter`'s reentrant-nesting
/// machinery is several extra call frames deep, and unlike Cortex-M/
/// RISC-V (which run interrupts on a dedicated exception stack), Xtensa's
/// level-3 handler here runs on whatever stack was live at the moment of
/// the interrupt — the *interrupted task's own*. Freeing a bootstrap slot
/// from inside `__level_3_interrupt` therefore has every one of
/// `critical::enter`'s extra frames come directly out of that task's
/// stack budget, and `smp_test.rs`'s 512-byte worker stacks (already
/// tight — see the linker script's own `.task_stacks` sizing notes) don't
/// have room for it: reproduced as a genuine `stack-overflow` fault on a
/// task doing nothing but a tight counting loop. A raw two-instruction
/// spin (this is never held across a call more than a few lines long) is
/// cheap enough in frames that it doesn't reopen the same problem.
struct BootstrapAlloc {
    next: usize,
    free: [usize; BOOTSTRAP_CAPACITY],
    free_len: usize,
}

struct BootstrapAllocCell(UnsafeCell<BootstrapAlloc>);
unsafe impl Sync for BootstrapAllocCell {}

static BOOTSTRAP_ALLOC: BootstrapAllocCell = BootstrapAllocCell(UnsafeCell::new(BootstrapAlloc {
    next: 0,
    free: [0; BOOTSTRAP_CAPACITY],
    free_len: 0,
}));

static BOOTSTRAP_LOCK: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[inline(always)]
fn with_bootstrap_alloc<R>(f: impl FnOnce(&mut BootstrapAlloc) -> R) -> R {
    while BOOTSTRAP_LOCK
        .compare_exchange_weak(
            false,
            true,
            core::sync::atomic::Ordering::Acquire,
            core::sync::atomic::Ordering::Relaxed,
        )
        .is_err()
    {
        core::hint::spin_loop();
    }
    // SAFETY: `BOOTSTRAP_LOCK` gives exclusive access for the duration of `f`.
    let r = f(unsafe { &mut *BOOTSTRAP_ALLOC.0.get() });
    BOOTSTRAP_LOCK.store(false, core::sync::atomic::Ordering::Release);
    r
}

/// Claim a bootstrap-table index, reusing a freed one when available.
fn alloc_bootstrap_index() -> usize {
    with_bootstrap_alloc(|a| {
        if a.free_len > 0 {
            a.free_len -= 1;
            a.free[a.free_len]
        } else {
            let i = a.next;
            assert!(
                i < BOOTSTRAP_CAPACITY,
                "rivet-arch-xtensa: more tasks spawned ({}) than the bootstrap table's capacity \
                 ({}, MAX_PTASKS {} + headroom {}) will ever fit — every task must be dispatched \
                 (interrupted) at least once to free its slot, or BOOTSTRAP_HEADROOM must be raised",
                i + 1,
                BOOTSTRAP_CAPACITY,
                MAX_PTASKS,
                BOOTSTRAP_HEADROOM
            );
            a.next = i + 1;
            i
        }
    })
}

/// Release a bootstrap-table index once its [`Bootstrap`] has been consumed
/// (fabricated into a real `Context`) and will never be read again.
fn free_bootstrap_index(index: usize) {
    with_bootstrap_alloc(|a| {
        debug_assert!(a.free_len < BOOTSTRAP_CAPACITY, "double-free of a bootstrap index");
        a.free[a.free_len] = index;
        a.free_len += 1;
    })
}

/// Tag a bootstrap-table index so it can never be confused with a real
/// task id (`0..MAX_PTASKS`) by any code that later sees this value as
/// `sp`: shifted up and out of that range entirely, matching how
/// `Tcb.sp` for an already-interrupted-at-least-once task is a bare task
/// id (see the module docs).
fn encode_bootstrap(index: usize) -> usize {
    MAX_PTASKS + index
}

fn decode_bootstrap(sp: usize) -> Option<usize> {
    sp.checked_sub(MAX_PTASKS)
}

/// Build the `Context` for a task that has never run before: `PC` points
/// at [`rivet_ptask_trampoline_impl`]'s first instruction (`entry a1,
/// 32`), `A1` at the top of its own, real, full-size stack (the whole
/// allocation — unlike the other two arches, nothing needs to be reserved
/// out of it for a saved frame, since that lives in [`CONTEXTS`] instead),
/// and `A2`/`A3` carry `arg`/`entry_fn` directly. This `Context` is
/// restored via `xtensa-lx-rt`'s own interrupt-exit assembly + `rfi`, not
/// a `call`, so nothing rotates the window on the way in; `PS_WOE` sets
/// `PS.WOE` (window overflow/underflow checking enabled, needed for the
/// task to make any windowed calls of its own) but leaves `PS.CALLINC` at
/// 0, meaning the trampoline's own `entry` rotates by *zero* registers —
/// its post-`entry` A2/A3 are exactly whatever was restored into A2/A3
/// here (unlike [`__rivet_arch_start_first_task`]'s manual bootstrap,
/// which reaches the trampoline via a real `call4` and so *does* need
/// its own +4-rotated A6/A7 — a genuinely different path with a
/// genuinely different convention, not the same rule applied twice).
fn fresh_task_context(b: &Bootstrap) -> Context {
    let mut ctx = Context::new();
    let top = (b.stack_base + b.stack_len) & !0xF;
    ctx.PC = rivet_ptask_trampoline_impl as *const () as usize as u32;
    ctx.PS = timer::PS_WOE;
    ctx.A1 = top as u32;
    ctx.A2 = b.arg as u32;
    ctx.A3 = b.entry_fn as u32;
    ctx
}

// ── Critical section / interrupt masking ───────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_save() -> usize {
    xtensa_lx::interrupt::disable() as usize
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_restore(token: usize) {
    // SAFETY: `token` is a mask previously returned by `disable()` (via
    // `irq_save`); restoring through the same `set_mask` the crate itself
    // uses for save/restore pairing, not calling `enable()`
    // unconditionally, is what keeps nested critical sections composing
    // correctly.
    unsafe {
        xtensa_lx::interrupt::set_mask(token as u32);
    }
}

// ── Misc Group A ─────────────────────────────────────────────────────

#[no_mangle]
extern "Rust" fn __rivet_arch_init() {
    // `VECBASE` is a genuine per-core SFR — `Reset` only sets it for
    // PRO_CPU (hart 0); APP_CPU's own entry point (`rivet_appcpu_entry`,
    // reached directly from the boot ROM, never through `Reset` at all —
    // see that function's docs) never touches it, so it must be set here
    // instead. Redundant-but-harmless on hart 0 (same value `Reset`
    // already wrote).
    unsafe extern "C" {
        static _init_start: u32;
    }
    unsafe {
        xtensa_lx::set_vecbase(core::ptr::addr_of!(_init_start) as *const u32);
    }

    // Cross-core IPI (plan.md Phase 24): route the *other* core's
    // "wake me up and reconsider what to run" signal to this core's own
    // level-3 line 23 (an `ExternLevel` type, level-3 line per
    // `xtensa-lx-rt`'s own `config/esp32s3.rs` — not otherwise used by
    // this port, distinct from Timer1/Software1). Each core maps only
    // the *one* `FROM_CPU_INTR` source the *other* core is expected to
    // signal it through, so a core writing its own outgoing register
    // never re-interrupts itself. `esp-hal`'s own `interrupt::map_raw`
    // does the identical `core_N_intr_map(source).map().bits(line)`
    // write for the same reason.
    // `CORE_n_INTR_MAP` has no named sub-fields (a raw whole-register
    // value in the SVD, matching `esp-hal`'s own `.map().bits(...)` call
    // shape conceptually, but this PAC version exposes no `map()`
    // accessor — write the raw bits directly instead).
    unsafe {
        if __rivet_arch_hart_id() == 0 {
            (*esp32s3::INTERRUPT_CORE0::ptr())
                .core_0_intr_map(ipi::FROM_APP_CPU_SOURCE)
                .write(|w| w.bits(ipi::IPI_CPU_LEVEL as u32));
        } else {
            (*esp32s3::INTERRUPT_CORE1::ptr())
                .core_1_intr_map(ipi::FROM_PRO_CPU_SOURCE)
                .write(|w| w.bits(ipi::IPI_CPU_LEVEL as u32));
        }
    }

    // Root cause (plan.md Phase 24), found on real dual-core hardware:
    // `INTENABLE` is a genuine per-core SFR (like `VECBASE` above) —
    // `rivet::init()`'s `tick_start` call (which unmasks Software1, and
    // Timer1 for whichever core owns the periodic tick) only ever runs
    // on hart 0 (`run_secondary_hart` deliberately skips `rivet::init()`
    // entirely — see its own doc comment). APP_CPU's own `INTENABLE`
    // therefore started, and stayed, all-zero: the moment
    // `start_secondary_hart`'s loop found nothing ready and called
    // `__rivet_arch_idle` (`waiti 0`), it blocked forever — with
    // *nothing* enabled to ever wake it, this is indistinguishable from
    // a hang (confirmed: a real two-core test reached this exact point,
    // printed nothing further, PRO_CPU included — its own dispatch was
    // never actually stuck, but had nothing left to observably print
    // once its own tasks were running quietly). Software1 (self-IPI) is
    // needed on *every* core; the cross-core IPI line only strictly
    // needs enabling on APP_CPU, but enabling it on both is harmless —
    // each core's own interrupt matrix (just above) only ever routes the
    // *one* source the other core is expected to signal it through.
    unsafe {
        xtensa_lx::interrupt::enable_mask(timer::SOFTWARE1_MASK | ipi::IPI_MASK);
    }
}


#[no_mangle]
extern "Rust" fn __rivet_arch_idle() {
    // SAFETY: `waiti 0` is always safe — it pauses until any enabled,
    // unmasked interrupt occurs, per the Xtensa ISA.
    unsafe {
        core::arch::asm!("waiti 0", options(nomem, nostack));
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_min_task_stack() -> usize {
    // No frame is reserved out of the task's own stack (see
    // `fresh_task_context`'s docs) — this only needs to cover real call
    // depth: the entry trampoline plus whatever the task's own code needs.
    256
}

/// No hardware guard mechanism on this arch at all (`guard_register` is a
/// no-op stub — stack-overflow detection here is purely the kernel's own
/// software watermark check) — unused for actual protection, still a
/// real power of two matching the historical guard size, since
/// `rivet::preempt::stack_pool`'s layout math needs *some* value.
#[no_mangle]
extern "Rust" fn __rivet_arch_min_guard_size() -> usize {
    64
}

#[no_mangle]
extern "Rust" fn __rivet_arch_cycle_count() -> u64 {
    // Xtensa's CCOUNT is a genuine 32-bit-only hardware register (no
    // wider pair exists, unlike RISC-V's mcycle/mcycleh) — zero-extending
    // it matches exactly how Cortex-M's 32-bit DWT CYCCNT is already
    // handled elsewhere in this workspace: every caller only ever takes
    // wrapping deltas, so the ~26.8s rollover period at typical clock
    // rates is invisible to them.
    xtensa_lx::timer::get_cycle_count() as u64
}

#[no_mangle]
extern "Rust" fn __rivet_arch_hart_id() -> usize {
    // `xtensa_lx::get_processor_id()` reads the `PRID` special register
    // (`rsr.prid`) — despite the name, this is *not* a small 0/1 core
    // index. On real hardware PRO_CPU read back `0xCDCD` (52685), which
    // an earlier version of this function treated as raw garbage and
    // hardcoded to 0 to work around (plan.md Phase 22). It wasn't
    // garbage: per Espressif's own convention (matching `esp-hal`'s
    // `Cpu::current()`), the *only* meaningful bit is bit 13 (`0x2000`)
    // — clear on PRO_CPU, set on APP_CPU. `0xCDCD & 0x2000 == 0`, i.e.
    // it correctly read as PRO_CPU all along; the crash it caused
    // (out-of-bounds into `rivet::critical`'s per-hart array) was from
    // using the *raw* register value as an index, not from the bit
    // itself being wrong. Masking to just that bit makes this correct
    // for both single-core (always 0, APP_CPU never released) and
    // dual-core (plan.md Phase 24).
    if xtensa_lx::get_processor_id() & 0x2000 != 0 {
        1
    } else {
        0
    }
}

/// Cross-core IPI (plan.md Phase 24): direct analog of
/// `rivet-arch-riscv::clint`'s `MSIP`-based `msip()`/`msip_for()` split —
/// a self-target uses the cheap local software interrupt, a genuinely
/// different core needs the SoC's real cross-core signal path.
mod ipi {
    /// `FROM_CPU_INTR0` (peripheral interrupt source 79): the register
    /// PRO_CPU writes to signal APP_CPU, and the only one APP_CPU's own
    /// interrupt matrix maps to a live line — confirmed against the PAC's
    /// own `Interrupt` enum (`FROM_CPU_INTR0 = 79`).
    pub const FROM_PRO_CPU_SOURCE: usize = 79;
    /// `FROM_CPU_INTR1` (peripheral interrupt source 80): the register
    /// APP_CPU writes to signal PRO_CPU, and the only one PRO_CPU's own
    /// interrupt matrix maps to a live line (`FROM_CPU_INTR1 = 80`).
    pub const FROM_APP_CPU_SOURCE: usize = 80;
    /// CPU interrupt line 23: level 3, `ExternLevel` (peripheral-routable)
    /// per `xtensa-lx-rt`'s own `config/esp32s3.rs` — deliberately not 15
    /// (Timer1) or 29 (Software1), both already owned by this port's
    /// existing tick/self-reschedule mechanism.
    pub const IPI_CPU_LEVEL: u8 = 23;
    /// `xtensa_lx::interrupt::get()`'s pending-bitmask bit for the same
    /// line, for `__level_3_interrupt`'s own pending check.
    pub const IPI_MASK: u32 = 1 << (IPI_CPU_LEVEL as u32);
}

/// Board-routed peripheral interrupts (plan.md Phase 25: `rivet::irq`).
///
/// The interrupt matrix maps a *peripheral source* (the `irq_num` the
/// board-defined `rivet-bsp-esp32s3::irq` constants and `rivet::irq`'s
/// port contract both use — matching a real `esp32s3::Interrupt` enum
/// discriminant, e.g. `UART0 = 27`) to one of 32 *CPU interrupt lines*.
/// This is a genuinely different namespace from the line number below
/// (which is also, coincidentally, 27) — see the constant's own doc.
mod periph_irq {
    /// CPU interrupt line 27: level 3, `ExternLevel`, peripheral-routable,
    /// per `xtensa-lx-rt`'s own `config/esp32s3.rs` — the only other
    /// level-3 `ExternLevel` line besides 23 (already owned by the
    /// cross-core IPI), so it's what's left for a generic board-registered
    /// peripheral interrupt. **Numerically coincidental** with
    /// `esp32s3::Interrupt::UART0`'s source number (also 27) — the two
    /// numbers are unrelated; nothing here depends on them matching.
    pub const LINE: u32 = 27;
    pub const MASK: u32 = 1 << LINE;
    /// esp-hal's own sentinel for "not routed to any line" (a real Timer-
    /// type line the interrupt matrix can't usefully route a peripheral
    /// source to, making the mapping inert) — matched here for the same
    /// reason, not independently derived.
    pub const DISABLED_TARGET: u32 = 16;

    /// Only one peripheral source can be live on line 27 at a time in
    /// this minimal implementation (this port doesn't yet fan a single
    /// CPU line out to multiple simultaneously-registered sources the way
    /// a real PLIC/NVIC claim register would) — good enough to prove the
    /// real vector-table-to-handler chain end to end (this phase's actual
    /// acceptance bar), not a general-purpose multi-source IRQ subsystem.
    /// Per-core: each core's interrupt matrix is independent.
    pub static CURRENT_SOURCE: [core::sync::atomic::AtomicU32; rivet::config::MAX_HARTS] =
        [const { core::sync::atomic::AtomicU32::new(u32::MAX) }; rivet::config::MAX_HARTS];
}

/// Cycle timestamp of the most recent reschedule request per hart
/// (plan.md Phase 12 precedent, extended here to Xtensa): set at every
/// `__rivet_arch_request_reschedule`/`_on` call, read back in
/// `__level_3_interrupt`'s Software1 branch to record
/// [`rivet::latency::Kind::IrqEntry`] — the same "request to actual
/// handler entry" latency `rivet-arch-cortex-m`'s `RESCHEDULE_REQUESTED_AT`
/// measures for PendSV. Per-hart (unlike Cortex-M, which is single-core):
/// hart 0 requesting a reschedule *on* hart 1 writes index 1 from hart 0's
/// context, so this must be a real shared array, not hart-local storage.
#[cfg(feature = "latency-histograms")]
static RESCHEDULE_REQUESTED_AT: [core::sync::atomic::AtomicU32; rivet::config::MAX_HARTS] =
    [const { core::sync::atomic::AtomicU32::new(0) }; rivet::config::MAX_HARTS];

#[cfg(feature = "latency-histograms")]
fn stamp_reschedule_requested(hart: usize) {
    RESCHEDULE_REQUESTED_AT[hart].store(
        rivet::port::arch::cycle_count() as u32,
        core::sync::atomic::Ordering::Relaxed,
    );
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule_on(hart: usize) {
    #[cfg(feature = "latency-histograms")]
    stamp_reschedule_requested(hart);
    if hart == __rivet_arch_hart_id() {
        // SAFETY: identical to `__rivet_arch_request_reschedule`'s own
        // self-IPI — this *is* that path, taken explicitly rather than
        // via the wrapper below to make the self-vs-other split visible
        // at this call site too.
        unsafe {
            xtensa_lx::interrupt::set(timer::SOFTWARE1_MASK);
        }
        return;
    }
    // SAFETY: `SYSTEM.cpu_intr_from_cpu(n)` is a real, fixed-address SoC
    // register; setting `cpu_intr` pends the source the *other* core's
    // `__rivet_arch_init` mapped to its own level-3 line 23 — this core's
    // own interrupt matrix (if it happens to also map the same source,
    // which `__rivet_arch_init` deliberately never does) is unaffected.
    unsafe {
        let system = &*esp32s3::SYSTEM::ptr();
        let source = if hart == 0 {
            ipi::FROM_APP_CPU_SOURCE
        } else {
            ipi::FROM_PRO_CPU_SOURCE
        };
        // `cpu_intr_from_cpu(n)` is indexed by the `FROM_CPU_INTRn`
        // register number (0 or 1 here), not the target hart — the
        // register that signals hart 0 is index 1 (`FROM_CPU_INTR1`,
        // the one hart 0 maps) and vice versa; `source` above already
        // encodes the *peripheral interrupt source number* (79/80), so
        // the register index is just `source - 79`.
        system
            .cpu_intr_from_cpu(source - ipi::FROM_PRO_CPU_SOURCE)
            .write(|w| w.cpu_intr().set_bit());
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_enable(irq_num: u32) {
    let hart = __rivet_arch_hart_id();
    periph_irq::CURRENT_SOURCE[hart].store(irq_num, core::sync::atomic::Ordering::Release);
    // SAFETY: `INTERRUPT_COREn` is a real, fixed-address SoC register
    // block; routing `irq_num` (a peripheral source, matching a real
    // `esp32s3::Interrupt` discriminant) to `periph_irq::LINE` on this
    // core only, matching the same per-core matrix pattern already used
    // for the cross-core IPI in `__rivet_arch_init`.
    unsafe {
        if hart == 0 {
            (*esp32s3::INTERRUPT_CORE0::ptr())
                .core_0_intr_map(irq_num as usize)
                .write(|w| w.bits(periph_irq::LINE));
        } else {
            (*esp32s3::INTERRUPT_CORE1::ptr())
                .core_1_intr_map(irq_num as usize)
                .write(|w| w.bits(periph_irq::LINE));
        }
        xtensa_lx::interrupt::enable_mask(periph_irq::MASK);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_disable(irq_num: u32) {
    let hart = __rivet_arch_hart_id();
    // Only actually mask the CPU line if `irq_num` is still the source
    // currently routed to it — a stale `disable` for an already-replaced
    // registration must not clobber whatever's live now.
    let current = periph_irq::CURRENT_SOURCE[hart].load(core::sync::atomic::Ordering::Acquire);
    if current != irq_num {
        return;
    }
    // SAFETY: same register block as `irq_enable`; routing to the
    // documented "disabled" sentinel target makes the mapping inert.
    unsafe {
        if hart == 0 {
            (*esp32s3::INTERRUPT_CORE0::ptr())
                .core_0_intr_map(irq_num as usize)
                .write(|w| w.bits(periph_irq::DISABLED_TARGET));
        } else {
            (*esp32s3::INTERRUPT_CORE1::ptr())
                .core_1_intr_map(irq_num as usize)
                .write(|w| w.bits(periph_irq::DISABLED_TARGET));
        }
        xtensa_lx::interrupt::disable_mask(periph_irq::MASK);
    }
}

#[no_mangle]
extern "Rust" fn __rivet_arch_irq_set_priority(_irq_num: u32, _priority: u8) {
    // Not implemented (plan.md Phase 25): this minimal port routes every
    // board-registered peripheral source through the single shared
    // level-3 line (see `periph_irq`'s docs), so there is no per-source
    // priority to set yet — matching Cortex-M's own default (a no-op
    // unless the `nvic` feature, which has real per-IRQn priority
    // registers, is enabled).
}

#[no_mangle]
extern "Rust" fn __rivet_arch_request_reschedule() {
    #[cfg(feature = "latency-histograms")]
    stamp_reschedule_requested(__rivet_arch_hart_id());
    // SAFETY: `set` with the Software1 bit (INT29, level 3 — see
    // `timer.rs`'s module docs) is the documented mechanism for pending a
    // software interrupt; only valid for software/edge-triggered
    // interrupts, which Software1 is.
    unsafe {
        xtensa_lx::interrupt::set(timer::SOFTWARE1_MASK);
    }
}

// ── APP_CPU (secondary hart) boot glue (plan.md Phase 24) ───────────
//
// Unlike PRO_CPU (which always boots through the ESP-IDF bootloader into
// `xtensa-lx-rt`'s `Reset` — bss/data init, `VECBASE`, the works), APP_CPU
// is released directly by writing a raw entry-point address to the boot
// ROM (`ets_set_appcpu_boot_addr`) and pulsing its reset/clock-gate/
// runstall bits (`rivet-bsp-esp32s3`'s job — a board-level fact, matching
// how `rivet-rt`'s RISC-V `_start` is the boot-glue layer, not this arch
// crate). Nothing about APP_CPU's arrival here is windowed-ABI-safe by
// default: there is no real "caller" to inherit a valid stack or
// `PS.CALLINC` from. Fully naked on purpose — every other bootstrap path
// in this crate that got this wrong (the `entry`/rotation bugs, plan.md
// Phase 22/23) did so via an *implicit* assumption about a caller's
// state; this one has no caller at all, so it sets up everything itself
// before the first real (`call4`, `CALLINC = 1`) windowed call.
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".literal .Lappcpu_stack_lit, _appcpu_stack_top",
    ".global rivet_appcpu_entry",
    "rivet_appcpu_entry:",
    "  l32r a1, .Lappcpu_stack_lit",
    "  call4 rivet_appcpu_rust_entry",
    "1:",
    "  j 1b",
);

#[no_mangle]
extern "C" fn rivet_appcpu_rust_entry() -> ! {
    // SAFETY: the very first thing to run on this core — nothing else
    // has touched its interrupt/timer state yet. Mirrors `esp-hal`'s own
    // `start_core1_init` (mask interrupts, zero the compare registers
    // this port's `timer` module might otherwise misread as already
    // armed) before anything else runs.
    unsafe {
        xtensa_lx::interrupt::set_mask(0);
        xtensa_lx::timer::set_ccompare0(0);
        xtensa_lx::timer::set_ccompare1(0);
        xtensa_lx::timer::set_ccompare2(0);
    }
    // Root cause (plan.md Phase 29), found on real hardware: this
    // function's own comment used to claim `rivet::run_secondary_hart()`
    // "spins on `rivet::kernel_ready()` before entering the scheduler" —
    // it never actually did (its real contract, per `rivet/src/lib.rs`,
    // is "call only after `kernel_ready()` is true" — the *caller*'s
    // responsibility, which `rivet-rt`'s RISC-V secondary-hart boot
    // upholds with an explicit wait loop and this port never had).
    // Without it, APP_CPU — released from `__rivet_board_init`, i.e.
    // inside `rivet::init()`, long before the app's `main()` has finished
    // its own `spawn_ptask!` calls — could reach `start_secondary_hart()`
    // and dispatch a task that had *just* become ready (a `spawn_ptask!`'s
    // own `ready_add` broadcasts a wake IPI unconditionally, whether or
    // not the kernel has actually started yet) while hart 0 was still
    // mid-spawn of a *different* task, corrupting `Tcb.sp` for whichever
    // task each hart ended up racing on.
    //
    // A first attempt at this fix (plain `Acquire`/`Release` ordering on
    // `KERNEL_READY`, matching every other atomic in this codebase) did
    // stop the corruption but introduced a *worse* regression: `smp_test.
    // rs` (independent per-task counters, no mutex contention) hung
    // indefinitely — hart 1 apparently spinning forever without ever
    // observing hart 0's write. Upgrading `KERNEL_READY`'s store/load to
    // `SeqCst` (see `rivet::kernel_ready`/`rivet::run`) closed that gap:
    // both `smp_test.rs` and `smp_latency_bench`'s forced-cross-core
    // scenario pass with this combination, where either fix alone did
    // not. Not fully explained why plain Acquire/Release wasn't
    // sufficient here specifically — this session's own real-time
    // characterization work (`docs/realtime.md` §10) independently found
    // reason to distrust plain-ordered cross-core atomic visibility on
    // this SoC/toolchain combination for a *different* piece of shared
    // state, so this may be the same underlying gap; flagged as a real
    // open question about this port's atomics rather than claimed as
    // fully understood.
    while !rivet::kernel_ready() {
        core::hint::spin_loop();
    }
    // plan.md Phase 30 (round-robin fairness): APP_CPU has no periodic
    // tick of its own — three fix shapes were tried on real hardware:
    // (1) giving APP_CPU its own independent `CCOMPARE1` running this
    // function's *entire* tick body (including `watchdog`/`poll_timers`)
    // fixed the fairness gap but broke `smp_latency_bench` by doubling
    // those hart-0-owned duties' rate across both cores; (2) the same
    // idea with those two calls guarded to hart 0 only hit a *different*,
    // immediate real-hardware panic at boot (`Interrupt: 1`, an unhandled
    // level-1 interrupt — likely `esp-hal`'s own per-core interrupt setup
    // not covering APP_CPU the way it covers PRO_CPU, not investigated
    // further given time cost); (3) periodically broadcasting the
    // existing cross-hart reschedule IPI from PRO_CPU's own tick (see
    // `timer::on_timer_irq`'s own comment) is what's actually shipped —
    // the only one of three fundamentally different designs that passed
    // real-hardware verification (18+ consecutive clean runs across all
    // three dual-core tests). See `timer::on_timer_irq` for the full
    // account and plan.md's Phase 30 notes for the complete history.
    rivet::run_secondary_hart();
}

// Memory-guard hooks: scoped out of v1 (plan.md Phase 21 — "not every arch
// ships every guard on day one", matching Cortex-M's own PMP-shaped stub).
#[no_mangle]
extern "Rust" fn __rivet_arch_guard_register(_guard_base: usize, _slot: usize) {}
#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_open(_base: usize, _size: usize) {}
#[no_mangle]
extern "Rust" fn __rivet_arch_scratch_close() {}
#[no_mangle]
extern "Rust" fn __rivet_arch_on_switch_to(_stack_base: usize, _stack_size: usize) {
    // No reprogrammable memory guard yet (see above) — nothing to do here.
}

// ── Preemptive tier: stack bootstrap ──────────────────────────────

// `l32r`'s immediate is a PC-relative *backward-only* offset with a
// limited range — it needs a real Xtensa literal-pool entry (the `.literal
// name, value` directive `xtensa-lx-rt`'s own boot asm uses for exactly
// this, e.g. `.literal sym_main, {main}`), not an ad-hoc `.word` label:
// using a plain `.word` here was tried first and produced a genuinely
// wrong address at link time (confirmed on real hardware — the resulting
// `l32r` loaded a garbage pointer into an unrelated ROM symbol's address
// range, taken by `callx4` straight into an `InstrProhibited` exception).
core::arch::global_asm!(
    ".section .text",
    ".align 4",
    ".literal .Lexit_core_lit, rivet_task_exit_core",
    ".global rivet_ptask_trampoline_impl",
    "rivet_ptask_trampoline_impl:",
    "  entry a1, 32",
    // a2 = arg, a3 = entry_fn (already in the right slots post-rotation
    // from whichever bootstrap path jumped/called in here — see
    // `__rivet_arch_start_first_task`'s and `fresh_task_context`'s own
    // docs). Root cause (plan.md Phase 23), found on real hardware: this
    // used to fall straight through to `callx4 a3` with arg still sitting
    // in a2 — but `callx4`'s *own* window rotation means the callee
    // (`entry_fn`) receives its first argument via the CALLER's a6, not
    // a2 (the outgoing-arg registers for a call4 are a6..a11, not
    // a2..a5 — a2 only looks like "the argument register" from the
    // callee's own post-rotation point of view). Forwarding a2 into a6
    // here is what actually places `arg` where `entry_fn`'s own `entry`
    // will find it. Confirmed via `objdump`+hardware: without this,
    // `entry_fn` read pure garbage through its arg pointer (`0xfffffff0`
    // — the literal `& !0xF` alignment-mask immediate this same function
    // materializes a few lines up, left sitting in a physical register
    // seven windows back purely by leftover-register coincidence) and
    // faulted on its very first instruction. Also widened the frame
    // reservation from 0 to 32 bytes: an `entry a1, 0` trampoline shares
    // its raw window with whichever caller jumped in, which is fragile
    // once this trampoline itself has live state (`a6`) to protect
    // across the `callx4`.
    "  mov a6, a2",
    "  callx4 a3",
    // entry_fn(arg) returned; its <=8-byte result is back in a2/a3 (the
    // windowed return convention this workspace already assumes for this
    // exact shape — see rivet-arch-riscv's identical assumption for its
    // own entry trampoline).
    "  l32r  a4, .Lexit_core_lit",
    "  callx4 a4",
    "1:",
    "  j 1b",
);

extern "C" {
    // Only referenced from the `global_asm!` text above (`.word
    // rivet_task_exit_core`), invisible to Rust's usage analysis.
    #[allow(dead_code)]
    fn rivet_task_exit_core(lo: usize, hi: usize) -> !;
    fn rivet_ptask_trampoline_impl();
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_init_task_stack(
    stack_ptr: *mut u8,
    stack_len: usize,
    entry_fn: usize,
    arg: usize,
) -> usize {
    let index = alloc_bootstrap_index();
    // SAFETY: `index < BOOTSTRAP_CAPACITY`, `alloc_bootstrap_index` hands out
    // each index exactly once (fresh or recycled), and this slot is
    // otherwise untouched until consumed.
    unsafe {
        *BOOTSTRAPS[index].0.get() = Bootstrap {
            stack_base: stack_ptr as usize,
            stack_len,
            entry_fn,
            arg,
        };
    }
    encode_bootstrap(index)
}

#[no_mangle]
unsafe extern "Rust" fn __rivet_arch_start_first_task(sp: usize) -> ! {
    // Root cause (plan.md Phase 23), found on real hardware: a genuinely
    // real interrupt race, only exposed by a test with *two* same-
    // priority tasks spawned before `run()` (a single task never hits
    // it). `rivet::preempt::start()` calls `sched::set_current`/
    // `on_dispatch` for the chosen "first" task *inside* a
    // `critical::enter`, but that critical section ends (interrupts
    // re-enabled) before this function's own bootstrap asm ever runs —
    // and `timer::tick_start`'s own doc comment turned out to be
    // aspirational, not actual: it assumes ticks can't fire until a
    // task's `Context` (with `PS.INTLEVEL = 0`) is restored, but
    // `PS.INTLEVEL` is already 0 from `Reset` onward on this port (never
    // explicitly raised), so `INTENABLE`'s per-source bits are the *only*
    // real gate — and once `tick_start` sets them (in `rivet::init()`,
    // long before `run()`), a tick can land in the small-but-nonzero gap
    // between `critical::enter` releasing and this function's own jump
    // into the trampoline. `sched::current()` was already set to
    // "first" at that point, so `__level_3_interrupt` would treat this
    // function's own in-progress, not-yet-bootstrapped execution state as
    // "first"'s legitimate resumable context — corrupting it — while
    // (with 2+ same-priority tasks) switching away to dispatch a
    // *different* task via `fresh_task_context`, whose otherwise-correct
    // state then gets discarded/never actually reached because the
    // interrupted "first" bootstrap never got to run at all. Fixed by
    // masking interrupts for the entire remainder of this function (they
    // were already fully masked, just via a different mechanism, for the
    // scheduling decision itself) and only re-enabling the two sources
    // `tick_start` armed as literally the last action before the jump —
    // closing the window completely rather than narrowing it.
    //
    // Update (plan.md Phase 29): a related but distinct dual-core variant
    // of this same failure mode was found and fixed separately, in
    // `rivet::preempt::start`/`start_secondary_hart` themselves — those
    // functions used two *separate* critical sections back-to-back (the
    // scheduling decision, then this dispatch), leaving a real gap where
    // local interrupts were briefly re-enabled between them, on real
    // dual-core hardware. See their own doc comments for the fix. A
    // second, independent contributor to the same symptom was also found
    // and fixed: `rivet::console`'s polling write path had no cross-hart
    // lock, so two harts printing concurrently (including a fault dump
    // racing this exact bug's own diagnostics) corrupted or lost each
    // other's output — see `rivet::console::write_bytes`'s own comment.
    xtensa_lx::interrupt::disable();
    let index = decode_bootstrap(sp)
        .expect("rivet-arch-xtensa: start_first_task given a non-bootstrap sp");
    // SAFETY: `index` was produced by `init_task_stack` above and this is
    // the first (and, since only one task can ever be "first", only)
    // consumer for it.
    let b = unsafe { *BOOTSTRAPS[index].0.get() };
    // Deliberately NOT calling `free_bootstrap_index` here (unlike the
    // interrupt-handler fabrication path below, which does): this
    // function's whole contract, extensively documented just below, is
    // that every interrupt source stays masked for its *entire* body via
    // a single raw `xtensa_lx::interrupt::disable()` — introducing
    // `free_bootstrap_index`'s cross-hart spinlock acquire here regressed
    // real dual-core hardware (`smp_test.rs`: "given a non-bootstrap sp"
    // panic on boot, confirmed by reverting exactly this one call site).
    // This function runs at most once per core, ever — leaking at most
    // `MAX_HARTS` bootstrap slots for the lifetime of the program is
    // negligible next to reopening a boot-time race in the one function
    // this port's history has most consistently found races in.
    let top = ((b.stack_base + b.stack_len) & !0xF) as u32;
    // Root cause (plan.md Phase 22/23), found on real hardware: this used
    // a bare `"j {trampoline}"` into the trampoline's `entry a1, 0`,
    // assuming (per the trampoline's own doc comment) that `entry`
    // rotates the window by exactly 4 registers — but a bare jump
    // doesn't set `PS.CALLINC`; the trampoline's `entry` inherits
    // whatever `CALLINC` was already in effect from however *this*
    // function itself was reached. Confirmed via disassembly of the real
    // call site (`rivet::port::arch::start_first_task`): the compiler
    // reaches this function via `callx8` (`CALLINC = 2`, an 8-register
    // rotation, not 4) — so the trampoline's `entry` was actually
    // rotating its window by 8, meaning its post-`entry` a2/a3 came from
    // *this* function's a10/a11, not the a6/a7 this code was writing.
    // `entry_fn` (in a7) landed 4 registers short of where the
    // trampoline's `callx4 a3` read it, explaining the small, PC-range
    // garbage value it jumped to. Fixed by using a real `call4` into the
    // trampoline instead of a bare jump: `call4` unconditionally sets
    // `CALLINC = 1` for the callee's next `entry`, regardless of how
    // *this* function was itself reached — making the trampoline's
    // +4-rotation assumption actually hold, deterministically, matching
    // its own doc comment. (Never returns here — `options(noreturn)` —
    // so using a real call instead of a bare jump costs nothing.)
    unsafe {
        core::arch::asm!(
            "mov a1, {top}",
            "wsr.intenable {mask}",
            "rsync",
            "call4 {trampoline}",
            top = in(reg) top,
            // Root cause (plan.md Phase 24), found on real dual-core
            // hardware: this `wsr.intenable` is a direct register
            // *write*, not an OR — before this fix it silently wiped out
            // whatever `__rivet_arch_init` had already armed for this
            // core (specifically the cross-core IPI line) every time
            // *any* task bootstrapped through this function, which is
            // every worker task in a real multi-task test, not just the
            // literal first one. Confirmed via a direct `INTENABLE`
            // dump from inside `__rivet_arch_idle`: APP_CPU's mask read
            // back `0x20008000` (Timer1 + Software1 — Timer1 is
            // meaningless there, this core never owns the tick, but
            // harmless) with `ipi::IPI_MASK` (bit 23) missing entirely —
            // exactly the bit the other core's broadcast wake IPI needs
            // to ever reach an idling APP_CPU. This is why the broadcast
            // fix worked once (before any worker had bootstrapped
            // through here on that core) and then never again.
            mask = in(reg) (timer::TIMER1_MASK | timer::SOFTWARE1_MASK | ipi::IPI_MASK),
            in("a6") b.arg as u32,
            in("a7") b.entry_fn as u32,
            trampoline = sym rivet_ptask_trampoline_impl,
            options(noreturn),
        );
    }
}

// ── Interrupt dispatch: tick + reschedule share level 3 ─────────────

// Not using `#[xtensa_lx_rt::interrupt(1)]`: as of `xtensa-lx-rt-proc-macros`
// 0.5.0 (the latest published version), its generated trampoline emits a
// bare `#[export_name = "..."]` on an `unsafe extern "C" fn`, which this
// toolchain's rustc now rejects ("unsafe attribute used without unsafe") —
// a genuine version-skew bug in the macro against this rustc, not
// something fixable from this crate. Implementing the documented
// `extern "Rust" fn __level_3_interrupt(&mut Context)` symbol directly
// (the exact thing the macro would have generated a trampoline for, per
// `xtensa_lx_rt::exception::context`'s own `unsafe extern "Rust" { fn
// __level_3_interrupt(...); }` declaration) sidesteps the macro entirely.
#[no_mangle]
extern "Rust" fn __level_3_interrupt(save_frame: &mut Context) {
    // Cycle-stamped as early as possible in the handler, mirroring
    // `rivet-arch-cortex-m::rivet_pendsv_rust`'s own `IrqEntry` timing —
    // used below (only in the Software1/reschedule branch, the only
    // source here with a matching `RESCHEDULE_REQUESTED_AT` stamp).
    #[cfg(feature = "latency-histograms")]
    let __entry_now = rivet::port::arch::cycle_count() as u32;
    // Root cause (plan.md Phase 22/23), found on real hardware: this used
    // to check `sched::current()` and return immediately if `None`,
    // *before* ever touching the hardware interrupt sources below. A tick
    // landing before `rivet::run()`/`start_first_task` (legitimately
    // possible — ticks start in `rivet::init()`, well before `run()`,
    // and `spawn_ptask!`'s own printing/setup takes long enough in
    // practice for the first tick deadline to arrive) took that early
    // return WITHOUT ever calling `timer::on_timer_irq()` to re-arm
    // `CCOMPARE1`. Since CCOUNT keeps incrementing past the stale compare
    // value, the exact same interrupt re-fires the instant this handler
    // returns — forever, with zero forward progress in the interrupted
    // code — an infinite interrupt storm that looks exactly like a silent
    // hang from the outside (confirmed: a console print mid-transmission
    // at the moment of the first tick was permanently cut off, never
    // resuming). The hardware ack/re-arm must happen unconditionally,
    // every time this handler runs, before any "is the scheduler even
    // up yet" decision.
    let pending = xtensa_lx::interrupt::get();
    if pending & timer::TIMER1_MASK != 0 {
        // SAFETY: only ever called from this handler.
        unsafe { timer::on_timer_irq() };
        // Root cause (plan.md Phase 24), found on real hardware: this
        // re-arms `CCOMPARE1` but — unlike
        // `rivet-arch-riscv::clint::on_timer_irq`, which calls
        // `rivet::timer::poll_timers(now_micros())` as part of the same
        // handler — never actually polled the deadline queue. Every
        // `Sleep`/timeout in this whole port was consequently a silent
        // no-op: nothing ever woke a task waiting on one (confirmed on
        // hardware — a dual-core test's async monitor task, waiting on
        // `Sleep::<5_000>`, never ran a second time). Only ever exercised
        // now because `demo.rs`/`preempt_test.rs` never used `Sleep` or
        // the async tier at all — a single-core-affecting gap, not a
        // dual-core one, just found via a dual-core test that happened
        // to be the first thing in this port to actually call `Sleep`.
        rivet::timer::poll_timers(rivet::port::board::now_us());
    }
    if pending & timer::SOFTWARE1_MASK != 0 {
        // SAFETY: Software1 is edge-triggered; clearing it here is the
        // documented acknowledgement.
        unsafe { xtensa_lx::interrupt::clear(timer::SOFTWARE1_MASK) };
        #[cfg(feature = "latency-histograms")]
        {
            let requested_at = RESCHEDULE_REQUESTED_AT[__rivet_arch_hart_id()]
                .load(core::sync::atomic::Ordering::Relaxed);
            rivet::latency::record(
                rivet::latency::Kind::IrqEntry,
                __entry_now.wrapping_sub(requested_at) as u64,
            );
        }
    }
    if pending & ipi::IPI_MASK != 0 {
        // Cross-core reschedule IPI (plan.md Phase 24): ack by clearing
        // whichever `FROM_CPU_INTRn` source *this* core's own
        // `__rivet_arch_init` mapped to this line — the other core's
        // `SYSTEM.cpu_intr_from_cpu(n)` write set it; this core clears
        // the same bit to acknowledge, exactly mirroring how
        // Software1's self-IPI is acked above, just via a different
        // register (peripheral-routed lines ack at the source, not via
        // `xtensa_lx::interrupt::clear`, which only knows about the
        // CPU's own internal Timer/Software lines).
        // SAFETY: `SYSTEM` is a real, fixed-address SoC register block.
        unsafe {
            let system = &*esp32s3::SYSTEM::ptr();
            let reg = if __rivet_arch_hart_id() == 0 {
                ipi::FROM_APP_CPU_SOURCE
            } else {
                ipi::FROM_PRO_CPU_SOURCE
            } - ipi::FROM_PRO_CPU_SOURCE;
            system.cpu_intr_from_cpu(reg).write(|w| w.cpu_intr().clear_bit());
        }
    }
    if pending & periph_irq::MASK != 0 {
        // Board-registered peripheral IRQ (plan.md Phase 25): unlike
        // Timer1/Software1/IPI above, this carries no scheduling decision
        // of its own — dispatch to the registered handler and return,
        // matching `rivet-arch-riscv::plic`'s identical
        // claim/dispatch/complete-then-return shape. The handler itself
        // is responsible for acking at the peripheral (this line is
        // level-triggered — see `periph_irq`'s docs — so an unacked
        // source re-fires the instant this handler returns, same
        // interrupt-storm hazard `timer::on_timer_irq` had to avoid).
        let hart = __rivet_arch_hart_id();
        let source = periph_irq::CURRENT_SOURCE[hart].load(core::sync::atomic::Ordering::Acquire);
        if source != u32::MAX {
            rivet::irq::dispatch(source);
        }
        return;
    }
    if pending & (timer::TIMER1_MASK | timer::SOFTWARE1_MASK | ipi::IPI_MASK) == 0 {
        return;
    }

    let Some(tid) = rivet::preempt::sched::current() else {
        // Preemptive tier hasn't started yet (a tick landed before
        // `rivet::run()`/`start_first_task`) — hardware is already
        // acked/re-armed above; nothing to dispatch yet, matching every
        // other arch's `on_tick` no-op-if-not-started contract.
        return;
    };

    // `CONTEXTS` is a *cross-hart-shared* array (`ContextCell`'s
    // `unsafe impl Sync` only reasoned about same-hart interrupt
    // reentrancy — never actually true on this dual-core target), but
    // each individual `CONTEXTS[id]` read/write was a plain, non-atomic
    // struct copy (several store/load instructions, not one) with *no*
    // synchronization at all — not even a lock spanning just the copy
    // itself. That leaves a real window: hart A can be mid-way through
    // `*CONTEXTS[tid] = *save_frame` while hart B's own, separately-timed
    // dispatch reads `CONTEXTS[tid]` back out mid-write — a torn read of
    // a live struct, not a benign race, and exactly the kind of fault
    // that gets more likely (not less) the more often both harts are
    // ticking concurrently, matching this crash's exact sensitivity to
    // `timer::on_timer_irq`'s broadcast rate. Wrapping *each* copy in its
    // own `critical::enter` closes that specific torn-copy race with the
    // smallest possible lock-hold time — deliberately *not* spanning the
    // scheduling decision itself (`on_tick` already protects that on its
    // own), since holding the cross-hart spinlock for the full
    // save-decide-restore span starved the other hart's own tick
    // handling badly enough to stall the whole system (found on real
    // hardware while verifying a first, wider-locking attempt at this
    // exact fix).
    // SAFETY: `tid < MAX_PTASKS` (a real task id).
    rivet::critical::enter(|| unsafe {
        *CONTEXTS[tid].0.get() = *save_frame;
    });

    let resume = rivet::preempt::on_tick(tid);
    if resume == tid {
        return; // no switch — save_frame is already this task's own state
    }

    if resume < MAX_PTASKS {
        // The candidate has been interrupted at least once before —
        // restore its last-saved full state.
        // SAFETY: `resume` is a task id this crate itself populated on a
        // prior switch-out.
        rivet::critical::enter(|| unsafe {
            *save_frame = *CONTEXTS[resume].0.get();
        });
    } else {
        // The candidate has never run before (dispatched for the first
        // time via this tick, not via `start_first_task` — only one task
        // in the whole system gets to be "first" that way). Fabricate its
        // initial state the same way `start_first_task` would have.
        // Never touches `CONTEXTS`, so no lock needed here.
        let index = decode_bootstrap(resume)
            .expect("rivet-arch-xtensa: on_tick returned an sp that is neither a task id nor a bootstrap marker");
        // SAFETY: consumed at most once, matching `start_first_task`'s
        // own contract for the same table.
        let b = unsafe { *BOOTSTRAPS[index].0.get() };
        free_bootstrap_index(index);
        *save_frame = fresh_task_context(&b);
    }
}
