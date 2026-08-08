//! ARM Cortex-M architecture port.
//!
//! Targets: Cortex-M3 on QEMU lm3s6965evb, any real Cortex-M MCU.
//!
//! # Preemptive context switch
//!
//! Tasks run in Thread mode using PSP (Process Stack Pointer); exceptions
//! (SysTick, PendSV, everything else) always run in Handler mode using MSP
//! (Main Stack Pointer) — this is automatic Cortex-M behavior, not
//! something we configure per-exception. That split matters: it means a
//! PendSV handler's own nested Rust calls (the scheduler, atomics, etc.)
//! run on MSP, never touching a task's PSP-based stack, so there's no
//! RISC-V-style risk of the scheduler's own call chain competing for
//! space with whatever a task had reserved for itself.
//!
//! Following ARM's recommended pattern: SysTick only *requests* a
//! reschedule (`SCB.ICSR.PENDSVSET`); the actual register save/restore and
//! scheduling decision happen in PendSV, which — being the lowest-priority
//! exception — never preempts a higher-priority ISR mid-flight.
//!
//! Cortex-M auto-stacks {r0-r3, r12, lr, pc, xPSR} on exception entry and
//! restores them on return; PendSV only needs to manually save/restore the
//! remaining callee-saved set {r4-r11} around that hardware frame. Unlike
//! RISC-V, there's no separate "saved PC register" or privilege-mode bit
//! to manage — the hardware frame carries the return PC and processor
//! state, and exception return handles the mode transition automatically.

use core::sync::atomic;

// ── Task stack minimum ────────────────────────────────────────────

/// Minimum task stack: the PendSV frame (32 bytes r4-r11 + 32 bytes
/// hardware-stacked r0-r3/r12/lr/pc/xPSR) plus slack for the entry
/// trampoline (plan.md §2.7).
pub const MIN_TASK_STACK: usize = 64 + 64;

// ── Memory Protection Unit (plan.md §3.1) ─────────────────────────
//
// Verified against QEMU's lm3s6965evb: MPU_TYPE = 0x0800 (8 data regions),
// MemManage faults work. Design with two regions:
//   Region 6 — the whole `.task_stacks` pool, AP=no-access, XN=1: denies
//              everything by default (tasks cannot touch each other's
//              stacks; an overflow past a stack's low end faults).
//   Region 7 — the *currently running* task's stack, AP=RW, XN=1,
//              reprogrammed on every context switch (three register
//              writes). On overlap the highest-numbered enabled region
//              wins, so region 7 re-enables just the running task's stack
//              inside the denied pool.
// PRIVDEFENA lets the kernel (flash, peripherals, kernel data) use the
// background map untouched.

const MPU_CTRL: *mut u32 = 0xE000_ED94 as *mut u32;
const MPU_RNR: *mut u32 = 0xE000_ED98 as *mut u32;
const MPU_RBAR: *mut u32 = 0xE000_ED9C as *mut u32;
const MPU_RASR: *mut u32 = 0xE000_EDA0 as *mut u32;

const RASR_XN: u32 = 1 << 28;
const RASR_AP_NO_ACCESS: u32 = 0b000 << 24;
const RASR_AP_RW: u32 = 0b011 << 24;
const RASR_ENABLE: u32 = 1 << 0;
const RBAR_VALID: u32 = 1 << 4;

/// Region number for the whole-pool deny.
const POOL_DENY_REGION: u32 = 6;
/// Region number for the running task's stack.
const CURRENT_STACK_REGION: u32 = 7;

fn mpu_write_region(region: u32, base: usize, size: usize, ap: u32, xn: bool) {
    // SAFETY: these are the fixed, memory-mapped MPU registers (volatile);
    // the MPU is exclusively owned by this module.
    unsafe {
        let size_field = size.trailing_zeros() - 1; // region = 2^(SIZE+1)
        core::ptr::write_volatile(MPU_RNR, region);
        core::ptr::write_volatile(MPU_RBAR, (base as u32) | RBAR_VALID | region);
        core::ptr::write_volatile(
            MPU_RASR,
            (if xn { RASR_XN } else { 0 }) | ap | (size_field << 1) | RASR_ENABLE,
        );
    }
}

/// Enable the MPU (PRIVDEFENA + ENABLE) and program the pool-deny region.
/// Region 7 is programmed on the first context switch. Must be called
/// before any task can run.
pub fn mpu_init() {
    let (base, len) = crate::preempt::stack_pool::pool_bounds();
    // The pool is 16 KiB aligned (linker script); its size is a power of two.
    debug_assert!(len.is_power_of_two());
    mpu_write_region(POOL_DENY_REGION, base, len, RASR_AP_NO_ACCESS, true);
    // SAFETY: fixed MPU control register; enable + PRIVDEFENA.
    unsafe {
        core::ptr::write_volatile(MPU_CTRL, 0b101);
    }
}

/// Reprogram region 7 to cover the currently-running task's stack.
/// Called from [`crate::arch::on_switch_to`] on every context switch.
pub fn mpu_set_current_stack(base: usize, size: usize) {
    if base == 0 {
        return;
    }
    debug_assert!(size.is_power_of_two());
    mpu_write_region(CURRENT_STACK_REGION, base, size, RASR_AP_RW, true);
}

/// Called on every actual context switch (plan.md §3.1).
pub fn on_switch_to(stack_base: usize, stack_size: usize) {
    mpu_set_current_stack(stack_base, stack_size);
}

/// Temporarily allow kernel access to the whole pool (used while
/// `preempt::spawn` fills and initializes a new stack inside the denied
/// pool, plan.md §3.1). The pool-deny region is the *highest-numbered*
/// region below the current-stack region, so any other enabled region
/// covering a stack would lose the overlap to it — the deny region itself
/// must be disabled for the window. The caller holds interrupts off
/// (critical section), so no other task can run during the window.
pub fn mpu_allow_scratch(_base: usize, _size: usize) {
    // SAFETY: fixed MPU registers; disable region 6.
    unsafe {
        core::ptr::write_volatile(MPU_RNR, POOL_DENY_REGION);
        core::ptr::write_volatile(MPU_RASR, 0);
    }
}

/// Re-enable the pool-deny region (closes the [`mpu_allow_scratch`]
/// window).
pub fn mpu_clear_scratch() {
    let (base, len) = crate::preempt::stack_pool::pool_bounds();
    mpu_write_region(POOL_DENY_REGION, base, len, RASR_AP_NO_ACCESS, true);
}

/// No PMP on Cortex-M (the MPU handles stack isolation).
pub fn pmp_register_guard(_guard_base: usize, _entry: usize) {}

// ── MemManage fault handling (plan.md §3.4) ────────────────────────

const CFSR: *const u32 = 0xE000_ED28 as *const u32;
const MMFAR: *const u32 = 0xE000_ED34 as *const u32;

/// Handle a MemManage fault: attribute to the running task and dispatch
/// to the fault policy. Under [`FaultPolicy::Panic`] this resets; under
/// [`FaultPolicy::IsolateTask`] it abandons the faulted task's frame and
/// returns the next ready task's *full* frame sp (manual r4-r11 save +
/// hardware frame). The asm `MemManage` entry (below) restores r4-r11 and
/// switches PSP to that frame, so the exception return un-stacks the *new*
/// task — a real context switch out of the fault handler, mirroring the
/// PendSV path.
#[no_mangle]
unsafe extern "C" fn rivet_memmanage_rust(faulted_sp: usize) -> usize {
    // SAFETY: fixed, memory-mapped system registers (volatile).
    let cfsr = unsafe { core::ptr::read_volatile(CFSR) };
    // SAFETY: as above — the MMFAR register (valid when CFSR.MMARVALID set).
    let mmfar = unsafe { core::ptr::read_volatile(MMFAR) };

    let fault_pc = if faulted_sp != 0 {
        // SAFETY: the faulted task's stacked frame is at faulted_sp; its
        // PC is at offset 24 (r0,r1,r2,r3,r12,lr,pc,xPSR).
        unsafe { core::ptr::read_volatile((faulted_sp + 24) as *const u32) as usize }
    } else {
        0
    };

    let task_id = crate::preempt::sched::current();
    if let Some(id) = task_id {
        if let Some(t) = crate::preempt::tcb::get(id) {
            // Persist the abandoned frame so the resume path would work if
            // this task were ever restarted.
            t.sp.store(faulted_sp, crate::sync::atomic::Ordering::Release);
        }
    }

    let address = if cfsr & (1 << 7) != 0 {
        mmfar as usize
    } else {
        0
    };
    let info = crate::fault::FaultInfo {
        task_id,
        kind: crate::fault::FaultKind::MemManage(cfsr),
        address,
        pc: fault_pc,
    };
    crate::fault::on_fault(&info)
}

core::arch::global_asm!(
    ".section .text.MemManage",
    ".global MemManage",
    ".thumb_func",
    "MemManage:",
    "  push {{lr}}",               // save EXC_RETURN (bl clobbers lr)
    "  mrs  r0, psp",              // faulted task's stacked frame
    "  bl   rivet_memmanage_rust", // returns next task's full frame sp
    "  ldmia r0, {{r4-r11}}",      // restore the new task's callee-saved
    "  adds r0, r0, #32",          // advance past the manual frame
    "  msr  psp, r0",              // PSP = hw frame; the exception return
    "  pop  {{lr}}",               // restore EXC_RETURN
    "  bx   lr",                   // un-stacks the new task's frame
);

// ── System tick ───────────────────────────────────────────────────

/// Tick counter. Counts *ticks* (not microseconds): a u32 tick counter at
/// 1 kHz wraps in ~49 days, versus ~71 minutes for a u32 microsecond
/// counter (plan.md [B5]). Conversion happens at the API boundary in
/// [`now_micros`]. The tick handler is the only writer.
static SYSTEM_TICKS: atomic::AtomicU32 = atomic::AtomicU32::new(0);

/// Configure SysTick's reload value for 1ms intervals, but deliberately do
/// NOT enable it here (ENABLE/TICKINT bits left clear). If SysTick (and
/// therefore PendSV) could fire this early, it could land while we're
/// still on the plain boot stack with PSP never set — PendSV's asm
/// unconditionally does `mrs r0, psp; stmia r0, {r4-r11}` assuming PSP is
/// valid, so an uninitialized PSP there faults immediately (in practice:
/// a UsageFault/UNALIGNED, since PSP's reset value isn't 4-byte aligned).
/// This is the Cortex-M analog of the RISC-V port's "don't set
/// mstatus.MIE until start_first_task" fix — same race, same shape of fix:
/// `systick_enable()` is called only once PSP is safely set up.
pub fn systick_init(sysclk_hz: u32) {
    let reload = sysclk_hz / 1000; // 1ms
                                   // SAFETY: `cortex_m::peripheral::SYST::PTR` is the statically-known
                                   // SysTick peripheral base address, valid on every Cortex-M; register
                                   // writes are volatile memory-mapped accesses.
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };

    // SAFETY: peripheral register write to the valid SysTick block; the
    // peripheral is only ever accessed here and in `systick_enable`.
    unsafe { syst.csr.write(0) }; // disable while configuring
                                  // SAFETY: as above — reload-value register write.
    unsafe { syst.rvr.write(reload - 1) };
    // SAFETY: as above — current-value register write.
    unsafe { syst.cvr.write(0) }; // clear current value
}

/// Enable SysTick (ENABLE + TICKINT). Call only once PSP has been set up
/// (i.e. from [`start_first_task`], not from [`early_init`]) — see
/// [`systick_init`] for why.
pub fn systick_enable() {
    // SAFETY: SYST::PTR is the statically-known SysTick base, valid on
    // every Cortex-M; see `systick_init` for the same justification.
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    // SAFETY: volatile memory-mapped write to the SysTick control register;
    // the peripheral is exclusively owned by this module.
    unsafe {
        syst.csr.write(
            (1 << 0)  // ENABLE
            | (1 << 1)  // TICKINT
            | (1 << 2), // CLKSOURCE (system clock)
        )
    };
}

/// Override the SysTick reload value (in system-clock ticks) after
/// [`systick_init`]. Safe to call before `run()` (the countdown starts
/// from the new value when SysTick is enabled); also resets the current
/// value so the first underflow uses the new period.
pub fn systick_reload(ticks: u32) {
    // SAFETY: SYST::PTR is the statically-known SysTick base (see
    // `systick_init`); RVR/CVR writes are volatile MMIO accesses.
    let syst = unsafe { &*cortex_m::peripheral::SYST::PTR };
    // SAFETY: peripheral register writes to the valid SysTick block.
    unsafe {
        syst.rvr.write(ticks);
        syst.cvr.write(0);
    }
}

/// Test hook: seed the tick counter so a test can start near the u32
/// boundary and observe a wrap crossing without running days of simulated
/// time (plan.md §2.2 [B5] acceptance — soak_time_wrap seeds just below
/// 2^32 µs and asserts Sleep still fires past it). Harmless in production:
/// it merely rewinds/advances the monotonic tick count.
pub fn systick_seed_ticks(v: u32) {
    SYSTEM_TICKS.store(v, atomic::Ordering::Release);
}

/// Call from `SysTick` exception handler: advance system time, wake
/// expired `Sleep` futures, then request a reschedule opportunity via
/// PendSV (never switches stacks directly — see module docs).
pub fn systick_handler() {
    // Count ticks, not microseconds (plan.md [B5] — u32 µs wraps in 71
    // minutes; u32 ticks at 1 kHz wraps in ~49 days).
    let tick = SYSTEM_TICKS.fetch_add(1, atomic::Ordering::Release) + 1;
    crate::watchdog::on_tick();
    crate::timer::poll_timers((tick as u64) * 1000);
    pend_executor();
}

/// Get current system time in microseconds: tick count × 1 ms, converted
/// at the API boundary (plan.md [B5]).
pub fn now_micros() -> u64 {
    (SYSTEM_TICKS.load(atomic::Ordering::Acquire) as u64) * 1000
}

// ── Sleep ─────────────────────────────────────────────────────────

pub fn sleep() {
    cortex_m::asm::wfi();
}

// ── Executor pend / preemptive-tier yield (PendSV) ────────────────

/// Set PendSV pending. Called from SysTick and from `yield_now()` — the
/// single trigger for every context switch, tick-driven or voluntary.
pub fn pend_executor() {
    // SAFETY: `SCB::PTR` is the statically-known System Control Block base,
    // valid on every Cortex-M; `ICSR` write is a volatile MMIO access and
    // the SCB is only accessed from this module and `early_init`.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.icsr.write(1 << 28); // PENDSVSET
    }
}

pub fn yield_now() {
    pend_executor();
}

// ── UART (NS16550 at 0x4000C000 on lm3s6965evb) ──────────────────

const UART0_BASE: u32 = 0x4000_C000;

/// Print a 32-bit value as 8 lowercase hex digits (diagnostics).
pub fn debug_print_hex32(mut n: u32) {
    let hex = b"0123456789abcdef";
    for _ in 0..8 {
        let d = ((n >> 28) & 0xF) as usize;
        // SAFETY: `hex` is a valid 16-byte table; `d < 16`.
        unsafe {
            crate::arch::debug_print(core::str::from_utf8_unchecked(core::slice::from_raw_parts(
                hex.as_ptr().add(d),
                1,
            )));
        }
        n <<= 4;
    }
}

pub fn debug_print(s: &str) {
    let uart_dr = UART0_BASE as *mut u32;
    let uart_fr = (UART0_BASE + 0x18) as *const u32;

    for &b in s.as_bytes() {
        // SAFETY: `uart_fr`/`uart_dr` point at the LM3S6965 PL011 UART
        // registers (fixed, memory-mapped, volatile); the UART is only
        // accessed from this function.
        while unsafe { core::ptr::read_volatile(uart_fr) } & (1 << 5) != 0 {
            core::hint::spin_loop();
        }
        // SAFETY: as above — UART data-register write.
        unsafe { core::ptr::write_volatile(uart_dr, b as u32) };
    }
}

// ── Semihosting ───────────────────────────────────────────────────

core::arch::global_asm!(
    ".section .text",
    ".global rivet_semihosting",
    ".thumb_func",
    "rivet_semihosting:",
    "  bkpt 0xAB",
    "  bx   lr",
    ".global rivet_exit_success",
    ".thumb_func",
    "rivet_exit_success:",
    "  movs r0, #0x18",
    "  ldr  r1, =0x20026",
    "  bkpt 0xAB",
    "1:",
    "  b    1b",
);

pub fn exit_success() -> ! {
    extern "C" {
        fn rivet_exit_success() -> !;
    }
    // SAFETY: `rivet_exit_success` is the semihosting exit sequence defined
    // in the global_asm! block above; it never returns.
    unsafe {
        rivet_exit_success();
    }
}

/// Cortex-M failure exit: prints a distinguishable marker, then halts.
/// QEMU ARM semihosting has no simple "exit with code N" path, so the
/// QEMU test harness asserts on the marker text instead of an exit code.
pub fn exit_failure(code: u32) -> ! {
    debug_print("\nRIVET_FAILURE code=");
    let mut digits = [0u8; 10];
    let mut n = code;
    let mut i = 0;
    if n == 0 {
        debug_print("0");
    } else {
        while n > 0 {
            digits[i] = b'0' + (n % 10) as u8;
            n /= 10;
            i += 1;
        }
        let mut buf = [0u8; 10];
        for j in 0..i {
            buf[j] = digits[i - 1 - j];
        }
        // SAFETY: `buf[..i]` is ASCII digits — valid UTF-8.
        if let Ok(s) = core::str::from_utf8(&buf[..i]) {
            debug_print(s);
        }
    }
    debug_print("\n");
    loop {
        core::hint::spin_loop();
    }
}

/// Cortex-M system reset via SCB AIRCR SYSRESETREQ (0x05FA0004).
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

// ── Early init ────────────────────────────────────────────────────

pub fn early_init() {
    mpu_init();
    systick_init(12_000_000); // LM3S6965 runs at 12 MHz by default on QEMU

    // PendSV must run at the lowest possible priority so it never preempts
    // a higher-priority ISR mid-flight — it only runs once everything else
    // has finished, which is what makes it safe to do the actual stack
    // switch there. Set SHPR3.PRI_14 (PendSV) and SHPR3.PRI_15 (SysTick)
    // to the lowest priority (0xFF, all implemented priority bits set).
    //
    // SAFETY: `SCB::PTR` is the statically-known System Control Block base,
    // valid on every Cortex-M; these SHPR/SHCSR writes are volatile MMIO
    // accesses and the SCB is exclusively owned by this module.
    unsafe {
        let scb = &*cortex_m::peripheral::SCB::PTR;
        scb.shpr[10].write(0xFF); // PendSV priority (SHPR3 byte 2)
        scb.shpr[11].write(0xFF); // SysTick priority (SHPR3 byte 3)
                                  // Enable the dedicated Bus/Usage/MemManage fault handlers; without
                                  // this they escalate straight to HardFault, hiding the real cause.
        scb.shcsr.write(
            (1 << 16) // MEMFAULTENA
            | (1 << 17) // BUSFAULTENA
            | (1 << 18), // USGFAULTENA
        );
    }
}

// ── Preemptive tier: PendSV context switch ────────────────────────

/// Rust-side PendSV logic. Called from the asm handler with `interrupted_sp`
/// (the interrupted task's PSP, pointing at its saved r4-r11 frame). Saves
/// the interrupted task's registers (already on the stack), asks the
/// scheduler what to run next, and returns the stack pointer to resume.
#[no_mangle]
unsafe extern "C" fn rivet_pendsv_rust(interrupted_sp: usize) -> usize {
    crate::preempt::on_tick(interrupted_sp)
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
    "  push {{lr}}",
    "  mrs  r0, psp",
    "  subs r0, r0, #32",
    "  stmia r0, {{r4-r11}}",
    "  bl   rivet_pendsv_rust",
    "  ldmia r0, {{r4-r11}}",
    "  adds r0, r0, #32",
    "  msr  psp, r0",
    "  pop  {{lr}}",
    // Symbol for the GDB context-switch verification script (tests/gdb):
    // r4-r11 have been restored from the frame; frame base = psp - 32.
    ".global rivet_pendsv_resume",
    "rivet_pendsv_resume:",
    "  bx   lr",
);

// ── First task start ──────────────────────────────────────────────

/// Set up the initial stack frame for a new task, then start the first
/// task's execution. This is called once, from `preempt::start`, with the
/// first task's already-built stack frame.
pub unsafe fn start_first_task(sp: usize) -> ! {
    // Set PSP to the first task's stack and switch to thread mode with PSP.
    // SAFETY: `sp` is the freshly-built initial frame of the first task;
    // PSP is set exactly once here, before any interrupt can fire.
    let frame = sp as *const u32;
    let arg = core::ptr::read(frame.add(8));
    let entry_fn = core::ptr::read(frame.add(14));

    core::arch::asm!(
        "msr psp, {sp}",
        "movs r2, #2",
        "msr control, r2", // SPSEL=1 (use PSP in Thread mode), stay privileged
        "isb",
        sp = in(reg) sp,
        out("r2") _,
    );

    // PSP is valid now — safe to let SysTick/PendSV start firing.
    systick_enable();

    core::arch::asm!(
        "mov r0, {arg}",
        "bx {entry}",
        arg = in(reg) arg,
        entry = in(reg) entry_fn,
        options(noreturn)
    );
}

/// Build the initial stack frame for a new preemptive task so that the
/// first context switch into it starts execution at `entry_fn(arg)`.
///
/// Frame layout (aligned to 8 bytes, 64 bytes total):
/// ```
/// [sp+0]  r4
/// [sp+4]  r5
/// [sp+8]  r6
/// [sp+12] r7
/// [sp+16] r8
/// [sp+20] r9
/// [sp+24] r10
/// [sp+28] r11
/// [sp+32] r0   <- arg
/// [sp+36] r1
/// [sp+40] r2
/// [sp+44] r3
/// [sp+48] r12
/// [sp+52] lr   <- entry_fn (with Thumb bit set)
/// [sp+56] pc   <- entry_fn (with Thumb bit set)
/// [sp+60] xPSR <- 0x01000000 (Thumb mode)
/// ```
///
/// The PendSV handler restores r4-r11 from the first 32 bytes; the
/// hardware un-stacks the remaining 32 bytes on exception return,
/// resuming at `entry_fn` with `r0 = arg`.
///
/// # Safety
/// `stack` must be suitably aligned and large enough for one context-switch
/// frame; `entry_fn` must be a valid function pointer taking one argument
/// and never returning; `arg` must be valid for the task's lifetime.
/// SVC-vectored kernel call: builds a new task's initial stack frame from
/// *Handler* mode, where the MPU does not apply. Thread-mode code cannot
/// write another task's stack: MPU region 6 denies the whole `.task_stacks`
/// pool and region 7 only permits the *current* task's stack — a spawner
/// faulting on the new task's stack is what `spawn_ptask!` hit before this
/// fix (plan.md §5.4 respawn exposed it).
///
/// The caller (`init_task_stack`) issues `svc 0` with r0 = stack ptr,
/// r1 = stack len, r2 = entry_fn, r3 = arg. Exception entry stacked those
/// registers in the caller's frame; we read them, build the frame, and
/// write the resulting stack pointer back into the saved r0 so the
/// exception return delivers it to the caller. The vector-table entry
/// points here (link-cm3.ld: `LONG(rivet_svc_handler)`).
///
/// Naked (no prologue): the exception frame base must be read from `sp`
/// *before* the compiler pushes anything, and the exception return value in
/// `lr` must be preserved across the call so the handler returns with `bx
/// lr` (EXC_RETURN), not a normal branch.
///
/// # Safety
/// Exception entry point; never called directly.
#[unsafe(naked)]
#[no_mangle]
unsafe extern "C" fn rivet_svc_handler() {
    // SAFETY: naked handler with no stack frame; the register-level
    // protocol with `rivet_svc_core` is documented in the doc comment.
    core::arch::naked_asm!(
        "mov r4, lr",     // preserve EXC_RETURN (r4 is callee-saved)
        "uxtb r1, r4",    // EXC_RETURN 0xFFFFFFFD = taken from thread
        "cmp  r1, #0xfd", // mode with PSP (spawn from a running task);
        "bne  1f",        // 0xF9 = thread mode with MSP (boot context)
        "mrs  r0, psp",   // frame on PSP
        "b    2f",
        "1:",
        "mov  r0, sp", // frame on MSP
        "2:",
        "bl  rivet_svc_core",
        "mov lr, r4", // restore EXC_RETURN
        "bx  lr",     // exception return
    );
}

/// Rust half of [`rivet_svc_handler`]: `frame` is the exception stack
/// frame ({r0,r1,r2,r3,r12,lr,pc,xPSR}) pushed by the `svc 0` issued from
/// [`init_task_stack`].
#[no_mangle]
fn rivet_svc_core(frame: *mut u32) {
    // SAFETY: the caller guarantees `frame` points at the live exception
    // stack frame ({r0,r1,r2,r3,...}) pushed by the `svc 0`; all four
    // slots are valid, word-aligned reads.
    let (stack_ptr, stack_len, entry, arg) = unsafe {
        (
            *frame.add(0) as *mut u8,
            *frame.add(1) as usize,
            *frame.add(2) as usize,
            *frame.add(3) as usize,
        )
    };

    // SAFETY: the caller passed a valid `&mut [u8]` slice split across
    // r0/r1 (as_mut_ptr / len).
    let sp = unsafe {
        // The ARMv7-M MPU applies in Handler mode too, so the write into
        // the denied `.task_stacks` pool would fault even here. Disable the
        // MPU for the duration of the frame write (real RTOSes do the
        // same); the SVC handler runs at the highest configurable priority
        // so nothing can preempt us mid-window.
        let mpu_ctrl = 0xE000_ED94usize as *mut u32;
        // SAFETY: fixed, memory-mapped MPU control register (the whole
        // block is inside the `unsafe` above).
        let saved = core::ptr::read_volatile(mpu_ctrl);
        core::ptr::write_volatile(mpu_ctrl, saved & !1);
        let sp = init_task_stack_impl(
            core::slice::from_raw_parts_mut(stack_ptr, stack_len),
            entry,
            arg,
        );
        // SAFETY: as above — restore the previous MPU state.
        core::ptr::write_volatile(mpu_ctrl, saved);
        sp
    };
    // Deliver the result via the exception frame's saved r0.
    // SAFETY: `frame` points at the live exception stack frame on MSP.
    unsafe {
        core::ptr::write_volatile(frame, sp as u32);
    }
}

/// Issue `init_task_stack_impl` from Handler mode via SVC (see
/// [`rivet_svc_handler`] for why the MPU requires it).
///
/// The caller holds a critical section (PRIMASK=1 on Cortex-M). An `svc`
/// issued with PRIMASK set runs at execution priority 0 — equal to the
/// SVC's own default priority — which the architecture escalates to
/// HardFault (QEMU's NVIC does exactly this). So PRIMASK is briefly
/// cleared around the `svc`. This is safe: the SVC handler runs at
/// priority 0, the highest configurable priority, so nothing (SysTick /
/// PendSV at 0xFF) can preempt the frame write; the critical section's
/// purpose — no task runs mid-initialization — is preserved.
pub unsafe fn init_task_stack(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    let ptr = stack.as_mut_ptr() as usize;
    let len = stack.len();
    let mut sp = 0usize;
    // SAFETY: the SVC handler reads r0-r3 from the exception frame, builds
    // the frame, and writes the new sp back into r0.
    unsafe {
        let mut primask: u32;
        // SAFETY: reading PRIMASK is always safe.
        core::arch::asm!(
            "mrs {0}, primask",
            out(reg) primask,
            options(nomem, nostack, preserves_flags),
        );
        if primask & 1 != 0 {
            // SAFETY: clearing PRIMASK re-enables interrupts (see doc).
            core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
        }
        core::arch::asm!(
            "svc 0",
            inout("r0") ptr => sp,
            in("r1") len,
            in("r2") entry_fn,
            in("r3") arg,
            options(nomem, nostack, preserves_flags),
        );
        if primask & 1 != 0 {
            // SAFETY: restoring PRIMASK (see doc).
            core::arch::asm!("cpsid i", options(nomem, nostack, preserves_flags));
        }
    }
    sp
}

unsafe fn init_task_stack_impl(stack: &mut [u8], entry_fn: usize, arg: usize) -> usize {
    const FRAME_WORDS: usize = 16; // 8 (r4-r11) + 8 (hw frame)
    const STACK_ALIGN: usize = 16;

    // SAFETY: `stack` is a valid mutable slice of at least 64 bytes (the
    // caller guarantees MIN_TASK_STACK); the writes below initialize the
    // frame INSIDE the slice (at the top, aligned down).
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
    core::ptr::write(frame.add(13), rivet_task_exit as *const () as usize as u32); // lr: entry's return path
    core::ptr::write(frame.add(14), entry_fn as u32); // pc
    core::ptr::write(frame.add(15), 0x0100_0000); // xPSR: Thumb bit (T=1) set

    frame_start
}
