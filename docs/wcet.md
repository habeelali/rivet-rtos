# Rivet RTOS: Formal WCET Analysis

This document gives worst-case execution time (WCET) and worst-case blocking-time
figures for every mechanism a hard real-time schedulability analysis needs:
interrupt latency, context-switch time, scheduling-decision cost, critical-section
hold time, priority-inheritance blocking time, and stack WCET, across all three
supported architectures (RISC-V/QEMU virt, Cortex-M3/QEMU LM3S6965, Xtensa LX7/
ESP32-S3 real hardware).

**See also `docs/wcet-stm32f401re.md`**: the same analysis for single-core
Cortex-M (STM32F401RE Nucleo, real hardware), which structurally avoids most
of the dual-core-specific hazards this document finds on the ESP32-S3 (no
cross-hart lock contention, hardware-isolated interrupt stacks) and reaches a
scoped, evidenced hard-real-time declaration.

**Every figure below is labeled by how it was obtained.** This matters for a hard
real-time claim: a number is only as trustworthy as its method.

- **MEASURED**: captured from real ESP32-S3 hardware via JTAG/UART, using the
  kernel's own `latency-histograms` instrumentation (cycle-exact, hardware
  `CCOUNT` register). The only category with genuine silicon backing in this
  environment.
- **DERIVED**: an exact instruction count, read directly from the actual
  compiled assembly (not estimated), converted to a cycle estimate using a
  stated per-instruction cost model for a reference core. The instruction count
  is exact; the cycle *time* depends on the stated model, since QEMU does not
  provide cycle-accurate timing for RV32/Cortex-M in this environment.
- **ARCHITECTURAL**: a figure specified by the ISA/vendor documentation itself
  (e.g. ARM's Cortex-M3 Technical Reference Manual exception-entry latency),
  not measured or derived here, but authoritative by definition.
- **ASSUMED**: a configuration value taken as given (e.g. clock rate) that this
  analysis did not independently verify against real silicon.

---

## 1. Configuration constants (the bounds everything else is expressed in terms of)

| Constant | Default | Max | Source |
|---|---|---|---|
| `MAX_PTASKS` | 16 | 32 | `RIVET_MAX_PTASKS` |
| `PRIORITIES` | 32 | 32 | `RIVET_PRIORITIES` |
| `MAX_HELD_MUTEXES` | 4 | 16 | `RIVET_MAX_HELD_MUTEXES` |
| `MAX_HARTS` | 1 | 8 | `RIVET_MAX_HARTS` (ESP32-S3 builds in this report: 2) |
| `TICK_HZ` | 1000 | 1,000,000 | `RIVET_TICK_HZ` |
| `MAX_IRQS` | 32 | 240 | `RIVET_MAX_IRQS` |

Every "O(MAX_PTASKS)" bound below is **exactly 16** unless a build overrides it.
This document uses the default throughout, since that is what ships.

---

## 2. Clock assumptions (ASSUMED: read this before trusting any time-domain figure)

| Board | Clock | Status |
|---|---|---|
| ESP32-S3 (Xtensa) | 40 MHz | **ASSUMED.** `rivet-bsp-esp32s3/src/lib.rs`'s own module docs: "the boot ROM's XTAL-derived default, not something this crate independently measures or configures." The chip is rated up to 240 MHz with PLL configuration Rivet does not perform. **Every microsecond/millisecond figure for this board in this document is only as accurate as this assumption.** Cycle counts (this document's actual measurements) are clock-independent and trustworthy regardless. |
| QEMU RISC-V virt | N/A | No real silicon in this environment. QEMU's TCG backend does not model cycle-accurate timing. Figures for this target are **instruction counts** (exact, from the real assembly) with a **derived** cycle estimate assuming a simple in-order RV32IMAC reference core (comparable to a SiFive E-class core) at 1 cycle/instruction baseline, plus 2 to 3 cycles per taken jump/branch. |
| QEMU Cortex-M3 (LM3S6965) | 12 MHz assumed in-tree (`systick_init`) | Same caveat as RISC-V for anything beyond ARM's own architecturally documented exception timing (§4.2), which is authoritative independent of clock rate (stated in cycles). |

---

## 3. Interrupt latency (assertion to first instruction of the scheduler's own code)

| Arch | Path measured | Cycles | Method |
|---|---|---|---|
| **Xtensa (ESP32-S3)** | Interrupt asserted → `on_tick` running (`Kind::IrqEntry`) | **max 224, typical 64-127** (log2 buckets: 2^6-2^7 dominant, 3 independent runs) | **MEASURED**, real hardware, 3 runs, deterministic (202-224 cycle spread) |
| **RISC-V (QEMU virt)** | `mtvec` dispatch → `rivet_trap_handler_rust` called | **34 instructions** exact (save path of `rivet_trap_entry`) plus 1 jump (vectored) or fixed offset (direct mode) to reach the handler | **DERIVED**, counted directly from `rivet-arch-riscv/src/lib.rs`'s `global_asm!` (lines 541-575); roughly 36-40 cycles at 1 CPI plus jump overhead for a reference in-order core |
| **Cortex-M3 (QEMU LM3S6965)** | Exception asserted → handler's first instruction | **12 cycles fixed** (best case, no bus wait) | **ARCHITECTURAL**, ARM Cortex-M3 Technical Reference Manual, exception entry latency. Automatic hardware stacking of r0-r3/r12/LR/PC/xPSR is included in this figure by ARM's own specification, independent of clock rate |

**Xtensa is the only figure in this table backed by real silicon.** The RISC-V and
Cortex-M numbers are correct instruction/cycle counts for the code as written, but
have never run on real RV32/Cortex-M3 silicon in this project, only QEMU, which
does not validate timing, only functional correctness.

---

## 4. Context-switch WCET (save outgoing + restore incoming register state)

### 4.1 Mechanism cost (excludes the scheduling *decision*: see §5)

| Arch | Save (asm insns) | Restore (asm insns) | Total | Cycles (model) |
|---|---|---|---|---|
| RISC-V | 28 `sw` + 6 setup = 34 | 4 setup + 28 `lw` + 2 = 34 | **76 instructions**, exact | roughly 80-95 cycles, DERIVED (1 CPI + `mret`/`call` overhead) |
| Cortex-M3 | 5 sw insns (r4-r11 `stmia`, 8 regs) + 12-cycle HW auto-stack (r0-r3/r12/LR/PC/xPSR) | 5 sw insns (`ldmia`) + 12-cycle HW auto-unstack | **10 sw insns + 24 HW cycles** | roughly 54 cycles, DERIVED (5 asm cost per ARM per-instruction table: `stmia`/`ldmia` of 8 registers is roughly N+1 cycles each on M3) + 24 ARCHITECTURAL |
| Xtensa (ESP32-S3) | `xtensa-lx-rt`'s `SAVE_CONTEXT`+`SPILL_REGISTERS` (full window flush) + this crate's `CONTEXTS[tid]` copy (136-byte `memcpy`) | mirror, `RESTORE_CONTEXT` + `CONTEXTS[resume]` copy | (not applicable, see cycles column) | **MEASURED** via `Kind::DispatchDecision`: dominant bucket 512-1023 cycles (uncontended, single-hart or lock-uncontested case); **worst-observed 97,133-166,597 cycles** under real dual-hart contention (see §6: this is `critical::enter` lock-wait time, not switch mechanism cost) |

### 4.2 A structural difference that matters for stack sizing (§7)

RISC-V and Cortex-M **both isolate the interrupt handler's own call stack**: RISC-V
swaps to a dedicated 2 KB per-hart `.isr_stack` (`csrrw sp, mscratch, sp`) before
calling into Rust; Cortex-M's exception handlers run in Handler mode on MSP, a
stack physically separate from any task's PSP. In both cases, only the small,
*fixed* save-frame (128 / 64 bytes) is charged against the interrupted task's own
stack. The scheduler, mutex, and dispatch logic's own call depth is not.

**Xtensa has no such isolation.** `xtensa-lx-rt`'s exception entry allocates its
256-byte frame *on top of whatever stack was live*: there is no separate ISR
stack on this port. The entire Rust handler (`on_tick`, `schedule()`, every
`critical::enter` call, mutex bookkeeping) runs on the interrupted task's own
stack. This is not hypothetical: it is exactly what forced the dual-hart race
fix documented in `docs/realtime.md` §15 to need a 4x task-stack increase
(4096 → 16384 bytes). Two extra `critical::enter` call frames were enough to
overflow the original budget on real hardware. **Any Xtensa task stack must
budget for its own deepest call chain *plus* the interrupt handler's full worst-
case call depth, not a fixed constant.**

---

## 5. Scheduling-decision WCET (`on_tick_locked` + `schedule()`)

All bounds are **DERIVED**, exact operation counts from the source
(`rivet/src/preempt/{mod,sched}.rs`), independent of architecture:

| Step | Bound | Why |
|---|---|---|
| `sched::current()` read, `Tcb.sp` store | O(1) | Single atomic load/store |
| Stack watermark check | O(1) | Single `read_volatile` of the lowest stack word |
| CPU-budget check | O(1) | Single comparison against a stored deadline |
| `schedule()`'s priority-bitmap scan | O(1) | `leading_zeros` on a 32-bit word, one hardware instruction on every supported ISA |
| `schedule()`'s least-recently-dispatched scan | **O(MAX_PTASKS) = 16 iterations, exact worst case** | Bounded scan of set bits in the winning priority's 32-bit ready word (`rivet/src/preempt/sched.rs:260-271`), a real loop bound, not asymptotic hand-waving: at most 16 iterations, each one atomic load + compare |
| `should_preempt` | O(1) | Two atomic loads + one comparison |
| Dispatch commit (`set_state`, `set_current`, `on_dispatch`) | O(1) + O(MAX_HARTS) | `ready_add`/`ready_remove` are O(1) bitmap ops; `wake_other_harts` loops at most `MAX_HARTS - 1` times (2 on this document's dual-hart ESP32-S3 builds) |

**Total worst case: a small, fixed number of O(1) operations plus one 16-iteration
bounded scan.** This is genuinely O(MAX_PTASKS), not larger. This is the mechanism's
*algorithmic* WCET; §6 covers why the *measured* time can still be much larger on
Xtensa specifically.

---

## 6. Critical-section WCET: the headline finding

`rivet::critical::enter` is the single primitive every bound above and below
assumes is short. Two independent findings from real-hardware measurement say
it is not always short, for reasons that have nothing to do with the scheduler's
own algorithm.

### 6.1 `console_write` under `critical::enter` is baud-rate-bound, not CPU-bound

`rivet-bsp-esp32s3/src/lib.rs`'s `__rivet_board_console_write` wraps its *entire*
byte-buffer write, not per byte, in one `critical::enter` call, and the inner
loop busy-waits on UART TX FIFO space:

```rust
rivet::critical::enter(|| {
    for &b in bytes {
        while uart0.status().read().txfifo_cnt().bits() >= 124 { spin_loop(); }
        uart0.fifo().write(...);
    }
});
```

Once the roughly 128-byte hardware FIFO saturates (any string longer than the
FIFO's headroom, printed faster than the wire drains: true by a huge margin at
40 MHz CPU clock versus serial baud rate), **the loop is waiting on physical
transmission time, not CPU cycles**. At an assumed 115200 baud (8N1, 10
bits/byte), that's **roughly 86.8 µs, 3472 cycles at 40 MHz, per byte**, for
the entire remainder of the string, with interrupts (and the cross-hart lock)
held the whole time.

**MEASURED**, `Kind::CriticalSection` max, exact values via added max-cycle
tracking (not just the log2-bucketed histogram, whose top bucket `[2^15, ∞)`
cannot itself answer "how large," only "at least 32768"):

| Workload | Config | `critsec` max (cycles) | approx time @ 40 MHz |
|---|---|---|---|
| `stress_load_bench` | dual-hart | 222,033-222,056 | 5.55 ms |
| `smp_latency_bench` | dual-hart | 222,506 | 5.56 ms |
| `report_test` | **single-hart** (no cross-hart contention possible) | **219,288** (identical across 1024-byte and 16384-byte task-stack builds) | 5.48 ms |

The single-hart result rules out lock contention as the cause (nothing to
contend with on one hart) and rules out task-stack size as a factor (identical
value across a 16x stack-size change). It isolates the cost to `report()`'s
console output length, consistent with the roughly 63-byte-at-115200-baud
arithmetic above. **This is a real, reproducible, roughly 5.5 ms critical
section, about three orders of magnitude larger than the scheduler's own
algorithmic bound in §5.**

**Consequence for a hard-RTOS deployment**: `rivet::console::write_str` /
`rivet::report()`, as currently implemented on the ESP32-S3 board, disables
*every* interrupt on *both* cores for however long the printed text takes to
physically clear the UART. This is unbounded in principle (grows linearly with
string length past roughly 128 bytes) and already reaches multiple
milliseconds with ordinary diagnostic output. **Any code path that can reach
this, including fault/panic handlers, must be excluded from a hard-real-time
task's blocking analysis, or the console must move to the already-implemented
interrupt-driven path (`write_bytes_irq`, SPSC-buffered, bounded by ring size
not string length) before this board can carry a genuine hard-real-time
guarantee.**

### 6.2 `critical::enter`'s cross-hart lock has no fairness bound

`rivet/src/critical.rs`'s cross-hart spinlock (`LOCK_OWNER`, a raw
`compare_exchange_weak` test-and-set) provides no FIFO or bounded-wait guarantee.
Formally, a hart spinning to acquire it has no algorithmic upper bound on wait
time, only an empirical one, driven by how long *other* critical sections
happen to run (which §6.1 shows can itself be large). `Kind::DispatchDecision`
(a timestamp that spans `critical::enter`'s own acquire wait plus the actual
decision) reached **166,597 cycles** worst-observed on real dual-hart
hardware, consistent with contending against a §6.1-class console write
elsewhere in the system.

**This is a genuine limitation of the current lock, not a measurement
artifact.** A formal hard-RTOS certification cannot derive a closed-form bound
on `critical::enter` contention from the algorithm alone. The practical bound
is "the longest critical section anywhere in the linked binary" (§6.1's
roughly 5.5 ms, today), which is a property of the *application*, not a
kernel-provided guarantee.

---

## 7. Priority-inheritance blocking-time bound (`PriorityMutex`)

### 7.1 Kernel-mechanism overhead (exact, DERIVED operation counts)

| Path | Operations | Bound |
|---|---|---|
| `lock()` (uncontended) | 1 CAS (`try_acquire`) + `push_held` (up to 4-slot linear scan) + `boost_holder` (O(1)) | **at most 6 bounded ops** |
| `lock()` (contended, blocks) | above + `add_waiter` (up to 16-slot CAS scan) | **at most 22 bounded ops** |
| `PriorityMutexGuard::drop` (unlock) | owner swap (O(1)) + `remove_held` (up to 4) + effective-priority recompute (up to 4 held mutexes × up to 16-slot `highest_waiter_priority` scan **each** = up to 64) + `set_effective_priority` (O(1)) + `wake_all_waiters` (up to 16 waiters × O(1) each: `unblock` + `cancel_ptask_deadline`) | **at most 87 bounded ops**, the single most expensive *bounded* kernel path in the crate |

All of these are compile-time-constant-bounded (`MAX_PTASKS=16`,
`MAX_HELD_MUTEXES=4`), genuinely O(1) in the formal sense for a fixed
configuration, just with a real constant (roughly 87) worth stating exactly
rather than waving away as "small."

### 7.2 Formal blocking bound (classical priority-inheritance protocol)

Using the standard Sha/Rajkumar/Lehoczky (1990) basic priority-inheritance bound,
adapted to this kernel's nesting support (`MAX_HELD_MUTEXES = 4` deep):

```
B ≤ min(n, m) × (C_max + O_unlock)
```

where **n** is the number of lower-priority tasks in the system that can lock a
mutex the blocked task also needs, **m** is the number of distinct mutexes the
blocked task can wait on (bounded by how many the *lower-priority* holder(s)
can nest, at most `MAX_HELD_MUTEXES = 4`), **C_max** is the longest
*application* critical-section body protected by any such mutex, and
**O_unlock**, roughly 87 bounded operations (§7.1), is the kernel's own fixed
unlock overhead added on top.

**The kernel can bound `O_unlock` exactly. It cannot bound `C_max`**, since that
is inherently an application property. §6.1 is the concrete warning: if any
mutex-protected critical section on this board can reach console I/O (directly,
via a nested call, or via a fault handler triggered while holding the lock),
`C_max` is not a small number. It is UART-baud-rate-bound and effectively
unbounded for certification purposes. A hard-real-time task set on this kernel
must audit every `PriorityMutex`-protected region for exactly this hazard before
`B` can be treated as bounded at all.

---

## 8. Stack WCET

| Arch | Fixed interrupt-frame cost (per nesting level) | Handler call-depth cost | `MIN_TASK_STACK` |
|---|---|---|---|
| RISC-V | **128 bytes**, exact (`FRAME_WORDS=32 × 4`), charged once regardless of handler complexity. Handler body runs on a separate 2 KB per-hart `.isr_stack` | None (isolated) | (not applicable) |
| Cortex-M3 | **64 bytes**, exact (32 HW-auto-stacked + 32 SW `r4`-`r11`), handler runs on MSP | None (isolated) | **128 bytes** (`rivet-arch-cortex-m/src/lib.rs`'s own `MIN_TASK_STACK` constant, PendSV frame + trampoline slack) |
| Xtensa (ESP32-S3) | **256 bytes** (`XT_STK_FRMSZ`, `xtensa-lx-rt`), plus `CONTEXTS[id]` is a further 136-byte struct (not stack-resident, but the same size class) | **Not isolated: full Rust handler call depth (scheduler + `critical::enter` + mutex logic) is charged against the interrupted task's own stack** | No fixed constant; empirically **4096 bytes was insufficient** post-fix (`docs/realtime.md` §15) for a task with a deep `console::write_str`→`print_u64`→`write_bytes` call chain at the preemption point; **8192 bytes was still insufficient**; **16384 bytes (4x original) was the smallest size that measured clean, 6/6 runs** |

**Recommendation for any Xtensa-based hard-real-time deployment**: do not treat
`MIN_TASK_STACK`-style constants as portable from RISC-V/Cortex-M reasoning to
this arch. Every task's stack budget must include the deepest interrupt-context
call chain *this kernel version* can execute (currently: `on_tick` → up to two
`critical::enter` calls → `schedule()`/mutex logic), re-measured whenever that
call chain changes, not derived analytically from a fixed frame size the way
RISC-V/Cortex-M allow.

---

## 9. Known limitations found during this analysis (reported, not fixed, per scope)

1. **`crate::latency::Kind::SchedulingWake`'s cycle-delta computation is not
   wraparound-safe.** `on_dispatch` computes `now.wrapping_sub(ready_at)` on a
   truncated 32-bit cycle count; once the hardware `CCOUNT` register wraps
   (roughly 107 s at the assumed 40 MHz), a sample straddling the wrap produces
   a bogus multi-billion-cycle "latency." Observed directly: max reported as
   4,282,267,520-4,282,837,995 cycles (roughly 107 s, the wraparound period
   itself, not a real latency) on longer-running workloads, while a
   short-lived workload (`report_test`, single-hart) reported a sane
   13,672-23,278 cycle max. **The `SchedulingWake` histogram's `max_cycles()`
   value is unreliable for any run long enough to wrap the counter and should
   not be used for certification until fixed** (e.g. a 64-bit monotonic
   timestamp, or explicit wraparound-aware subtraction). The bucketed
   histogram's own top bucket is likely contaminated by the same artifact.
2. **`critical::enter`'s cross-hart lock has no fairness/bounded-wait
   guarantee** (§6.2), not a bug, a documented design choice
   (`critical.rs`'s own module docs call it a "raw test-and-set"), but one a
   formal WCET analysis must treat as an open, not closed, bound.
3. **`console_write`'s critical section is baud-rate-, not CPU-, bound**
   (§6.1), the most severe finding in this document. Not fixed here (out of
   scope for an analysis pass), but it should be treated as blocking for any
   actual hard-real-time certification of the ESP32-S3 board as shipped.
4. **ESP32-S3's `CPU_HZ = 40_000_000` is unverified against real silicon**
   (§2). Every time-domain (µs/ms) figure for that board inherits this
   uncertainty; the cycle-domain figures (this document's actual measurements)
   do not.

---

## 10. Summary table (exact figures, one screen)

| Metric | RISC-V (QEMU) | Cortex-M3 (QEMU) | Xtensa ESP32-S3 (real HW) |
|---|---|---|---|
| Interrupt latency (assert → handler) | 34 insns, DERIVED | 12 cycles, ARCHITECTURAL | **202-224 cycles, MEASURED** |
| Context switch (mechanism only) | 76 insns, DERIVED | 10 insns + 24 cyc HW, DERIVED | 512-1023 cyc typical; **97,133-166,597 cyc worst-observed (lock contention)**, MEASURED |
| Scheduling decision, algorithmic bound | O(1) + 16-iter scan, DERIVED (arch-independent) | same | same |
| Worst measured critical section | not measured (no real HW) | not measured (no real HW) | **219,288-222,506 cycles, roughly 5.5 ms, MEASURED** (console I/O, §6.1) |
| Mutex unlock, kernel overhead | at most 87 bounded ops, DERIVED (arch-independent) | same | same |
| Interrupt-frame stack cost | 128 B, fixed, isolated | 64 B, fixed, isolated | 256 B **+ full handler call depth, not isolated** |
| Task stack sizing basis | fixed constant works | fixed constant works (`MIN_TASK_STACK`) | **must re-measure per kernel version, no fixed constant is safe** |

**Bottom line**: the scheduler's own algorithm is genuinely, provably bounded
(§5, §7.1). Every kernel-internal operation traced here terminates in a
small, exact, `MAX_PTASKS`/`MAX_HELD_MUTEXES`-driven constant. The actual
obstacles to a hard-real-time guarantee on the currently shipping ESP32-S3 board
are external to that algorithm: an unfair lock (§6.2), a baud-rate-bound console
path that can hold it (§6.1), and a stack-sizing discipline that isn't portable
from the other two architectures' isolated-ISR-stack model (§8).
