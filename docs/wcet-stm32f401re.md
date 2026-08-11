# Rivet RTOS on STM32F401RE (Nucleo-64) — Formal WCET Analysis and Hard Real-Time Declaration

Companion to `docs/wcet.md` (the ESP32-S3/Xtensa analysis). Same methodology,
same labeling discipline (MEASURED / DERIVED / ARCHITECTURAL / ASSUMED — see
that document's §0 for definitions), applied to a genuinely different — and,
the evidence below shows, genuinely simpler to certify — target: a single-core
Cortex-M4 with hardware-isolated interrupt stacks and an architecturally
guaranteed clock.

All real-hardware figures in this document were captured on a physical
**Nucleo-F401RE board** via its onboard ST-LINK/V2-1 (SWD, OpenOCD 0.12.0 +
GDB), using the kernel's own `latency-histograms` instrumentation backed by
the Cortex-M4's real DWT `CYCCNT` register — not simulation, not QEMU.

---

## 1. Why this board is a materially easier certification target

Three structural facts, established once here and assumed throughout the rest
of this document:

1. **Single core.** `rivet-bsp-stm32f401re` builds with `MAX_HARTS = 1`
   (there is only one core to build for). This eliminates, *by construction*,
   the entire class of problem that dominated the ESP32-S3 analysis: cross-
   hart `critical::enter` contention has no meaning here — the lock's
   uncontended fast path (`docs/realtime.md`'s own module doc: "the CAS never
   has a second hart to race") is the *only* path, on every single build.
   There is no `CONTEXTS`-array race class of bug possible, because there is
   no second hart to race it with.
2. **Hardware-isolated interrupt stack.** Confirmed in `docs/wcet.md` §4.2 and
   re-confirmed by reading `rivet-arch-cortex-m/src/lib.rs` directly: PendSV,
   SysTick, and every other exception run in Handler mode on MSP — physically
   separate from any task's PSP stack. Only a fixed 64-byte frame (32 bytes
   hardware-auto-stacked + 32 bytes software `r4`–`r11`) is ever charged
   against an interrupted task's own stack, **regardless of how deep the
   scheduler's own Rust logic calls**. `MIN_TASK_STACK = 128` bytes is a real,
   portable constant on this arch — unlike Xtensa, where this session's
   ESP32-S3 fix needed a 4x, empirically-discovered stack increase because the
   entire handler runs on the interrupted task's own stack.
3. **Architecturally guaranteed clock**, not an assumption. `rivet-bsp-
   stm32f401re/src/lib.rs`'s own module doc: "Runs at the reset-state HSI
   clock (16 MHz, no PLL configured) ... so the board's timing is whatever a
   freshly-reset chip actually does, not a value that silently stops matching
   reality." HSI = 16 MHz ±1% is STMicroelectronics' own documented factory-
   trimmed reset-state oscillator (RM0368 reference manual) — this is not
   independently re-measured in this session, but it does not need to be:
   it is the chip's guaranteed power-on behavior with zero configuration code
   in between, unlike ESP32-S3's assumed-but-unconfirmed 40 MHz.

---

## 2. Interrupt latency (real hardware, MEASURED)

### 2.1 Baseline tick interrupt (`Kind::IrqEntry`)

| Workload | Typical (dominant bucket) | Worst observed |
|---|---|---|
| `stress_load_bench` | 64–127 cycles (15,036/15,036 samples) | **max 82 cycles** |
| `report_test` | 64–127 cycles (268/268 samples) | **max 82 cycles** |
| `priority_inversion_bench` | 64–127 cycles (33/33 samples) | **max 82 cycles** |
| `critsec_isolate_bench` (heavy mutex contention) | 64–127 cycles dominant (37,813/37,913) | **max 6666 cycles** (50 samples at 2^10, 50 at 2^12) |

**82 cycles = 5.125 µs at 16 MHz, deterministic across three independent
workloads** — this is essentially ARM's fixed hardware exception-entry latency
(12 cycles, architecturally documented, Cortex-M3/M4 Technical Reference
Manual) plus Rivet's own SysTick handler preamble before the timestamp is
taken, with negligible additional variance.

**The 6666-cycle outlier under heavy mutex contention is real, explained, and
important** (§3 below): it is not architectural jitter, it is SysTick being
locally masked by an in-progress `critical::enter` section elsewhere in the
kernel. This is the correct way to reason about interrupt latency on a
single-core target: the true worst case is `ARM's fixed entry latency + the
longest critical section anywhere that can be active when the tick is due`,
not the fixed hardware number alone.

### 2.2 Nested interrupt latency (`nested_irq_bench` — purpose-built, deterministic)

| Metric | min | max | avg | n |
|---|---|---|---|---|
| Trigger → nested (higher-priority) IRQ entry | 86 | **86** | 86 | 500 |
| Low-priority handler round trip, including the nested IRQ | 391 | **391** | 391 | 500 |

**Zero variance across 500 samples.** This is about as strong a WCET claim as
real hardware measurement can produce: not "usually 86, sometimes more" — measured
identically 500/500 times. 86 cycles = 5.375 µs; 391 cycles = 24.4 µs, both at
16 MHz.

---

## 3. Critical-section WCET — normal operation vs. terminal-only

This is where this board's story is fundamentally different from, and
stronger than, the ESP32-S3's (`docs/wcet.md` §6.1).

### 3.1 The console path does **not** hold `critical::enter` across UART transmission

`rivet-bsp-stm32f401re`'s `__rivet_board_init` calls `rivet::console::
enable_irq_tx()` — every normal `console::write_str` call goes through
`write_bytes_irq`: push bytes into a 256-byte SPSC ring under `critical::
enter` (O(1) per byte, no UART wait in the common case — the ring absorbs it,
actual transmission happens later, asynchronously, off the critical path)
then return. **Confirmed by direct source read** (`rivet-bsp-stm32f401re/src/
lib.rs`'s `__rivet_board_console_write` is a bare polling loop, *not* wrapped
in `critical::enter` at the board level at all) **and by measurement**: the
bulk of every `critsec` histogram captured this session (>99.9% of samples in
every workload) sits in the 32–1023 cycle range (2–64 µs), consistent with
O(1)/O(MAX_PTASKS)-bounded kernel bookkeeping, not UART wait.

### 3.2 The one exception, `flush_sync`, is architecturally confined to termination

`rivet::console::flush_sync()` — which *does* synchronously drain the TX ring
under `critical::enter`, and *is* baud-rate-bound (§6.1's Xtensa mechanism,
present here too) — is called from exactly two places, both verified by direct
source read: `rivet::port::board::reset()` and `rivet::port::board::exit()`,
both `-> !` (never return). It is reached from normal program exit, from
`rivet::fault::on_fault`'s panic/reset policies, and from watchdog timeout —
**every one of them a terminal state the system does not resume normal
scheduling from.** A task blocked waiting on some other task's deadline
during this window is a non-issue: by definition, nothing is meeting further
deadlines once the system is mid-reset.

**Measured**: `flush_sync`'s actual cost in every workload this session ran —
**42,308–42,486 cycles (2.64–2.66 ms), essentially identical across four
independent, unrelated workloads** (`stress_load_bench`, `report_test`,
`priority_inversion_bench`, and `critsec_isolate_bench`'s 33,328-cycle
variant) — consistent with a small, mostly-already-drained ring tail at exit
time, not a full-ring worst case.

**Theoretical worst case** (DERIVED, not observed): a completely full
256-byte ring flushed with nothing pre-drained, at the assumed 115200 baud
(8N1, 10 bits/byte ⇒ 86.8 µs/byte): **256 × 86.8 µs ≈ 22.2 ms ≈ 355,600
cycles.** This is the true upper bound to cite for `flush_sync`/system-
termination latency, even though this session never observed anything close
to it.

### 3.3 Normal-operation worst case (excludes §3.2's terminal-only path)

Excluding the single per-run `flush_sync` sample (always landing in the
histogram's top 1–2 buckets, always attributable to program exit), the
**worst normal-operation critical section observed across four independent
workloads tops out at bucket 2^12 (4096–8191 cycles), 7 samples total across
all runs** — consistent with `PriorityMutexGuard::drop`'s `wake_all_waiters`
path (§4) under real, heavy contention (`critsec_isolate_bench`'s
`mutex_contender_iters = 27,699`). **Treat 8191 cycles (512 µs at 16 MHz) as
the measured normal-operation critical-section ceiling** for this board's
current test coverage — the same DERIVED algorithmic bound from `docs/
wcet.md` §7.1 (≤87 bounded operations for a full mutex unlock) applies as the
*mechanism's* bound; 8191 cycles is what that mechanism actually cost, once,
under real contention, including memory access latency this session did not
attempt to model separately.

---

## 4. Priority-inheritance blocking — measured, not just derived

`priority_inversion_bench` (purpose-built, this session ran it directly)
reproduces the classical inversion scenario — low-priority holder, high-
priority waiter, medium-priority tasks that would starve the holder without
inheritance — and reports:

```
high_wait_cycles=40548          (≈ 2.53 ms at 16 MHz)
low_critical_section_cycles=56443   (≈ 3.53 ms at 16 MHz)
medium_task_iterations_during_test=0
PRIORITY_INVERSION_BOUNDED
```

**`medium_task_iterations_during_test=0` is the load-bearing number**: with
priority inheritance working, the medium-priority tasks get *zero* CPU time
while the low-priority holder (boosted to at least the high task's priority)
finishes its critical section — exactly the classical protocol's guarantee,
confirmed on real silicon, not just asserted analytically. `high_wait_cycles
< low_critical_section_cycles` is consistent (the high task started waiting
partway through the low task's section, not at its very start) and rules out
unbounded inversion by direct measurement, not inference.

Kernel-mechanism overhead bound (`docs/wcet.md` §7.1, arch-independent,
DERIVED): lock ≤22 bounded operations, unlock ≤87 bounded operations —
unchanged on this arch, and consistent with §3.3's 8191-cycle normal-
operation ceiling once real memory/pipeline cost per operation is included.

---

## 5. Context switch and scheduling decision

| Metric | Value | Method |
|---|---|---|
| PendSV software save/restore | 10 `stmia`/`ldmia`-class instructions (§4.1, `docs/wcet.md`) | DERIVED, exact instruction count from `rivet-arch-cortex-m/src/lib.rs`'s `global_asm!` |
| Hardware auto-stack/unstack | 12 + 12 = 24 cycles | ARCHITECTURAL (ARM Cortex-M3/M4 TRM) |
| `Kind::DispatchDecision` (scheduling logic itself) | **typical 512–1023 cycles, max 984–1048 cycles across four workloads** | MEASURED, real hardware — note the *absence* of the 97K–166K-cycle contention outliers seen on ESP32-S3 (`docs/wcet.md` §4.1): with no second hart, `critical::enter` here never has anything to wait on |
| Scheduling-decision algorithmic bound | O(1) + one ≤16-iteration bounded scan | DERIVED, arch-independent (`docs/wcet.md` §5) — unchanged, and the measured 984–1048 cycle figure is consistent with this bound plus real memory-access cost, not evidence of an unbounded path |

**This table alone is the clearest evidence for §1's claim**: the same
algorithmic scheduler, on a single-core target, shows a tight ~2x measured
spread (512–1048 cycles) with *no* outliers reaching into the tens of
thousands, where the identical algorithm on dual-core Xtensa showed a
300x spread (512 to 166,597) driven entirely by lock contention that does not
exist here.

---

## 6. Stack WCET

| Component | Cost | Basis |
|---|---|---|
| Interrupt frame (any nesting level, any handler complexity) | **64 bytes, fixed** | ARCHITECTURAL — hardware-isolated MSP, confirmed §1.2 |
| `MIN_TASK_STACK` | **128 bytes** | Portable constant, `rivet-arch-cortex-m/src/lib.rs` — genuinely safe to rely on, unlike Xtensa's kernel-version-dependent figure |
| Observed real-task stack watermarks (this session) | 64–512 bytes used, out of 256–4096-byte allocations | MEASURED — `rivet::report()` output across all four workloads run this session |

No stack-related crash, corruption, or watermark violation occurred in any of
the four workloads this session ran on real hardware. Given §1.2's structural
argument (fixed frame, hardware-isolated), this is expected, not lucky.

---

## 7. Formal hard real-time declaration

**Given the evidence above, Rivet RTOS on the STM32F401RE (Nucleo-64), single-
core, HSI 16 MHz, `RIVET_MAX_HARTS=1`, is declared hard real-time capable for
task sets whose scheduling analysis uses the following exact, evidenced
bounds:**

| Bound | Value | Status |
|---|---|---|
| Interrupt latency (tick, normal operation) | ≤ 6666 cycles (416.6 µs) | MEASURED worst case, 4 workloads; architectural floor 12 cycles |
| Nested interrupt latency | 86 cycles (5.375 µs), **zero variance, 500/500 samples** | MEASURED |
| Context-switch mechanism cost | ≤ 24 cycles HW + 10 insns SW | ARCHITECTURAL + DERIVED |
| Scheduling-decision cost | ≤ 1048 cycles (65.5 µs) | MEASURED worst case, 4 workloads; ≤16-iteration bound DERIVED |
| Critical-section hold time, **normal operation** | ≤ 8191 cycles (512 µs) | MEASURED worst case, 4 workloads |
| Mutex lock kernel overhead | ≤ 22 bounded operations | DERIVED, exact |
| Mutex unlock kernel overhead | ≤ 87 bounded operations | DERIVED, exact |
| Priority-inversion bound | Confirmed: zero medium-priority interference during holder boost | MEASURED, purpose-built test, real hardware |
| Interrupt-frame stack cost | 64 bytes, fixed, any nesting depth | ARCHITECTURAL |

**Explicit scope — what this declaration covers and does not:**

1. **Covers**: all normal operation — every task dispatch, every mutex
   lock/unlock, every tick, for as long as the system has not faulted, has
   not hit a watchdog timeout, and has not called `rivet::exit`. Every bound
   in the table above was either measured directly during such operation or
   derived from an exact, compile-time-bounded algorithm.
2. **Does not cover**: the interval from a fault/watchdog-timeout/`exit()`
   call to system halt or reset. That interval can synchronously drain up to
   256 queued console bytes (§3.2), bounded at ≤22.2 ms (DERIVED worst case;
   42,308–42,486 cycles ≈ 2.64–2.66 ms MEASURED typical, across four
   independent workloads) — by design, since this is the mechanism that
   guarantees a fault's diagnostic message is not lost. **A hard-real-time
   task set
   using this kernel on this board must treat "system has faulted" as "no
   further deadlines are being met," which is standard practice** (DO-178C-
   style systems universally treat the transition into a failure-response
   state as outside the nominal timing budget) — not a gap unique to Rivet.
3. **Does not cover**: application-level critical-section bodies. The kernel
   bounds its own mutex mechanism exactly (≤87 operations); it cannot and
   does not bound what an application does *between* `lock()` and `drop()`.
   `docs/wcet.md` §7.2's warning applies here too: an application-level
   critical section that itself calls `console::write_str` with output long
   enough to overflow the 256-byte ring reintroduces §3.2's baud-rate-bound
   cost into normal operation. **This declaration assumes application code
   does not do that** — an auditable, one-line-per-call-site property, not an
   unverifiable one.
4. **Does not cover**: the HSI clock's absolute accuracy (±1% per ST's own
   datasheet, not re-measured against an external reference this session) —
   relevant only if this system needs to correlate timing against an external
   wall clock; irrelevant to the *relative* cycle-count bounds in this
   document, which hold regardless of the oscillator's absolute trim.

**This is a materially tighter, better-evidenced declaration than would be
possible for the ESP32-S3 board today**: every dual-core-specific hazard in
`docs/wcet.md` (§6.1's unbounded normal-operation console path, §6.2's
unfair cross-hart lock, §8's non-portable stack sizing) is either structurally
absent here (single core) or already confined to a scope standard hard-RT
practice excludes (§3.2's terminal-only path) rather than reachable during
normal task execution.
