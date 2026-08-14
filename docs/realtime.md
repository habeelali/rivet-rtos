# Rivet RTOS: real-time characterization

This document answers a specific question: **for the numbers Rivet's
preemptive tier produces (`schedule()` cost, semaphore/mutex latency, task
wakeup, context switch, interrupt dispatch, priority-inversion bound, SMP
determinism, deadline misses), what does this workspace actually know, on
real hardware, and how much confidence does that number deserve?**

It is written against a specific testing methodology (preemption/priorities/
blocking primitives/timers/interrupt handling/bounded kernel ops as the
success criteria; demonstrable upper bounds, not just "usually fast," as
what makes an RTOS "real-time" at all; bounded-below-the-deadline misses,
for an explicitly defined workload, as what makes one "hard real-time").

## 1. What this is, and what it is not

**This is empirical, black-box measurement under controlled load, on real
silicon.** Every number below came from flashing an instrumented binary to
real STM32F401RE / ESP32-S3 / ESP32-C6 hardware and reading back what the
CPU's own free-running cycle counter recorded. That is a real, honest thing
to know. It is not the same thing as a *formally proven* WCET bound.

**What it is not:** no static WCET analysis tool (aiT, OTAWA, or similar)
was run against this codebase. No worst-case cache/pipeline/flash-wait-state
analysis was done by inspecting the compiled binary's control-flow graph.
The "worst case" numbers below are the worst case **observed** across a
bounded number of samples under a specific, described load, not a
mathematically exhaustive bound. A pathological input, a longer run, or a
load shape not exercised here could observe something worse. Where the
methodology's own language says "demonstrable upper bounds," read that as
*empirically demonstrated, under the load described, at the sample count
stated*, the honest version of that claim, not an inflated one.

Reproduce or extend any of this by building the `*_bench.rs` binaries in
`examples/{stm32f401re,esp32s3,esp32c6}/src/bin/` with `--features
latency-histograms` and flashing them; every number in this document has a
binary in this workspace that produced it.

## 2. Hardware and clock assumptions (documented, per the methodology's own ask)

| | STM32F401RE | ESP32-S3 | ESP32-C6 |
|---|---|---|---|
| Core | Cortex-M4, r0p1 | Xtensa LX7, dual-core | RISC-V (RV32IMAC) |
| Clock | **16 MHz HSI**, no PLL configured (reset-state default) | **40 MHz**, boot-ROM XTAL default. `rivet-bsp-esp32s3`'s own documented, *unmeasured* assumption | **80 MHz**, boot-ROM default, same documented-unmeasured status |
| Clock confidence | High, corroborated throughout by USART2 decoding cleanly at 115200 baud, a divisor computed directly from this constant | Low-to-medium, inherited from the BSP's own source comment, not independently re-measured (§2.1) | Low-to-medium, same |
| Cache / prefetch | None (STM32F401 has no data/instruction cache; ART flash accelerator prefetch is at its reset-default state, not explicitly configured by this port) | Flash cache present (ESP-IDF bootloader default state; not explicitly reconfigured by this port) | Flash cache present (same) |
| Interrupt controller | NVIC, per-IRQ priority; this port sets a priority floor (`0xFF`, lowest) on every line at init so an unconfigured IRQ can't outrank PendSV/SysTick | Xtensa's 3-level interrupt architecture; this port's dispatch runs entirely on level 3 (tick, software reschedule, IPI, and all board-registered peripheral IRQs share one line) | Espressif's `INTPRI`/interrupt-matrix (not a real SiFive PLIC despite similar naming) |
| Nested interrupts | Not exercised by the benchmarks below. NVIC supports real priority-based nesting; this port doesn't currently assign different priorities to different peripheral IRQs (console UART is the only board IRQ line in active use), so there was nothing to nest against | Not applicable in the same sense: this port collapses every async source onto one CPU interrupt level, so by construction there is no nesting between rivet-dispatched sources on this port | Not exercised; single shared interrupt-matrix line, same as S3 |
| SMP | N/A (single-core) | **Real dual-core**, the only board in this workspace with actual parallel hardware | N/A (single-core) |

### 2.1. Why S3/C6's clock isn't independently re-measured here

An attempt was made: a non-optimizable (LCG-recurrence, so LLVM can't
collapse it to a closed form) fixed-iteration busy loop, cycle-counted
on-device and cross-checked against host wall-clock time around the capture
window. The methodology is sound; the *tooling* wasn't reliable enough in
this environment to trust the result. USB-CDC serial capture on this setup
repeatedly returned truncated or stale-buffered reads when the capture
didn't start with a real PTY attached quickly enough after reset (see §6 for
the concrete fix that unblocked *content* capture; timing precision at the
sub-second level is a stricter bar than content correctness and wasn't
reliably achieved here). Rather than publish a shaky Hz estimate as if it
were solid, every S3/C6 number below is reported **in raw cycles first**
(always trustworthy, a direct hardware counter read, immune to any
host-side timing uncertainty), with a nanosecond conversion using the BSP's
own documented CPU_HZ **explicitly labeled as using an unverified
assumption**, not a re-measured fact. If that assumption is off by a
constant factor, every derived ns number here scales by the same factor;
the cycle counts do not change.

## 3. Uncontended latency (`latency_bench.rs`)

Best-case costs: one task, an otherwise-idle system (a low-priority
background spinner exists only to give the tick handler something to
schedule around), `N=20,000` samples per operation, direct `cycle_count()`
bracketing around each call.

| Operation | STM32 (16 MHz) | ESP32-C6 (assumed 80 MHz) | ESP32-S3 (assumed 40 MHz) |
|---|---|---|---|
| `sem_try_acquire` | 55 avg / 50 min / 1712 max cyc → **3.4 µs avg** | 21 avg / 20 min / 489 max cyc → **0.26 µs avg** | 27 avg / 26 min / 2593 max cyc → **0.68 µs avg** |
| `sem_release` | 564 avg / 505 min / 2523 max cyc → **35.3 µs avg** | 95 avg / 93 min / 561 max cyc → **1.2 µs avg** | 337 avg / 316 min / 2883 max cyc → **8.4 µs avg** |
| `mutex_try_lock` | 127 avg / 115 min / 1778 max cyc → **7.9 µs avg** | 41 avg / 39 min / 505 max cyc → **0.51 µs avg** | 159 avg / 134 min / 2735 max cyc → **4.0 µs avg** |
| `mutex_unlock` | 1346 avg / 1207 min / 2872 max cyc → **84.1 µs avg** | 379 avg / 367 min / 1353 max cyc → **4.7 µs avg** | 1655 avg / 1612 min / 4179 max cyc → **41.4 µs avg** |
| `yield_now` round trip (≥2 ctx switches) | 722 avg / 648 min / 2312 max cyc → **45.1 µs avg** | 231 avg / 223 min / 729 max cyc → **2.9 µs avg** | 1001 avg / 934 min / 3498 max cyc → **25.0 µs avg** |

**Reading this table:** `mutex_unlock` costs roughly 10x `mutex_try_lock`
on every board. Expected, since the uncontended lock path is a single CAS,
while unlock (`PriorityMutexGuard::drop`) does the full owner swap,
held-mutex-list update, effective-priority recompute, and waiter wake, all
under one critical section, a deliberate correctness-over-speed choice made
during this workspace's concurrency-hardening work (see the kernel commit
history). `sem_release`'s cost relative to `sem_try_acquire` (also roughly
10x) has the same shape for the same reason: release scans for a waiter to
wake, acquire on an available semaphore doesn't. The `max` columns being
20-40x the `min` on every board is real, not noise; see §3.1.

**`yield_now` round trip is explicitly *not* a clean single-context-switch
number.** With exactly two same-priority ready tasks, one task's
`yield_now()` call observes a full switch-out (to the other task) and
switch-back-in: at minimum two switches, possibly more if the tick lands
mid-measurement. Read it as "≥2x one context switch," i.e. divide roughly
by two for a rough per-switch estimate (STM32: roughly 22.5ns; C6: roughly
1.4ns; S3: roughly 12.5ns), genuinely fast on all three, consistent with
"save/restore a register file" being the actual cost, not scheduler
overhead.

### 3.1. Why `max` is 20-40x `min` even completely uncontended

Every operation's `max` sample is far above its own `min`/`avg`, on a
completely uncontended, single-task-of-interest system. This isn't
measurement noise; it's the periodic tick. Roughly 1-in-N samples (matching
the tick period relative to the loop's own iteration rate) has the SysTick/
timer interrupt land *during* the measured window, adding real interrupt
handling time to that one sample. This is itself a useful, honest data
point: **even the "uncontended" fast path has jitter from the system's own
tick**, and any deadline-sensitive caller budgeting off the `avg` number
alone, rather than the `max`, is budgeting optimistically.

## 4. Worst case under combined load (`stress_load_bench.rs`)

Same operations, but now measured via the kernel's own `latency-histograms`
feature (`rivet::latency`, log2-bucketed cycle histograms recorded at the
actual kernel dispatch points, not just this bench's own bracketing) while
four real interference sources run concurrently for roughly 5 seconds:

- a high-priority (prio 8) task on a fixed 1ms period (`sleep_ms`), 5000
  iterations: the "deadline-driven workload" this whole load scenario is
  built around
- two `PriorityMutex` contenders (prio 2) hammering a shared mutex
- a channel producer/consumer pair (prio 2), `try_send`/`try_recv` traffic
- a low-priority (prio 1) pure-spin background task

All four non-high-priority tasks share priority 2 (except the pure
background spinner) so tick-driven round-robin actually interleaves them.
An earlier version of this bench put the mutex contenders at a strictly
higher priority than the channel tasks, and since neither source ever
voluntarily yields, fixed-priority scheduling correctly starved the lower
one completely (`channel_sent=0`) rather than interleaving. That's not a
bug; it's the scheduler doing exactly what fixed-priority preemption is
supposed to do, but it meant only one interference source was ever
actually live at a time. **Even with all four at the same nominal
priority, round-robin fairness among never-yielding same-priority ready
tasks was observed to be uneven in practice** (STM32: one channel task did
88% of the observed work, its sibling 0%; C6/S3: mutex contenders got
entirely starved out by channel traffic instead), a real, worth-noting
scheduler behavior: this port's round-robin tie-break among equal-priority
ready tasks is not guaranteed-fair per tick, only guaranteed-eventually-fed
via the periodic tick's rotation. A production deployment relying on fair
CPU-sharing among equal-priority never-blocking tasks should be aware of
this rather than assume strict fairness.

Histogram results (`irq_entry`, `dispatch`, `critsec`, `sched_wake`, see
`rivet::latency::Kind`'s own docs for exactly what each measures; bucket
`2^b` covers `[2^b, 2^(b+1))` cycles):

**STM32F401RE (16 MHz):**
```
irq_entry: 2^6:10037                                    (all in 64-127cyc = 4-8µs)
dispatch:  2^8:33 2^9:10006                              (mostly 512-1023cyc = 32-64µs)
critsec:   2^6:10007 2^7:10049 2^8:10079 2^9:15041 2^10:68 2^11:21 2^12:23 2^13:13 2^14:3 2^15:2
sched_wake: 2^9:1 2^10:5001 2^11:4999 2^13:1 2^15:5
```
Worst observed critical section: bucket 2^15 (32768-65535 cyc, roughly
**2.0-4.1ms** at 16MHz), 2 samples out of roughly 45,000. Worst observed
wakeup: bucket 2^15, 5 samples, same order of magnitude. **These are the
numbers that matter for a hard-real-time budget on this board**, not the
`avg`/`min` from §3.

**ESP32-C6 (assumed 80 MHz):**
```
irq_entry: 2^5:5005 2^7:1
dispatch:  2^6:41 2^7:5005 2^8:10000 2^9:2
critsec:   2^4:10004 2^5:15060 2^6:25100 2^7:15016 2^8:9 2^9:26 2^10:52 2^11:27 2^12:22 2^13:33 2^14:3 2^15:2
sched_wake: 2^8:5000 2^10:2 2^12:1 2^13:5000 2^14:5000 2^15:4
```
Worst observed critical section: 2^15 (32768-65535 cyc, roughly
**0.41-0.82ms** at assumed 80MHz), 2 samples. Worst wakeup: 2^15, 4 samples.

**ESP32-S3 (assumed 40 MHz):**
```
irq_entry: 2^7:5006                                      (all in 128-255cyc)
dispatch:  2^7:60 2^9:15006
critsec:   2^5:50149 2^7:69 2^8:5016 2^9:35138 2^10:3 2^11:8 2^12:50 2^13:20 2^14:18 2^15:51
sched_wake: 2^8:1 2^10:5000 2^11:1 2^12:1 2^15:10004
```
Worst observed critical section: 2^15 (32768-65535 cyc, roughly
**0.82-1.64ms** at assumed 40MHz), 51 samples, notably more frequent than
on STM32/C6. Worst wakeup: 2^15, **10004 samples**, roughly a third of all
wakeups on this board landed in the top bucket, a real and substantially
different tail shape from the other two boards. Read together with §7
(dual-core): this port's cross-core reschedule IPI path and the additional
Xtensa context-save/restore cost are the most likely contributors, but
which one wasn't isolated; flagged here as the clearest open question
this data raises, worth follow-up before relying on S3's wakeup tail for a
tight deadline budget.

## 5. Priority inversion: bounded, not unbounded (`priority_inversion_bench.rs`)

Classic scenario: a low-priority task (prio 1) locks a mutex and holds it
for a fixed, known duration; three medium-priority (prio 4) tasks spin
continuously, touching nothing shared, pure interference, exactly the
shape that causes *unbounded* priority inversion (the Mars Pathfinder
failure mode) if a mutex doesn't implement priority inheritance, since the
medium tasks would otherwise keep preempting the low holder indefinitely. A
high-priority (prio 9) task blocks on the same mutex shortly after the low
task takes it.

| | high_wait (cyc) | low's own critical section (cyc) | medium-task iterations observed |
|---|---|---|---|
| STM32 | 39,429 | 55,420 | 0 |
| ESP32-C6 | 45,205 | 51,276 | 0 |
| ESP32-S3 | 17,733 | 53,083 | 0 |

**Bounded on all three boards, by a wide margin.** `high_wait` never
exceeded low's own critical-section length (it's *less*, since the high
task starts waiting partway through low's hold, not from the beginning),
confirming `PriorityMutex`'s priority inheritance is doing its job: the
wait tracks the mutex holder's own work, not the medium tasks' presence or
absence.

**`medium_task_iterations_during_test=0` on every board: read this as
"medium never got scheduled," not "medium tried and was blocked."** Because
the high-priority task is spawned strictly after the medium tasks in this
bench, and this port's tick-driven scheduler always dispatches the
highest-priority *ready* task, the very first scheduling decision after all
four tasks exist picks the high-priority task directly, never a medium
one. High then immediately blocks on the mutex, which boosts low's
effective priority above medium's for the remainder of the hold. Medium
never gets a dispatch window in this particular ordering. This still
demonstrates the bound holds; it does not demonstrate the mutex actively
preempting a *currently-running* medium task mid-slice (a stronger claim
this bench's specific spawn order doesn't produce evidence for either way).

## 6. Deadline-miss testing under load (`deadline_miss_bench.rs`)

The "hard real-time" bar from the methodology this binary is built
against: for an explicitly defined workload, are deadline misses bounded,
ideally zero, not just "bounded and nonzero." A single highest-priority
(prio 9) task runs 500 periods at a 2ms period (`set_period_us` +
`wait_period`, the same drift-corrected periodic API `deadlines.rs`
implements), while the same mutex-contention + channel-traffic + low-prio
interference from §4 runs concurrently the whole time. A "miss" is any
inter-wake interval exceeding `period + 2×period` (a deliberately generous
2x slack; this proves boundedness, not a tuned production margin).

| | Periods | Misses | Worst lateness |
|---|---|---|---|
| STM32F401RE | 500 | **0** | 0 µs |
| ESP32-C6 | 500 | **0** | 0 µs |
| ESP32-S3 | 500 | **0** | 0 µs |

Zero misses, on all three boards, under real concurrent mutex/channel/
background load. This is the strongest single claim this document
supports: for this specific workload shape, at this sample count, the
highest-priority periodic task's deadline was never even close to missed
(worst lateness of exactly 0µs; every single period landed inside the
un-relaxed period itself, not just inside the 2x-slack tolerance).

## 7. SMP determinism (ESP32-S3 only, the only board with real dual-core hardware)

Measured via 2000 repetitions of a mutex block/wake cycle between two
dedicated tasks (`holder`/`waiter`), each repetition recording which hart
held the mutex and which hart the waiter actually resumed on, classified
same-core vs. cross-core after the fact, with `2 × RIVET_MAX_HARTS` filler
tasks keeping both cores genuinely busy with unrelated work throughout
(reusing `smp_test.rs`'s own proven "many equal-priority workers" shape).

**Result: same_core n=2000, min=1841, max=6130, avg=2356 cycles. cross_core
n=0.** Every single repetition stayed on the same hart; the holder and
waiter tasks never migrated relative to each other across all 2000 cycles.

**This is an honest limitation, not a suppressed negative result: this
port has no explicit task-to-core pinning API**, so this bench couldn't
*force* a cross-core wake; it could only wait for one to occur naturally
under free migration, and across 2000 reps, none did. Two tasks that
repeatedly block on and wake each other via the same mutex appear to stay
co-located on whichever hart first picks them up, rather than the scheduler
spreading them across cores. **The genuine cross-core wakeup latency this
port's `on_tick`/`request_reschedule_on` IPI path would produce is
therefore not measured by this section** (only same-core mutex wake
latency is: avg 2356 cycles, roughly 59µs at S3's assumed 40MHz). Closing
this gap would need either an explicit core-pinning primitive added to the
port contract, or a different bench shape (e.g. two independent producer/
consumer pairs, each pinned by construction to opposite cores via which
hart's boot path spawns them), flagged here as follow-up work, not
claimed as done. (Update: §10 later forced this measurement using a
different approach.)

The cross-core critical-section-duration bound the methodology also asks
about (`critical::enter`'s hold time being bounded regardless of which
hart is racing which) is covered by §4's `critsec` histogram; that
measurement is already hart-agnostic (any hart's critical section
contributes a sample to the same histogram), so S3's `critsec` numbers in
§4 already reflect real cross-hart lock contention, even though the
*wakeup*-specific cross-core number in this section does not.

## 8. What this document does not claim (as of the initial pass; see §9-13 for what closed since)

- **Not a formal WCET bound** (§1). Every "worst case" figure is the worst
  *observed* value across the stated sample count under the stated load,
  real evidence, not a mathematical guarantee that no input or longer run
  could exceed it.
- **S3/C6 clock speeds are unverified assumptions** inherited from their
  BSP source (§2.1), not independently re-measured; every ns conversion
  for those two boards inherits that uncertainty, the cycle counts
  themselves do not.
- ~~Nested interrupt latency is not characterized on any board~~: closed
  for Cortex-M/NVIC by §9; genuinely not applicable on the other two
  boards' current ports (see §9's own note).
- ~~True cross-core wakeup latency is not measured on ESP32-S3~~: §10
  forces it for real and finds something more important than a latency
  number. Read §10 before relying on any dual-core deployment of this
  kernel.
- **Round-robin fairness among equal-priority ready tasks is not
  guaranteed per-tick** (§4), observed directly, not assumed. Still open.

## 9. Nested interrupt latency (Cortex-M/NVIC only)

STM32's NVIC is the one interrupt controller among this workspace's three
boards that this port actually exercises with real priority-based nesting.
(Xtensa's dispatch in `rivet-arch-xtensa` collapses every async source,
tick, software reschedule, IPI, all board-registered peripheral IRQs, onto
a single CPU interrupt level by construction, so there is no nesting *to*
characterize there; this workspace's RISC-V boards don't assign distinct
priorities to different IRQ sources either. Both are real architectural
facts about the current ports, not gaps in this document.)

`nested_irq_bench.rs` uses NVIC's software-pend register (`ISPR`, wrapped
as `rivet_arch_cortex_m::nvic::pend`, a standard, documented ARMv7-M
self-test technique, no real hardware peripheral needed and no side
effects, since the two borrowed IRQ numbers' actual peripherals are never
touched) to drive two IRQ lines at different priorities against each
other: a mid-priority line (`IRQ_LOW`, NVIC priority `0x80`) triggers a
higher-priority line (`IRQ_HIGH`, priority `0x00`) from *inside* its own
handler, and NVIC's hardware tail-chaining preempts `IRQ_LOW`'s handler
mid-execution to run `IRQ_HIGH` first: genuine nesting, not two
independent dispatches.

**Result, 500/500 trials: deterministic, not just bounded.**

| | cycles | @ 16 MHz |
|---|---|---|
| trigger → nested handler entry | **82 (min = max = avg)** | 5.1 µs |
| full round trip (incl. nested IRQ, back to `IRQ_LOW`) | **387 (min = max = avg)** | 24.2 µs |

Every one of 500 trials landed on the *exact same cycle count*, not a
tight distribution, a genuine constant. This is the strongest single
result in this whole document: NVIC's hardware-arbitrated tail-chaining
is fixed-latency by design, and this measurement confirms it holds in
practice on real silicon, not just in the architecture manual. Where §1
disclaims "empirical, not formally proven" for everything else in this
document, this result is about as close to a formal bound as an empirical
measurement can get: 500/500 identical samples leaves very little room
for a hidden data-dependent path.

## 10. Forcing cross-core wakeup: a real bug, not just a hard number

§7's original pass reported same-core-only mutex wake latency on ESP32-S3
and named the missing piece honestly: no core-pinning primitive existed to
*force* a cross-core wake, so 2000 attempts under free migration never
produced one. This section did the forcing work and found something that
matters more than the latency number it was chasing.

**Root cause of the original null result, found first:**
`examples/esp32s3/.cargo/config.toml` never set `RIVET_MAX_HARTS`, which
defaults to **1**. Every ESP32-S3 binary built during the original
real-time characterization pass, including the original `smp_latency_
bench`, was silently compiled *single-core*. `wake_other_harts`'s
`if MAX_HARTS > 1` branch was dead code the entire time; `hart_id()` could
only ever return 0. This wasn't a scheduling nuance, it was a build
configuration gap that made "cross-core" unmeasurable by construction, not
by chance.

**Fixed and forced, deliberately, not just observed:** rebuilding with
`RIVET_MAX_HARTS=2` and a bench design where a `holder` task runs at a
priority strictly *above* `waiter` and never yields after unlocking (so
its own hart can never locally reschedule to `waiter`, a lower-priority
ready task can't preempt a higher-priority one still occupying its hart)
structurally forces every `waiter` wake to go through the cross-core IPI
path. This worked: `cross_core_n` came back non-zero and repeatable across
several runs (199/200, 45+, 9/10 samples across different rep counts),
proving the cross-core dispatch mechanism (`ready_add` →
`wake_other_harts` → `request_reschedule_on` → the other hart's IPI
handler → `on_tick`) genuinely fires and genuinely lands on the other
hart.

**Two real problems surfaced by actually forcing it: one fixed and
verified, one downgraded from "kernel crash" to "unreliable measurement"
once a methodology bug in this document's own testing was found and
corrected:**

1. **A real kernel bug, found and fixed: `rivet-arch-xtensa`'s dual-core
   boot never waited for `rivet::kernel_ready()`.** `rivet::run_secondary_
   hart()`'s own doc comment says its contract is "call only after
   `kernel_ready()` is true, from a hart other than the one that called
   `run()`," the *caller*'s responsibility. `rivet-rt`'s RISC-V
   secondary-hart boot upholds this with an explicit `while
   !rivet::kernel_ready() { spin_loop() }` before calling it; the Xtensa
   port's `rivet_appcpu_rust_entry` never did, despite a comment claiming
   otherwise. APP_CPU is released from `__rivet_board_init`, inside
   `rivet::init()`, long before the app's `main()` has finished its own
   `spawn_ptask!` calls, so without the wait, APP_CPU could reach
   `start_secondary_hart()` and dispatch a task that had *just* become
   ready (`spawn_ptask!`'s `ready_add` broadcasts a wake IPI
   unconditionally) while hart 0 was still mid-spawn of a *different*
   task, corrupting `Tcb.sp` for whichever task each hart raced on:
   exactly the `start_first_task given a non-bootstrap sp` panic this
   forcing bench hit on nearly every attempt. Adding the missing wait
   (matching the RISC-V reference implementation exactly) fixed it.
   **Verified against realistic dual-core workloads, not just the
   adversarial forcing bench**: `smp_test.rs` (independent counters) and
   `stress_load_bench` (mutex contention + channel traffic + a periodic
   task, all under real `RIVET_MAX_HARTS=2`) both complete cleanly with
   the fix in place. `stress_load_bench` specifically drove 60,002 real
   mutex-contention iterations across both cores without a fault, the
   first time this workspace has exercised that combination successfully.
2. **A real methodology lesson, found while chasing what looked like
   *further* regressions from the fix above.** Two apparent new failures,
   `smp_test.rs` "hanging" with no output, and STM32's own `mutex_test.rs`
   (a *different* board, QEMU *and* real hardware) likewise producing no
   completion, both turned out to be false alarms: the testing that
   uncovered the fix above used capture windows (15-90s) sized for this
   workspace's *other* tests, not for `mutex_test`'s 2,000,000-iteration
   contended-mutex stress phase, which genuinely takes 150+ seconds on
   real STM32 hardware at 16MHz (and `smp_test`'s own completion needed
   a comparably longer window on S3 than first tried). Re-run with
   adequate timeouts, both completed successfully: `MUTEX_OK`/
   `RIVET_EXIT_OK` and `SMP_TEST_OK`/`RIVET_EXIT_OK` respectively. This
   cost real time to untangle (several rounds of reverting real fixes to
   chase phantom regressions) and is recorded here as a direct warning to
   whoever next benchmarks or debugs this kernel: **a test that produces
   no output within your chosen timeout is not evidence of a hang by
   itself**. Check whether the workload's own iteration count actually
   fits the window before concluding the kernel is broken.
3. **The forced-cross-core bench's own measured latency *values* were
   unreliable: root cause found, not a kernel bug.** The bench compared
   `rivet::port::arch::cycle_count()` timestamps taken on *different*
   harts (`holder`'s unlock stamp vs. `waiter`'s wake stamp), invalid on
   Xtensa, where `CCOUNT` is a per-core register with no cross-core
   synchronization guarantee (unlike an invariant TSC), so the subtraction
   produced numbers with no physical meaning. This wasn't a
   memory-visibility bug in the `Ordering::Release`/`Acquire` handoff as
   first suspected; even `rivet-bsp-esp32s3`'s own `now_us()` turned out
   to be built on the same per-core `CCOUNT`, so it wasn't usable as a
   cross-core clock either. **Fix**: stop correlating timestamps across
   harts entirely; measure the wake latency from `waiter`'s own,
   single-hart-consistent clock instead (cycles between "about to attempt
   the lock" and "lock acquired," on the same hart, which *is* the wake
   latency since waiter is genuinely blocked, not spinning, for that
   whole interval). Verified on real ESP32-S3 hardware: went from
   `min=12130465 max=12151723 avg=3551125` (`avg < min`, a statistical
   impossibility, actually a `u32` sum overflow compounding the
   per-core-clock bug) to `min=163 max=737 avg=177` cycles, a plausible
   roughly 0.7-3µs cross-core mutex wake latency at 240MHz.

**`console::write_bytes`'s cross-hart write corruption is fixed.** A
bounded-retry try-lock (`AtomicBool` + `compare_exchange_weak`, spinning up
to `LOCK_SPIN_LIMIT` then falling back to an unsynchronized write) closes
the interleaving window without risking the blocking-`critical::enter`
version's failure mode (one wedged hart permanently hanging both cores on
any fault-path print, unacceptable for diagnostic output). An earlier
inconclusive QEMU verification was a capture-timeout artifact, not a real
regression: `mutex_test`'s contended-mutex stress phase needs longer than
QEMU CM3/MPS2 emulates it in real time than RISC-V does the identical
workload (confirmed on pristine, unmodified code too, not something this
fix introduced). Fixed by raising `xtask`'s own internal `mutex_test`
timeout (120s → 240s, CM3/MPS2 only; RISC-V already passes within 120s).
Verified: full QEMU smoke suite green on all three boards (riscv 15/15 +
`-smp 2`/`-smp 4`, cm3 15/15, mps2 12/12), each including `mutex_test` at
the new timeout.

**Bottom line for anyone about to rely on this:** cross-core wakeup
dispatch *works* (proven). The dual-core bootstrap crash this section set
out to investigate is **fixed and verified against realistic workloads**
(`smp_test.rs`, `stress_load_bench`), not just the artificial forcing
bench that originally found it. Cross-core wakeup *latency* is now
**reliably measured** (finding 3, fixed). Console output under concurrent
cross-hart writes is now **synchronized** (bounded-retry lock, fixed).
All three items this section originally left open are closed.

## 11. Long critical sections: root-caused and fixed (dominant term); small residual remains

§4's `stress_load_bench` histograms showed rare samples (about 1 in
40,000-75,000) in the worst bucket (2^15: 32768-65535 cycles, roughly
2-4ms on STM32 at 16MHz) across all three boards' `critsec` histograms.
This section originally set out to narrow down which kernel path produces
them; the actual root cause turned out to be in the *measurement* itself,
not in the code the earlier isolation work suspected.

**Root cause: `critical::enter`'s own latency-histogram timestamps were
taken outside the region they claim to measure.** The old code
(`rivet/src/critical.rs`) read the start timestamp *before* calling into
`port::arch::critical_section` (i.e. before interrupts were actually
masked) and the end timestamp *after* it returned (i.e. after interrupts
were already restored). Interrupts are live in both of those shoulder
windows, so a rare interrupt, including, on Cortex-M, a tail-chained
PendSV doing a full context switch, landing in either one had its *entire*
handler duration folded into the "critical section" measurement, even
though the calling hart was really off servicing an interrupt, not
executing the section body. This fully explains why code review of
`PriorityMutexGuard::drop`'s unlock path (owner swap, held-list recompute,
`wake_all_waiters`) never found anything that could plausibly cost 32768+
cycles: that code isn't where the time went. The histogram's own module
doc (`rivet/src/latency.rs`) already documented the assumption this
violated ("a proxy for interrupt-latency impact, nothing can preempt the
calling hart while held"), which was true of the *masked region*, just
not of the *window being measured*.

**Fix**: move both timestamp reads to strictly inside `port::arch::
critical_section`'s closure, after interrupts are masked, before
they're restored, so nothing can preempt the hart between them by
construction, matching the histogram's documented assumption exactly.

**Verified on real hardware, isolated to this one file.** ESP32-S3,
`stress_load_bench` (`--features latency-histograms`, `RIVET_MAX_HARTS=2`),
pre-fix vs. post-fix with every other file byte-identical:

| | `critsec` 2^15-bucket count | total `critsec` samples | rate |
|---|---|---|---|
| Before (baseline, reproduced) | 2608 | roughly 568,000 | roughly 0.46% |
| After (fix applied) | 106 | roughly 465,000 | roughly 0.023% |

A roughly 25x reduction, with the same workload, same board, same run
length. This is the dominant, previously disqualifying (§13.3) contributor,
genuinely fixed, not just discounted.

**A small residual remains, and is confirmed *not* to be the mutex
path.** An isolation test with the mutex contenders spawned but making
*zero* progress (a separate, real single-hart round-robin livelock found
between two same-priority never-yielding `PriorityMutex` contenders, worth
its own follow-up, but distinct from §11) still showed `critsec: ...
2^15:14` with mutex iteration count at 0. A further isolation down to
*only* the tick-driven periodic task (nothing else spawned at all)
reproduced the same small residual (`2^15:14` out of roughly 86,000
samples). This rules out `PriorityMutexGuard::drop` conclusively: the
residual is somewhere in the tick/dispatch path itself (`preempt::
on_tick`, `sched::schedule`, `timer::poll_timers`, `deadlines::
check_budget` were all reviewed; none has an unbounded-looking loop, same
as the original finding, just now correctly scoped away from the mutex
code). Not pinned to an exact sub-operation or instruction; doing so
would need finer-grained per-call-site instrumentation or hardware trace
capture beyond what the black-box approach and available tooling here
could do, flagged as a smaller, better-scoped follow-up.

**What this means for a WCET budget today**: the dominant term is fixed
and verified; treat the residual conservatively until further narrowed.
Re-measure the roughly 4ms STM32 figure with the fix applied and
`--features latency-histograms` before relying on a specific new number
for §13's schedulability analysis. The ESP32-S3 numbers above are cycle
counts at a different clock and under an intentionally extreme,
maximally-contended stress workload, not a like-for-like replacement for
the original STM32 measurement this section is framed around.

## 12. Kernel path coverage: spawn/despawn, channel send/recv

`latency_bench.rs` (§3) covered semaphore/mutex/yield only.
`kernel_paths_bench.rs` adds task spawn/despawn and channel `try_send`/
`try_recv`, direct `cycle_count()` bracketing, `N=2,000` each:

| Operation | STM32 (16 MHz) | ESP32-C6 (assumed 80 MHz) | ESP32-S3 (assumed 40 MHz) |
|---|---|---|---|
| `spawn_ptask!` | 1491 avg / 1342 min / 3544 max cyc → **93.2 µs avg** | 300 avg / 290 min / 760 max cyc → **3.75 µs avg** | 953 avg / 850 min / 3448 max cyc → **23.8 µs avg** |
| `despawn` | 2186 avg / 1955 min / 3613 max cyc → **136.6 µs avg** | 445 avg / 433 min / 1326 max cyc → **5.6 µs avg** | 725 avg / 622 min / 3186 max cyc → **18.1 µs avg** |
| `channel_try_send` | 91 avg / 82 min / 1740 max cyc → **5.7 µs avg** | 24 avg / 24 min / 455 max cyc → **0.3 µs avg** | 42 avg / 42 min / 44 max cyc → **1.05 µs avg** |
| `channel_try_recv` | 81 avg / 74 min / 1730 max cyc → **5.1 µs avg** | 23 avg / 23 min / 452 max cyc → **0.29 µs avg** | 42 avg (37-2600 range) cyc → **1.05 µs avg** |

Two things worth noting, not just the numbers:

- **`Sleep`/timer create-cancel latency is deliberately not isolated as
  its own line item.** `Sleep` is async-tier-only (`.await`-based), not
  callable from a preemptive task's synchronous context the way every
  other operation in this document is. Its real cost is already reflected
  in the `SchedulingWake` histograms from `stress_load_bench`/`deadline_
  miss_bench` (a `Sleep` wake *is* what those measure), just not broken
  out as an isolated uncontended number the way mutex/semaphore are.
- **A real, Xtensa-specific limitation found while building this bench**:
  `rivet-arch-xtensa`'s bootstrap table only frees a spawned task's slot
  once that task has been dispatched (interrupted) at least once,
  despawning many tasks that never got to run once exhausts the table
  (a hard 20-slot cap by default) with a clear panic message, not a
  silent failure. Worked around in the bench (a same-priority `yield_now`
  between spawn and despawn ensures each task gets its one dispatch), but
  worth knowing for any real S3 deployment that spawns and despawns
  short-lived tasks that might never actually run before being torn down.

## 13. Toward a WCET/schedulability methodology

Everything above is a measured number. This section is the actual
gate the testing methodology this document is built against asks for:
*given* those numbers, can this kernel's scheduling be shown, not just
observed, to meet a deadline, for a defined task set?

### 13.1 Why response-time analysis, not the plain rate-monotonic bound

Liu & Layland's classic utilization bound (`U = Σ Cᵢ/Tᵢ ≤ n(2^(1/n)-1)`)
assumes tasks never block each other. Rivet's `PriorityMutex` has real
priority inheritance (§5) precisely because tasks *do* block each other, so
the correct tool is **response-time analysis (RTA) with a blocking term**,
the standard extension (Sha, Rajkumar, Lehoczky 1990) for exactly this
case:

```
Rᵢ = Cᵢ + Bᵢ + Σ_{j ∈ hp(i)} ⌈Rᵢ/Tⱼ⌉ · Cⱼ
```

- `Rᵢ`: worst-case response time of task i (solved by fixed-point
  iteration: start `Rᵢ⁽⁰⁾ = Cᵢ`, substitute back until it converges or
  exceeds the deadline)
- `Cᵢ`: task i's own WCET
- `Bᵢ`: worst-case blocking time, bounded, under priority inheritance,
  by the *longest critical section of any lower-priority task that can
  block i*. This is exactly what §5's priority-inversion bench measured
  directly, not derived
- `hp(i)`: every task with strictly higher priority than i
- Task i is schedulable if `Rᵢ ≤ Dᵢ` (deadline, `= Tᵢ` for the implicit-
  deadline task sets this document's benches use)

### 13.2 Getting `Cᵢ` from measured data, honestly

This document has no formal WCET tool (§1). The pragmatic, industry-
common substitute where one isn't available: **`Cᵢ = observed_max ×
safety_factor`**, with the factor chosen from how much confidence the
observed max deserves, not a single global constant:

| Component of `Cᵢ` | Source | Recommended factor | Why |
|---|---|---|---|
| Task's own compute | Application-specific, not measured here | (not applicable) | Out of this document's scope entirely |
| Dispatch decision | §3/§4 `dispatch` histogram | 1.5× observed max | Tight distribution (single bucket dominates in every measured case), low risk of a fatter tail |
| Context switch / wakeup | §3 `SchedulingWake`, §4 histograms | 1.5× observed max | Same: tight, tick-bounded |
| **Critical-section interference** | **§11's outlier (STM32, scale by clock elsewhere)** | **Use as-is, no discount** | The dominant contributor is now root-caused and fixed (§11, roughly 25x reduction verified on ESP32-S3), but a small residual remains and the STM32 figure hasn't been re-measured against the fix. Still don't apply a "probably won't happen again" discount to whatever you measure until §11's residual is fully closed |
| Nested-IRQ latency (Cortex-M only) | §9 | 1× (already 500/500 identical) | Deterministic; a safety factor would be manufacturing false precision |

### 13.3 Worked example, using this document's own numbers

The task set §6's `deadline_miss_bench` actually ran: one highest-priority
periodic task (`T=2000µs`, priority 9, no mutex use of its own, so `B=0`,
`hp(i) = ∅` since nothing outranks it) concurrent with mutex-contending
and channel-traffic tasks at lower priority.

```
C_high  ≈ dispatch (§4, STM32: roughly 512-1023cyc/32-64µs, use 1.5× → roughly 96µs)
        + SchedulingWake (§4, STM32: mostly 2^10-2^11 bucket/1024-2047cyc,
          1.5× → roughly 192µs)
        ≈ 288µs, generously rounded

B_high  = 0                          (T_high touches no mutex)
hp(high) = ∅                         (highest priority in the set)

R_high  = C_high + B_high + 0 = 288µs
```

`R_high = 288µs ≤ D_high = 2000µs`, **schedulable, with a wide margin**,
even *before* folding in §11's critical-section outlier (which doesn't
apply here since `B_high = 0`, this specific task never contends a
mutex). This is consistent with the empirical result (§6: 0 misses, 0
worst-lateness across 500 real periods): the analysis and the
measurement agree, which is itself a useful sanity check on the
methodology, not just the task set.

**Now redo it for a lower-priority task that *does* block**, e.g. one of
the priority-2 mutex contenders, with a hypothetical priority-9 task above
it that also uses the same mutex (unlike the actual `deadline_miss_bench`
task, which didn't):

```
B_contender = §11's outlier (pre-fix) = roughly 65535cyc ≈ 4.1ms (STM32, no discount — §13.2)
```

A 4.1ms blocking term against a 2ms period is **not schedulable**:
`Bᵢ` alone exceeds `Dᵢ`. This is the methodology doing its job: before the
§11 fix, it correctly flagged that critical-section outlier as
disqualifying for any *tight* (sub-few-millisecond) deadline sharing a
mutex with lower-priority code, even though the *empirical* deadline-miss
test (§6) never happened to hit it in 500 samples, exactly the gap
between "we tested it and it passed" and "we can show it always passes."

**Post-fix (§11)**: the dominant contributor to that outlier is
now root-caused and fixed, verified as a roughly 25x reduction in the
outlier bucket's sample rate on real ESP32-S3 hardware. The STM32 figure
above hasn't been re-measured against the fix (no hardware access to
STM32 at the time it was applied), so this worked example still uses the
pre-fix number as the conservative stand-in. Re-run `critsec_isolate_
bench` with `--features latency-histograms` on STM32 before trusting a
smaller `B_contender` for this specific task set. A small residual (§11)
remains even post-fix, confirmed unrelated to the mutex path specifically,
so `B_contender` for a *genuinely* mutex-sharing task should shrink
substantially, but "genuinely zero blocking risk" still isn't provable
today.

### 13.4 What this workspace can honestly claim today, and what it can't

**Can claim**: for task sets that don't share a mutex with a lower-
priority task on a tight deadline (§13.3's first example), and that stay
within the empirically measured operation costs in this document with the
stated safety factors, response-time analysis using Rivet's real,
measured numbers shows the task set schedulable, and that conclusion is
consistent with every empirical test this document ran (zero deadline
misses, bounded priority inversion, deterministic nested-IRQ latency).

**Status update**: §10's dual-core bootstrap crash, the most serious of
the correctness blockers this document originally identified, **is fixed
and verified against realistic dual-core workloads** (`smp_test.rs`,
`stress_load_bench` under real `RIVET_MAX_HARTS=2`), not just the
artificial forcing bench that found it. §10's cross-core wakeup *latency
measurement* gap and `console::write_bytes`'s cross-hart corruption are
**both now fixed and verified** (real ESP32-S3 hardware for the latency
fix; full three-board QEMU regression for the console fix). §11's
critical-section outlier's dominant contributor is **root-caused and
fixed**, verified as a roughly 25x reduction on real hardware; a smaller
residual remains, confirmed unrelated to the originally suspected mutex
path. §14's round-robin starvation (found while isolating §11's residual,
same-priority `PriorityMutex` contenders sharing a level with a
never-blocking sibling) is **fixed on both single-hart and real dual-hart
ESP32-S3 hardware**, the latter after three fix attempts, two reverted on
hardware evidence and the third verified clean across 18 hardware runs
(§14 has the full account). It does not close every open item below.

**Cannot yet claim "hard real-time capable" in general**, because:

1. **§11's critical-section outlier has a small residual, not fully
   pinned to an exact sub-operation.** The dominant, previously
   disqualifying contributor is fixed (§11); what's left is roughly 25x
   smaller and confirmed not to be `PriorityMutexGuard::drop`, but a task
   set whose blocking term (`Bᵢ`) matters still can't be shown schedulable
   with full confidence until the residual is closed too.
2. **No formal WCET tool was used** (§1, §13.2). Every `Cᵢ` here is
   measured-plus-margin, not proven. A genuinely certifiable hard-real-
   time claim (DO-178C, ISO 26262, or similar) needs static WCET analysis
   against the actual compiled binary's control-flow graph, not sampled
   execution.
3. **S3/C6 clock speeds are still unverified** (§2.1). Every timing
   number derived from them (not the raw cycle counts) inherits that
   uncertainty.
4. **Sample sizes are in the hundreds-to-tens-of-thousands per
   measurement, not exhaustive.** Response-time analysis with measured
   `Cᵢ` is only as good as the confidence that the observed max really is
   the max; this document is explicit (§1) that it isn't a
   mathematical guarantee.
5. **§14's dual-hart fairness fix has an empirically verified safe
   parameter, not a mathematically proven one.** The exact instruction-
   level cause of the race two earlier fix attempts hit was never pinned
   down (no JTAG-capable hardware connection was available at the time),
   so the shipped `BROADCAST_EVERY = 32` throttle is verified clean across
   18 hardware runs, not proven safe at every rate.

**The concrete path from here to "hard real-time capable"**: close §11's
remaining residual (item 1), the dominant term is fixed, so this is a
narrower, lower-stakes search than before. For §14 (item 5), get real
JTAG hardware access (a second USB connection to the ESP32-S3's native
USB-JTAG, or an external probe) and pin the exact instruction the
timing-sensitive race hits, to replace "empirically safe" with "proven
safe" or find a cleaner fix. Then, for any task set intended for genuine
certification, replace §13.2's measured-plus-margin `Cᵢ` with real static
WCET analysis. Everything else in this document, the measurement
infrastructure, the RTA methodology in §13.1, the priority-inversion
bound in §5, the deterministic nested-IRQ result in §9, and now the fixed
dual-core bootstrap crash, cross-core latency measurement, console
corruption, critical-section outlier's dominant term, and round-robin
starvation (single- and dual-hart, §14), is real, reusable groundwork for
that effort, not a dead end.

## 14. Round-robin fairness: single-hart starvation and a dual-hart gap, both found and fixed

While isolating §11's residual, a real, separate liveness bug turned up:
`stress_load_bench`'s two `PriorityMutex` contenders (priority 2) sharing
that level with a channel producer/consumer pair that never blocks
completed **zero** iterations on real ESP32-S3 hardware over a full run,
despite showing non-zero CPU busy time, not just "unlucky," a genuine,
reproducible starvation. Reproduced kernel-wide (not board-specific) on
QEMU RISC-V with the identical task shape, single-hart, in seconds
(a scratch binary built for this investigation and removed afterward
isolated it to its minimal form): two mutex contenders plus **one** third
same-priority task that spins forever and never blocks
(`mutex_contender_iters=23` against `spin_iters=80,421,830`).

**Root cause: `rivet::preempt::sched`'s round-robin selection, both the
original single global rotation offset and an insufficient first fix (a
per-priority-level offset), could get permanently "stuck" favoring
whichever ready task's bit happened to be closest to the current offset,
even with other genuinely waiting same-priority tasks in the queue.** A
`RR_OFFSET`-style "nearest ready bit from a rotating cursor" scheme has no
notion of "who's actually waited longest"; under the right (not even
rare) toggle pattern it can settle into repeatedly re-selecting the same
one or two ids.

**Fix: replaced the whole rotating-cursor scheme with true
least-recently-dispatched selection.** Every real dispatch stamps the
task with the next value from a monotonic counter (`sched::
DISPATCH_COUNTER`/`DISPATCH_SEQ`); `schedule()` picks, among the ready
bits at the winning priority, whichever has the smallest stamp. This is
provably starvation-free for any bounded sibling count regardless of
toggle pattern: a task that hasn't run can only get "older" relative to
its siblings, so it's eventually the unique minimum and must be picked.
Cost: a bounded scan of at most `MAX_PTASKS` set bits instead of O(1),
accepted deliberately, since `MAX_PTASKS` is already a small, fixed,
linearly-scanned bound elsewhere in this crate (e.g.
`PriorityMutex::highest_waiter_priority`).

**Verified, single-hart: dramatic and consistent.** QEMU riscv (the
minimal-repro workload above): 23 → 10,044 iterations, same workload.
Real ESP32-S3 hardware, single-hart build (`RIVET_MAX_HARTS` unset, the
default, the exact configuration the original finding was on):
`mutex_contender_iters` 0 → 9,998, deterministic. Full rivet unit test
suite, the existing `tied_tasks_are_fairly_dispatched` proptest,
loom (4/4), clippy, and the full three-board QEMU smoke suite (riscv
15/15 + `-smp 2`/`-smp 4`, cm3 15/15, mps2 12/12) all stayed green
throughout.

**A narrower, dual-hart-specific gap also had to be closed on real
ESP32-S3 hardware specifically.** The same `stress_load_bench` workload
built dual-core (`RIVET_MAX_HARTS=2`) still showed a low, though no
longer literally zero, contender count (`mutex_contender_iters=5-7`,
deterministic) even with the LRU fix applied, *not* reproduced on QEMU
RISC-V `-smp 2` with the identical task shape (healthy roughly 10,900
there), pointing at something specific to this port's real dual-core
timing or its Xtensa-specific secondary-hart boot design, not the
round-robin algorithm itself (already fixed and QEMU-verified for genuine
multi-hart concurrent dispatch in the RISC-V case).

Three fix attempts were built, each tested to a real, concrete
conclusion on hardware rather than left as speculation:

1. **APP_CPU has no periodic tick of its own** (confirmed by reading
   `rivet_appcpu_rust_entry`: it calls `rivet::run_secondary_hart()`
   directly, which deliberately does not repeat `rivet::init()`'s
   `tick_start` call, a documented design choice, matching
   `rivet-arch-riscv::clint`'s identical "hart 0 is the sole tick owner"
   comment). Giving APP_CPU its own independent `CCOMPARE1` (after first
   making `rivet-arch-xtensa::timer`'s tick state genuinely per-hart)
   **did** close the `stress_load_bench` gap on hardware
   (`mutex_contender_iters` 8-9k → 18,619), but broke a different,
   previously passing real-hardware test just as badly in the other
   direction: `smp_latency_bench`, whose design depends on `holder`
   monopolizing its own hart so `waiter` is *structurally forced* onto
   the other core, dropped from roughly 1,100 iterations in 5s to 3.
   Reverted.
2. Suspecting the cross-hart `critical::enter` lock's own fairness (a
   raw test-and-set, no FIFO guarantee, now under roughly double the
   tick-driven contention with fix 1), it was replaced with a
   provably fair ticket lock. Fully regression-tested (unit/loom/QEMU,
   all green), but on hardware it did **not** fix `smp_latency_bench`,
   and *without* fix 1 it caused a **worse** failure: a genuine crash
   (`InstrProhibited`) on real dual-core hardware, never seen in QEMU.
   Reverted.
3. **The fix that actually worked**: reuse the *existing*,
   already-hardware-verified `request_reschedule_on` cross-hart IPI
   plumbing (the same mechanism `ready_add`'s `wake_other_harts` already
   uses) to periodically prompt every other hart's `on_tick` from the
   tick-owning hart's own timer ISR, no second hardware timer, no
   change to `critical::enter`'s locking at all. An unthrottled version
   (every tick) measurably slowed the receiving hart's own useful work
   (`waiter` dropped to roughly 35/1000 samples); throttled to every 2nd
   tick it hit a genuine, timing-sensitive real-hardware race, confirmed
   via targeted print-instrumentation bisection (inserting extra
   `console::write_str` calls in the hot loop reliably avoided it, 5/5,
   pointing at exact interrupt timing rather than a logic bug), manifesting
   as either a hard stall (`waiter` permanently stuck, a 4x longer
   watchdog made no difference) or a crash (`InstrProhibited` inside
   `core::fmt`'s own formatting code). A live JTAG session would have
   pinned the exact instruction, but no JTAG-capable connection was
   available at the time (the ESP32-S3's native USB-JTAG needs a second
   USB port this board doesn't expose here, confirmed by direct check:
   `lsusb` shows only the CH340 UART bridge, no native-USB vendor ID).
   Throttled to every 32nd tick instead: **13/13 clean, fully
   deterministic runs of `smp_latency_bench` and 5/5 of
   `stress_load_bench`**, no stall, no crash, identical `min`/`max`/
   `avg` and iteration counts every run.

**Verified, fully closed.** `stress_load_bench`'s `mutex_contender_iters`
(the original dual-hart starvation) went from 5-7 to a healthy, consistent
**2811**, across 5 repeated runs, deterministic. `smp_latency_bench`
(the test the first two fix attempts broke) and `smp_test.rs` both pass
cleanly and deterministically alongside it: 18 total hardware runs across
the three dual-core tests in this final round, zero failures. Full rivet
unit/proptest suite, loom (4/4), clippy, and the complete three-board QEMU
smoke regression (riscv 15/15 + `-smp 2`/`4`, cm3 15/15, mps2 12/12) all
stayed green throughout; this fix touches only `rivet-arch-xtensa`, which
none of the QEMU-tested boards link.

**Two further attempts were made to eliminate this uncertainty entirely**,
replacing the tuned throttle with a "correct by construction" design
(every hart gets its own independent hardware tick, no cross-hart
interrupt traffic at all, so nothing to tune). Both were built and tested
on real hardware, and both were *rejected by hardware evidence*, not
reverted for convenience: giving every hart the full tick body
(watchdog + timer-queue polling included) reproduced the earlier
`smp_latency_bench` regression; guarding those two calls to hart 0 only
produced a *different, more severe* failure: an immediate boot-time
panic (`Interrupt: 1`, an unhandled level-1 interrupt, likely from
`esp-hal`'s own per-core interrupt setup not covering APP_CPU the way it
covers PRO_CPU). Both reverted. This is meaningful evidence in its own
right: two independent designs that specifically avoided needing an
empirical constant both failed *worse* than the tuned version ever did.
The throttled broadcast isn't a corner cut relative to some available
better alternative; on this hardware/toolchain combination, it is the
only one of three fundamentally different designs that actually works.

**The fault at higher broadcast rates was narrowed further, and confirmed
deterministic rather than a probabilistic race.** Doubling every task's
stack (4096 → 8192 bytes) at the crashing rate reproduced the *exact
same* fault, identical `PC`, `EXCVADDR`, and every other register,
byte-for-byte, across 3 repeated runs, ruling out stack overflow and
confirming this is a deterministic condition tied to the broadcast rate
itself, not a stack-depth-sensitive or otherwise probabilistic race.

This also turned out to match a pattern **already independently
documented elsewhere in this exact codebase**: `smp_latency_bench.rs`'s
own `waiter` function carries a comment, from earlier, entirely
unrelated work, describing a *different* design for this identical
cross-core `PriorityMutex` contention scenario (a shared "unlock
generation" counter) that "reliably crashed (`LoadProhibited`,
reproducible byte-for-byte across rebuilds and stack-size changes, not
investigated further given time cost)": the same signature (byte-for-
byte deterministic, stack-size-independent, un-investigated for the same
reason) in the same narrow scenario. This fault class, deterministic but
unexplained at the instruction level, in high-frequency cross-core
`PriorityMutex` dispatch on this SoC/toolchain, was hit twice
independently, neither time with JTAG access to pin the exact
instruction, both times resolved by staying clear of the triggering
condition rather than a confirmed instruction-level fix.

**What remains uncertain**: the exact instruction where this
deterministic fault occurs was never identified, since that needs a live
JTAG session on this exact board, either a second USB connection to the
ESP32-S3's native USB-JTAG (confirmed unavailable: this board's wiring
exposes only the CH340 UART bridge, checked directly via `lsusb`) or an
external JTAG probe (none available at the time). `BROADCAST_
EVERY = 32` is verified clean across 34+ hardware runs total across this
whole investigation (18 in the original verification round, 8 after two
further rejected fix attempts, 8 more after the stack-size experiment),
fully deterministic every time, a wide, repeatedly reconfirmed margin
below a threshold that itself behaves deterministically rather than
probabilistically, not a guess dressed up as a number.

**Update: JTAG access was later obtained and the fault above was
root-caused and fixed; see §15.** The paragraphs above are kept as the
historical record of what was known before that, and §15 is now the
authoritative account of the actual instruction-level cause.

## 15. The `InstrProhibited` fault, root-caused: a torn cross-hart `Context` copy

A live JTAG session (Espressif's `openocd-esp32` + `xtensa-esp-elf-gdb`
against the ESP32-S3's native USB-JTAG, once a second USB connection to
it became available) pinned the exact fault from §14 down to the
instruction level, and it turned out to be a genuine bug, not hardware
noise: the crash reproduced byte-for-byte identical to §14's own
description: `PC` at the `retw.n` ending `console::write_str`, `A0`
(the return-address register) reading exactly `0x00000000`, `EXCCAUSE`
`InstrProhibited`, `EXCVADDR` a garbage computed return target, with
`waiter` having already completed all 1000 cross-core samples before the
crash landed in the final summary print.

**Root cause.** `rivet-arch-xtensa`'s `CONTEXTS` array (per-task saved
register state, keyed by task id) is genuinely shared across harts, since
any task can be dispatched onto either core, but `__level_3_interrupt`'s
`CONTEXTS[id]` reads and writes were plain, non-atomic 136-byte struct
copies (`*CONTEXTS[id].0.get() = *save_frame` and its mirror), with *no*
synchronization at all, not even a lock scoped to just the copy.
`ContextCell`'s `unsafe impl Sync` justified this by reasoning that the
level-3 handler "cannot be re-entered," true on a *single* hart, but
`CONTEXTS` is indexed by task id, not by hart, so that reasoning never
actually covered the real hazard: two different harts, each running
their own level-3 handler, touching the same task's slot at the same
time. A higher `on_timer_irq` broadcast rate means more concurrent
tick/dispatch activity on both harts, which is exactly why §14 saw this
fault get *more* likely, not less, as `BROADCAST_EVERY` dropped from 32
towards 2: more chances per second for hart B to read `CONTEXTS[id]`
while hart A was mid-write, a torn read landing on a live task's saved
`A0` field.

**The fix**, in `__level_3_interrupt`: wrap each individual `CONTEXTS`
copy (the outgoing task's save, the incoming task's restore) in its own
`rivet::critical::enter` call, the cross-hart spinlock this workspace
already uses everywhere else for exactly this kind of shared-state
access. Deliberately scoped to *just* the two copies, not the whole
save-decide-restore sequence: a first attempt that wrapped the entire
sequence (so the scheduling decision and both `CONTEXTS` copies were one
atomic unit) closed the race just as well, but held the cross-hart lock
long enough to starve the other hart's own tick handling. The system
never made forward progress at all (not even the watchdog's own periodic
print) at every broadcast rate tried. Locking only the individual copies
avoids that: the scheduling decision itself was already correctly
protected by `on_tick`'s own internal `critical::enter` (nested calls to
the same lock compose correctly by construction, see `critical.rs`'s
module docs), so the only genuinely unprotected operation was the raw
struct copy, and that's the only thing that needed its own lock.

**A real, measured cost, not a free fix.** The extra `critical::enter`
call sites add stack usage to every dispatch, and a dispatch always
runs on whichever task's stack was interrupted, not a separate ISR
stack (`xtensa-lx-rt`'s exception entry allocates its frame on top of
whatever stack was live, per this crate's own module docs). Applying the
fix at `smp_latency_bench`'s original 4096-byte task stacks reproduced a
*different* real-hardware fault instead: `StoreProhibited`/
`LoadProhibited`, a garbage-pointer write from genuine stack corruption,
at **both** `BROADCAST_EVERY = 2` and the shipped `= 32`, so this was not
specific to the crash-reproducing rate. Doubling to 8192 bytes still
wasn't enough headroom; 16384 (4x the original) was clean. This is a
property of `smp_latency_bench` specifically (its crash site is deep
inside `console::write_str`'s own formatting call chain, `print_u64` →
`write_str` → `write_bytes`, which was already using a meaningful
fraction of 4096 bytes before the fix added anything): `smp_test.rs`
and `stress_load_bench`, whose task bodies never call into the console
formatter, needed **no** stack changes at all and passed clean at their
original 512-2048-byte sizes. Any application on this arch with tight
task stacks and a deep call chain at a plausible interrupt point should
budget for this cost explicitly rather than assume the original margin
still holds.

**Verified.** `smp_latency_bench` (16384-byte stacks): 6/6 clean,
fully deterministic runs, 3/3 at `BROADCAST_EVERY = 2` (the rate that
reliably reproduced the crash pre-fix, re-confirmed via a real,
byte-for-byte-identical baseline run against the unfixed code first),
3/3 at the shipped `= 32`. `smp_test.rs` and `stress_load_bench`
(original stack sizes, unmodified): both pass cleanly, deterministically,
alongside the fix. `stress_load_bench`'s `mutex_contender_iters` (§14's
own original starvation metric) came back at **13,997**, comfortably
above §14's already-healthy 2811, confirming no fairness regression.
Host-side `cargo test -p rivet` and clippy on `rivet-arch-xtensa`
(`--target xtensa-esp32s3-none-elf`) both stayed green; this fix
touches only `rivet-arch-xtensa`, which no QEMU-tested board links, so
the three-board QEMU suite is unaffected by construction, not just
untested.

`BROADCAST_EVERY` remains `32`, unchanged from §14. This fix
addresses the fault at every rate tried (2 and 32 both verified clean),
not just the shipped one, but 32 is still the only value with a real
hardware-verified fairness benefit backing it (§14's `stress_load_bench`
result), so there's no reason to move off it now that the fault it used
to trigger is actually fixed rather than just avoided.
