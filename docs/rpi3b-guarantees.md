# Rivet on a Raspberry Pi 3B: what is guaranteed, and what is only measured

A companion to [rpi3b-benchmarks.md](rpi3b-benchmarks.md), which reports
what this port does. This one says which of those figures you may design
against, on what conditions, and which you may not.

The distinction matters more here than on any other board in this
workspace. Every other target owns its silicon. This one shares a die with
Linux, and a number measured while sharing is a statement about the run it
came from unless something structural makes it hold in general.

Claims are labelled the same way as [wcet.md](wcet.md) and
[wcet-stm32f401re.md](wcet-stm32f401re.md):

- **ARCHITECTURAL**: follows from the hardware or the code's structure. No
  measurement could refute it; a measurement disagreeing would mean a bug.
- **DERIVED**: arithmetic from an exactly bounded quantity.
- **MEASURED**: the worst observed over a stated sample count under a
  stated load. A longer run may observe worse.
- **ASSUMED**: taken on someone else's authority, not re-verified here.

## 1. Configuration this document is about

Everything below is scoped to exactly this setup. Change any line and the
declaration no longer applies.

| | |
|---|---|
| Board | Raspberry Pi 3 Model B v1.2, BCM2837 |
| rivet | core 3, EL1, MMU on, `RIVET_MAX_HARTS=1` |
| Linux | Raspberry Pi OS Lite 64-bit, cores 0 to 2 |
| Core reservation | `/cpus/cpu@3` deleted from the device tree |
| Memory | `/reserved-memory` `no-map` at 0x30000000, 18 MiB |
| Clock | `force_turbo=1`, 1.2 GHz pinned |
| Tick | 10 kHz (`RIVET_TICK_HZ=10000`) |
| Timebase | `CNTPCT_EL0`, 19.2 MHz |

## 2. Architectural guarantees

These hold by construction. They are the part of this port you can actually
design against, and they are the reason the measured figures are as stable
as they are.

| Guarantee | Basis | Status |
|---|---|---|
| No Linux thread ever runs on core 3 | The device tree node is deleted, so Linux never enumerates the CPU and never writes its spin-table slot | ARCHITECTURAL |
| rivet cannot read or write Linux's memory | Its page tables map only the owned window, the shared window, the low 2 MiB, and the peripheral and ARM-local blocks. All other addresses are unmapped and fault | ARCHITECTURAL |
| Linux's allocator never hands out rivet's memory | `/reserved-memory` with `no-map` keeps the region out of the kernel's linear map entirely | ARCHITECTURAL |
| The timebase does not move with CPU frequency | `CNTPCT_EL0` is driven by the 19.2 MHz crystal, not the core PLL. Linux owns cpufreq and cannot affect it | ARCHITECTURAL |
| The tick grid cannot drift | `CNTP_CVAL_EL0` is advanced by a fixed interval from the previous comparator value, never reloaded relative to now. A late tick yields one late tick | ARCHITECTURAL |
| Deadline lateness is at most one tick period plus the wake cost | A tick-driven timer can only wake on the tick grid | DERIVED |
| No timer wake can be skipped by the early-out | The cached earliest deadline is never greater than the true earliest: arming lowers it, cancelling leaves it, and only the sweep that recomputed it from the arrays may raise it, inside that same critical section | ARCHITECTURAL |
| Priority inversion is bounded by the holder's critical section | `PriorityMutex` boosts the holder to the waiter's priority for the duration of the hold | ARCHITECTURAL |
| Cross-tier GPIO writes cannot race | `GPSET`/`GPCLR` are write-to-set and write-to-clear, not read-modify-write, so the two sides can drive different pins in one bank with no lock | ARCHITECTURAL |

Two of these were verified against the hardware even though they do not
depend on measurement, because a structural claim that is merely intended
is worth nothing. Linux reports three CPUs online, and rivet's level-two
page table, read back from Linux, maps zero blocks of Linux's RAM.

The memory isolation guarantee has one deliberate hole: root on Linux can
reach rivet's region through `/dev/mem`. That is not a leak, it is the
loader. It does mean the isolation is against accident and against
unprivileged code, not against a hostile root.

## 3. Measured envelopes

Worst observed, with sample counts, from
[rpi3b-benchmarks.md](rpi3b-benchmarks.md). "Loaded" means three shell CPU
hogs plus memory and disk pressure on Linux's three cores.

| Quantity | Idle worst | Loaded worst | n | Status |
|---|---|---|---|---|
| Interrupt latency, hardware to handler | 1041 ns | 1250 ns | 30000 | MEASURED |
| Tick handler cost | 1614 ns | 2291 ns | 30000 | MEASURED |
| Scheduler decision, no task change | 3645 ns | 5156 ns | 20000 | MEASURED |
| Task-to-task switch | 364 ns mean | 364 ns mean | 4000 | MEASURED |
| Mutex handoff, contended | 1093 ns | 1250 ns | 100 | MEASURED |
| Semaphore try/release | 2656 ns | 4479 ns | 20000 | MEASURED |
| Deadline lateness, 10 kHz | 94 us | 95 us | 500 | MEASURED |

The `>1us` columns in the benchmark document matter as much as these
maxima. A worst case that happened once in thirty thousand and one that
happens a third of the time are different systems, and a maximum reports
them identically.

## 4. Declaration

**Rivet on core 3 of a BCM2837, configured as in section 1, is declared
suitable for hard real-time task sets whose schedulability analysis uses
the following bounds.**

Periodic kernel overhead, the term every task in the set pays:

| Term | Measured worst, loaded | Declared bound |
|---|---|---|
| Interrupt latency | 1250 ns | 2500 ns |
| Tick handler cost | 2291 ns | 4582 ns |
| Per tick, total | 3541 ns | **7082 ns** |
| Tick utilisation at 10 kHz | 3.54 % | **7.08 % of core 3** |

The declared column is the measured worst doubled. That factor is an
engineering margin, chosen and stated rather than derived, and it exists
because of section 5: the dominant variable is Linux's effect on a shared
cache, and no measurement bounds an adversary. Using the measured figure
directly would be treating an observation as a proof.

Release granularity is one tick, 100 us, ARCHITECTURAL.

### Worked example

A 1 kHz control task with a 50 us execution time, alone on the core:

```
  ticks per period          10
  tick interference         10 x 7082 ns  = 70.8 us
  release quantisation                      100.0 us
  execution                                  50.0 us
  response time R                           220.8 us
  deadline                                 1000.0 us
  margin                                    779.2 us  (78 %)
```

Feasible with a wide margin. The same arithmetic at shorter periods:

| Period | R | |
|---|---|---|
| 1000 us | 221 us | feasible |
| 500 us | 185 us | feasible |
| 200 us | 164 us | feasible |
| 100 us | 157 us | **infeasible** |

The floor is release quantisation, not throughput. At a 100 us period the
task's release jitter is its entire period. A task set needing periods
near or below the tick has to raise the tick rate, which raises the
utilisation term proportionally: 70.8 % of the core at 100 kHz, against
7.08 % at 10 kHz.

### Scope

**Covers** normal operation: every tick, every dispatch, every mutex
operation, for as long as the system has not faulted, has not tripped a
watchdog and has not called `rivet::exit`.

**Does not cover** the interval from a fault to halt, on the same reasoning
as [wcet-stm32f401re.md](wcet-stm32f401re.md) section 7: a task set must
treat "faulted" as "no further deadlines are being met."

**Does not cover** application code between `lock()` and `drop()`. The
kernel bounds its own mutex mechanism; it cannot bound what you do while
holding one.

**Does not cover** anything reached through the shared window. Those paths
are measured in the benchmark document and explicitly excluded here, for
the reasons in section 5.

## 5. What is not guaranteed, and why

This is the substantive half of the document. Each item is a real limit of
this arrangement, not a caveat added for form.

**The L2 cache is shared and cannot be partitioned.** Four Cortex-A53
cores share one 512 KiB L2 on this part, and it offers no way to
partition, lock or colour it. Linux's working set evicts rivet's lines at
will. This is not a small effect and it is not hypothetical: it is the
entire reason the tick handler cost moves from 312 ns to 677 ns, and
chasing it down is written up in the benchmark document. Every measured
figure here is bounded empirically against one load shape. A different
Linux workload can do worse, and nothing in the hardware stops it.

**The memory controller is shared with no quality-of-service.** There is
no bandwidth reservation for core 3.

**DMA is unbounded and invisible.** Linux's SD card, USB (which is also
where the Ethernet controller lives) and the VideoCore all move data across
the same interconnect. rivet can neither bound this traffic nor observe it.

**The VideoCore runs firmware this port has no visibility into.** It
initialises the SoC, owns several clocks, and continues running.

**Thermal throttling is still possible.** `force_turbo=1` pins the
frequency against DVFS, which removed the dominant source of interrupt
latency outliers, but the firmware still reduces clocks above 85 °C. The
architected counter is unaffected, so timing measurements stay valid while
instruction throughput drops, which is the worst combination: the symptom
is missed deadlines with no change in the clock the system reads.

**No static WCET analysis has been performed.** No aiT, no OTAWA, no
control-flow analysis of the compiled binary. Every "worst" here is a
worst observed.

**Peripheral interrupts cannot be routed to core 3 alone.**
`GPU_INTERRUPTS_ROUTING` is one system-wide choice, so in practice rivet
gets the per-core timer and the per-core mailbox, and peripherals are
polled or left to Linux.

**Only the 10 kHz tick is characterised.** The quantisation relationship is
arithmetic and holds at any rate, but the utilisation figures were measured
at one.

**Linux is not modelled as an adversary.** The load used is three CPU hogs
plus memory and disk pressure. It is not a calibrated profile, and it is
certainly not a worst case. A workload built specifically to thrash the
shared L2 would be worse, and this document cannot tell you by how much.

## 6. What would raise the confidence level

In rough order of how much each would buy:

1. A cache-thrashing adversarial load, sized to the L2, to find the real
   shape of the tail rather than the one a generic load produces.
2. Long-duration runs. Thirty thousand ticks is three seconds. A rare event
   at one in ten million would not have appeared once here.
3. Static WCET analysis of the tick handler and the scheduler, which are
   both small and loop-bounded, to replace the measured terms with derived
   ones.
4. External validation of the whole chain, extending `scope_demo`'s
   approach past the doorbell.
5. Thermal characterisation under sustained load, to find whether
   throttling is reachable in a realistic enclosure.

Until at least the first three, the honest summary is: the architectural
guarantees in section 2 are solid and are what make this arrangement worth
using, and the numbers in section 4 are well-evidenced engineering figures
with a stated margin, not proofs.
