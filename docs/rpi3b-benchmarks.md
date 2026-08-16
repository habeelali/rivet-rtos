# Rivet on a Raspberry Pi 3B: measured real-time characterisation

Numbers for the AArch64 port, taken on a Raspberry Pi 3 Model B v1.2 with
rivet owning core 3 and Linux running on cores 0 to 2. Both result tables
below came off that board; nothing here is estimated, extrapolated, or
carried over from another port.

The tool that produces them is `examples/rpi3b-kernel/src/bin/rt_bench.rs`,
with the two Linux-side figures produced by `rivet-amp bench`.

Which of these you may design against, and on what conditions, is
[rpi3b-guarantees.md](rpi3b-guarantees.md). Read both alongside
[realtime.md](realtime.md), which sets out what a measured number does and
does not entitle you to claim. The short version
applies here too: this is empirical black-box measurement under a described
load, not a formally proven WCET bound. A longer run or a load shape not
exercised here could observe something worse.

## The timebase, and the floor it imposes

Every figure comes from `CNTPCT_EL0`, the architected counter, which runs
at a fixed 19.2 MHz on the BCM2837. One tick is 52.083 ns, and conversion
to nanoseconds is exact rather than approximate: `ns = ticks * 625 / 12`.

Two properties make it the right clock here. It does not move with CPU
frequency, which matters because Linux owns the cpufreq driver for a
cluster rivet shares. And Linux userspace can read the same counter
through `CNTVCT_EL0`, with `CNTVOFF_EL2` zero on this board, so the two
sides can timestamp against one clock. Without that, the only honest
cross-tier figure would be a round trip; with it, one-way latency is
measurable.

The counter's resolution is also the measurement floor. A row reporting
52 ns means "at or below one tick", not "exactly 52 ns". The suite times a
bare counter read first so that floor appears in the output rather than
being something you have to know.

`CNTPCT_EL0` traps to EL1 from userspace and dies with `SIGILL`, which is
why the Linux side reads the virtual counter and not the same register.

## Running it

Three phases, because `rivet-amp bench` and `rivet-amp console` are both
readers of the console ring and would otherwise eat each other's bytes.

```sh
# On the build machine
cd examples/rpi3b-amp
cargo build --release
rust-objcopy -O binary \
    ../../target/aarch64-unknown-none/release/rt_bench_amp rt_bench.img
scp rt_bench.img pi:/tmp/

# On the Pi
sudo rivet-amp load /tmp/rt_bench.img
sudo rivet-amp console      # self-contained results, then it waits
                            # Ctrl-C once it says it is waiting
sudo rivet-amp bench 400    # round trip and one-way throughput
sudo rivet-amp console      # the doorbell row, then RT_BENCH_OK
```

A spin-table release latches, so a core already running an image cannot be
restarted in place. **Reboot between runs.** Loading a second image onto a
core still executing the first one silently does nothing, which looks
exactly like a hang.

For the loaded numbers, start the load before `rivet-amp load`:

```sh
for i in 1 2 3; do (while :; do :; done) & done
(while :; do cat /dev/urandom | gzip -1 > /dev/null; done) &
(while :; do dd if=/dev/zero of=/tmp/lf bs=1M count=64; sync; rm -f /tmp/lf; done) &
```

Three CPU hogs for three cores, plus memory and I/O pressure. The intent is
to oversubscribe every resource rivet shares with Linux: the cluster clock,
the L2, and the memory controller.

## Results

Both columns are from the same binary and the same board, differing only in
whether the load above was running. Sample counts are per row in the tool's
output; the large loops are 20000 samples, the interrupt rows about 30000
over a three-second window.

Nanoseconds, except where noted. `>1us` counts samples over one
microsecond, which is the column that makes the others readable: on a tight
distribution the integer mean lands on the minimum and stops carrying
information, and a maximum alone says nothing about how often the worst
case happens.

| | idle min / mean / max | idle >1us | loaded min / mean / max | loaded >1us |
|---|---|---|---|---|
| counter read (floor) | 0 / 0 / 1145 | 1 / 20000 | 0 / 0 / 1145 | 1 / 20000 |
| reschedule, no task change | 729 / 729 / 3645 | 154 / 20000 | 729 / 729 / 5156 | 153 / 20000 |
| task-to-task switch | 364 mean | | 364 mean | |
| semaphore try/release | 156 / 156 / 2656 | 3 / 20000 | 156 / 156 / 4479 | 22 / 20000 |
| mutex try_lock/unlock | 677 / 677 / 3333 | 144 / 20000 | 677 / 677 / 5000 | 138 / 20000 |
| mutex handoff, contended | 937 / 937 / 1093 | 1 / 100 | 937 / 937 / 1250 | 1 / 100 |
| Signal to async task | 781 / 885 / 5312 | 20 / 200 | 781 / 1145 / 7708 | 77 / 200 |
| 64 writes, Normal cached | 156 / 156 / 2812 | 1 / 2000 | 156 / 156 / 989 | 0 / 2000 |
| 64 writes, Device shared | 468 / 468 / 1145 | 11 / 2000 | 468 / 572 / 11822 | 98 / 2000 |
| dc cvac x8 + dsb ish | 52 / 52 / 2447 | 1 / 2000 | 52 / 52 / 729 | 0 / 2000 |
| **interrupt, hardware to handler** | **52 / 52 / 1041** | **0 / 30000** | **52 / 156 / 1250** | **3 / 30000** |
| tick handler cost | 260 / 312 / 1614 | 6 / 30000 | 260 / 677 / 2291 | 5408 / 30000 |
| Linux doorbell to task | 2187 / 2500 / 51979 | 400 / 400 | 2343 / 7500 / 49895 | 400 / 400 |
| Linux round trip | 5156 / 5781 / 56666 | | 5677 / 11458 / 55364 | |

Coarser figures, in their own units:

| | idle | loaded |
|---|---|---|
| tick-to-tick interval, 10 kHz | 99010 / 100000 / 100885 ns | 98854 / 99947 / 101145 ns |
| deadline lateness, 10 kHz tick | 80 / 86 / 94 us | 81 / 87 / 95 us |
| ring write, rivet side | 107 MiB/s | 84 MiB/s |
| ring one-way, rivet to Linux | 63 MiB/s | 20 MiB/s |

## Reading the numbers

**The isolation holds, and this is the row that says so.** Interrupt
latency is what core isolation exists to protect. Saturating the three
Linux cores moves its worst case from 1041 ns to 1250 ns, and three
samples in 30000 crossed a microsecond. The mean goes from 52 ns to
156 ns, which in counter ticks is one to three: the quantity is at the
instrument's floor in both conditions. Deadline lateness barely moves at
all, 94 us to 95 us.

**The tick handler is still where the load lands**, and it is the one row
worth watching: 6 long samples out of 30000 idle against 5408 loaded. An
earlier version of this port was much worse, 22 against 9168, and running
that down is written up under [the tick handler
investigation](#the-tick-handler-investigation) below. What remains is
real work: at a 10 kHz tick, the sweeps that survive are the ones that
actually have a timer to fire. It does not reach the deadline figures
because it stays far below the 100 us tick period, but it is the term
that would grow first at a higher tick rate.

**`min` equal to `mean` is not a bug.** Several rows report the same value
for both. The distribution is tight enough that the integer mean lands on
the minimum, which means the mean has stopped carrying information and the
maximum and the `>1us` count are the columns worth reading.

**Tick jitter is small and does not accumulate.** The tick-to-tick interval
at 10 kHz stays within about 400 ns idle and 1 us loaded, and its mean is
exactly the nominal 100000 ns. That last part is the important one. The
handler advances `CNTP_CVAL_EL0` by a fixed step rather than reloading
`CNTP_TVAL_EL0`, so a late tick produces one late tick instead of
permanently skewing the grid. Writing `TVAL` sets the deadline relative to
whenever the handler happens to run, which quietly folds exception entry
and register save into every period.

**Deadline lateness is tick quantisation, not a defect.** At a 10 kHz tick
a sleeper can only be woken on a 100 us grid, so lateness is bounded by one
tick period by construction, and 82 to 97 us is where a uniformly
distributed deadline lands inside that grid. Raise the tick rate and the
number falls proportionally: at 1 kHz the bound is 1001 us, at 100 kHz it
is 10 us. Note that this row does not carry a `>1us` count, because at a
100 us period every sample would clear that bar and the column would just
restate the sample count. This was chased down as a suspected clock-drift bug before the
arithmetic settled it; see the AMP README for that.

**Device memory costs about 3x Normal memory, and that is the cheaper
option.** The shared window is mapped Device-nGnRnE so both tiers agree on
visibility with no cache maintenance, and 64 writes take 468 ns against
156 ns cached. The alternative is keeping it Normal cacheable and cleaning
lines by hand, and `dc cvac` on eight lines plus a `dsb ish` is at or below
one counter tick when the lines are already in cache. That looks like the
better trade until you count what a real implementation needs: maintenance
on both sides of every transfer, in both directions, with the barriers to
order them. The Device mapping pays a fixed, predictable, and
easy-to-reason-about price instead, which on a real-time core is worth more
than the throughput.

**The Linux-side figures are dominated by Linux.** The doorbell one-way
minimum is about 2.1 us and its maximum near 49 us even with nothing else
running. rivet's side of that path is the interrupt entry already measured
at 52 ns; essentially all of the rest is the sending process being
scheduled, faulting in its `/dev/mem` mapping, and the write reaching the
interconnect. Quote these as properties of the channel, not of the RTOS.

**One-way ring throughput measures the reader.** rivet writes into the ring
at about 105 MiB/s and Linux drains it at 63 idle, 30 under load. The
producer overwrites rather than blocking, so any shortfall is genuinely
bytes the reader could not keep up with, and the tool reports how many. The
reader is the constraint here because the shared window is Device memory,
where every access makes its own trip to the interconnect. Reading 64 bits
at a time instead of one byte took this from 7 MiB/s to 63 and took
overwritten bytes from 119 KiB to none. Unaligned and
vector accesses fault on Device memory, so the wide path is used only where
the offset is aligned and the run does not wrap.

## What each row actually measures

- **counter read (floor)** — two back-to-back `CNTPCT_EL0` reads. The
  measurement floor, and its maximum is a preemption landing inside the
  instrument.
- **reschedule, no task change** — `request_reschedule()` where the caller
  is still the highest-priority runnable task. The scheduler's decision
  cost with the context switch excluded.
- **task-to-task switch** — two tasks at equal priority handing off 2000
  times, divided by the 4000 switches. Mean only: the exchange is timed in
  bulk, so there are no per-switch extremes to report.
- **semaphore, mutex try_lock** — the uncontended fast paths.
- **mutex handoff, contended** — a priority-4 task blocked on a
  `PriorityMutex` held by a priority-1 task. The holder timestamps
  immediately before releasing and the waiter immediately after acquiring,
  so the figure is the wake and switch alone, with the hold duration
  excluded by construction. This is the path priority inheritance exists to
  bound, and it is the tightest row in the table: 937 ns minimum and mean
  in both conditions, with a single sample past a microsecond out of a
  hundred either way. Linux load does not touch it.
- **Signal to async task** — `Signal::signal()` to the awaiting task
  running. Crosses from the preemptive tier into the async one.
- **64 writes cached / Device / dc cvac** — the coherency cost discussed
  above.
- **interrupt, hardware to handler** — `CNTP_CVAL_EL0`, the instant the
  comparator matched, to the first instruction of the handler. Covers
  exception entry, the full register save and board dispatch, with nothing
  estimated, because the comparator value is the real hardware event time.
- **tick handler cost** — time inside the handler.
- **tick-to-tick interval** — consecutive handler entries.
- **deadline lateness** — `sleep_until` against an absolute deadline, not
  the gap between wakeups. Gap is the difference of two quantisation errors
  and swings a full tick while nothing is actually late.
- **Linux doorbell to task** — Linux stamps `CNTVCT_EL0`, writes the
  command, rings core 3's mailbox; rivet stamps `CNTPCT_EL0` on arrival
  before parsing anything. One clock, so this is a true one-way latency.
- **Linux round trip** — the same send, timed until rivet's reply becomes
  visible in the console ring. Detected by watching the write pointer move
  rather than by parsing text, since parsing would time the parser.

## The tick handler investigation

The tick handler cost row originally read 22 long samples out of 30000
idle against 9168 loaded. A step change like that is not gradual
degradation, so it was worth running down rather than quoting.

The first hypothesis offered was that the tick handler polls the shared
mailbox or ring on every tick, that the line sits cheaply in rivet's core
while Linux is idle and gets invalidated by coherency traffic when the
other cores are busy, and that the fix is to move the check onto the
mailbox interrupt. That is a coherent story and it is wrong here, in three
separate ways worth recording because each one is a thing this SoC does
differently:

- **The tick handler does not touch shared memory.** It calls exactly two
  things, `rivet::watchdog::on_tick` and `rivet::timer::poll_timers`.
  Neither goes near the shared window.
- **The doorbell is already an interrupt.** There is nothing to move.
  Linux rings an ARM-local mailbox, `IRQ_SOURCE_MBOX0` fires, and the
  handler signals a task. There is also no GIC on a BCM2837 to move it to.
- **The shared window is Device-nGnRnE, so it is never cached.** There is
  no line to hold and none to evict, and no coherency traffic is generated
  for it in either direction. That is a deliberate choice, made so both
  tiers agree on visibility without cache maintenance.

The way to settle it was to measure rather than argue, so `tick_anatomy`
times each stage of the handler separately and adds a control: eight
read-modify-writes against private statics, doing no useful work, touching
about one cache line. Idle against loaded:

| stage | idle mean | loaded mean | idle >1us | loaded >1us |
|---|---|---|---|---|
| bookkeeping (own statics) | 104 | 104 | 1 | 0 |
| watchdog | 0 | 0 | 0 | 0 |
| timer wheel | 520 | 781 | 16 | 6586 |
| control (private RMW) | 104 | 156 | 0 | 0 |

That rules out both hypotheses at once. Nothing shared is involved, and
generic memory contention is not the answer either, because the control
barely moved. All of the effect is in `poll_timers`.

`poll_timers` swept `TIMER_SLOTS` and `PTASK_DEADLINES` in full on every
tick, armed or not: 256 plus 128 bytes at the default sizes, six cache
lines, touched once per 100 us and not otherwise. That is precisely the
footprint that lives in L2 rather than L1, and the four Cortex-A53 cores
on this part share one unified L2. Linux's working set evicts those lines
between ticks and each tick refetches them from DRAM.

The control is what makes this quantitative rather than a story. It moved
52 ns for its one line; the sweep moved 261 ns for its six. A 5.0x ratio
of deltas against a 6x ratio of cache lines is the mechanism, measured.

The fix is a cached earliest-deadline, checked before touching either
array, so a tick with nothing due reads one value and no array at all.
Correctness rests on a one-sided invariant: the cached value is never
greater than the true earliest deadline. Arming lowers it, cancelling
leaves it alone, and only the sweep that recomputed it from the arrays may
raise it, inside the same critical section. Understating it costs a
redundant sweep; overstating it would skip a wake, so the code is arranged
so only the first can happen.

Effect on the unchanged benchmark:

| | before | after |
|---|---|---|
| tick cost, idle mean | 572 ns | 312 ns |
| tick cost, idle >1us | 22 / 30000 | 6 / 30000 |
| tick cost, loaded mean | 885 ns | 677 ns |
| tick cost, loaded >1us | 9168 / 30000 | 5408 / 30000 |
| interrupt latency, idle max | 416 ns | 1041 ns |
| cheapest possible tick | 520 ns | 260 ns |

The floor halving, 520 to 260 ns, is the cleanest evidence that the sweep
is what went away. The loaded tail roughly halves rather than returning to
the idle figure, and that is the honest ceiling on this change: the sweeps
that remain are ones with a timer genuinely due. `rt_bench` has two tasks
sleeping at 1 ms against a 100 us tick, so about one tick in five has real
work. Removing the rest of the tail means reducing how often timers fire,
not how much a firing costs.

The interrupt latency maximum is the one figure that did not improve and
it moves around between runs regardless (416, 1041 and 1302 ns have all
been observed idle, always with zero or one sample over a microsecond).
Treat that column as run-to-run variation, not as a result.

To reproduce:

```sh
cd examples/rpi3b-amp
cargo build --release --features tick-phases
# load tick_anatomy_amp and read the console
```

The feature is off by default because it puts three counter reads, each
with an ISB, on a path that runs ten thousand times a second, inside the
interval the handler reports as its own cost. Compare its columns against
each other, not against the main table.

## Measuring it from outside the machine

Everything above is rivet timing itself. That is a real measurement, but
it is an argument the software makes on its own behalf using a counter it
reads with instructions it also schedules. `scope_demo` puts the same
interval on three pins so an oscilloscope or logic analyser can measure it
instead.

The doorbell is the only figure where this is possible, and that is why it
is the one chosen. Its start event happens on the Linux side, so an
external observer can see both ends. A purely internal quantity like the
context switch has no externally visible start, and instrumenting one with
GPIO would mostly measure the instrumentation.

### Wiring

Four adjacent pins in the corner of the 40-pin header, so three probes and
a ground reach without spanning the board.

| Header pin | GPIO | Channel | Driven by |
|---|---|---|---|
| 38 | 20 | A | Linux, immediately before ringing the doorbell |
| 40 | 21 | B | rivet, first statement in the interrupt handler |
| 37 | 26 | C | rivet, in the task the doorbell wakes |
| 39 | — | GND | ground for all three probes |

Trigger on A rising. One capture gives three intervals:

- **A to B** is Linux's MMIO write reaching rivet's interrupt handler.
- **B to C** is the scheduler waking the task that was awaiting the
  doorbell.
- **A to C** is the whole path, and the figure the table above reports as
  "Linux doorbell to task".

### Running it

```sh
sudo rivet-amp load /tmp/scope_demo.img
sudo rivet-amp console &        # prints a pulse count every 2 s
sudo rivet-amp scope 200 5      # 200 pulses, 5 ms apart
```

`rivet-amp scope` asks for `SCHED_FIFO` and `mlockall`, and says so if it
cannot have them. Neither makes the send deterministic, since the tail of
this distribution belongs to Linux either way, but they remove the
easiest sources of outliers.

It also reads the pads back through `GPLEV` while pulsing, and reports
whether each of rivet's two pins was ever observed high. Without that, a
wiring mistake and a doorbell that never arrives look identical on a scope
that is not yet connected. Confirmed on hardware, idle and under the load
described above: 200 rings produced 200 pulses both times, with both pins
seen high.

### Reading the trace honestly

A GPIO write here is a store to Device memory, so each edge costs a trip
to the peripheral bus, and that cost sits inside the measured interval on
both sides. The scope will therefore read somewhat longer than the table
above, which stamps a counter register instead.

Neither number is wrong and they answer different questions. The scope
figure includes the cost of being observed, which is what you want if you
intend to react to the doorbell by driving a pin. The software figure is
what you want if you intend to react to it in software.

### Why the two sides need no lock

Both sides drive pins in the same GPIO bank with no coordination at all,
which is safe for a specific reason: `GPSET` and `GPCLR` are write-to-set
and write-to-clear, not read-modify-write. Writing bit 20 from Linux
cannot disturb bit 21 or 26, whatever rivet is doing at the time.

`GPFSEL`, which selects pin function, does not have that property. So all
three pins, including the one only Linux ever drives, are configured by
rivet's image at startup. If each side configured its own, the two
read-modify-write updates could lose each other.

## Known gaps

- Nothing here is a proven bound. See [realtime.md §1](realtime.md).
- The load generator is shell loops rather than a calibrated tool like
  `stress-ng`, which is not installed on the stock image. It saturates all
  three cores and the I/O path, but it is not a reproducible load profile
  in the sense a benchmark suite would want.
- The doorbell and round-trip maxima are Linux scheduling artefacts and
  would need `SCHED_FIFO` on the sending process to say anything about the
  channel's own worst case.
- The scope demonstration covers the doorbell path only. The other rows
  remain self-timed.
- Only the 10 kHz tick is tabulated. The quantisation relationship is
  arithmetic, but the other rates are not separately measured here.
