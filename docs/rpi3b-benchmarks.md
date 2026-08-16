# Rivet on a Raspberry Pi 3B: measured real-time characterisation

Numbers for the AArch64 port, taken on a Raspberry Pi 3 Model B v1.2 with
rivet owning core 3 and Linux running on cores 0 to 2. Both result tables
below came off that board; nothing here is estimated, extrapolated, or
carried over from another port.

The tool that produces them is `examples/rpi3b-kernel/src/bin/rt_bench.rs`,
with the two Linux-side figures produced by `rivet-amp bench`.

Read this alongside [realtime.md](realtime.md), which sets out what a
measured number does and does not entitle you to claim. The short version
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
| counter read (floor) | 0 / 0 / 1458 | 1 / 20000 | 0 / 0 / 1406 | 1 / 20000 |
| reschedule, no task change | 729 / 729 / 3750 | 156 / 20000 | 729 / 729 / 4843 | 152 / 20000 |
| task-to-task switch | 364 mean | | 364 mean | |
| semaphore try/release | 156 / 156 / 2708 | 19 / 20000 | 156 / 156 / 3906 | 30 / 20000 |
| mutex try_lock/unlock | 677 / 677 / 5052 | 140 / 20000 | 677 / 677 / 5833 | 137 / 20000 |
| mutex handoff, contended | 937 / 937 / 1145 | 1 / 100 | 937 / 937 / 1250 | 1 / 100 |
| Signal to async task | 781 / 885 / 5260 | 19 / 200 | 781 / 1145 / 7187 | 87 / 200 |
| 64 writes, Normal cached | 156 / 156 / 1093 | 3 / 2000 | 156 / 156 / 2864 | 5 / 2000 |
| 64 writes, Device shared | 468 / 468 / 3072 | 9 / 2000 | 468 / 468 / 1458 | 8 / 2000 |
| dc cvac x8 + dsb ish | 52 / 52 / 989 | 0 / 2000 | 52 / 52 / 989 | 0 / 2000 |
| **interrupt, hardware to handler** | **52 / 52 / 416** | **0 / 30000** | **52 / 104 / 1145** | **1 / 30000** |
| tick handler cost | 520 / 572 / 1562 | 22 / 30000 | 520 / 885 / 2395 | 9168 / 30000 |
| Linux doorbell to task | 2135 / 2552 / 48802 | 400 / 400 | 2291 / 7395 / 54947 | 400 / 400 |
| Linux round trip | 5364 / 5833 / 54375 | | 5572 / 11614 / 59531 | |

Coarser figures, in their own units:

| | idle | loaded |
|---|---|---|
| tick-to-tick interval, 10 kHz | 99739 / 100000 / 100364 ns | 99062 / 100000 / 100989 ns |
| deadline lateness, 10 kHz tick | 81 / 87 / 95 us | 81 / 88 / 97 us |
| ring write, rivet side | 106 MiB/s | 102 MiB/s |
| ring one-way, rivet to Linux | 63 MiB/s | 30 MiB/s |

## Reading the numbers

**The isolation holds, and this is the row that says so.** Interrupt
latency is what core isolation exists to protect. Saturating the three
Linux cores moves its worst case from 416 ns to 1145 ns, and exactly one
sample in 30000 crossed a microsecond. The mean doubles, from 52 ns to
104 ns, which in counter ticks is one to two: the quantity is at the
instrument's floor in both conditions. Deadline lateness barely moves at
all, 95 us to 97 us.

**The tick handler is where the load actually lands.** Handler cost is the
one row that degrades sharply: 22 long samples out of 30000 idle against
9168 loaded, with the mean going 572 to 885 ns. That is memory-system
contention, since the handler touches the timer registers and the timer
wheel while three other cores hammer the same L2 and memory controller. It
does not reach the deadline figures because it stays far below the 100 us
tick period, but it is the term that would grow first at a higher tick
rate.

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

## Known gaps

- Nothing here is a proven bound. See [realtime.md §1](realtime.md).
- The load generator is shell loops rather than a calibrated tool like
  `stress-ng`, which is not installed on the stock image. It saturates all
  three cores and the I/O path, but it is not a reproducible load profile
  in the sense a benchmark suite would want.
- The doorbell and round-trip maxima are Linux scheduling artefacts and
  would need `SCHED_FIFO` on the sending process to say anything about the
  channel's own worst case.
- Only the 10 kHz tick is tabulated. The quantisation relationship is
  arithmetic, but the other rates are not separately measured here.
