# Rivet alongside Linux on a Raspberry Pi 3

Linux on cores 0-2, rivet with core 3 to itself. Linux keeps the serial
console and the peripherals; rivet gets a core, its own timer, and memory
Linux was told not to manage.

This builds directly on the bare-metal `core3_demo`, which already proved
the handover works when the releasing core was our own code. The only
thing that changes is who does the releasing.

## How the pieces fit

| | |
|---|---|
| `maxcpus=3` | Linux brings up cores 0-2 and leaves core 3 spinning in the firmware's mailbox loop, exactly where the stub parked it |
| `mem=768M` | Linux manages only the first 768 MiB. Everything above is invisible to it, which is both how rivet gets memory and why `/dev/mem` will map it |
| image at `0x3000_0000` | Just above that boundary. Set by `RIVET_RPI3B_LOAD_ADDR`, checked at runtime against where the image actually landed |
| ring at `0x3100_0000` | Shared console, mapped Device on the rivet side to match Linux's uncached `/dev/mem` view |
| spin table at `0xf0` | Core 3's mailbox. Write the entry address, `SEV`, and it goes |

## Confirmed against a real Linux boot

From a Pi OS boot (kernel 6.12.75, arm64) with the options above:

```
Memory limited to 768MB
Reserved memory: created CMA memory pool at 0x1e000000, size 256 MiB
OF: reserved mem: 0x1e000000..0x2dffffff  map reusable linux,cma
NUMA: Faking a node at [mem 0x00000000-0x2fffffff]
SMP: Total of 3 processors activated.
Root IRQ handler: bcm2836_arm_irqchip_handle_irq
arch_timer: cp15 timer(s) running at 19.20MHz (phys)
vc_mem.mem_base=0x3ec00000
simple-framebuffer: framebuffer at 0x3eaa9000
```

Which pins the layout:

| Range | Owner |
|---|---|
| `0x0000_0000`-`0x2fff_ffff` | Linux, CMA included |
| `0x3000_0000`-`0x3eaa_8fff` | free. Rivet's image and shared ring sit at the bottom of this |
| `0x3eaa_9000`+ | framebuffer, then VideoCore from `0x3ec0_0000` |

So the reserved region is genuinely clear at both ends: CMA stops at
`0x2dff_ffff`, well below rivet, and the framebuffer starts far above it.

Two things worth knowing. CMA takes 256 MiB out of the 768, leaving Linux
about 453 MiB actually usable, so `cma=64M` is worth adding if Linux needs
the room. And Linux brings up exactly three CPUs and never touches the
fourth, which is the whole premise holding.

## The console problem

There is one UART on the header once `disable-bt` has moved Bluetooth
aside, and Linux is already using it. Rather than interleave two consoles
onto one line, rivet writes into a ring in the shared window and
`rivet-amp console` drains it from the Linux side.

The `amp` feature also removes the boot-time UART checkpoints and the core
guard from the entry path, for the same reason: poking the PL011 would cut
into Linux's output mid-line, and the core that arrives is by definition
the one rivet was handed.

## Build

```bash
cd examples/rpi3b-amp
cargo build --release
rust-objcopy -O binary ../../target/aarch64-unknown-none/release/amp_demo rivet-amp.img
```

## Provisioning a card from scratch

`linux/provision.sh` applies every change this arrangement needs to a
freshly imaged Pi OS Lite card, from the machine with the card reader:

```bash
./linux/provision.sh /media/you/bootfs /media/you/rootfs rivet.img
```

It is idempotent and backs up everything it replaces as `.stock`. Prefer
it over hand-editing, because several of the steps are decisions rather
than settings and the script records why. `linux/mkimage.sh` then clones
the finished card into one distributable `.img`.

Boot it and rivet starts by itself:

```bash
journalctl -b -u rivet -u rivet-console   # what the RTOS said, at boot
sudo rivet-amp send ping                  # talk to it
```

## Memory: reserved-memory, not mem=

The carve-out is a `/reserved-memory` node marked `no-map`, which Linux
confirms at boot:

```
OF: reserved mem: 0x30000000..0x311fffff (18432 KiB) nomap non-reusable rivet@30000000
```

`no-map` means Linux creates no mapping for that range at all, which is a
stronger statement than not allocating from it. It also replaced an
earlier `mem=768M`, which worked but made Linux ignore *everything* above
the line: switching to the node handed 157 MiB back (748 MiB to 909 MiB
of usable RAM) for no loss of protection.

Worth being exact about what that protects against. Linux will not touch
the region in normal operation, and no process can reach it through
ordinary allocation. Root with `/dev/mem` still can, and always will,
because that is precisely how the loader writes the image in. That hole
cannot be closed without giving up the loader, so it is stated rather
than papered over.

## Configure the boot partition

`cmdline.txt` gains two options, on the single line it already has:

```
maxcpus=3 mem=768M
```

`config.txt` stays as the Linux one (`config.txt.pi-os-backup` on the card
is the original), with `enable_uart=1` so the console works.

## On the Pi

```bash
cc -O2 -o rivet-amp linux/rivet-amp.c

sudo ./rivet-amp probe               # is this machine set up?
sudo ./rivet-amp load rivet-amp.img  # copy in, release core 3
sudo ./rivet-amp console             # watch rivet's output
```

`probe` reports each prerequisite separately rather than failing as a
unit, because they fail for different reasons and want different fixes.

## The spin table maps, so plain userspace is enough

This was the open question, and it is settled: on Pi OS Lite with kernel
6.18.34, `/dev/mem` maps all three regions.

```
map reserved 0x30000000 : OK
map shared   0x31000000 : OK
map spintable 0xd8      : OK
```

The reserved and shared windows map because `mem=768M` keeps them out of
Linux's memory map entirely. The spin table maps for a different reason,
visible in `/proc/iomem`:

```
00000000-2fffffff : System RAM
  00000000-00000fff : reserved
```

The first page is carved out as `reserved` inside System RAM, so
`STRICT_DEVMEM` does not treat it as ordinary memory. Neither fallback
below is needed. They are kept because a different kernel build could
easily decide otherwise, and because the armstub route stays the better
design for anything permanent.

## The fallbacks, if a kernel ever refuses

Mapping the spin table. `mem=768M` keeps the reserved region out of
Linux's memory map, so `CONFIG_STRICT_DEVMEM` permits mapping it. The spin
table at `0xd8` is at the bottom of memory, which Linux may well consider
System RAM and refuse. `probe` answers that on a given kernel before
anything is written, which is why it exists.

If it does refuse, there are two ways forward:

**A kernel module**, which reaches the same physical address through
`ioremap` and is not subject to `/dev/mem`'s policy. Needs
`raspberrypi-kernel-headers` on the Pi. This is the smaller change.

**A custom `armstub`**, which is the better long-term answer and needs
nothing from Linux at all. `config.txt` can load an arbitrary blob to an
arbitrary address with `initramfs rivet-amp.img 0x30000000`, and
`armstub=` replaces the EL3 stub that runs before any kernel. A stub that
sends core 3 straight to `0x3000_0000` and lets cores 0-2 boot Linux
normally removes the loader, the `/dev/mem` question and the release
timing all at once. It is more work, and a wrong stub means the board does
not boot at all until the card is rewritten.

## What the demo measures, and what it measured

A task waking every 10 ms, reporting the worst deviation in the last
second, the worst since warm-up, and how many wakeups missed their slot
by more than half a tick.

Measured on a Pi 3B with rivet on core 3 and Linux on 0-2:

| Linux state | typical jitter | slipped wakeups |
|---|---|---|
| idle | 1 us | 1 in 57 s |
| all three cores spinning, plus `dd` memory traffic | 2-3 us | unchanged |

Saturating every core Linux owns, and the shared memory system with it,
moves the typical figure by about two microseconds. Nothing Linux
schedules can preempt a core it does not own.

### Chasing the outlier

The first version of this reported a flat `worst=999` forever, which was
not jitter at all: the very first wakeup aligns onto the tick grid and
carries up to a full tick of phase error, and a lifetime maximum with no
warm-up captured that and never let go. Hence the warm-up period and the
per-window figure.

With that fixed, a real outlier appeared: a single-tick slip roughly
every twelve seconds, at the same rate whether Linux was idle or loaded.

The obvious suspect was clock mismatch, since deadlines are kept in
`now_us` time (the 1 MHz System Timer) while wakeups land on architected
timer ticks. `linux/clockdrift.c` measures that directly, and it is
**2.5 ppm**, which predicts one slip every 395 seconds. Far too small.
Hypothesis wrong.

The actual cause was in the tick itself. Re-arming with
`CNTP_TVAL_EL0 = interval` sets the next expiry relative to *the moment
of the write*, so every single tick silently absorbed the exception
entry, the register save and an MMIO read. A small constant, added
forever, walking the tick grid away from real time. Advancing the
absolute comparator `CNTP_CVAL_EL0` instead cannot drift: a late handler
gives one late tick, not a permanently skewed grid.

That took slips from one per twelve seconds to one in fifty-seven. The
remainder is consistent with the residual 2.5 ppm, though with a single
observation the rate is not really characterised.

## Rivet cannot reach Linux's memory

The identity map covers only what this image owns. Read back out of
rivet's own translation tables, from Linux, while it was running:

```
0x000000000000-0x00001fffff  Normal      spin table, for releasing cores
0x000000200000-0x002fffffff  unmapped    Linux
0x000030000000-0x0030ffffff  Normal      rivet
0x000031000000-0x00311fffff  Device      shared ring
0x000031200000-0x003effffff  unmapped
0x00003f000000-0x003fffffff  Device      peripherals
```

Zero blocks covering Linux's RAM are mapped. A stray pointer in rivet now
takes a translation fault with the address in `FAR_EL1`, rather than
silently corrupting another operating system to be discovered much later
somewhere else entirely.

## Kernel features and peripherals, on core 3 alongside Linux

Measured on the board, with Linux running on cores 0-2:

```
==== rivet rpi3b kernel features ====
  skip watchdog: system-wide and owned by Linux in this build
  ok   Channel round-tripped five values
  ok   Signal woke an async task from the preemptive tier
  ok   Semaphore acquired, held and released
  ok   async task ran and slept through the executor
  ok   preemptive sleep_ms kept its deadline
KERNEL_FEATURES_OK

==== rivet rpi3b peripherals ====
  skip PL011 loopback: the UART belongs to Linux in this build
  ok   GPIO output read back its own driven level
  note GPIO24 did not follow GPIO23: no jumper, as expected bare
  note SPI0 transfer completed, read back 00000000 (no jumper)
  ok   I2C1 scanned 112 addresses, all NAKed (nothing on the bus)
PERIPH_TEST_OK
```

Two things cannot be exercised while sharing the machine, and both are
skipped rather than faked. The UART is Linux's console. The watchdog is
worse than shared: it is a single system-wide countdown that resets the
whole SoC, systemd claims it at boot, and arming it from here took the
board down mid-run about two seconds after the test stopped feeding it.
That generalises to every global BCM2837 resource, `GPU_INTERRUPTS_ROUTING`
and `core_freq` included. Per-core facilities are fine, which is exactly
why the tick works.

## Live tracing

Built with `--features trace`, the kernel emits PulseTrace frames from
the scheduler, fault and IRQ paths on its own. They go to a second ring
in the shared window, never the console: the wire format is framed
binary and a log line landing mid-frame corrupts it.

```bash
sudo rivet-amp load trace_demo_amp.img
sudo rivet-amp trace /tmp/rivet.ptrace     # binary frames
sudo rivet-amp console                     # text, at the same time
```

Captured from core 3 while Linux ran on the others, sixteen seconds of a
two-task workload produced 175344 bytes: 5485 frames, uniformly 32 bytes
apart, sync words spanning the whole file with no gaps. The text console
reported the trace ring staying near empty throughout, which is the
reader keeping up rather than the producer stalling.

## Commanding rivet from Linux

The rings above only report; the command ring runs the other way and
makes this a channel. Linux writes a line into it and rings rivet's
mailbox doorbell:

```bash
sudo rivet-amp send ping
sudo rivet-amp send "period 50"
sudo rivet-amp send stats
```

The doorbell is the part worth having. A ring alone only supports
polling, which on a real-time core is wrong twice over: it burns the core
while idle and still adds latency when busy. The ARM-local mailboxes are
per-core and raise an interrupt on the target, which makes them the only
interrupt source on this SoC that can be aimed at one core; peripheral
IRQs all go wherever the single global routing register points. So rivet
sits in `WFI` until Linux asks for something.

Measured: the doorbell count matched the command count exactly, so no
spurious or missed interrupts.

## Pinning the clock

BCM2837 has one ARM clock domain for the whole cluster and Linux owns the
cpufreq driver, so by default the real-time core's speed is decided by
what the other three are doing. `force_turbo=1` in `config.txt` pins it,
and a `performance` governor unit does the same from the Linux side.
Verified: with it set, forcing the `powersave` governor and waiting
fifteen seconds leaves the clock at 1.2 GHz.

It is worth doing for latency, not just throughput. Frequency transitions
stall the core while the PLL relocks, and those transitions turned out to
be the dominant source of worst-case interrupt latency:

| | before pinning | after pinning |
|---|---|---|
| IRQ latency, max | 989 ns | **52 ns** |
| tick handler, max | 1979 ns | **572 ns** |
| Signal wake, max | 2447 ns | **989 ns** |

52 ns is one tick of the 19.2 MHz counter, so with the clock pinned the
worst observed interrupt latency over 3001 samples equals the best: the
measurement floor. `force_turbo=1` alone does not set the OTP warranty
bit; only combining it with `over_voltage` does, and this does not.

## The 1 ms outlier, explained

A periodic task showed an occasional ~1 ms deviation, one or two per five
hundred wakeups, which survived both the comparator fix and pinning the
clock. It turned out not to be a defect at all, and the way it was chased
is worth recording because the first hypothesis was wrong.

The guess was that a 10 ms period is an exact multiple of the 1 kHz tick,
so every deadline lands on a grid boundary where noise decides which tick
catches it. Testing an off-grid period should then have removed the
outliers. It did the opposite: at 10.5 ms the gap error became a
*constant* 500 us, because deadlines alternate either side of the grid
and consecutive gaps alternate 10.0 and 11.0 ms.

What the experiment did show is that the metric was wrong. Gap between
wakeups is the difference of two quantisation errors, so it swings by a
whole tick while nothing is actually late. Lateness against the absolute
deadline is the honest measure, and at both periods its maximum was
almost exactly one tick:

| tick rate | max lateness | mean lateness | tick cost |
|---|---|---|---|
| 1 kHz | 1001 us | 332 us | 0.05% |
| 10 kHz | 100 us | 36 us | 0.5% |
| 100 kHz | 10 us | 5 us | 5% |

The bound tracks the tick period exactly, an order of magnitude at a
time, which is the signature of quantisation and of nothing else.
`sleep_until` wakes at the first tick at or after the deadline, so
lateness is uniform in `[0, tick)` by construction. Deadlines are
absolute (`next += period`), so nothing drifts: a late wakeup is followed
by an early one and the long-run rate is exact.

So the fix is a configuration choice, not a patch. The AMP build now runs
a 10 kHz tick, which costs about half a percent of the core (the handler
measures 520 ns) and takes the worst-case deadline from 1000 us to 100
us. 100 kHz is available and works, at 5% of the core.

Below that, the floor stops being the tick and becomes interrupt latency,
around 1 us. Getting there needs a different design: programming the
comparator for the next deadline rather than polling deadlines on a fixed
grid.

## Jumpers, for the two tests that need wires

Everything above runs on a bare board. Two checks have a stronger form
that needs a wire, and report the weaker one until it is there:

| Wire | Header pins | Turns into |
|---|---|---|
| GPIO23 to GPIO24 | 16 to 18 | one pin proven to drive another, rather than a pin reading back its own level |
| MOSI to MISO | 19 to 21 | a real SPI0 loopback, rather than a transfer that completes into a floating input |

Nothing else changes: `periph_test` detects both and says which case it
measured.

## Running it

`rivet` is the front door. Two commands with different names read as two
projects, so everything is behind one:

```sh
rivet status              what is running, on which core, where
rivet images              what else can run
rivet boot <image> -r     switch and reboot
rivet console             follow the RTOS console
rivet send ping           talk to it
rivet --help              the rest
```

It starts itself at boot. Four units, all enabled:

| Unit | Does |
|---|---|
| `rivet-build` | compiles `rivet-amp` if it is missing, once |
| `rivet` | loads the selected image onto core 3, then announces it to `dmesg` |
| `rivet-console` | relays the RTOS console into the journal, forever |
| `rivet-health` | watches the heartbeat, fails loudly if it stops |

`rivet.target` groups them, so a service that needs the RTOS can order
itself `After=rivet.target` without naming the individual units.

### Identity

`rivet-identity.sh` re-badges the presentation layer: `/etc/os-release`
becomes `Rivet RTOS + Linux 0.3.0 (Raspberry Pi 3B)` with
`VARIANT_ID=rivet-amp`, and `/etc/issue` and `/etc/motd` match. The
Debian-derived fields stay exactly as they were, because package tooling
depends on them; only what a human sees changes. The original is kept at
`/etc/os-release.stock`.

The RTOS also announces itself into the kernel ring buffer, so it lands in
`dmesg` next to the kernel's own boot messages instead of in a separate
log nobody correlates with them:

```
[   18.870343] rivet: RTOS 0.2.0 (channel_demo_amp) on core 3, 10000 Hz
               tick, running, system 0.3.0 build v0.1.0-44-g0d73b3c3
```

### One source of truth

The memory map used to be declared independently in the board crate, the
cargo config, the Linux loader and `provision.sh`, with nothing checking
they agreed. A change to one produced silent corruption rather than an
error, because every ring magic still matched and every pointer still
pointed somewhere.

It now lives in the device tree, in the `/reserved-memory` node the
provisioner already creates, and `rivet-amp` reads it at run time:

```dts
rivet@30000000 {
    compatible = "rivet,amp-core";
    reg = <0x30000000 0x1200000>;
    no-map;
    rivet,core = <3>;
    rivet,shared-offset = <16777216>;
    rivet,tick-hz = <10000>;
    rivet,abi = <1>;
};
```

The offset is written in decimal deliberately. `fdtput -t u` accepts a
`0x` literal without complaining and stores zero, which put the rings on
top of the image and left every reader waiting for a ring that was never
going to be there. `rivet-amp` now range-checks what it reads and says so
rather than using it.

### Health

The RTOS publishes a header in the last 4 KiB of the shared window: ABI
version, system and RTOS versions, build id, image name, tick rate, core,
memory windows, state, and a heartbeat.

That last one closed a real gap. A hung core and an idle one used to be
indistinguishable from Linux, because the console ring simply stopped
producing, which is also what a healthy system with nothing to say looks
like. Now:

```
  health ----------------------------------------
  heartbeat OK  30 beats in 300 ms at 100 Hz
  rivet uptime       17 s (1741 beats)
```

The heartbeat comes from the tick handler, once per hundred ticks, so it
proves the timer interrupt is still being taken. It does not prove the
scheduler is making progress: a task spinning forever at the top priority
would keep it beating, and that case belongs to the watchdog.

`state` distinguishes an orderly stop from a failure, so a finished
benchmark does not read as a crash. The exit and panic paths mask
interrupts before halting, so a stopped kernel actually looks stopped
rather than leaving the tick running and the pulse alive.

The ABI field is a handshake. An image and a loader built at different
times say so:

```
rivet image speaks header ABI 99, this tool speaks 1
  the image and the loader were not built together
```

### Talking to it after a cold boot

```sh
systemctl is-active rivet rivet-console rivet-health   # active active active
rivet status
rivet send ping                                        # -> [rivet] pong
```

### Choosing what runs

```sh
rivet-select                 # list images, star the current one
rivet-select rt_bench        # select it, takes effect next boot
rivet-select rt_bench -r     # select it and reboot now
```

**Switching images requires a reboot, and this is not a convenience
choice.** Releasing a core through the spin table latches: once core 3 has
left the firmware's pen and started an image, writing a different one into
memory does nothing, because nothing sends the core back to the pen to
pick up the new entry point. It cannot be restarted in place. Load a second
image onto a running core and it looks exactly like a hang, which is the
one failure worth knowing about in advance.

Installed images:

| Image | What it is |
|---|---|
| `channel_demo` | the default. Idles on the doorbell, answers commands, never exits |
| `rt_bench` | the characterisation suite |
| `scope_demo` | doorbell latency on three GPIO pins |
| `tick_anatomy` | per-stage timing inside the tick handler |
| `kernel_features` | tasks, mutexes, semaphores, channels, watchdog |
| `periph_test` | GPIO, SPI, I2C |
| `trace_demo` | emits PulseTrace frames while scheduling |

### The one conflict to know about

`rivet-console` holds the console ring, and so do `rivet-amp console` and
`rivet-amp bench`. Two readers on one ring eat each other's bytes. Stop the
service before running either by hand:

```sh
sudo systemctl stop rivet-console
...
sudo systemctl start rivet-console
```

### The benchmark suite

```sh
rivet-select rt_bench -r
# wait for it to come back
sudo systemctl stop rivet-console
sudo rivet-amp console      # self-contained rows, then it waits; Ctrl-C
sudo rivet-amp bench 400    # round trip and one-way throughput
sudo rivet-amp console      # the doorbell row, then RT_BENCH_OK
```

For the loaded numbers, start this before the run and reboot afterwards to
clear it:

```sh
for i in 1 2 3; do setsid bash -c "while :; do :; done" & done
setsid bash -c "while :; do cat /dev/urandom | gzip -1 > /dev/null; done" &
setsid bash -c "while :; do dd if=/dev/zero of=/tmp/lf bs=1M count=64; sync; rm -f /tmp/lf; done" &
```

Results in [docs/rpi3b-benchmarks.md](../../docs/rpi3b-benchmarks.md), and
which of them are guarantees in
[docs/rpi3b-guarantees.md](../../docs/rpi3b-guarantees.md).

### The scope demo

Probe header pins 37, 38 and 40 with ground on 39, then:

```sh
rivet-select scope_demo -r
sudo rivet-amp scope 200 5
```

Trigger on GPIO20 (pin 38) rising. GPIO21 (pin 40) is rivet's interrupt
handler, GPIO26 (pin 37) is the task it wakes. It reads the pads back as it
goes, so it reports whether rivet's pins moved even with no probe attached.

### Tracing

```sh
rivet-select trace_demo -r
sudo systemctl stop rivet-console
sudo rivet-amp trace /tmp/rivet.ptrace
```

Frames go to their own ring, not the console: the wire format is framed
binary and a log line in the middle of a frame corrupts it.

### Deploying a new image

From the build machine:

```sh
cd examples/rpi3b-amp
cargo build --release
rust-objcopy -O binary \
    ../../target/aarch64-unknown-none/release/channel_demo_amp channel_demo.img
scp channel_demo.img pi:/tmp/
ssh pi 'sudo cp /tmp/channel_demo.img /usr/local/lib/rivet/ && rivet-select channel_demo -r'
```

`tick_anatomy` is the one image that needs a feature:
`cargo build --release --features tick-phases --bin tick_anatomy_amp`.

## What is not done yet

- Peripheral interrupts still go wherever `GPU_INTERRUPTS_ROUTING` at
  `0x4000_000c` points, which is one global choice for the whole system
  rather than per-core. Rivet's core realistically wants timer-driven work
  and polled peripherals, leaving the aggregated peripheral IRQ to Linux.
  See the GIC note in `examples/rpi3b/README.md`.
- The load generator used for the benchmark numbers is shell loops, not a
  calibrated tool. It saturates every core and the I/O path, but it is not
  a reproducible load profile.
