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

## Jumpers, for the two tests that need wires

Everything above runs on a bare board. Two checks have a stronger form
that needs a wire, and report the weaker one until it is there:

| Wire | Header pins | Turns into |
|---|---|---|
| GPIO23 to GPIO24 | 16 to 18 | one pin proven to drive another, rather than a pin reading back its own level |
| MOSI to MISO | 19 to 21 | a real SPI0 loopback, rather than a transfer that completes into a floating input |

Nothing else changes: `periph_test` detects both and says which case it
measured.

## What is not done yet

- Peripheral interrupts still go wherever `GPU_INTERRUPTS_ROUTING` at
  `0x4000_000c` points, which is one global choice for the whole system
  rather than per-core. Rivet's core realistically wants timer-driven work
  and polled peripherals, leaving the aggregated peripheral IRQ to Linux.
  See the GIC note in `examples/rpi3b/README.md`.
- Nothing stops rivet writing into Linux's memory. The identity map covers
  all of RAM as Normal, and narrowing it to just the reserved region would
  turn an errant pointer into a fault instead of silent corruption of
  another OS.
