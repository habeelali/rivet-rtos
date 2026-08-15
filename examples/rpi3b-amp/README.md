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

## The one genuinely uncertain step

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

## What the demo measures

Once running, rivet reports every second:

```
[rivet] t=5s wakeups=500 worst_jitter_us=<n>
```

A task waking every 10 ms, with the worst deviation it has seen. That
number is the actual claim being made: it should stay flat regardless of
what Linux is doing on the other three cores, because nothing Linux
schedules can preempt a core it does not own.

Worth being honest about the ceiling. Core isolation removes scheduler and
interrupt interference, not memory-system interference: DRAM and the L2 are
shared, so heavy traffic from the Linux cores can still show up as jitter.
Measuring that is the point of printing the worst case rather than an
average.

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
