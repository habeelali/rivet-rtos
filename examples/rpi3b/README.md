# Raspberry Pi 3B bring-up

Bare-metal AArch64 on a Raspberry Pi 3 Model B (BCM2837, quad Cortex-A53).
This is the first milestone of a new port: prove the toolchain, the boot
path and the serial console on real silicon. There is no kernel here yet,
and no Linux.

Three binaries:

- `bringup` — reports the state the firmware handed over, brings up both
  UARTs, drops to EL1, then heartbeats.
- `faultcheck` — provokes the fault that gated the port, and proves the
  exception vectors work.
- `mmucheck` — installs an identity map, enables the MMU and both caches,
  and re-runs the atomics that `faultcheck` shows aborting without it.

The kernel itself lives in `examples/rpi3b-kernel`, a separate package so
that these three keep linking neither it nor the arch crate, which is
what makes them tests of the bare hardware rather than of rivet.

Status: confirmed on hardware up to and including the kernel running,
scheduling and preempting. What remains is releasing the other three
cores, and then the Linux-alongside arrangement.

## Build and run under QEMU

`qemu-system-aarch64 -M raspi3b` identifies itself as "Raspberry Pi 3B
(revision 1.2)", the same board this targets, and covers everything
except the firmware and the pin muxing.

```bash
cargo build --release
rust-objcopy -O binary ../../target/aarch64-unknown-none/release/bringup kernel8.img

# PL011 is serial_hd(0), the mini UART is serial_hd(1).
qemu-system-aarch64 -M raspi3b -kernel kernel8.img -display none \
    -serial file:pl011.txt -serial file:miniuart.txt
```

Piping QEMU's stdio into `head` or `grep` can swallow the output through
buffering; capture to files instead.

Source-level debugging, which stands in for the JTAG this board cannot
practically offer (see below):

```bash
qemu-system-aarch64 -M raspi3b -kernel kernel8.img -display none -serial null -S -s &
gdb-multiarch ../../target/aarch64-unknown-none/release/bringup \
    -ex 'target remote :1234' -ex 'break rust_main' -ex continue
```

## Build an SD card

`./mkboot.sh [binary] [outdir]` builds the image, fetches the Pi firmware
(all blobs pinned to one commit, since `start.elf` and `fixup.dat` are a
matched pair) and writes a `config.txt`. Copy the staged contents onto a
FAT32 first partition in an MBR table (type `0x0C`); the Pi 3 boot ROM
does not read GPT.

```bash
./mkboot.sh bringup boot-staging
```

## Wiring

A 3.3 V USB-to-TTL adapter, cross-wired to the header. Do not connect the
adapter's power pin; power the Pi from its own supply.

| Adapter | Pi header |
|---|---|
| GND | pin 6 (GND) |
| RXD | pin 8 (GPIO14, TXD) |
| TXD | pin 10 (GPIO15, RXD) |

Then `picocom -b 115200 /dev/ttyUSB0` (exit with `Ctrl-A Ctrl-X`), started
*before* powering the board so nothing is missed.

Worth doing once before trusting any of this: short the adapter's own TX
to its own RX and type into the terminal. Characters echoing back proves
the adapter, cable, driver and baud rate all work, and removes four
variables from any later failure.

## Reading the output

The boot sequence emits single-character checkpoints from its first
instructions, so a boot that dies halfway still says how far it got.

```
A0000000000080000    raw PL011 poke, then the image's true load address
BCD                  .bss zeroed / FP untrapped / vectors installed
==== rivet rpi3b bring-up ====
...
TICK n=0
```

| What appears | What it means |
|---|---|
| Nothing at all | Card, firmware files or wiring. Not the binary: check MBR + FAT32 first partition, and that all firmware came from one commit. |
| Firmware boot log, then silence | `uart_2ndstage` worked, so the whole serial path is proven. The image did not load, or died before checkpoint `A`. |
| `A` and an address, then silence | Executing. The printed address should be `0000000000080000`; anything else means `kernel_address` was ignored. |
| Checkpoints, banner, then `TICK` | Done. |
| Steady mojibake | Baud mismatch, so `init_uart_clock` was not 48 MHz. The banner prints the divisors the firmware left behind, which gives the real clock. |
| PL011 text but no mini UART text, or the reverse | Settles which UART reaches the header pins. |

The banner also reports `CurrentEL`, `MPIDR_EL1`, `CNTFRQ_EL0` and
`SCTLR_EL2`, so one boot confirms on real silicon the entry state that is
otherwise taken on trust from the firmware's ARM stub.

## Confirmed on hardware

Measured on a Pi 3B, `boardrev a02082`, booting the image this directory
builds. Recorded because several of these were assumptions until the
board printed them, and because QEMU disagrees with two of them.

| | Value | Note |
|---|---|---|
| Entry EL | `CurrentEL` = 2 | EL2h, as the firmware's ARM stub sets up |
| `SCTLR_EL2` | `0x30c50830` | RES1 bits only: MMU, caches and alignment checking all off. QEMU reports `0x0`. |
| `CNTFRQ_EL0` | 19200000 Hz | QEMU reports 62500000, so never hardcode this |
| Load address | `0x80000` | `kernel_address` honoured; firmware logs the same |
| `MPIDR_EL1` | `0x80000000` | core 0, bit 31 RES1 |
| DTB pointer | `0x2eff7400` | `x0` on entry, near the top of the 948 MB the firmware leaves the ARM |
| PL011 divisors left by firmware | IBRD 26, FBRD 3 | Identical to what this driver computes, which confirms `init_uart_clock` really is 48 MHz |
| Atomic RMW with MMU off | aborts, ESR `0x96000035` | EC 0x25, DFSC 0x35. **QEMU permits it instead.** |
| Atomic RMW with MMU on | works | `fetch_add` and `compare_exchange` both correct, once RAM is Normal Inner-Shareable |
| `SCTLR_EL1` after enable | `0x30d01805` | M, C and I set over the `0x30d00800` the EL2 drop leaves |
| `TCR_EL1` / `MAIR_EL1` | `0x200803520` / `0xff` | Identical to QEMU, so the table format is right on both |
| Kernel tick accuracy | 500025 us for a 500 ms sleep | 0.005% off, against 500530 under QEMU: the architected timer is crystal-derived here and emulated there |
| Preemptive scheduling | two workers, 48 iterations each | Equal progress at equal priority, so tasks really are being suspended and resumed around each other |

Both UARTs reach GPIO14/15, so either can be the console. The PL011
remains the better choice for the reasons above.

## Why there is no JTAG here

`enable_jtag_gpio=1` does work, and is applied by the firmware before any
kernel loads, so it is OS-independent. But the signals it exposes
(GPIO22-27) include no `nSRST`, so connect-under-reset and halt-at-reset
are impossible: a debugger can only attach to a core already running.
The failure this milestone is most exposed to, an image that never starts,
is exactly the one JTAG could not have diagnosed anyway. Hence the
checkpoint-heavy design, and QEMU plus GDB as the substitute.

An ST-Link cannot serve as the probe regardless: the Pi exposes JTAG
rather than SWD, and ST-Link's JTAG is limited to STM32 parts.

## What QEMU does not cover

Worth being explicit, because these are exactly what the physical test is
for:

- **GPIO alt-function muxing is not modelled.** Both UARTs respond no
  matter what `GPFSEL1` says, so QEMU will happily pass a completely
  wrong pin mux.
- **The firmware never runs.** `bootcode.bin` and `start.elf` are not
  involved, so `config.txt`, the load address, overlay processing and
  `init_uart_clock` are all unverified until the card boots.
- **Atomics behave differently.** See below.

## The constraint that gates the rest of the port

With the MMU off, AArch64 treats all memory as Device-nGnRnE, which has
no exclusive monitor. The `LDXR`/`STXR` pair behind
`AtomicUsize::fetch_add` and friends is documented to fault on this SoC
with ESR `0x96000035` (data abort, fault status `0x35`, "unsupported
exclusive or atomic access"). The kernel uses atomic read-modify-write in
a few dozen places, so it cannot be linked in until an identity map
exists describing RAM as Normal Inner-Shareable Write-Back memory. That
is the next milestone, and the reason this image depends on no `rivet`
crate at all.

The `atomics-polyfill` approach used for the RP2040 is not a substitute.
It masks interrupts, which buys atomicity against preemption on one core
and nothing against the other three coherent A53s.

**QEMU does not reproduce this.** Its model permits the exclusive access
and `fetch_add` simply succeeds. `faultcheck` reports whichever way it
goes rather than assuming, and then executes a `BRK` regardless so the
vector table and fault decoder are proven either way. Confirming the
abort on real hardware is one of the open questions the physical test
settles.
