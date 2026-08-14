# Rivet RTOS

[![crates.io](https://img.shields.io/crates/v/rivet-rtos.svg)](https://crates.io/crates/rivet-rtos)
[![docs.rs](https://img.shields.io/docsrs/rivet-rtos)](https://docs.rs/rivet-rtos)

> **[Read the full documentation →](docs/DOCUMENTATION.md)**: architecture,
> every feature, the port contract, configuration, and how to write an
> application. This README is the quick tour; that document is the
> complete reference.

```bash
cargo add rivet-rtos
```

Published as `rivet-rtos` on crates.io (`rivet` was already taken) but
imported as `rivet`. Every example below and in the docs uses `rivet::`
directly, no rename needed on your end.

**The kernel contains no allocator, and never will.** Every resource is a
fixed pool sized at compile time (see `rivet/build.rs` / the `RIVET_*`
environment variables): task stacks live in one `.task_stacks` pool, the
task registry, timer queue, waker bitmaps, and semaphore waiter bitmaps are
static arrays. Worst-case memory is bounded and statically known; there is
no fragmentation and no allocator in the scheduling or interrupt path.

A zero-allocation, dual-tier RTOS for ARM Cortex-M, RISC-V, and Xtensa, written in Rust (`no_std`). A **preemptive tier** (real per-task stacks, real timer-driven context switches, priority-inheritance mutex) sits above a **cooperative tier** (`async fn` tasks, single shared stack). The async executor runs as the lowest-priority preemptive task, so any real preemptive task immediately interrupts it. Two boards, an ESP32-S3 (dual-core) and an STM32F401RE Nucleo, are validated on **real hardware**, not just QEMU; the STM32F401RE has a formal, real-hardware-measured hard-real-time declaration (see [Hard real-time analysis](#hard-real-time-analysis)).

## What actually works today

- **Real preemption, not just cooperative scheduling.** A preemptive task that never calls anything cooperative, no `.await`, no yield, still gets forcibly switched out by the timer tick. Verified on every target, including real hardware: two same-priority tasks that just spin and increment a counter come out genuinely interleaved, not one running to completion before the other starts. See [Design notes](#design-notes) for how each arch's context switch actually works. All three had real, non-obvious correctness bugs that only a second pair of eyes, QEMU's own trap logs, or (for the Xtensa dual-hart race) live JTAG debugging actually pinned down.
- **`embedded-hal` / `embedded-hal-async` 1.0 support**, live-hardware-verified: typestate GPIO on 5 boards (stm32f401re, rp2040, esp32c6, esp32s3, lm3s6965), async SPI (PL022) and I2C (Stellaris + STM32F4 legacy I2C), and async `digital::Wait` on stm32f401re, all driven through a new `rivet::sync::Signal` completion primitive. See [`docs/driver-authoring.md`](docs/driver-authoring.md) for how to write your own driver.
- **Priority-inheritance mutex** (`preempt::PriorityMutex`), verified against the actual scenario it exists for: a low-priority task holds it, a higher-priority task blocks waiting, and a *third*, medium-priority task that never yields is ready the whole time. On the STM32F401RE this was measured directly on real hardware (`PRIORITY_INVERSION_BOUNDED`, zero medium-priority interference during the boosted holder's critical section), not just asserted by construction.
- **Real dual-core SMP** on the ESP32-S3 (`RIVET_MAX_HARTS=2`): cross-hart IPI-driven round-robin fairness, verified on real hardware after finding and fixing a genuine cross-hart data race via live JTAG (`docs/realtime.md` §15).
- **Parameterized preemptive tasks**: `fn(&'static Arg) -> !` with real, typed arguments (via `spawn_ptask!`), each on its own statically-allocated stack. No heap, no TAIT.
- **Real `async fn` tasks** via `#[rivet::task(priority = N)]` for the cooperative tier: genuine compiler-generated `Future` state machines, not hand-rolled state enums, and no nightly/TAIT required (see [Design notes](#design-notes)).
- **Interrupt-driven peripherals** (`rivet::irq`, NVIC-backed on Cortex-M): the STM32F401RE's USART2 console is built on it end-to-end, not just polling.
- **Typestate GPIO** (`rivet_bsp_lm3s6965::gpio`) for the Cortex-M board: pin direction is tracked in the type, not a runtime flag; calling `.set_high()` on a pin still typed as `Input` is a compile error.
- **Async sync primitives** for the cooperative tier: `Semaphore::acquire().await`, `Channel::send().await`/`recv().await`, lock-free, ISR-safe on the signaling side.
- **Tickless `Sleep`**: registers a deadline with a timer queue instead of busy-polling; the executor genuinely reaches `WFI` between events.
- **Seven validated boards** across three architectures: RISC-V (QEMU `virt`, ESP32-C6), ARM Cortex-M3 (QEMU `lm3s6965evb`, `mps2-an385`), ARM Cortex-M4 (**STM32F401RE, real hardware**), and Xtensa LX7 (**ESP32-S3, real hardware, dual-core**), proving the arch/board split actually holds, not just working on the one board it was written against. See `docs/porting.md` to add your own board. (An eighth, RP2040/Cortex-M0+, is in the workspace but explicitly experimental: compiles and partially boots, not yet a validated port. See `docs/DOCUMENTATION.md` §3.)
- **Optional live event tracing** (`trace` feature, off by default): `rivet::trace` emits scheduler dispatch, IRQ, mutex, and fault events over whatever transport a board's `port::board::trace_write` implements, for external tooling to consume. Zero cost when disabled.
- **Formal WCET analysis**, exact figures (not estimates) for interrupt latency, context-switch time, scheduling-decision cost, critical-section hold time, and priority-inheritance blocking time, each labeled by how it was obtained. See [Hard real-time analysis](#hard-real-time-analysis).
- **33+ host-side tests** (`cargo test -p rivet-rtos`), covering the scheduler, priority inheritance, waker bitmap, sync primitives, and an end-to-end async producer/consumer test driven through the real polling machinery.

## Hard real-time analysis

Two formal WCET documents, both method-labeled (every figure marked
**measured** on real hardware, **derived** from exact instruction counts,
**architectural** per the ISA's own spec, or **assumed**) rather than
presented as uniformly certain:

- **[`docs/wcet-stm32f401re.md`](docs/wcet-stm32f401re.md)**: STM32F401RE
  (Nucleo-64), single-core Cortex-M4, real hardware via ST-LINK/OpenOCD/GDB.
  Reaches a scoped **hard real-time declaration** for normal operation:
  zero-variance nested-interrupt latency (86 cycles, identical across
  500/500 samples), measured `PRIORITY_INVERSION_BOUNDED` with zero
  medium-priority interference, no cross-hart contention (single core).
- **[`docs/wcet.md`](docs/wcet.md)**: ESP32-S3, dual-core Xtensa LX7, real
  hardware via JTAG/UART. Finds the scheduler's own algorithm is genuinely
  bounded, but surfaces two real hazards specific to this board: a
  console-output path that holds a lock across a UART-baud-rate-bound wait
  (measured ~5.5 ms), and a cross-hart lock with no fairness guarantee,
  both documented as open items, not swept under the rug.

## What's not here yet

- No MPU sandboxing beyond stack-overflow guard bands, no RTT/defmt logging.
- No `embedded-hal` GPIO driver for `qemu-virt` (QEMU's RISC-V `virt` machine has no GPIO hardware to drive, not a closable gap) or `mps2-an385` (QEMU stubs its GPIO block as an unimplemented no-op device, nothing to verify against).
- A provably-fair (FIFO) cross-hart lock, needed to close the ESP32-S3's one remaining hard-RT gap (see above); not needed on any single-hart board.
- ESP32-C3 real-hardware support: architecturally blocked, not just untested. It lacks the RISC-V atomic extension the kernel's lock-free scheduler needs throughout (see `docs/DOCUMENTATION.md` §18).
- Preemptive tasks are `fn(&'static Arg) -> !`, no return value, no join/await-a-preemptive-task. That's a deliberate simplicity tradeoff for v0.1, not an oversight, but it means a preemptive task can't cleanly signal "I'm done" to anything except through its own side effects (a semaphore, a shared flag).

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│ Preemptive tier — priority 1..31, own stack per task      │
│   timer tick / yield_now() → save full context → schedule │
│   → restore a *different* task's context if warranted     │
├─────────────────────────────────────────────────────────┤
│ Cooperative tier — priority 0 (lowest), ONE shared stack   │
│   the async executor: polls ready #[rivet::task]s,         │
│   runs whenever no real preemptive task is ready            │
└─────────────────────────────────────────────────────────┘
```

Both tiers share one priority space conceptually, but only the preemptive tier's priorities are compared by the scheduler directly. The cooperative tier is just "whatever the async executor, sitting at preemptive-priority 0, decides to poll next" using its own separate priority-bitmap system among `#[rivet::task]`s.

A preemptive task's full context, every general-purpose register plus (arch-specific) the program counter, is saved on every switch. That's what makes it interruptible *anywhere*, not just at yield points. A cooperative task's "context" is just its `Future`'s state, live only across `.await`.

## Design notes

**Why real `async fn` tasks work without nightly Rust.** The compiler-generated `Future` type of an `async fn` is unnameable on stable Rust: you can't write `static TASK: TaskCell<F>` because `F` has no name (that needs `type_alias_impl_trait`, which is why Embassy requires nightly for its task macro). Rivet sidesteps this. `TaskCell<const SIZE: usize>` is generic over a **byte count**, not the future type; `SIZE` is just a `usize`, always nameable. The proc macro emits a thin, non-generic wrapper that calls a **generic** `TaskCell::poll::<F>()` method, monomorphized separately per task by the compiler, which reads/writes the future's bytes into the cell's buffer. `F` never has to be named. Size/alignment are checked at runtime on first poll (`assert!`, not silent UB).

**The RISC-V context switch is one unified trap path, and getting it right took two real, separately-diagnosed bugs.** RISC-V hardware auto-saves *nothing* on trap entry (unlike Cortex-M's automatic r0-r3/r12/lr/pc/xPSR stacking), so `rivet_trap_entry` is hand-written `global_asm!` that saves the full GPR file plus `mepc`, and the Rust dispatch function can return a *different* stack pointer to resume from. That's what turns "an interrupt handler" into "a real context switch." Bug #1: interrupts were enabled during early boot, before the first task was truly running. A tick landing in that window corrupted the first task's saved state before it ran a single instruction (fixed by deferring the global enable to the bootstrap `mret`). Bug #2, found only after an independent second-opinion review of the raw assembly plus QEMU's `-d int`/`-d cpu` trap logs: `mstatus.MPP` (the privilege mode `mret` returns to) was never set, defaulting to U-mode. QEMU faults `mret` itself when returning to U-mode with no PMP rules configured, and the fault handler at the time silently ignored non-interrupt causes, so the CPU looped on the same faulting `mret` forever, which looked *exactly* like "task A never gets scheduled," not like a crash. Fixed by asserting `MPP=M-mode` on every resume, and, the real lesson, by making unhandled traps `panic!()` loudly instead of silently doing nothing.

**Why priority/index don't need to be passed manually to `Semaphore`/`Channel`/`Sleep`.** They read "which task is this" from `executor::current_task()` (cooperative tier). The executor records it right before polling, right after registering a waiter would otherwise need it threaded through every call site.

**The ESP32-S3's dual-hart fairness fix exposed a genuine cross-hart data race, root-caused via live JTAG.** A periodic cross-hart IPI (added so both cores actually reconsider what they're running, closing a round-robin starvation bug) made a rare, real-hardware-only `InstrProhibited` crash far more likely. `rivet-arch-xtensa`'s per-task saved-register array (`CONTEXTS`, shared across harts since any task can run on either core) had its reads/writes as plain, unsynchronized 136-byte struct copies: a `Sync` justification that only ever reasoned about *same-hart* interrupt reentrancy, never actually true on a dual-core target. A live JTAG session (`openocd-esp32` + `xtensa-esp-elf-gdb` against the chip's native USB-JTAG) pinned the exact instruction: `retw.n` computing a garbage return address because the return-register field of a torn-read `Context` copy landed on exactly zero. Fixed by wrapping each individual copy in the kernel's existing cross-hart lock, deliberately *not* the whole save-decide-restore sequence, since a first attempt at that starved the other hart badly enough to stall the system outright. That was found the same way: by testing on real hardware, not assuming the wider fix was safer because it looked more thorough. Full account: `docs/realtime.md` §15.

## Build

```bash
# QEMU boards (RISC-V, Cortex-M3)
rustup target add riscv32imac-unknown-none-elf thumbv7m-none-eabi

# Real hardware: STM32F401RE (no extra toolchain, just the target + openocd)
rustup target add thumbv7em-none-eabi
sudo apt install openocd gdb-multiarch

# Real hardware: ESP32-S3 (needs Espressif's separate `esp` Rust toolchain
# fork, https://github.com/esp-rs/espup, not `rustup target add`)
```

`rivet` is a pure kernel with **no MMIO and no `#[cfg(target_arch)]` of its
own**; hardware is reached through the port contract (`rivet::port`) and
supplied by separate arch/board crates. See **[`docs/DOCUMENTATION.md`
§15](docs/DOCUMENTATION.md#15-project--crate-map)** for the full crate map
and **[`docs/DOCUMENTATION.md` §16](docs/DOCUMENTATION.md#16-building-running-and-testing)**
for the complete build/flash workflow on every board, including the two
real-hardware ones.

## Tests

```bash
cargo test -p rivet-rtos -- --test-threads=1
```

## QEMU

Both examples run the same three-part demo:

- **Part 1**, priority inheritance: `pi_low` locks a `PriorityMutex`, spawns `pi_medium` (never yields, higher base priority) and `pi_high` (blocks on the same mutex). `pi_low` finishes its critical section and releases despite `pi_medium` being ready the whole time, only possible because inheritance boosted it.
- **Part 2**, real preemption: two same-priority preemptive tasks, `A` and `B`, neither ever yielding, forced to interleave by the timer tick.
- **Part 3**, the cooperative tier: a `heartbeat` task (`Sleep`-based, toggles a real GPIO pin on Cortex-M), a `producer`/`consumer` pair over a `Channel`, and a `finisher` that waits on a `Semaphore` and exits.

```bash
# RISC-V (QEMU virt)
apt install qemu-system-misc   # if needed
./scripts/run-qemu.sh

# Cortex-M3 (QEMU lm3s6965evb)
apt install qemu-system-arm    # if needed
./scripts/run-cm3.sh
```

Expected output shape (both):
```
Rivet RTOS v0.1.0 <RISC-V|Cortex-M3> (preemptive + async demo)
Phase 0: priority inheritance (avoiding priority inversion):
[pi_low: acquiring mutex]
[pi_low: holds mutex, spawning medium+high]
[pi_high: trying to acquire mutex]
[pi_low: critical section done, releasing]
[pi_high: got mutex, priority inheritance worked]
Phase 1: two same-priority preemptive tasks (A, B), no yielding:
ABABABABABABABABABABABAB+-+-+-.+-+-
consumer: sum=15
SUCCESS
```
Exit code 0. The exact A/B interleaving pattern varies run to run (timer-tick timing); what matters is that it's mixed, not `AAAA...BBBB...`.

## Writing tasks

**Cooperative** (`async fn`, no arguments, shared state via `static`s):

```rust
use rivet::sync::{Channel, Semaphore};
use rivet::time::Sleep;

static CHAN: Channel<u32, 4> = Channel::new();
static DONE: Semaphore<1> = Semaphore::new(0);

#[rivet::task(priority = 1)]
async fn producer() {
    let (mut tx, _) = CHAN.split();
    for i in 1..=5u32 {
        Sleep::<30_000>::new().await; // 30ms
        tx.send(i).await;
    }
}

#[rivet::task(priority = 2)]
async fn consumer() {
    let (_, mut rx) = CHAN.split();
    let mut sum = 0;
    for _ in 0..5 {
        sum += rx.recv().await;
    }
    DONE.release();
}
```

**Preemptive** (own stack, real priority preemption, typed argument):

```rust
struct Config { threshold: u32 }
static CFG: Config = Config { threshold: 100 };

fn critical_task(cfg: &'static Config) -> ! {
    loop {
        // Runs with genuine priority preemption — no .await required
        // anywhere for a higher-priority task to interrupt this.
    }
}

// Inside a #[rivet::main] fn, before rivet::run():
rivet::spawn_ptask!(stack = 2048, priority = 10, entry = critical_task, arg = CFG);
```

A complete binary links one arch crate and one BSP crate, then declares its
entry point with `#[rivet::main]` (which calls `rivet::init()`: arch/board
bring-up, `#[rivet::task]` discovery, spawning the async executor as the
lowest-priority preemptive task, before your function body runs):

```rust
#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _; // or rivet_bsp_lm3s6965, or your own board
use rivet_rt as _;            // boot glue + default panic handler

#[rivet::main]
fn main() -> ! {
    rivet::spawn_ptask!(stack = 2048, priority = 10, entry = critical_task, arg = CFG);
    rivet::run() // starts the preemptive scheduler — never returns
}
```

## Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md). Short version: every claim in
this project (a bug is fixed, a number is a real bound) is backed by a test,
a QEMU golden-output diff, or a real-hardware trace, not just "should work."

## License

Licensed under the [MIT License](LICENSE).
