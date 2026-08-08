# Rivet RTOS

**The kernel contains no allocator, and never will.** Every resource is a
fixed pool sized at compile time (see `rivet/build.rs` / the `RIVET_*`
environment variables): task stacks live in one `.task_stacks` pool, the
task registry, timer queue, waker bitmaps, and semaphore waiter bitmaps are
static arrays. Worst-case memory is bounded and statically known; there is
no fragmentation and no allocator in the scheduling or interrupt path
(plan.md §4). 

A zero-allocation, dual-tier RTOS for ARM Cortex-M and RISC-V, written in Rust (`no_std`). A **preemptive tier** (real per-task stacks, real timer-driven context switches, priority-inheritance mutex) sits above a **cooperative tier** (`async fn` tasks, single shared stack) — the async executor runs as the lowest-priority preemptive task, so any real preemptive task immediately interrupts it.

## What actually works today

- **Real preemption, not just cooperative scheduling.** A preemptive task that never calls anything cooperative — no `.await`, no yield — still gets forcibly switched out by the timer tick. Verified on both targets: two same-priority tasks that just spin and increment a counter come out genuinely interleaved (`ABABABABAB...`), not `AAAA...BBBB...`. See [Design notes](#design-notes) for how each arch's context switch actually works — both had real, non-obvious correctness bugs that only a second pair of eyes and QEMU's own trap logs actually pinned down.
- **Priority-inheritance mutex** (`preempt::PriorityMutex`), verified against the actual scenario it exists for: a low-priority task holds it, a higher-priority task blocks waiting, and a *third*, medium-priority task that never yields is ready the whole time. Without inheritance the low-priority holder would starve forever (classic priority inversion) and the high-priority waiter would never unblock. With it, the holder's effective priority is boosted for as long as anyone waits, finishes its critical section despite the medium-priority task being ready, and releases promptly.
- **Parameterized preemptive tasks** — `fn(&'static Arg) -> !` with real, typed arguments (via `spawn_ptask!`), each on its own statically-allocated stack. No heap, no TAIT.
- **Real `async fn` tasks** via `#[rivet::task(priority = N)]` for the cooperative tier — genuine compiler-generated `Future` state machines, not hand-rolled state enums, and no nightly/TAIT required (see [Design notes](#design-notes)).
- **Typestate GPIO** (`rivet_bsp_lm3s6965::gpio`) for the Cortex-M board — pin direction is tracked in the type, not a runtime flag; calling `.set_high()` on a pin still typed as `Input` is a compile error. Real register writes (GPIODIR/GPIODEN/GPIODATA on the LM3S6965), verified fault-free in QEMU.
- **Async sync primitives** for the cooperative tier — `Semaphore::acquire().await`, `Channel::send().await`/`recv().await`, lock-free, ISR-safe on the signaling side.
- **Tickless `Sleep`** — registers a deadline with a timer queue instead of busy-polling; the executor genuinely reaches `WFI` between events.
- **Two validated targets**: RISC-V (QEMU `virt`) and ARM Cortex-M3 (QEMU `lm3s6965evb`) — both run priority inheritance, real preemption, and the async tier back to back, and both exit QEMU cleanly with code 0.
- **33 host-side tests** (`cargo test -p rivet`), covering the scheduler, priority inheritance, waker bitmap, sync primitives, and an end-to-end async producer/consumer test driven through the real polling machinery.

## What's not here yet

- No MPU sandboxing, no RTT/defmt logging, no compile-time timing bounds, no multi-core support, no GPIO driver for the RISC-V target. Real gaps against the original spec.
- ESP32-C3 real-hardware validation — the RISC-V arch code should port over, but hasn't been flashed/tested on real silicon.
- Preemptive tasks are `fn(&'static Arg) -> !` — no return value, no join/await-a-preemptive-task. That's a deliberate simplicity tradeoff for v0.1, not an oversight, but it means a preemptive task can't cleanly signal "I'm done" to anything except through its own side effects (a semaphore, a shared flag).

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

Both tiers share one priority space conceptually, but only the preemptive tier's priorities are compared by the scheduler directly — the cooperative tier is just "whatever the async executor, sitting at preemptive-priority 0, decides to poll next" using its own separate priority-bitmap system among `#[rivet::task]`s.

A preemptive task's full context — every general-purpose register plus (arch-specific) the program counter — is saved on every switch, which is what makes it interruptible *anywhere*, not just at yield points. A cooperative task's "context" is just its `Future`'s state, live only across `.await`.

## Design notes

**Why real `async fn` tasks work without nightly Rust.** The compiler-generated `Future` type of an `async fn` is unnameable on stable Rust — you can't write `static TASK: TaskCell<F>` because `F` has no name (that needs `type_alias_impl_trait`, which is why Embassy requires nightly for its task macro). Rivet sidesteps this: `TaskCell<const SIZE: usize>` is generic over a **byte count**, not the future type — `SIZE` is just a `usize`, always nameable. The proc macro emits a thin, non-generic wrapper that calls a **generic** `TaskCell::poll::<F>()` method, monomorphized separately per task by the compiler, which reads/writes the future's bytes into the cell's buffer. `F` never has to be named. Size/alignment are checked at runtime on first poll (`assert!`, not silent UB).

**The RISC-V context switch is one unified trap path, and getting it right took two real, separately-diagnosed bugs.** RISC-V hardware auto-saves *nothing* on trap entry (unlike Cortex-M's automatic r0-r3/r12/lr/pc/xPSR stacking), so `rivet_trap_entry` is hand-written `global_asm!` that saves the full GPR file plus `mepc`, and the Rust dispatch function can return a *different* stack pointer to resume from — that's what turns "an interrupt handler" into "a real context switch." Bug #1: interrupts were enabled during early boot, before the first task was truly running — a tick landing in that window corrupted the first task's saved state before it ran a single instruction (fixed by deferring the global enable to the bootstrap `mret`). Bug #2, found only after an independent second-opinion review of the raw assembly plus QEMU's `-d int`/`-d cpu` trap logs: `mstatus.MPP` (the privilege mode `mret` returns to) was never set, defaulting to U-mode; QEMU faults `mret` itself when returning to U-mode with no PMP rules configured, and the fault handler at the time silently ignored non-interrupt causes — so the CPU looped on the same faulting `mret` forever, which looked *exactly* like "task A never gets scheduled," not like a crash. Fixed by asserting `MPP=M-mode` on every resume, and — the real lesson — by making unhandled traps `panic!()` loudly instead of silently doing nothing.

**Why priority/index don't need to be passed manually to `Semaphore`/`Channel`/`Sleep`.** They read "which task is this" from `executor::current_task()` (cooperative tier) — the executor records it right before polling, right after registering a waiter would otherwise need it threaded through every call site.

## Build

```bash
rustup target add riscv32imac-unknown-none-elf thumbv7m-none-eabi
```

Workspace layout — `rivet` is a pure kernel with **no MMIO and no
`#[cfg(target_arch)]` of its own**; hardware is reached through the port
contract (`rivet::port`) and supplied by separate arch/board crates. See
`plan.md` for the full design and `docs/porting.md` for how to bring
Rivet up on a new board:

- `rivet/` — the kernel: scheduler, TCB, executor, timers, sync, fault
  policy. Builds on any host target with zero board/arch crates present
  (`cargo build -p rivet` never touches MMIO).
  - `preempt/` — the preemptive tier: TCB, scheduler, `PriorityMutex`
  - `executor.rs`, `task.rs`, `waker.rs`, `sync/`, `time.rs` — the cooperative tier
  - `port/` — the `extern "Rust"` contract arch/board crates implement
- `rivet-arch-riscv/`, `rivet-arch-cortex-m/` — CPU ports: context switch,
  trap/PendSV entry, MPU/PMP programming. No board knowledge.
- `rivet-bsp-qemu-virt/`, `rivet-bsp-lm3s6965/` — board support: memory
  map, clocks, console, tick source, exit/reset, watchdog, linker script.
  `rivet-bsp-lm3s6965/src/gpio.rs` is the typestate GPIO driver.
- `rivet-bsp-support/` — shared BSP helpers (software-watchdog fallback,
  NS16550 UART) so a new board isn't reimplementing them from scratch.
- `rivet-rt/` — boot glue (`_start`/`Reset`, bss/data init, default panic
  handler) and the `#[rivet::main]` entry-point macro.
- `rivet-macros/` — `#[rivet::task]` and `#[rivet::main]` proc macros.
- `examples/qemu-riscv/`, `examples/qemu-cm3/` — runnable demos, each just
  `rivet` + one arch crate + one BSP crate + `rivet-rt` + application logic.

## Tests

```bash
cargo test -p rivet -- --test-threads=1
```

## QEMU

Both examples run the same three-phase demo:

- **Phase 0** — priority inheritance: `pi_low` locks a `PriorityMutex`, spawns `pi_medium` (never yields, higher base priority) and `pi_high` (blocks on the same mutex). `pi_low` finishes its critical section and releases despite `pi_medium` being ready the whole time — only possible because inheritance boosted it.
- **Phase 1** — real preemption: two same-priority preemptive tasks, `A` and `B`, neither ever yielding, forced to interleave by the timer tick.
- **Phase 2** — the cooperative tier: a `heartbeat` task (`Sleep`-based, toggles a real GPIO pin on Cortex-M), a `producer`/`consumer` pair over a `Channel`, and a `finisher` that waits on a `Semaphore` and exits.

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
[pi_high: got mutex — priority inheritance worked]
Phase 1: two same-priority preemptive tasks (A, B), no yielding:
ABABABABABABABABABABABAB+-+-+-.+-+-
consumer: sum=15
SUCCESS
```
Exit code 0. The exact A/B interleaving pattern varies run to run (timer-tick timing) — what matters is that it's mixed, not `AAAA...BBBB...`.

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
entry point with `#[rivet::main]` (which calls `rivet::init()` — arch/board
bring-up, `#[rivet::task]` discovery, spawning the async executor as the
lowest-priority preemptive task — before your function body runs):

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

## License

Licensed under the [MIT License](LICENSE).
