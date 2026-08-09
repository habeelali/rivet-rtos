# Rivet RTOS — Complete Documentation

> A zero-allocation, dual-tier real-time operating system for embedded
> targets, written in Rust. This document covers everything: what it is,
> what it does, how it's built, how it works internally, and how to use it.

---

## Table of contents

1. [What Rivet is](#1-what-rivet-is)
2. [Key features](#2-key-features)
3. [Supported hardware](#3-supported-hardware)
4. [Architecture: the layering model](#4-architecture-the-layering-model)
5. [The kernel: dual-tier scheduling](#5-the-kernel-dual-tier-scheduling)
6. [Memory safety and fault isolation](#6-memory-safety-and-fault-isolation)
7. [Synchronization primitives](#7-synchronization-primitives)
8. [Time, sleeping, and timers](#8-time-sleeping-and-timers)
9. [Task lifecycle: spawn, join, despawn, pause/resume](#9-task-lifecycle-spawn-join-despawn-pauseresume)
10. [Logging and diagnostics](#10-logging-and-diagnostics)
11. [The watchdog](#11-the-watchdog)
12. [The port contract (how a board plugs in)](#12-the-port-contract-how-a-board-plugs-in)
13. [Configuration](#13-configuration)
14. [Writing an application](#14-writing-an-application)
15. [Project / crate map](#15-project--crate-map)
16. [Building, running, and testing](#16-building-running-and-testing)
17. [Verification and quality bar](#17-verification-and-quality-bar)
18. [Known limitations and honest gaps](#18-known-limitations-and-honest-gaps)
19. [Roadmap](#19-roadmap)

---

## 1. What Rivet is

Rivet is a **real-time operating system (RTOS) kernel** for microcontrollers,
written in `no_std` Rust. It gives embedded applications two ways to run
concurrent work under one priority scheduler:

- **Preemptive tasks** — ordinary functions, each with its own stack, that
  the timer tick can forcibly switch out at *any* instruction, not just at
  a yield point. This is what "real-time" means in practice: a
  higher-priority task interrupts a lower-priority one immediately, even if
  the lower one never calls anything cooperative.
- **Cooperative tasks** — `async fn`s compiled to real Rust `Future` state
  machines, polled on a single shared stack. Zero per-task stack cost,
  ideal for I/O-bound or event-driven logic (waiting on a semaphore, a
  channel, a timer).

Both tiers share one 32-level priority space. The cooperative tier runs as
an ordinary preemptive task at the lowest priority, so any real preemptive
task immediately preempts it — the async executor only gets the CPU when
nothing more urgent is ready.

The kernel is split from the hardware it runs on: `rivet` itself contains
**no memory-mapped I/O and no architecture-specific code at all**. Everything
that touches real silicon — the CPU's context-switch mechanism, a board's
clock rate, its UART, its watchdog — is supplied by separate crates through
a documented symbol contract. This is what makes "port Rivet to your own
board" mean "write a new small crate," not "fork the kernel."

**Guiding constraint: the kernel contains no heap allocator, and never
will.** Every resource — task stacks, the task registry, the timer queue,
waker bitmaps, semaphore waiter lists — is a fixed-size array or pool, sized
at compile time via environment variables. Worst-case memory use is bounded
and knowable before you flash anything; there is no fragmentation and no
allocator in the scheduling or interrupt path.

---

## 2. Key features

| Feature | What it means in practice |
|---|---|
| **Real preemption** | A tight-loop preemptive task with no `.await` and no yield still gets forcibly context-switched by the timer tick. Verified on every supported board: two same-priority spinning tasks come out genuinely interleaved, not one-then-the-other. |
| **Priority inheritance** | `PriorityMutex` boosts a lock holder's effective priority to match the highest-priority task waiting on it, closing the classic priority-inversion hole. Nested-lock inheritance is tracked correctly (releasing one held mutex doesn't clobber the boost held for another). |
| **Real `async fn` tasks, stable Rust** | No nightly features, no `type_alias_impl_trait`. A small trick (generic over a *byte count*, not the unnameable `Future` type) lets `#[rivet::task]` store real compiler-generated state machines in a `static`. |
| **Task lifecycle** | Preemptive tasks can return a typed value, be `join()`ed (blocking or `.await`), `despawn()`ed and have their slot/stack recycled, and `pause()`/`resume()`d. Generation-counted handles catch stale references after a slot is reused (ABA safety). |
| **Fault isolation** | A wild pointer or stack overflow produces a diagnosed fault (task id, fault kind, address, PC), not silent corruption — backed by a real MPU (Cortex-M) or PMP (RISC-V) guard. Two policies: `Panic` (dump and reset) or `IsolateTask` (poison the faulting task's held mutexes, run a user hook, keep the system running). |
| **Zero-allocation everywhere** | Every kernel data structure is a fixed-size static array. Capacities are compile-time environment variables, not runtime parameters. |
| **Deferred, ISR-safe logging** | `rivet::log!()` pushes a `{level, task, timestamp, message}` frame into a lock-free ring buffer in O(1) time — safe to call from interrupt context — and a drain task formats it to the console later, off the hot path. |
| **Kernel introspection** | `rivet::report()` dumps every live task's priority, state, and stack high-water mark, plus registry-wide slot usage, in one call. |
| **Software watchdog framework** | Both a hardware-watchdog path (for boards that have one) and a task-level "checkin" watchdog that catches an individual hung task, independent of whether the board has real WDT hardware. |
| **Portable by design** | The kernel is proven to run unmodified across three genuinely different boards on two architectures (RISC-V and ARM Cortex-M), through a symbol-based port contract — see [§4](#4-architecture-the-layering-model) and [§12](#12-the-port-contract-how-a-board-plugs-in). |
| **Deep verification discipline** | Host-side unit/integration tests, `loom` permutation testing of the lock-free core, `proptest` model-based scheduler/timer testing, `cargo-fuzz` targets, and a QEMU harness that asserts exit codes, ordered golden output, *and* that the emulator's own trap log is clean unless a fault is explicitly expected. |

---

## 3. Supported hardware

Rivet runs today on three QEMU-emulated boards, deliberately chosen to be
different from each other so the architecture/board split is proven, not
just assumed:

| Board | Architecture | Notes |
|---|---|---|
| **QEMU `virt` (RV32)** | RISC-V, RV32IMAC | CLINT-based tick/IPI, NS16550 UART, `riscv.sifive.test` exit/reset device, software watchdog. |
| **`lm3s6965evb`** | ARM Cortex-M3 | PL011 UART, real `luminary-watchdog` hardware block (genuine hardware-independent watchdog, verified in QEMU), typestate GPIO driver, SysTick/PendSV context switch. |
| **`mps2-an385`** | ARM Cortex-M3 | A *different* memory map and peripheral set from lm3s6965 — CMSDK APB UART (different register layout than PL011), CMSDK/SP805-compatible watchdog. Added purely as a new board-support crate, without touching the kernel or the Cortex-M architecture port, as proof the boundary holds. |

A fourth target — ESP32-C3 (RV32IMC) — was evaluated and found to be a real,
architectural blocker rather than a simple porting task: it lacks the RISC-V
atomic (`A`) extension that the kernel's lock-free scheduler relies on
throughout. Supporting it would require switching the kernel to
software-emulated atomics, a cross-cutting change, not a new board. This is
documented, not silently ignored — see [§18](#18-known-limitations-and-honest-gaps).

Real hardware has not been tested; everything above runs in QEMU. See
[§18](#18-known-limitations-and-honest-gaps) for exactly what that does and
doesn't validate.

---

## 4. Architecture: the layering model

Rivet is organized into four strict layers. Dependencies point only
downward — the kernel knows nothing about any layer above it, and nothing
below it knows about a specific board:

```
  application code            uses rivet + one BSP crate; no MMIO, no boot code
        |
  rivet-rt                    boot glue: _start/Reset, bss/data init, RISC-V
        |                     multi-hart park guard, default panic handler,
        |                     the #[rivet::main] entry-point macro
        |
  rivet-bsp-<board>           board support: memory map, clock rates, console
        |                     UART, tick source wiring, exit/reset, watchdog,
        |                     GPIO/peripheral drivers, the linker script
        |
  rivet-arch-<isa>            CPU port: context switch, trap/exception entry,
        |                     critical-section primitive, MPU/PMP programming
        |
  rivet                       the kernel: scheduler, task control blocks,
                               executor, timers, sync primitives, fault
                               policy. Zero MMIO. Zero #[cfg(target_arch)].
```

### Why this split, concretely

Before this layering existed, board-specific memory-mapped I/O (CLINT
addresses, UART registers, watchdog registers, clock rates) was scattered
directly inside the kernel's architecture files, gated by
`#[cfg(target_arch = "riscv32")]` / `#[cfg(target_arch = "arm")]`. That meant:

- The kernel could never be built for a board it didn't already know about.
- Two boards sharing the same CPU architecture (e.g. two different
  Cortex-M3 boards) couldn't coexist in one workspace — the arch selection
  was keyed on the *target triple*, not the board.
- Adding hardware support meant editing the kernel itself.

Today, `rivet` is checkably board-free: `cargo build -p rivet` compiles for
any target with zero board or architecture crate present (an `rlib` doesn't
need its `extern "Rust"` symbols resolved until final link time). Building
an actual runnable binary without a real board crate fails at the *link*
step with an error naming the exact missing symbol — not a silent wrong
answer, not a compile-time type error, a precise "you forgot to implement
`__rivet_board_console_write`."

### The binding mechanism: symbols, not generics or trait objects

The layers are connected by `extern "Rust"` function declarations rather
than a `trait Board` object or generic parameters threaded through the
kernel. This was a deliberate choice:

- **No dynamic dispatch cost** — the linker resolves the concrete
  implementation directly; there's no vtable, no runtime board lookup.
- **No generics explosion** — the kernel's statics (the task registry, the
  waker bitmaps, the timer queue) don't need to become generic over a
  `Board` type parameter, which would ripple through the entire kernel.
- **A missing implementation is a link error, not type-system gymnastics.**
- This is the same pattern the widely-used `critical-section` crate uses
  for its own provider mechanism — not a novel or risky approach.

Concretely, the kernel declares two groups of symbols in `rivet::port`:

- **Group A ("arch")** — `rivet::port::arch`: context switch, trap entry,
  interrupt masking, MPU/PMP guard registration. Implemented by a
  `rivet-arch-*` crate.
- **Group B ("board")** — `rivet::port::board`: clock/board bring-up, the
  monotonic clock, the tick source, the console, exit/reset, the watchdog.
  Implemented by a `rivet-bsp-*` crate.

See [§12](#12-the-port-contract-how-a-board-plugs-in) for the exact symbol
list and what each one is responsible for.

---

## 5. The kernel: dual-tier scheduling

### The preemptive tier

Each preemptive task is a plain function, `fn(&'static Arg) -> T`, given its
own statically-allocated stack via `spawn_ptask!`. The scheduler tracks, per
task: a base priority, an *effective* priority (which can be temporarily
boosted — see priority inheritance below), a run state (`Ready` / `Running`
/ `Blocked`), and its stack bounds.

On every timer tick (and on any voluntary yield or a mutex/semaphore
release), the architecture-specific trap handler saves the interrupted
task's full register file to its own stack, asks the scheduler
`preempt::on_tick()` "who should run now?", and resumes whatever it
returns — which may be a completely different task. This is what makes a
task interruptible at *any* instruction, not just at an explicit yield
point: the register save/restore is total, driven by hardware, not by
cooperative checkpoints in the task's own code.

Scheduling is strict priority-based: the highest-priority `Ready` task
always runs next. Tasks at the same priority get round-robin fairness.

### The cooperative tier

`#[rivet::task(priority = N)]` declares an `async fn` that the compiler
turns into a state machine. These are stored in fixed-size cells (sized by
a `stack = N` byte-count attribute, not by naming the unnameable `Future`
type — see the note in the macro's own documentation for the trick that
makes this work on stable Rust) and polled by a single executor.

The executor itself runs as one ordinary preemptive task, always at
priority 0 (the lowest). This is the key design point: the cooperative tier
is not a special second scheduler running in parallel — it's *one task* in
the same priority space as everything else. Any real preemptive task at any
higher priority immediately preempts the whole async tier. The async tier
exists to make good use of CPU time that would otherwise be idle, not to
compete with real-time work.

When every task (preemptive and cooperative) is blocked or parked, the
executor calls into the architecture's `idle()` hook (`wfi` on both
supported architectures) — genuine tickless idle, not a busy-poll loop.

### Priority inheritance

`preempt::PriorityMutex<T>` implements priority inheritance: when a
higher-priority task blocks trying to acquire a mutex a lower-priority task
holds, the holder's *effective* priority is boosted to match, for as long
as the waiter is blocked. This closes the classic priority-inversion
scenario — a medium-priority task that never yields can no longer starve a
low-priority lock holder indefinitely while a high-priority task waits.

Nested locking is handled correctly: releasing one held mutex recomputes
the effective priority from *all remaining* held mutexes, not by blindly
resetting to the base priority — so releasing the inner of two nested locks
doesn't accidentally drop the boost still owed for the outer one.

---

## 6. Memory safety and fault isolation

Rivet has no MMU and no virtual memory — this is bare-metal microcontroller
territory. What it does have is real, verified use of each architecture's
memory-protection hardware:

- **ARM Cortex-M (MPU):** a two-region design gives full mutual stack
  isolation with just two of the eight-plus available regions. One region
  denies the *entire* task-stack pool by default; a second region,
  reprogrammed on every context switch, re-enables exactly the currently
  running task's own stack within that pool. Any other task's stack —
  including an overflow past the bottom of your own — is denied.
- **RISC-V (PMP):** PMP entries that affect machine mode must be locked at
  boot and are then immutable until reset, so RISC-V isolation is
  necessarily boot-time-static rather than reprogrammed per switch. Each
  task's stack gets a small locked guard band at its low end (a limited
  hardware resource — 16 total entries, budgeted across tasks) that faults
  on overflow; tasks beyond that budget fall back to software stack
  watermarking.

### Fault policy

When either mechanism traps (a MemManage fault on Cortex-M, an access fault
on RISC-V), the architecture-specific trap handler builds a `FaultInfo`
(task id, fault kind, faulting address, program counter) and hands it to
`rivet::fault`, which dispatches on the configured policy:

- **`Panic`** (default) — dump the diagnosis plus every task's current
  stack watermark to the console, then reset or halt with a distinguishable
  exit code.
- **`IsolateTask`** — mark the faulting task `Faulted`, **poison every
  `PriorityMutex` it currently holds** (so a task later trying to lock one
  of them gets a clean `Err(Poisoned)` instead of deadlocking forever),
  invoke a user-registered hook with the fault details, and switch to the
  next ready task. The rest of the system keeps running. This reuses the
  exact same "return an arbitrary stack pointer to resume" primitive that
  ordinary preemption already relies on — there's no separate isolation
  mechanism to trust.

### Stack watermarking

Every task stack is filled with a `0xAA` pattern at spawn time. Because
that pattern is never legitimately written by running code, scanning down
from the top for the first non-`0xAA` byte gives a cheap, universal
high-water-mark measurement (`rivet::preempt::stack_usage()`), used both by
the fault dump and by `rivet::report()` — and it works identically on both
architectures, independent of whether the hardware guard caught the
overflow or not.

---

## 7. Synchronization primitives

All of these live under `rivet::sync` and `rivet::preempt`, and are
`'static`-friendly (no heap, declared as plain `static`s):

| Primitive | Tier | What it's for |
|---|---|---|
| `preempt::PriorityMutex<T>` | preemptive | Mutual exclusion with priority inheritance (see [§5](#5-the-kernel-dual-tier-scheduling)). `lock()` (blocking), `try_lock()`, `lock_timeout(Duration)`. Poisoned after `IsolateTask` isolates a holder. |
| `sync::Semaphore<MAX>` | cooperative | Counting semaphore. `acquire().await`, `try_acquire()`, `release()`. Backed by a per-priority waiter bitmap (not a single slot), so multiple concurrent waiters are never silently dropped. |
| `sync::Channel<T, N>` | cooperative | Lock-free single-producer/single-consumer ring buffer. `split()` is one-shot — it returns `Some((Sender, Receiver))` exactly once, `None` on any later call, which prevents the SPSC contract from being silently violated by two producers. `send().await` / `try_send()`, `recv().await` / `try_recv()`. |
| `sync::Once<T>` | either | One-time initialization cell for handing a `Sender`/`Receiver` half (or anything else) from boot code to a task that needs it. |
| `sync::atomic` | either | A thin shim over `core::sync::atomic`, swapped for `loom`'s atomics under the `loom` feature — this is what lets the lock-free core be exhaustively permutation-tested (see [§17](#17-verification-and-quality-bar)). |

---

## 8. Time, sleeping, and timers

- **`rivet::time::Duration`** and **`rivet::time::Sleep<const MICROS: u64>`**
  — a const-generic, zero-allocation sleep future. `Sleep::<100_000>::new().await`
  sleeps 100ms. Dropping a `Sleep` before it fires correctly releases its
  timer-queue slot (no leak, no spurious wake against whatever the task is
  doing later).
- **`rivet::timer`** — the fixed-size deadline queue backing `Sleep`. The
  architecture's tick handler calls `poll_timers()` on every tick, waking
  any task whose deadline has passed. This is what makes the kernel's idle
  wait a real `wfi`, not a spin loop: between ticks, nothing needs polling.
- Time itself comes from the board's monotonic clock
  (`__rivet_board_now_us`) — on RISC-V boards with a CLINT, this reads the
  hardware `mtime` register directly (tear-free even on a 32-bit
  architecture, via a hi/lo/hi-recheck read protocol), so it can never
  drift from the actual hardware clock.

---

## 9. Task lifecycle: spawn, join, despawn, pause/resume

```rust
// Spawn: own stack, typed argument, returns a value.
let handle = rivet::spawn_ptask!(stack = 2048, priority = 10, entry = my_task, arg = CFG)?;

fn my_task(cfg: &'static Config) -> u32 {
    // ... real work ...
    42 // the entry returning normally is a real return value, not `-> !`
}

// From another task:
match handle.join::<u32>() {
    Ok(42) => { /* ... */ }
    Err(JoinError::Faulted) => { /* the task died under IsolateTask, no result */ }
    Err(JoinError::Stale) => { /* handle outlived a slot recycle */ }
    Err(_) => { /* SelfJoin / AlreadyJoined */ }
}
handle.despawn();       // release the slot + stack back to the pool
handle.pause();         // stop scheduling it (progress halts)
handle.resume();        // resume scheduling it
handle.request_stop();  // cooperative cancellation — the task must poll should_stop()
```

Every `TaskHandle` carries a **generation counter** alongside its slot id.
When a slot is recycled for a new task, the generation increments, so a
handle to the *old* occupant is detected as stale (`is_valid()` returns
`false`, `join()` returns `Err(JoinError::Stale)`) rather than silently
operating on the wrong task — this is the standard fix for the ABA problem
that comes with reusing fixed-size slots.

`-> !` tasks (the common embedded pattern — a task that never returns) still
work exactly as before; a non-`!` return type is the addition, not a
replacement.

---

## 10. Logging and diagnostics

### `rivet::log!` — ISR-safe deferred logging

```rust
use rivet::log::Level;
rivet::log!(Level::Info, "worker finished");
```

This does **no formatting and no blocking** at the call site — it pushes a
`{level, task_id, timestamp, message}` frame into a lock-free ring buffer
in O(1) time, which is why it's safe to call from interrupt context, not
just task context. A drain task (either your own loop calling
`rivet::log::drain_one()`, or the built-in `rivet::log::drain_forever()`)
formats and writes frames to the console later, off whatever hot path
logged them.

Logging is inherently **multi-producer** — any task or any ISR might log
concurrently — which the underlying SPSC ring buffer doesn't support raw;
every producer path is routed through a critical section to serialize
concurrent callers into one logical producer. A full ring drops the
oldest-pending frame and counts it (`rivet::log::dropped_frames()`) rather
than blocking whatever tried to log.

The current implementation stores each frame's message as a plain
`&'static str` pointer, not an interned index into a linker-section format
table — which means no `printf`-style argument interpolation yet. See
[§18](#18-known-limitations-and-honest-gaps) for what a fuller version
would add.

### `rivet::report()` — one-call kernel state dump

```rust
rivet::report();
```

```text
=== rivet::report() ===
task 0 prio=0 state=running stack=248/4096
task 1 prio=2 state=blocked stack=176/512
task 2 prio=2 state=blocked stack=176/512
ptask slots: 3/16
timer slots: 0/16
log: 0 dropped frame(s)
=== end report ===
```

Prints every live task's id, base/effective priority, current state, and
stack high-water mark, plus registry-wide slot usage and dropped log-frame
count. Reading *another* task's stack safely on Cortex-M required reusing
the same MPU scratch-window primitive the stack pool itself uses internally
(a pool-allocated task's stack is outside whichever task is *currently*
running's MPU window) — handled transparently, you don't need to think
about it as a caller.

Deliberately **not** included: per-task execution-time accounting,
deadline-miss counts, or budget-overrun tracking — those need timing
infrastructure (a cycle counter, period/deadline bookkeeping) that doesn't
exist yet. `report()` growing those columns later is an additive change to
this same function, not a redesign.

---

## 11. The watchdog

`rivet::watchdog` gives you one API regardless of whether the underlying
board has real watchdog hardware:

```rust
rivet::watchdog::init(Duration::from_millis(500));
// ... periodically, from whichever task is responsible for liveness ...
rivet::watchdog::feed();
```

- On boards with a real hardware watchdog (e.g. `lm3s6965evb`'s
  `luminary-watchdog`), this arms genuine, CPU-independent hardware — it
  keeps counting even if the CPU wedges with interrupts disabled, and
  resets on expiry without any software intervention.
- On boards without one, a software fallback checks a deadline against the
  clock on every tick. **This is explicitly not CPU-independent** — a tick-
  driven check cannot catch a hang that stops the ticks themselves (e.g.
  interrupts disabled in a spin loop). Every board's own documentation says
  which case it is; this isn't glossed over.

A separate, purely software **task-level checkin watchdog**
(`rivet::watchdog::checkin()` / `enable_checkins()`) catches one specific
task going silent longer than its configured timeout, independent of the
board-level watchdog path.

---

## 12. The port contract (how a board plugs in)

This is the exact, current symbol contract. Full guidance on implementing
it for a new board — including how to probe an unfamiliar board's memory
map safely, the linker-script requirements, and a worked example — lives in
[`docs/porting.md`](porting.md); this section is the reference list.

### Group A — `rivet::port::arch` (CPU port)

| Symbol | Responsibility |
|---|---|
| `__rivet_arch_init` | One-time arch bring-up: install the trap vector, arm boot-time-static memory guards. |
| `__rivet_arch_idle` | Enter a low-power wait for the next interrupt (`wfi`). |
| `__rivet_arch_request_reschedule` | Trigger the same trap/exception path tick-driven preemption uses — the single reschedule entry point, voluntary or not. |
| `__rivet_arch_irq_save` / `__rivet_arch_irq_restore` | Interrupt masking for critical sections, composable under nesting. |
| `__rivet_arch_init_task_stack` | Build a new task's initial stack frame so the first switch into it starts at `entry(arg)`. |
| `__rivet_arch_start_first_task` | Transfer control to the first task. Never returns. |
| `__rivet_arch_on_switch_to` | Called on every context switch with the new task's stack range (Cortex-M reprograms its MPU region here; a no-op on RISC-V, whose guards are boot-time-static). |
| `__rivet_arch_guard_register` | Register a locked overflow guard for one task-stack allocation (RISC-V PMP; a no-op on arches — Cortex-M — that don't need per-task entries). |
| `__rivet_arch_scratch_open` / `__rivet_arch_scratch_close` | Temporarily grant kernel access to a stack range inside an otherwise memory-guard-denied pool. |
| `__rivet_arch_min_task_stack` | Minimum viable task stack size for this architecture. |

Provided today by `rivet-arch-riscv` and `rivet-arch-cortex-m`.

### Group B — `rivet::port::board` (board port)

| Symbol | Responsibility |
|---|---|
| `__rivet_board_init` | One-time board bring-up: clocks/PLL, console hardware init. |
| `__rivet_board_now_us` | Monotonic, tear-free time since boot, in microseconds. |
| `__rivet_board_tick_start` | Start the periodic tick at the kernel's configured rate. |
| `__rivet_board_console_write` | Write raw bytes to the debug console. |
| `__rivet_board_reset` | Trigger a system reset. Never returns. |
| `__rivet_board_exit` | Terminate with a status code (0 = success) — under QEMU, whatever exit device/semihosting path the board has. |
| `__rivet_board_wdt_init` / `__rivet_board_wdt_feed` | Arm/kick the watchdog (real hardware or software fallback). |
| `__rivet_board_wdt_check` | Called every tick; a no-op for real hardware watchdogs, the actual deadline check for software ones. |

Provided today by `rivet-bsp-qemu-virt`, `rivet-bsp-lm3s6965`, and
`rivet-bsp-mps2-an385`.

### The "Group C" case: optional stock drivers

Some mechanisms are genuinely universal within an ISA family but need a
board-supplied parameter to work — the clearest example is RISC-V's CLINT
(near-universal base address, but the clock rate driving `mtime` varies per
platform). Rather than force every board through the same mechanism, the
arch crate ships the mechanism as a feature-gated optional module (e.g.
`rivet-arch-riscv`'s `clint` feature); a board with a CLINT enables it and
supplies two numbers, a board without one (a future ESP32-C3-style target)
leaves it off and supplies its own tick/reschedule symbols directly. The
symbol-contract design doesn't care which crate defines a given symbol,
only that exactly one does — this flexibility is load-bearing, not
incidental.

---

## 13. Configuration

Every kernel capacity is a compile-time constant, generated from
environment variables by `rivet/build.rs` into `$OUT_DIR/config.rs`:

| Variable | Default | Max | Governs |
|---|---|---|---|
| `RIVET_MAX_PTASKS` | 16 | 32 | Preemptive task registry slots. |
| `RIVET_MAX_TIMERS` | 16 | 64 | Outstanding `Sleep` deadlines. |
| `RIVET_MAX_COOP_TASKS` | 16 | 32 | `#[rivet::task]` slots per priority (hard-capped by a `u32` bitmap representation). |
| `RIVET_PRIORITIES` | 32 | 32 | Number of priority levels. |
| `RIVET_TICK_HZ` | 1000 | 1,000,000 | The scheduler tick rate every board's `tick_start` programs to. |
| `RIVET_MAX_HELD_MUTEXES` | 4 | 16 | Nested `PriorityMutex` locks tracked per task (for correct inheritance recomputation on unlock). |

Out-of-range values are a `compile_error!`, not a silent clamp. There is
deliberately **no `alloc` feature** — see [§1](#1-what-rivet-is)'s guiding
constraint.

---

## 14. Writing an application

A complete binary links exactly one architecture crate and one board-support
crate (both transitively, via the board crate depending on the right arch
crate), plus `rivet-rt` for boot glue, and declares its entry point with
`#[rivet::main]`:

```rust
#![no_std]
#![no_main]

use rivet_bsp_qemu_virt as _; // or your own board's BSP crate
use rivet_rt as _;            // boot glue + default panic handler

use rivet::sync::{Channel, Semaphore};
use rivet::time::Sleep;

static CHAN: Channel<u32, 4> = Channel::new();
static DONE: Semaphore<1> = Semaphore::new(0);

// Channel::split() is genuinely one-shot for the pair (it returns `None`
// on any call after the first) — this is what stops the SPSC contract
// from being silently violated by two producers. Split it exactly once,
// from main() below, and hand each half to its task through a Once cell.
static TX: rivet::sync::Once<rivet::sync::Sender<'static, u32, 4>> = rivet::sync::Once::new();
static RX: rivet::sync::Once<rivet::sync::Receiver<'static, u32, 4>> = rivet::sync::Once::new();

// Cooperative: fine for I/O-bound logic.
#[rivet::task(priority = 1)]
async fn producer() {
    let tx = TX.get().expect("set in main() before tasks run");
    for i in 1..=5u32 {
        Sleep::<30_000>::new().await; // 30ms
        tx.send(i).await;
    }
}

#[rivet::task(priority = 2)]
async fn consumer() {
    let rx = RX.get().expect("set in main() before tasks run");
    let mut sum = 0;
    for _ in 0..5 {
        sum += rx.recv().await;
    }
    DONE.release();
}

// Preemptive: genuinely can't be starved by a lower/equal-priority task
// that never yields.
struct Config { threshold: u32 }
static CFG: Config = Config { threshold: 100 };

fn critical_task(cfg: &'static Config) -> ! {
    loop {
        // real work, no .await required anywhere for this to be preempted
    }
}

#[rivet::main]
fn main() -> ! {
    let (tx, rx) = CHAN.split().expect("split exactly once, here, at boot");
    let _ = TX.set(tx);
    let _ = RX.set(rx);

    rivet::spawn_ptask!(stack = 2048, priority = 10, entry = critical_task, arg = CFG);
    rivet::run() // starts the preemptive scheduler — never returns
}
```

`#[rivet::main]` expands to a `#[no_mangle] extern "C" fn rivet_main()`
that `rivet-rt`'s boot code calls after bss/data init, calling
`rivet::init()` automatically before your function body runs —
`rivet::init()` does arch/board bring-up, `#[rivet::task]` discovery, and
spawns the async executor as the lowest-priority preemptive task.

---

## 15. Project / crate map

```
rivet/                 the kernel — scheduler, TCB, executor, timers, sync,
                        fault policy, log, report. Builds on any host target
                        with zero board/arch crates present.
  src/preempt/          the preemptive tier: TCB, scheduler, PriorityMutex,
                        lifecycle (join/despawn/pause/resume), stack pool
  src/executor.rs, task.rs, waker.rs, sync/, time.rs, timer.rs
                        the cooperative tier and its sync primitives
  src/port/             the extern "Rust" contract arch/board crates implement
  src/fault.rs          fault policy and dispatch
  src/log.rs, report.rs deferred logging, kernel state dump
  src/watchdog.rs       watchdog policy (hardware-agnostic)

rivet-arch-riscv/       RV32 ISA port: trap entry/dispatch, PMP guards,
                        optional CLINT tick/IPI backend (feature "clint")
rivet-arch-cortex-m/    Cortex-M ISA port: PendSV/MemManage, MPU guards,
                        optional SysTick backend (feature "systick")

rivet-bsp-qemu-virt/    board support: QEMU RISC-V "virt"
rivet-bsp-lm3s6965/     board support: QEMU lm3s6965evb (+ typestate GPIO)
rivet-bsp-mps2-an385/   board support: QEMU mps2-an385 (the "second board"
                        proof board)
rivet-bsp-support/      shared BSP helpers: software-watchdog fallback,
                        NS16550 UART driver

rivet-rt/               boot glue (_start/Reset, bss/data init, mhartid
                        park guard, default panic handler) and the
                        #[rivet::main] macro's runtime half
rivet-macros/           #[rivet::task] and #[rivet::main] proc macros

examples/qemu-riscv/    demo + 11 QEMU test binaries for the riscv board
examples/qemu-cm3/      demo + 10 QEMU test binaries for the cm3 board
examples/mps2-an385/    demo + 8 QEMU test binaries for the mps2 board

xtask/                  the QEMU test harness: board registry, per-test
                        golden-output/exit-code/qemu-log assertions
fuzz/                   cargo-fuzz targets for the pure-logic modules
tests/gdb/              GDB-scripted context-switch verification
tests/golden/           captured golden outputs + a running log of every
                        pre-existing and newly-found bug, fixed or not
docs/porting.md         step-by-step guide to adding a new board
plan.md                 the layering-overhaul design doc and its running
                        implementation log (what's done, what's deferred,
                        and why, phase by phase)
```

---

## 16. Building, running, and testing

```bash
# One-time setup
rustup target add riscv32imac-unknown-none-elf thumbv7m-none-eabi
sudo apt install qemu-system-misc qemu-system-arm   # for the demos/tests

# Run a demo directly
./scripts/run-qemu.sh    # RISC-V
./scripts/run-cm3.sh     # Cortex-M3 (lm3s6965evb)

# Host-side kernel tests (unit + integration + property-based)
cargo test -p rivet

# The full QEMU test harness
cargo xtask boards                          # list registered boards
cargo xtask list --target riscv             # list that board's test cases
cargo xtask test --target riscv --suite smoke
cargo xtask test --target cm3 --suite smoke
cargo xtask test --target mps2 --suite smoke
cargo xtask test --target riscv --suite gdb # context-switch verification (needs gdb-multiarch)
cargo xtask soak --target riscv --sim-hours 4   # bounded soak-invariant proof (see plan.md Phase 9)

# Deeper verification (see §17)
cargo +nightly miri test -p rivet --lib
RUSTFLAGS='--cfg loom' cargo test -p rivet --features loom --test loom --release
cargo +nightly fuzz run fuzz_sched -- -max_total_time=60
cargo llvm-cov -p rivet --tests --summary-only
```

Every example binary is a `#![no_std] #![no_main]` crate depending on
`rivet` + one `rivet-arch-*` + one `rivet-bsp-*` + `rivet-rt`. The demo
(`main.rs` in each `examples/*` package) walks through three phases in
order and prints its progress: priority inheritance, real preemption, then
the cooperative async tier — see any of the three `examples/*/src/main.rs`
files for the exact narrated walkthrough.

---

## 17. Verification and quality bar

Rivet's testing strategy is layered specifically to catch different classes
of bugs that "run the demo and eyeball it" cannot:

- **Host-side unit + integration tests** (`cargo test -p rivet`) — the
  scheduler, waker bitmap, sync primitives, and an end-to-end async
  producer/consumer test driven through the real polling machinery, run
  under a serialized-and-reset harness (`kernel_test!`) so parallel test
  threads don't race on shared kernel statics.
- **`loom`** — exhaustive permutation testing of the lock-free core
  (waker/semaphore/channel), the only tool that can actually *validate* the
  acquire/release orderings otherwise justified only by comments.
- **`proptest`** — model-based testing: a naïve reference scheduler and
  reference timer queue, checked against the real implementation across
  random operation sequences, so correctness isn't only checked against
  test cases the same author thought of.
- **`cargo-fuzz`** — three targets (scheduler, waker bitmap, channel ring)
  fuzzing the pure-logic modules directly on the host.
- **The QEMU harness (`xtask`)** — asserts the **exit code**, an **ordered
  golden-output sequence** (not just "contains the right words somewhere"),
  and that **QEMU's own trap/guest-error log is empty** unless a test
  explicitly declares it expects a trap. This last check is what would have
  caught, for example, a stack-overflow guard silently failing to fire.
- **`cargo-llvm-cov`**, wired into CI as an informational (non-gating)
  report — current baseline and the reasoning for not yet enforcing a hard
  floor are in `plan.md`'s Phase 9 section.
- **A GDB-scripted context-switch verifier** (`tests/gdb/ctx_switch.py`) —
  the only thing that actually proves the hand-written assembly
  context-switch is correct: it compares the live register file against
  the saved frame at every single switch, on both architectures.

CI (`.github/workflows/ci.yml`) runs all of the above per board, plus
`clippy -D warnings` scoped correctly per target (the kernel checked on
host; every arch/board/example crate checked on its own real target,
since they contain genuine architecture-specific assembly that simply
cannot build for the host).

---

## 18. Known limitations and honest gaps

Documented here rather than discovered the hard way:

- **No real hardware has been tested.** Everything is verified in QEMU.
  QEMU's TCG emulation has no caches, wait states, bus contention, or flash
  latency — it validates *mechanism* (does the guard fire, does the switch
  restore correctly), not *timing magnitude*. Any WCET/latency number from
  QEMU is not a hardware number.
- **RISC-V M-mode memory protection is inherently weaker than Cortex-M's.**
  PMP entries affecting machine mode must be locked at boot and are
  immutable until reset, so RISC-V gets boot-time-static overflow *guards*,
  not Cortex-M's fully reprogrammable-per-switch mutual isolation.
- **Multi-core is explicitly out of scope beyond a safety guard.** The
  entire kernel's synchronization model rests on "disable interrupts = 
  mutual exclusion," which is simply false under true SMP. `-smp N > 1` is
  made *safe* (every hart but 0 parks without touching kernel state,
  verified as a permanent regression check) but not *useful* — true SMP
  would be a rewrite of the synchronization layer, not a feature addition.
- **ESP32-C3 (and any RV32IMC target without the atomic extension) cannot
  currently be supported.** The kernel's lock-free code needs
  `fetch_add`/`compare_exchange`, which don't exist on that ISA without
  switching to software-emulated atomics — a real, cross-cutting kernel
  change, verified by directly attempting it (`cargo check -p rivet
  --target riscv32imc-unknown-none-elf` fails with 36 missing-method
  errors), not assumed.
- **`rivet::log!` has no argument interpolation yet** — it takes a level
  and a plain string, not a `format_args!`-style template. See
  [§10](#10-logging-and-diagnostics).
- **No interrupt-driven peripheral drivers, no IRQ dispatch API, no
  `embedded-hal` implementation.** The design for all three is fully
  specified in `plan.md`'s Phase 8 section (down to which crate owns which
  piece — the controller driver is arch, the IRQ number map is board) but
  not implemented; each is a substantial project in its own right.
- **No chaos/fault-injection testing, no Kani formal verification, no
  enforced coverage floor.** All three were evaluated and are documented as
  concrete next steps rather than attempted partially.

The single source of truth for exactly what's done, what's deferred, and
the reasoning behind every scope decision is `plan.md` — it's a running
log, not just a forward-looking design doc, and it's kept current as work
lands.

---

## 19. Roadmap

In roughly the order the project's own planning ranks them:

1. **IRQ dispatch** — a registration table + `enable`/`disable`/
   `set_priority` API in the kernel, backed by an NVIC driver
   (`rivet-arch-cortex-m`) and a PLIC driver (`rivet-arch-riscv`), with the
   IRQ-number-to-peripheral map living in each board crate. This is the
   actual missing primitive everything else in this list depends on.
2. **Interrupt-driven peripheral drivers**, starting with UART RX/TX ring
   buffers, as BSP-crate code (not kernel code) — proving the IRQ mechanism
   end-to-end.
3. **`embedded-hal` 1.0 + `-async`** trait implementations in the BSP
   crates, opening up the existing Rust embedded driver ecosystem.
4. **Execution-time accounting, deadlines, and latency histograms**,
   extending `rivet::report()` once a cycle-counter port symbol exists.
5. **Argument interpolation for `rivet::log!`**, plus the originally-
   planned interned-format-string/host-decoder design if the simpler
   current version turns out not to be enough.
6. **A real multi-hour soak run** (scaling `soak_smoke`'s iteration count
   and the harness timeout together), randomized tick-jitter and
   GDB-driven fault-injection chaos testing, and an enforced coverage
   floor.
7. **Real hardware validation**, once the above is further along — the
   project is explicit that this is a separate phase, not a stepping stone
   to rush toward.
