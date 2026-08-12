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

Rivet runs on seven boards across three architectures, three of them real
silicon, chosen to keep the architecture/board split honest rather than
implicitly written against just one target:

| Board | Architecture | Notes |
|---|---|---|
| **QEMU `virt` (RV32)** | RISC-V, RV32IMAC | CLINT-based tick/IPI, NS16550 UART, `riscv.sifive.test` exit/reset device, software watchdog. |
| **`lm3s6965evb`** | ARM Cortex-M3 (QEMU) | PL011 UART, real `luminary-watchdog` hardware block, typestate GPIO driver, SysTick/PendSV context switch. |
| **`mps2-an385`** | ARM Cortex-M3 (QEMU) | A *different* memory map and peripheral set from lm3s6965 — CMSDK APB UART, CMSDK/SP805-compatible watchdog. Added purely as a new board-support crate, without touching the kernel or the Cortex-M architecture port, as proof the boundary holds. |
| **ESP32-C6** | RISC-V, RV32IMAC | QEMU-validated board support. |
| **ESP32-S3** | Xtensa LX7, dual-core | **Real hardware.** The only dual-core (`RIVET_MAX_HARTS=2`) target — cross-hart IPI-driven fairness, a genuine cross-hart data race found and fixed via live JTAG (`docs/realtime.md` §15), real-hardware-verified priority inheritance and round-robin fairness. Built with Espressif's separate `esp` Rust toolchain fork (excluded from the main workspace — see §15). |
| **STM32F401RE (Nucleo-64)** | ARM Cortex-M4, single-core | **Real hardware** (ST-LINK/V2-1 onboard debugger, SWD). The current best hard-real-time story in this project: `docs/wcet-stm32f401re.md` gives a scoped, real-hardware-measured hard-RT declaration — zero-variance nested-interrupt latency (86 cycles, 500/500 identical samples), measured `PRIORITY_INVERSION_BOUNDED` with zero medium-priority interference, no cross-hart contention class of problem (single core). |

An eighth target — ESP32-C3 (RV32IMC) — was evaluated and found to lack the
RISC-V atomic (`A`) extension the kernel's lock-free scheduler relies on
throughout. This was originally documented as a hard architectural
blocker; it's narrower than that now. Bringing up the RP2040 port (Cortex-
M0+/ARMv6-M, which has the identical problem — no LDREX/STREX) added an
`atomics-polyfill` feature: `rivet::sync::atomic` swaps its re-export from
`core::sync::atomic` to `portable-atomic`, a byte-for-byte API-compatible
drop-in that provides the missing `compare_exchange`/`fetch_or`/etc. via a
critical-section-guarded fallback, with every call site elsewhere in the
kernel unaffected. ESP32-C3 support would still need a RISC-V
`critical_section::Impl` registered (the RP2040 port uses `cortex-m`'s
ready-made one; ESP32-C3 has no equivalent off the shelf yet) — real work,
not zero work — but "switch to software-emulated atomics" is no longer a
cross-cutting kernel change, it's enabling an existing feature plus writing
one new critical-section backend. A good first issue for RISC-V experience.

## RP2040 (Raspberry Pi Pico) — experimental, not yet validated

A ninth port, `rivet-arch-cortex-m0` + `rivet-bsp-rp2040`, is in the
workspace but **does not yet meet this project's real-hardware bar** and
is deliberately not listed in the table above. What's actually confirmed
on real hardware: the board boots, brings up its own clock tree (XOSC +
dual PLL — RP2040 resets on an imprecise ring oscillator, unlike this
project's other boards' factory-trimmed defaults), and a USB CDC-ACM
console (`rivet-bsp-rp2040::usb`) enumerates and transmits real bytes.
What's *not* yet confirmed: the scheduler/context-switch/task-spawn path
running end to end — the full demo (priority inheritance, forced
preemption, the async tier) has been flashed but hasn't yet produced
observable output past board bring-up. A real, reproducible finding from
this work: RP2040's `BUFF_STATUS` USB interrupt reliably completes EP0
control transfers (enumeration works every time) but never once drove a
CDC bulk-data transfer to completion in testing — only direct, repeated,
synchronous `poll()` calls did. Root cause not isolated; a real open
question, not swept under the rug (see the open issues for the current
state). Treat this port as "compiles and partially boots," not "works,"
until this section is updated.

**Real hardware has been tested** — ESP32-S3 and STM32F401RE above, both
via live JTAG/SWD debugging (OpenOCD + GDB), not just flash-and-hope. See
`docs/wcet.md` and `docs/wcet-stm32f401re.md` for exact, method-labeled
(measured / derived / architectural / assumed) timing figures gathered this
way — including real findings QEMU could never have surfaced, like a
baud-rate-bound critical section discovered only by measuring on real
silicon. The QEMU-only boards above still have the caveats in
[§18](#18-known-limitations-and-honest-gaps): QEMU's TCG emulation has no
caches, wait states, or flash latency, so it validates *mechanism*, not
*timing magnitude*, on those specific targets.

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

Today, `rivet` is checkably board-free: `cargo build -p rivet-rtos` compiles for
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

Published to crates.io as **`rivet-rtos`** (`[package] name`) — the
crate name `rivet` was already taken; `[lib] name = "rivet"` in
`rivet/Cargo.toml` keeps every `use rivet::...` in this workspace and any
downstream consumer's code unchanged (`cargo add rivet-rtos` and then
`use rivet::...`, same as here).

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
  src/trace.rs          optional (feature "trace") live event emission —
                        scheduler/IRQ/fault events over whatever transport
                        the board's port::board::trace_write implements,
                        for external tooling. Off by default; zero cost
                        when the feature isn't enabled.

rivet-arch-riscv/       RV32 ISA port: trap entry/dispatch, PMP guards,
                        optional CLINT tick/IPI backend (feature "clint")
rivet-arch-cortex-m/    Cortex-M3/M4/M7 (ARMv7-M) ISA port: PendSV/
                        MemManage, MPU guards, NVIC IRQ dispatch, DWT
                        cycle counter, optional SysTick backend
                        (feature "systick")
rivet-arch-cortex-m0/   Cortex-M0/M0+ (ARMv6-M) ISA port — a separate
                        crate from rivet-arch-cortex-m, not a #[cfg]
                        branch inside it: ARMv6-M has no MPU, no DWT, no
                        32-bit Thumb-2 STM/LDM (PendSV shuttles r8-r11
                        through r4-r7 with `mov` instead), and no
                        MemManage/BusFault/UsageFault. Experimental — see
                        §3's RP2040 section.
rivet-arch-xtensa/      Xtensa LX7 ISA port (ESP32-S3 only): context
                        switch via xtensa-lx-rt's Context struct, dual-hart
                        (RIVET_MAX_HARTS=2) dispatch, cross-hart IPI
                        fairness broadcast. Excluded from the main
                        workspace (needs Espressif's separate `esp` Rust
                        toolchain fork) — build via `cd rivet-arch-xtensa
                        && cargo +esp build`, never `cargo build --workspace`.

rivet-bsp-qemu-virt/    board support: QEMU RISC-V "virt"
rivet-bsp-lm3s6965/     board support: QEMU lm3s6965evb (+ typestate GPIO)
rivet-bsp-mps2-an385/   board support: QEMU mps2-an385 (the "second board"
                        proof board)
rivet-bsp-esp32c6/      board support: ESP32-C6 (RISC-V, QEMU-validated)
rivet-bsp-esp32s3/      board support: ESP32-S3, real hardware, dual-core.
                        Excluded from the main workspace alongside
                        rivet-arch-xtensa (same toolchain reason).
rivet-bsp-stm32f401re/  board support: STM32F401RE Nucleo-64, real
                        hardware. Interrupt-driven USART2 console, IWDG
                        watchdog, NVIC priority floor.
rivet-bsp-rp2040/       board support: Raspberry Pi Pico (RP2040), real
                        hardware. Experimental — see §3. Boot2 second-
                        stage bootloader, hand-rolled XOSC/PLL clock tree
                        plus rp2040-hal's for PLL_USB, USB CDC-ACM debug
                        console (rp2040-hal's UsbBus + usb-device/
                        usbd-serial — the one dependency in this
                        workspace on a peripheral driver crate that isn't
                        a bare PAC, for the same reason rivet-boot2 is a
                        pre-built blob: DPRAM/endpoint/SIE management is
                        genuinely intricate hardware not worth hand-
                        rolling blind).
rivet-bsp-support/      shared BSP helpers: software-watchdog fallback,
                        NS16550 UART driver

rivet-rt/               boot glue (_start/Reset, bss/data init, mhartid
                        park guard, default panic handler) and the
                        #[rivet::main] macro's runtime half
rivet-macros/           #[rivet::task] and #[rivet::main] proc macros

examples/qemu-riscv/    demo + QEMU test binaries for the riscv board
examples/qemu-cm3/      demo + QEMU test binaries for the cm3 board
examples/mps2-an385/    demo + QEMU test binaries for the mps2 board
examples/esp32c6/       demo + QEMU test binaries for the esp32c6 board
examples/esp32s3/       demo + real-hardware test/bench binaries for the
                        esp32s3 board (smp_latency_bench, stress_load_bench,
                        smp_test — the dual-hart fairness/race regression
                        suite). Excluded from the main workspace (same
                        toolchain reason as rivet-arch-xtensa).
examples/stm32f401re/  demo + real-hardware test/bench binaries for the
                        stm32f401re board, including the purpose-built WCET
                        benchmarks (nested_irq_bench, critsec_isolate_bench,
                        priority_inversion_bench, deadline_miss_bench).
examples/rp2040/        demo binary for the rp2040 board. Experimental —
                        see §3; builds and partially boots, not yet a
                        validated test suite.

xtask/                  the QEMU test harness: board registry, per-test
                        golden-output/exit-code/qemu-log assertions
fuzz/                   cargo-fuzz targets for the pure-logic modules
tests/gdb/              GDB-scripted context-switch verification
tests/golden/           captured golden outputs + a running log of every
                        pre-existing and newly-found bug, fixed or not
docs/porting.md         step-by-step guide to adding a new board
docs/realtime.md        real-time characterization log: every timing bug
                        found and fixed on real hardware, phase by phase,
                        including the ESP32-S3 dual-hart race (§15)
docs/wcet.md            formal WCET analysis, ESP32-S3/Xtensa — interrupt
                        latency, context-switch, scheduling, critical-
                        section, and blocking-time figures, each labeled
                        by method (measured / derived / architectural /
                        assumed)
docs/wcet-stm32f401re.md
                        formal WCET analysis and a scoped hard-real-time
                        declaration for STM32F401RE — the same figures,
                        gathered on real hardware, for a target that
                        structurally avoids most of the ESP32-S3's
                        dual-core-specific hazards
```

`plan.md` (the layering-overhaul design doc and phase-by-phase implementation
log this project has kept throughout its development) is a working document,
not part of the tracked repository — see the root `.gitignore`.

---

## 16. Building, running, and testing

```bash
# One-time setup — QEMU boards
rustup target add riscv32imac-unknown-none-elf thumbv7m-none-eabi
sudo apt install qemu-system-misc qemu-system-arm   # for the demos/tests

# Run a demo directly
./scripts/run-qemu.sh    # RISC-V
./scripts/run-cm3.sh     # Cortex-M3 (lm3s6965evb)

# Host-side kernel tests (unit + integration + property-based)
cargo test -p rivet-rtos

# The full QEMU test harness
cargo xtask boards                          # list registered boards
cargo xtask list --target riscv             # list that board's test cases
cargo xtask test --target riscv --suite smoke
cargo xtask test --target cm3 --suite smoke
cargo xtask test --target mps2 --suite smoke
cargo xtask test --target riscv --suite gdb # context-switch verification (needs gdb-multiarch)
cargo xtask soak --target riscv --sim-hours 4   # bounded soak-invariant proof (see plan.md Phase 9)

# Deeper verification (see §17)
cargo +nightly miri test -p rivet-rtos --lib
RUSTFLAGS='--cfg loom' cargo test -p rivet-rtos --features loom --test loom --release
cargo +nightly fuzz run fuzz_sched -- -max_total_time=60
cargo llvm-cov -p rivet-rtos --tests --summary-only
```

Every example binary is a `#![no_std] #![no_main]` crate depending on
`rivet` + one `rivet-arch-*` + one `rivet-bsp-*` + `rivet-rt`. The demo
(`main.rs` in each `examples/*` package) walks through three phases in
order and prints its progress: priority inheritance, real preemption, then
the cooperative async tier — see any of the three `examples/*/src/main.rs`
files for the exact narrated walkthrough.

### Real hardware — STM32F401RE (Nucleo-64)

No extra toolchain needed beyond the standard `thumbv7em-none-eabi` target
and a system `openocd` (0.12+ has `board/st_nucleo_f4.cfg` built in) —
everything flashes over the onboard ST-LINK/V2-1 via SWD:

```bash
rustup target add thumbv7em-none-eabi
openocd -f interface/stlink.cfg -f board/st_nucleo_f4.cfg &   # GDB server on :3333

cd examples/stm32f401re
cargo build --release --bin demo
gdb-multiarch -batch \
  -ex "target extended-remote :3333" -ex "monitor reset halt" \
  -ex load -ex "monitor reset" -ex detach -ex quit \
  ../../target/thumbv7em-none-eabi/release/demo

# Console output is on the same ST-LINK's CDC-ACM virtual COM port:
# /dev/ttyACM<N> at 115200 8N1 (picocom, minicom, screen, or a plain
# pyserial read all work).
```

### Real hardware — ESP32-S3

Needs Espressif's separate `esp` Rust toolchain fork (`espup`) and
`espflash`, since `rivet-arch-xtensa`/`rivet-bsp-esp32s3`/`examples/esp32s3`
are excluded from the main workspace for exactly that reason:

```bash
# One-time: espup install, then `source ~/export-esp.sh` in every new shell
export RIVET_MAX_HARTS=2   # for the dual-core fairness/race regression suite
cd examples/esp32s3
cargo build --release --bin demo --target xtensa-esp32s3-none-elf
espflash flash --monitor target/xtensa-esp32s3-none-elf/release/demo
```

---

## 17. Verification and quality bar

Rivet's testing strategy is layered specifically to catch different classes
of bugs that "run the demo and eyeball it" cannot:

- **Host-side unit + integration tests** (`cargo test -p rivet-rtos`) — the
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

- **QEMU-only boards (`virt`, `lm3s6965evb`, `mps2-an385`, `esp32c6`) have
  not been hardware-validated.** QEMU's TCG emulation has no caches, wait
  states, bus contention, or flash latency — it validates *mechanism* (does
  the guard fire, does the switch restore correctly), not *timing
  magnitude*. Any WCET/latency number from those boards is not a hardware
  number. This caveat does **not** apply to ESP32-S3 or STM32F401RE — both
  are real-hardware-validated, including live JTAG/SWD measurement; see
  `docs/wcet.md` and `docs/wcet-stm32f401re.md`.
- **The cross-hart critical-section lock (`critical::enter` on a
  multi-hart build) has no FIFO/bounded-wait guarantee.** It's a raw
  test-and-set, correct but not fair — a formal WCET analysis cannot derive
  a closed-form upper bound on lock-acquisition wait time from the
  algorithm alone on a multi-hart board (`docs/wcet.md` §6.2). This does not
  affect any single-hart board, where the same lock's uncontended fast path
  is the only path there is.
- **`rivet::console::write_str` on the ESP32-S3 board holds `critical::
  enter` across a UART-FIFO-drain busy-wait that scales with output length
  once it exceeds the hardware FIFO's ~128-byte headroom** — bound by baud
  rate, not CPU speed, measured at ~5.5 ms for ordinary `report()` output
  (`docs/wcet.md` §6.1). Not present as a normal-operation hazard on
  STM32F401RE, whose console is interrupt-driven/ring-buffered during
  normal operation; the equivalent unbounded-by-length path there
  (`flush_sync`) is architecturally confined to system termination, not
  reachable from a running task (`docs/wcet-stm32f401re.md` §3).
- **RISC-V M-mode memory protection is inherently weaker than Cortex-M's.**
  PMP entries affecting machine mode must be locked at boot and are
  immutable until reset, so RISC-V gets boot-time-static overflow *guards*,
  not Cortex-M's fully reprogrammable-per-switch mutual isolation.
- **True multi-core SMP exists only on ESP32-S3 (Xtensa, `RIVET_MAX_HARTS=
  2`).** Getting it right there required finding and fixing a genuine
  cross-hart data race (a torn, unsynchronized `Context` struct copy — see
  `docs/realtime.md` §15) via live JTAG; the fix has a measured stack-size
  cost documented in the same section. RISC-V's own multi-hart path
  (`-smp N > 1`) remains a safety guard only — every hart but 0 parks
  without touching kernel state (verified as a permanent regression check),
  not genuine concurrent SMP scheduling.
- **ESP32-C3 (and any RV32IMC target without the atomic extension) is not
  yet supported, though the blocker is narrower than it used to be.** The
  kernel's lock-free code needs `fetch_add`/`compare_exchange`, which
  don't exist on that ISA natively — verified by directly attempting it
  (`cargo check -p rivet-rtos --target riscv32imc-unknown-none-elf` fails
  with 36 missing-method errors). The `atomics-polyfill` feature (added
  for the RP2040 port, which has the identical problem on ARMv6-M) now
  provides exactly the missing-method fallback via `portable-atomic`; what
  ESP32-C3 still needs is a RISC-V `critical_section::Impl` for that
  fallback to use (the RP2040 port gets this for free from `cortex-m`'s
  `critical-section-single-core` feature; nothing equivalent exists yet
  for this target). Good first issue.
- **The RP2040 port is experimental, not real-hardware-validated yet** —
  see [§3](#3-supported-hardware). Boots, brings up its own clock tree,
  and transmits real bytes over a USB CDC-ACM console; the scheduler/
  task-spawn path running end to end is not yet confirmed.
- **`rivet::log!` has no argument interpolation yet** — it takes a level
  and a plain string, not a `format_args!`-style template. See
  [§10](#10-logging-and-diagnostics).
- **`rivet::irq` (registration table, `enable`/`disable`/`set_priority`,
  NVIC-backed on Cortex-M) exists and is exercised on real hardware** (the
  STM32F401RE board's USART2 console is built entirely on it), but no
  `embedded-hal` trait implementation exists yet in any BSP crate.
- **No chaos/fault-injection testing, no Kani formal verification, no
  enforced coverage floor.** All three were evaluated and are documented as
  concrete next steps rather than attempted partially.
- **`crate::latency::Kind::SchedulingWake`'s cycle-delta computation is not
  wraparound-safe** on a 32-bit cycle counter — found on real ESP32-S3
  hardware, producing a bogus multi-billion-cycle "latency" on longer runs
  (`docs/wcet.md` §9). Reported, not fixed, since it was found during an
  analysis pass, not a debugging one; the histogram's bucketed data is
  likely contaminated by the same artifact on any sufficiently long run.

The single source of truth for exactly what's done, what's deferred, and
the reasoning behind every scope decision is `plan.md` — it's a running
log, not just a forward-looking design doc, and it's kept current as work
lands.

---

## 19. Roadmap

Delivered since this list was first written — kept here so the history is
visible rather than quietly rewritten:

- ✅ **IRQ dispatch** (`rivet::irq`, NVIC-backed on Cortex-M) — done,
  exercised on real STM32F401RE hardware (interrupt-driven USART2 console).
- ✅ **Interrupt-driven peripheral drivers** — the STM32F401RE console is
  one; RX/TX ring buffers, BSP-crate code, not kernel code, as planned.
- ✅ **Execution-time accounting, deadlines, and latency histograms**
  (`rivet::exec_time`, `rivet::deadlines`, `rivet::latency`) — done, and
  used directly to produce `docs/wcet.md`/`docs/wcet-stm32f401re.md`'s
  real-hardware figures.
- ✅ **Real hardware validation** — done, on two boards (ESP32-S3 dual-core
  Xtensa, STM32F401RE Cortex-M4), via live JTAG/SWD, not just flash-and-hope.
  Found and fixed a genuine cross-hart race in the process
  (`docs/realtime.md` §15).

Remaining, in roughly the order the project's own planning ranks them:

1. **`embedded-hal` 1.0 + `-async`** trait implementations in the BSP
   crates, opening up the existing Rust embedded driver ecosystem.
2. **Fix the `SchedulingWake` cycle-counter wraparound bug** found during
   the WCET analysis pass (`docs/wcet.md` §9) — a 64-bit monotonic
   timestamp, or explicit wraparound-aware subtraction.
3. **Move `rivet::console::write_str` on ESP32-S3 off the FIFO-polling
   critical section** (`docs/wcet.md` §6.1) — the interrupt-driven path
   this crate already has for other boards (and that STM32F401RE already
   uses) closes the one normal-operation gap standing between the ESP32-S3
   board and a STM32F401RE-style scoped hard-RT declaration.
4. **Argument interpolation for `rivet::log!`**, plus the originally-
   planned interned-format-string/host-decoder design if the simpler
   current version turns out not to be enough.
5. **A real multi-hour soak run** (scaling `soak_smoke`'s iteration count
   and the harness timeout together), randomized tick-jitter and
   GDB-driven fault-injection chaos testing, and an enforced coverage
   floor.
6. **A provably-fair (FIFO) cross-hart lock** for `critical::enter` on
   multi-hart boards, closing `docs/wcet.md` §6.2's open bound — needed for
   an ESP32-S3-class hard-RT declaration, not needed on any single-hart
   board.
