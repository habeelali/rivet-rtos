# Writing a `rivet` peripheral driver

This is the practical companion to `embedded-hal-plan.md` (the design doc
that shaped the primitives below) — read this one if you're adding a new
interrupt-driven peripheral driver to a `rivet-bsp-*` crate, not designing
one from scratch. Every rule here was found the hard way, on real hardware,
building the drivers already in this tree (`pl022`, `stellaris_i2c`, the
STM32F401 I2C and EXTI drivers) — cross-reference their source for worked
examples, not just this doc.

## The shape every driver takes

One struct per peripheral instance, holding a fixed register base and a
`&'static rivet::sync::Signal`. Two trait families on the *same* struct:

- `embedded_hal::*` (sync) — for preemptive tasks, which have no
  cooperative suspend primitive. Polls raw status registers in a spin
  loop; correct, if not efficient.
- `embedded_hal_async::*` (async) — for cooperative `#[rivet::task]`s.
  Arms the interrupt, `.await`s the `Signal`, and returns once the ISR
  wakes it.

Don't try to make one method do both by, say, having the sync trait poll a
`Signal` synchronously — `RivetDelay`'s `embedded_hal_async::delay::DelayNs`
already does something like this deliberately (blocking the calling
preemptive task, since that tier has nothing better), and that's the one
sanctioned exception, not a pattern to copy. Two clean trait impls beat one
impl that pretends to serve both tiers.

## `Signal`: the completion primitive

[`rivet::sync::Signal`](../rivet/src/sync/signal.rs) is a one-shot,
latching handoff between an ISR and exactly one waiting task. Construct it
via a `_instance!` macro (see below), `reset()` it before arming hardware,
`.await` it from the cooperative task, `signal()` it from the ISR.

**The register-then-recheck contract.** `Signal::wait()`'s `Future::poll`
does: try_take → register the current task as waiter → try_take *again* →
`Pending`. This closes the race where `signal()` fires between your first
check and registering — study `Wait::poll` in `signal.rs` before writing
anything that hand-rolls similar logic. Every driver in this tree relies on
this to correctly handle "the hardware condition was already true/pending
before the future was ever polled" — which is the *common* case, not an
edge case, once you understand why (see "arm, then trigger" below).

**Arm, then trigger — even when triggering yourself.** If you're
self-triggering a peripheral for a test (software-interrupt registers,
loopback modes), do the *hardware* arming (unmask/enable bits) before the
trigger, not after. `stm32_wait_test.rs`'s own history is the cautionary
tale: triggering `EXTI_SWIER` before the awaited future's body had even
run (async fn bodies don't execute until first polled — the trigger call
that appears to come "after" `pin.wait_for_rising_edge()` in the source
actually runs *before* any of that method's own arming code) left `IMR`
still masked at the moment the software interrupt pended, and the
interrupt never fired — confirmed by a JTAG breakpoint on the ISR that
was never hit. This is not a bug in `Signal`; it's what happens when the
*hardware* trigger outraces your own arming code, same as a human pressing
a button before you've enabled the interrupt. Fixed by exposing separate
`arm()`/`wait_armed()` steps so the test could sequence them explicitly.

**Statics inside generic functions are not monomorphized per const-generic
instantiation.** A naive `fn isr<const BASE: usize>()` with an inline
`static Signal` would silently share *one* `Signal` across every
instantiation of that function — invisible in a single-instance test,
catastrophic the moment a board has two of the same peripheral. Every
`_instance!` macro in this tree (`pl022_instance!`, `stellaris_i2c_instance!`,
`stm32_i2c_instance!`, `stm32_exti13_instance!`) exists specifically to
force a distinct `static Signal` + named `fn()` ISR pair per call site:

```rust
rivet_bsp_support::pl022_instance!(SPI0_SIG, spi0_isr, base = 0x4000_8000);
```

Write the same shape for a new peripheral rather than trying to be clever
with generics.

## The `fn()`-only ISR constraint

`rivet::irq::register(irq_num: u32, handler: fn())` — plain function
pointers only, confirmed by the signature itself
(`rivet/src/irq.rs:50`). No closures, no captures. This is *why* the
`_instance!` macro pattern exists: the generated `fn() { isr(&SIG) }`
shim is how a closure-shaped "call this ISR with this Signal" need gets
expressed through a bare function pointer.

## Masking a level-latched status flag is not optional

Most peripheral status flags this project has dealt with are
level-latched, not edge-triggered: they stay asserted until *something*
explicitly clears the condition (draining a FIFO, writing a data
register, clearing a pending bit). If your ISR calls `signal()` without
first masking the interrupt enable at the peripheral (not just
acknowledging it), the NVIC will re-enter your ISR the instant it returns
— before the awaiting task ever gets a chance to run the code that would
actually clear the condition. This is a real, previously-shipped bug in
this exact codebase: the STM32 I2C driver's event/error ISRs originally
called `signal()` without masking `CR2`'s `ITEVTEN`/`ITERREN`, live-verified
via JTAG to cause a genuine interrupt-storm livelock (PC pinned in Handler
mode, Thread-mode code — including the very code that would clear the
condition — never getting to run at all). `pl022`'s ISR masks `IMSC` for
the same reason; `stm32_i2c`'s masks `CR2`.

The corollary: if your driver has more than one `.await` point per
transaction (STM32 I2C's `start_async` then `send_address_async`), **every
`.await` past the first must re-arm the interrupt enables itself** —
the ISR unconditionally disarms them on every entry, so nothing else will.
This was the second real bug found building that driver: `send_address_async`
inherited the enables `start_async` had armed, but the ISR had already
cleared them after the first interrupt, so the second phase's completion
interrupt never fired.

## `TaskCell` sizing — a runtime panic, not a compile error

`TaskCell::<SIZE>::poll` asserts (not `static_assert`s — a genuine runtime
`assert!`, `rivet/src/task.rs:144-154`) that the concrete future fits in
`SIZE` bytes and doesn't need stricter alignment than the cell provides.
This means a future that's too large for `#[rivet::task(stack = N)]`'s `N`
compiles cleanly and only fails at the task's *first poll*, on hardware —
not in CI, not at link time. A driver holding buffers across an `.await`
(a multi-byte I2C/SPI transaction future, for instance) can easily exceed
the 512-byte default. Size generously and document the reasoning at the
call site (`#[rivet::task(stack = 1024)]`, with a comment on why), rather
than discovering the panic on real hardware.

Register the waiter lazily, on first `poll`, never in the constructor —
`Sleep`/`Semaphore`/`Channel`/`Signal::wait` all do this because the
future is constructed on the caller's stack and moved into the `TaskCell`
before its first poll; registering earlier would register the wrong
address.

## Cancellation does not disarm hardware

This is the sharpest footgun in the whole design, worth stating bluntly:
**dropping an in-flight `.await` on a `Signal::wait()` clears only the
software registration, never arms/disarms anything at the peripheral.**
`Wait::drop` (in `signal.rs`) explicitly does *not* clear the latch either
— a signal that fired during cancellation must stay observable to
whatever calls `try_take()` next. If your driver's async method enables an
interrupt and the task holding that future gets dropped mid-transaction
(a timeout wrapping it, a `select!`-style race against another future,
task despawn), the peripheral's interrupt mask is left exactly as it was
— potentially still enabled, potentially mid-transaction, with no task
left to receive the next `signal()`. That ISR will fire into nobody.

**Every driver whose async method leaves hardware armed must implement
`Drop` to disable that peripheral's interrupt enable (and abort/flush any
in-flight transfer) — the `Signal`'s own `Drop` will never do this for
you.** None of the drivers in this tree today hold a `Signal::wait()`
across a cancellation-prone `.await` in a way that's been exercised this
way in practice (every existing test runs its transaction to completion),
so this is a documented gap to close *before* shipping a driver whose
callers might reasonably cancel it, not a solved problem to copy from
existing code.

## Real quirks worth knowing before you hit them yourself

- **QEMU's PL022 model**: `RXIM` only asserts once the RX FIFO holds ≥ 4
  bytes — a transfer shorter than that won't raise the interrupt under
  emulation. `TXIM` asserts almost permanently (QEMU's transfers are
  instantaneous) — only ever unmask `RXIM`.
- **QEMU's `stellaris-i2c` model**: address-NAK sets `MCS.ERROR` but never
  raises an interrupt — check it synchronously right after issuing `RUN`,
  not via a `.await` (an await-only NAK path hangs forever under this
  model). `MIMR` can't be re-masked once set — clear via `MICR` instead.
  Repeated-START is broken in the model — use STOP-then-START.
- **STM32's legacy I2C peripheral**: `ADDR` must be cleared by reading
  `SR1` then `SR2`, in that order — reading `SR1` alone leaves the
  address-match condition latched and the bus stuck. The last byte of a
  multi-byte read needs `ACK` cleared and `STOP` set *before* reading the
  second-to-last byte's `DR`. The last transmitted byte of a transaction
  needs `BTF`, not just `TXE` — `TXE` alone can issue `STOP` while the
  shift register is still clocking the byte out.
- **I2C needs a real pull-up, internal or external.** Unlike SPI (separate
  MOSI/MISO), I2C's SCL/SDA are a shared open-drain bus — with no pull-up
  at all, the bus can never reach electrical idle-high, and `START`
  generation spins forever waiting for `SB`. Found live on real STM32F401RE
  hardware this project: enabling `GPIOx_PUPDR`'s internal pull-up on both
  pins was what actually fixed it.
- **`EXTI_SWIER` pends the same interrupt a real edge would**, independent
  of `RTSR`/`FTSR`'s trigger-direction configuration — useful for a
  human-free hardware test of an edge-driven `Wait` impl, as long as you
  arm before you trigger (see above).
