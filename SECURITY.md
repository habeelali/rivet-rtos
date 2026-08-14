# Security Policy

## What "security" means for a `no_std` kernel

Rivet isn't a network service. It has no listening socket, no parser
for untrusted wire formats in the kernel itself, no attack surface in
the usual web-application sense. The security-relevant surface here is
narrower and more specific:

- **Memory safety in `unsafe` code.** The kernel crate builds with
  `#![forbid(clippy::undocumented_unsafe_blocks)]` (every `unsafe` block
  has a `// SAFETY:` comment stating the invariant it relies on), but a
  documented invariant can still be *wrong*. A bug that lets safe code
  trigger undefined behavior, a data race the safety comment missed, an
  out-of-bounds access in the lock-free scheduler core, a torn read/write
  loom/miri didn't catch, is a real security issue here, not just a
  correctness one, because this kernel runs with no memory protection
  between itself and the tasks it schedules on most boards.
- **Fault-isolation bypass.** Where a board *does* have MPU/PMP-backed
  stack isolation (see `docs/DOCUMENTATION.md` for which boards), a bug
  that lets one task's stack overflow or read/write into another task's
  region, or into kernel state, without being caught is in scope.
- **Scheduler-correctness bugs reachable by an adversarial task.** Not
  "the scheduler is occasionally unfair," something a task genuinely
  controls (spawn pattern, mutex/semaphore usage, timing) that lets it
  starve every other task indefinitely, corrupt the ready-queue bitmap,
  or defeat priority inheritance in a way that causes unbounded
  (not just measured-and-documented) priority inversion.
- **Watchdog/fault-policy bypass.** A path where a hung or faulting task
  is supposed to be caught (by the configured `FaultKind` policy or the
  watchdog) but isn't.

## What's out of scope

- **Physical/hardware attacks**: voltage/clock glitching, JTAG/SWD access
  by someone with the board in hand, side-channel analysis. Rivet targets
  microcontrollers with no secure boot or debug-lock story; this is a
  correctness project, not a hardened-secure-element one.
- **Boards explicitly marked experimental** in `README.md`/
  `docs/DOCUMENTATION.md` (currently: the RP2040 port). File these as
  regular issues, not security reports, unless the same bug is also
  reachable on a board that isn't marked experimental.
- **The host-side tooling** (trace visualizers, debuggers): those live
  in separate, unpublished repositories and aren't covered by this
  policy.
- Anything requiring `unsafe` *misuse* by the application itself (e.g.
  handing `spawn_ptask!` a stack that isn't actually `'static` and
  exclusively owned, which the macro's own safety contract explicitly
  requires the caller to uphold).

## Supported versions

Pre-1.0: only the latest published `0.1.x` release is supported. There's
no LTS branch yet; fixes land on `main` and the next release.

## Reporting

Use GitHub's private vulnerability reporting
([Security tab → "Report a vulnerability"](https://github.com/habeelali/rivet-rtos/security/advisories/new))
so the report and any discussion stay private until a fix ships. If you'd
rather not use GitHub for this, email **habeelali023@gmail.com** with
"RIVET SECURITY" in the subject.

Include what you'd include in any other bug report here (see
`CONTRIBUTING.md`'s "claims need evidence" standard): which board or
target, a reproduction (a failing test, a QEMU trace, a real-hardware
observation), and, if you have one, the specific invariant or `SAFETY:`
comment you believe is wrong and why.

## What to expect

This is a solo-maintained project, so there's no SLA. A genuine memory-
safety or fault-isolation report gets priority over a feature PR sitting
in the same queue, and I'll acknowledge a report within a few days. There
is no bug bounty; the ask is simply coordinated disclosure: give a fix
(or an explicit "this is a known, documented limitation, not a bug") a
chance to land before public disclosure.
