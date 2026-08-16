# Contributing to Rivet RTOS

Thanks for looking at this. Rivet is a small, opinionated project. The
conventions below aren't bureaucracy; they're what actually kept a
zero-allocation, multi-architecture kernel correct across three real ISAs
and two real boards. Read this before your first PR; it'll save you a
review round-trip.

## The one rule everything else follows: claims need evidence

Every fix, every "this works," every WCET figure in this repo is backed by
something concrete: a passing test, a QEMU golden-output diff, a real
hardware trace, or an exact instruction count read from real assembly. Not
"should work," not "looks right." This project has a documented history of
plausible-looking fixes that failed on real hardware in ways QEMU couldn't
show and vice versa (`docs/realtime.md` is the log of exactly that,
including several rejected fix attempts kept in the record on purpose).
When you open a PR:

- If you fixed a bug, say **how you confirmed it's fixed**: which test,
  which board, how many runs. "Should be fixed" is not a changelog entry.
- If you're claiming a timing/latency number, label how you got it:
  measured (real hardware, cite the tool), derived (exact instruction
  count, cite the source lines), architectural (cite the ISA/vendor
  manual), or assumed (say so explicitly). See `docs/wcet.md` for the
  standard this project holds itself to.
- If a fix only worked in QEMU, or only on one board, say that too. Don't
  let a partial verification read as a complete one.

## Before you start

- Read `docs/DOCUMENTATION.md` in full. It's the complete reference:
  architecture, the port contract, every feature, configuration, and the
  known-limitations list (§18) so you don't rediscover a documented gap.
- Skim `docs/realtime.md` and `docs/wcet.md` / `docs/wcet-stm32f401re.md`
  if you're touching anything timing-sensitive (the scheduler,
  `critical::enter`, interrupt entry/exit, the context-switch path on any
  arch). Real, hard-won findings live there, not just in code comments.
- **`plan.md` is a local development log, not part of this repository.**
  It's `.gitignore`d deliberately. Source comments across this codebase
  reference it ("plan.md Phase 19", "plan.md §4.1", etc.) as historical
  context for *why* a decision was made, written by and for the person
  doing that work at the time. A fresh clone won't have it, and that's
  expected. If a comment's `plan.md` reference matters for understanding a
  change you're making, ask, or reconstruct the reasoning from
  `docs/realtime.md`/git history instead of assuming the file exists.

## Code conventions this project actually enforces

- **`rivet` (the kernel) has zero MMIO and zero `#[cfg(target_arch)]`.**
  If you're adding kernel functionality that needs to touch hardware,
  you're adding a `port::arch`/`port::board` symbol, not an `#[cfg]`
  branch. See `docs/DOCUMENTATION.md` §12 and §4 for the port contract
  and why it's enforced this strictly (`cargo build -p rivet` must never
  touch MMIO, checked in CI).
- **No comments explaining *what* code does, only *why*, when the why
  isn't obvious from reading it.** A comment that just restates the next
  line in English gets deleted in review. A comment explaining a hidden
  constraint, a workaround for a specific found bug, or a non-obvious
  invariant stays. Every comment in this codebase should survive the
  question "would a future reader be confused without this?"
- **No unsafe without a `// SAFETY:` comment directly above it**
  (`#![forbid(clippy::undocumented_unsafe_blocks)]` in the kernel crate).
  State the actual invariant being relied on, not just "this is fine."
- **Don't add abstractions, feature flags, or defensive error handling for
  cases that can't happen.** Trust the kernel's own invariants; validate
  only at real boundaries (user input, external hardware state).
- **New board = new `rivet-bsp-*` crate, not a kernel change.** If you find
  yourself needing to modify `rivet`, `rivet-arch-*`, or `rivet-rt` to
  bring up a board, that's very possibly a real kernel bug worth fixing,
  but treat it as its own, clearly justified change, not a board-specific
  workaround bundled into the port. `docs/porting.md` walks through this
  with three real worked examples.

## Testing expectations

Match the level of verification to what you changed:

- **Pure kernel logic** (`rivet/src/**`, no arch/board involved): the host
  test suite must pass, `cargo test -p rivet` (debug, `--release`, and
  `--profile release-checked`; release-mode-only UB is a real category of
  bug this project has hit). New scheduler/sync-primitive logic should get
  a `loom` test if there's any cross-task/cross-hart interaction, and a
  `proptest` case if there's a property worth stating generally rather
  than as one example.
- **Arch/board code**: the QEMU golden-output suite for every affected
  board (`cargo xtask test --target <board> --suite smoke`, plus `--suite
  stress`/`--suite gdb` if you touched the scheduler or context switch).
  `cargo xtask boards` lists what's registered.
- **Anything claiming real-hardware behavior**: actually run it on real
  hardware (ESP32-S3 or STM32F401RE, per what you're changing) and say so
  in the PR, with what you observed, not "should also work on hardware."
  If you don't have the hardware, say that explicitly rather than
  implying you tested something you didn't.
- **`cargo clippy --all-targets -- -D warnings`**, scoped per target (see
  `.github/workflows/ci.yml`: the kernel lints on host, everything
  arch/board-specific lints on its own real target, since it contains
  genuine architecture-specific assembly that can't build for host).

CI (`.github/workflows/ci.yml`) runs the host suite, `miri`, `loom`, the
full QEMU board matrix, and clippy per target. A PR that doesn't pass CI
locally first is a slower review, not a faster one.

### QEMU versions

CI installs QEMU unpinned (`apt-get install qemu-system-arm
qemu-system-misc` on `ubuntu-latest`), so the QEMU in use drifts as
runner images update. The board suite is verified against **QEMU 8.2.2**
(the maintainers' local baseline); newer QEMUs (10.2.x) log additional
machine-model guest-error lines on the ARM boards (e.g. `NVIC: Bad read
offset 0xdfc`, `PL011 data written to disabled UART`). These are
known-benign QEMU model quirks, not kernel bugs — the per-board
`ignore_log_lines` lists in `xtask/src/main.rs` exist to absorb them. If
a QEMU you're testing with logs lines the allowlists don't cover, add
them with a comment explaining why they're benign **and** an entry in
`tests/golden/KNOWN_FAILURES.md` (the project's convention: record, don't
silently hide) rather than chasing each one as a kernel regression.

## Commit and PR style

- Commit messages explain **why**, not just what changed. Match the
  existing log (`git log --oneline`) rather than generic "fix bug"/"update
  file" messages.
- One logical change per PR. A bug fix doesn't need an unrelated cleanup
  riding along with it, even a small one; split it.
- If your change resolves or narrows something in `docs/DOCUMENTATION.md`
  §18 (known limitations) or a `docs/wcet*.md` open item, update that
  document in the same PR. Stale limitations lists are exactly the kind
  of drift this project tries not to accumulate.

## Questions

Open an issue, or start the PR description with the question. A PR that's
genuinely "here's my attempt, not sure about X" is welcome and reviewed
differently from one presented as finished.
