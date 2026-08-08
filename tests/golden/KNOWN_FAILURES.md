# Known pre-existing failures (baseline, before layering refactor)

Captured 2026-08-08, before any layering changes. These are pre-existing bugs
unrelated to the RTOS/BSP layering work and are explicitly out of scope for
that refactor. They are recorded here so later phases can confirm they fail
**identically** (not worse, not differently) rather than being silently
papered over.

## cm3/fault_overflow

`examples/qemu-cm3/src/bin/fault_overflow.rs` overflows a 512-byte task stack
with a 2048-byte local array. The write drives PSP ~1536 bytes into the MPU
deny region (region 6, the whole `.task_stacks` pool minus the current
task's region 7). On Cortex-M, exception entry auto-stacks {r0-r3, r12, lr,
pc, xPSR} onto whatever stack was active pre-exception (PSP, since the
kernel runs tasks in Thread mode on PSP) — and that auto-stack push lands at
the same (already-denied) SP, so the MemManage fault's own entry stacking is
*also* denied. Per ARMv7-M, a fault that recurs during a fault handler's own
exception-entry sequence escalates to HardFault rather than delivering a
clean, diagnosable MemManage.

Observed: `Rivet CM3 fault_overflow\nabout to overflow a stack\nHARD_FAULT\n`
then the CM3 `HardFault` handler spins forever -> 30s xtask timeout.
Expected (never delivered): `RIVET FAULT ... memmanage ... RIVET_FAILURE code=250`.

This is a real gap in the two-region MPU design (old plan.md §3.1): it can
cleanly catch a *shallow* overflow (a few bytes past the guard) but not a
*deep* one that also swallows the fault entry's own stacking. Fixing it
would need either a dedicated Handler-mode-only stacking path (impossible on
armv7-M for the initial hardware frame) or a much larger guard band plus
stack-limit pre-checking. Left unfixed here; tracked for a future
fault-isolation hardening pass, not the layering overhaul.

Status after each phase of the layering refactor: **must still fail exactly
this way** (same three lines, same HardFault, same timeout). If it starts
passing or fails differently, that is a real regression signal worth
investigating (could indicate the refactor accidentally changed MPU
programming or stack layout).

## cm3/fault_isolate

Same root cause as `cm3/fault_overflow` above (deep stack overflow -> PSP
lands in the MPU deny region -> the MemManage handler's own hardware
auto-stack-push also lands there -> ARMv7-M escalates to HardFault instead
of delivering MemManage). Observed:
`Rivet CM3 fault_isolate\nfaulting task will be isolated\nHARD_FAULT\n`,
then hangs to the 30s timeout. Not fixed here; same disposition as
`fault_overflow`.

## cm3/stress_max_ptasks

Fails with `HARD_FAULT` shortly after boot (`Rivet CM3 stress_max_ptasks\n`
then `HARD_FAULT\n`, hangs to the 40s timeout). Root cause not
investigated — distinct symptom from the overflow tests above (this test
does not deliberately overflow anything; it fills the task registry to
`MAX_PTASKS` with plain 512-byte worker stacks, well within the 16 KiB
pool by a naive size sum). Left as a pre-existing, unexplained failure;
out of scope for the layering refactor. Flagged for a future investigation
pass (possibly stack-pool alignment padding, or an MPU region-7
reprogramming race under rapid spawn).

## Fixed during Phase 1 (harness bugs, not kernel bugs)

Two `xtask` test-table typos were corrected as part of establishing this
baseline (metadata-only, zero effect on kernel/example code):

- `cm3/join_test` was missing `extra_qemu_args: &["-machine", "lm3s6965evb"]`
  entirely, so QEMU exited immediately with "No machine specified" (exit
  code 1, zero bytes of guest output) — not a kernel issue, a missing test
  declaration.
- `cm3/join_test` and `cm3/respawn_test` had `ignore_log_lines: &["Timer
  with period zero, disabled"]` (past tense) but QEMU's stellaris-watchdog
  model actually emits "Timer with period zero, **disabling**" (present
  participle) — the mismatched string meant the benign line was never
  filtered, failing the qemu.log-must-be-clean assertion.

Both now pass. These are recorded here (not silently fixed) because they
change which tests are green in the "before" baseline used for regression
diffing in later phases.
