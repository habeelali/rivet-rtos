#!/usr/bin/env python3
"""Coverage floor check (plan.md Phase 17).

Enforced only on the modules the *host* test suite can actually reach —
sched/waker/mutex/channel/timer — at a floor just above their measured
baseline (see plan.md Phase 9's coverage note for why the crate-wide
number isn't a meaningful gate: most of the gap is code the QEMU suite
exercises instead, which llvm-cov can't see). Everything else in the
report stays informational, exactly as the existing `coverage` CI job
already treats it.

Usage: cargo llvm-cov -p rivet-rtos --tests --json --output-path cov.json
       python3 scripts/check-coverage-floor.py cov.json
"""

import json
import sys

FLOOR_PERCENT = 70.0

# (path suffix to match, human label)
GATED_FILES = [
    ("preempt/sched.rs", "sched"),
    ("waker.rs", "waker"),
    ("preempt/mutex.rs", "mutex"),
    ("sync/channel.rs", "channel"),
    ("timer.rs", "timer"),
]


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <cov.json>", file=sys.stderr)
        return 2

    with open(sys.argv[1]) as f:
        report = json.load(f)

    files = report["data"][0]["files"]
    failures = []
    for suffix, label in GATED_FILES:
        matches = [f for f in files if f["filename"].replace("\\", "/").endswith(suffix)]
        if not matches:
            failures.append(f"{label}: no coverage data found for `{suffix}` (renamed/moved?)")
            continue
        pct = matches[0]["summary"]["lines"]["percent"]
        status = "OK" if pct >= FLOOR_PERCENT else "FAIL"
        print(f"[{status}] {label}: {pct:.2f}% (floor {FLOOR_PERCENT}%)")
        if pct < FLOOR_PERCENT:
            failures.append(f"{label}: {pct:.2f}% < {FLOOR_PERCENT}% floor")

    if failures:
        print("\nCoverage floor violations:", file=sys.stderr)
        for msg in failures:
            print(f"  - {msg}", file=sys.stderr)
        return 1

    print(f"\nAll gated modules meet the {FLOOR_PERCENT}% floor.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
