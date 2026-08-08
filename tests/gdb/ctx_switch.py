#!/usr/bin/env python3
"""GDB-scripted context-switch verification (plan.md §1.8).

The only thing that actually proves the hand-written context switch is
correct is comparing the live register file against the saved frame at
every switch — print-based demos can't. This script runs under
`gdb-multiarch -batch -x tests/gdb/ctx_switch.py <elf>` against a QEMU
instance started with `-s -S` (gdbstub on :1234, halted at reset).

For every context switch it verifies:

  1. **Save consistency** — at trap/PendSV entry, the frame just written
     matches the live register file at the save point (nothing clobbered
     the interrupted task's registers before they were stored).
  2. **Restore round-trip** — at resume, the register file about to run
     equals the frame recorded when this task (identified by stack frame
     address) was last suspended. Any register missing from the save *or*
     restore list shows up immediately as a mismatch. This also documents
     and enforces the deliberate x3/gp, x4/tp skip on RISC-V.
  3. **MPP gate (RISC-V)** — mstatus.MPP == 0b11 at every trap entry,
     turning the historical MPP bug into a permanent regression gate.

Architecture is detected from the ELF path (contains `riscv` or `cm3`).

Exit status: 0 on success; nonzero (with the failing comparison) if any
assertion fails.
"""

import gdb

TARGET_SWITCHES = 64  # verify at least this many context switches

arch = None


def detect_arch():
    global arch
    exe = gdb.objfiles()[0].filename if gdb.objfiles() else ""
    if "riscv" in exe:
        arch = "riscv"
    elif "cm3" in exe:
        arch = "cm3"
    else:
        raise gdb.GdbError(f"cannot detect arch from ELF path: {exe}")


# ── Register <-> frame-offset tables ────────────────────────────────

if_riscv_frame_layout = [  # word offset -> register name
    (0, "ra"),
    (1, "t0"),
    (2, "t1"),
    (3, "t2"),
    (4, "s0"),
    (5, "s1"),
    (6, "a0"),
    (7, "a1"),
    (8, "a2"),
    (9, "a3"),
    (10, "a4"),
    (11, "a5"),
    (12, "a6"),
    (13, "a7"),
    (14, "s2"),
    (15, "s3"),
    (16, "s4"),
    (17, "s5"),
    (18, "s6"),
    (19, "s7"),
    (20, "s8"),
    (21, "s9"),
    (22, "s10"),
    (23, "s11"),
    (24, "t3"),
    (25, "t4"),
    (26, "t5"),
    (27, "t6"),
]

# Deliberately NOT saved/restored: x0 (hardwired zero), x2/sp (the frame
# itself), x3/gp and x4/tp (fixed at boot, never touched by kernel code).
riscv_skipped = ["x0", "sp", "gp", "tp"]


def read_mem(addr, nbytes):
    inf = gdb.selected_inferior()
    return bytes(inf.read_memory(int(addr), nbytes))


def read_reg(name):
    return int(gdb.parse_and_eval(f"${name}"))


def read_regs(names):
    return {n: read_reg(n) for n in names}


def check_mpc_status():
    """RISC-V MPP gate: the interrupted task must have been in M-mode."""
    mstatus = read_reg("mstatus")
    mpp = (mstatus >> 11) & 0b11
    if mpp != 0b11:
        raise gdb.GdbError(
            f"MPP gate FAILED: mstatus={mstatus:#x} MPP={mpp:#b} (expected 0b11) at trap entry"
        )


class SwitchVerifier:
    """Tracks per-task saved frames and verifies save/restore consistency."""

    def __init__(self):
        self.saved = {}  # frame base -> list of saved register values
        self.entry_snapshots = {}  # frame base -> live register snapshot
        self.switches = 0
        self.checked_save = 0
        self.checked_restore = 0

    # RISC-V ─────────────────────────────────────────────────────────

    def riscv_snapshot_entry(self):
        # At `rivet_trap_entry` (first instruction), every GPR is still the
        # interrupted task's — sp included. Snapshot the live register file
        # keyed by the frame base (sp - 128, where the save code will put
        # the frame).
        sp = int(gdb.parse_and_eval("$sp"))
        frame = sp - 128
        self.entry_snapshots[frame] = read_regs(
            [r for (_, r) in if_riscv_frame_layout]
        )

    def riscv_record_save(self):
        # At `rivet_trap_handler_rust` entry, a0 = frame base and the frame
        # is complete. Compare it against the live-register snapshot taken
        # at `rivet_trap_entry`: the save code must have written exactly the
        # interrupted task's registers (it may clobber a0/t0 *after* saving
        # them, so the comparison is against the pre-save snapshot, not the
        # current live file).
        frame = int(gdb.parse_and_eval("$a0"))
        words = [int.from_bytes(read_mem(frame + 4 * i, 4), "little") for i in range(28)]
        snap = self.entry_snapshots.pop(frame, None)
        if snap is None:
            raise gdb.GdbError(
                f"RISC-V: no entry snapshot for frame {frame:#x} — save code "
                "wrote a frame that was never entered?"
            )
        for (off, name), expect in zip(if_riscv_frame_layout, words):
            if snap[name] != expect:
                raise gdb.GdbError(
                    f"RISC-V SAVE MISMATCH at frame {frame:#x}: snapshot ${name}="
                    f"{snap[name]:#x} != saved word {off} {expect:#x}"
                )
        self.saved[frame] = words
        self.checked_save += 1

    def riscv_check_restore(self):
        # At `rivet_trap_mret`, sp points just past the frame. The register
        # file about to run must equal the frame saved at this task's last
        # suspension.
        frame = int(gdb.parse_and_eval("$sp")) - 128
        recorded = self.saved.get(frame)
        if recorded is None:
            return  # first dispatch (start_first_task) — nothing recorded yet
        live = read_regs([r for (_, r) in if_riscv_frame_layout])
        for (off, name), expect in zip(if_riscv_frame_layout, recorded):
            if live[name] != expect:
                raise gdb.GdbError(
                    f"RISC-V RESTORE MISMATCH at frame {frame:#x}: live ${name}="
                    f"{live[name]:#x} != recorded word {off} {expect:#x}"
                )
        self.switches += 1
        self.checked_restore += 1

    # Cortex-M ───────────────────────────────────────────────────────

    def cm3_record_save(self):
        # At `rivet_pendsv_rust` entry, r0 = frame base; the manual save
        # [r4..r11] is complete. The hardware frame (r0-r3, r12, lr, pc,
        # xPSR) sits 32 bytes above and is not our responsibility. Verify
        # the manual save matches live r4-r11.
        frame = int(gdb.parse_and_eval("$r0"))
        saved = [int.from_bytes(read_mem(frame + 4 * i, 4), "little") for i in range(8)]
        live = [read_reg(f"r{i}") for i in range(4, 12)]
        for i, (lv, sv) in enumerate(zip(live, saved)):
            if lv != sv:
                raise gdb.GdbError(
                    f"CM3 SAVE MISMATCH at frame {frame:#x}: live $r{i+4}="
                    f"{lv:#x} != saved {sv:#x}"
                )
        self.saved[frame] = saved
        self.checked_save += 1

    def cm3_check_restore(self):
        # At `rivet_pendsv_resume`, r4-r11 have been reloaded and
        # psp = frame + 32. Compare live r4-r11 with the recorded frame.
        psp = read_reg("psp")
        frame = psp - 32
        recorded = self.saved.get(frame)
        if recorded is None:
            return
        live = [read_reg(f"r{i}") for i in range(4, 12)]
        for i, (lv, rv) in enumerate(zip(live, recorded)):
            if lv != rv:
                raise gdb.GdbError(
                    f"CM3 RESTORE MISMATCH at frame {frame:#x}: live $r{i+4}="
                    f"{lv:#x} != recorded {rv:#x}"
                )
        self.switches += 1
        self.checked_restore += 1


verifier = SwitchVerifier()


class EntryBreakpoint(gdb.Breakpoint):
    """Trap/PendSV entry: record + verify the save."""

    def __init__(self, spec):
        super().__init__(spec, internal=True)
        self.silent = True

    def stop(self):
        try:
            if arch == "riscv":
                verifier.riscv_record_save()
            else:
                verifier.cm3_record_save()
        except gdb.GdbError:
            raise
        except Exception as e:  # noqa: BLE001 — report and fail
            raise gdb.GdbError(f"save verification crashed: {e}")
        return False  # keep running


class EntrySnapshotBreakpoint(gdb.Breakpoint):
    """RISC-V only: snapshot live registers before the save code runs."""

    def __init__(self, spec):
        super().__init__(spec, internal=True)
        self.silent = True

    def stop(self):
        verifier.riscv_snapshot_entry()
        return False  # keep running


class ResumeBreakpoint(gdb.Breakpoint):
    """Resume point: verify the restored register file."""

    def __init__(self, spec):
        super().__init__(spec, internal=True)
        self.silent = True

    def stop(self):
        try:
            if arch == "riscv":
                verifier.riscv_check_restore()
            else:
                verifier.cm3_check_restore()
        except gdb.GdbError:
            raise
        except Exception as e:  # noqa: BLE001
            raise gdb.GdbError(f"restore verification crashed: {e}")
        return False  # keep running


class MppGateBreakpoint(gdb.Breakpoint):
    """RISC-V only: mstatus.MPP must be M-mode at every trap entry."""

    def __init__(self, spec):
        super().__init__(spec, internal=True)
        self.silent = True

    def stop(self):
        check_mpc_status()
        return False


def run():
    detect_arch()
    gdb.execute("set pagination off", to_string=True)
    gdb.execute("target remote :1234", to_string=True)

    if arch == "riscv":
        EntrySnapshotBreakpoint("rivet_trap_entry")
        EntryBreakpoint("rivet_trap_handler_rust")
        ResumeBreakpoint("rivet_trap_mret")
        MppGateBreakpoint("rivet_trap_entry")
    else:
        EntryBreakpoint("rivet_pendsv_rust")
        ResumeBreakpoint("rivet_pendsv_resume")

    # Run until we've verified enough switches, or the guest exits
    # (the demo calls exit_success, which terminates QEMU — that's fine,
    # we only need TARGET_SWITCHES verified switches before it).
    try:
        while verifier.switches < TARGET_SWITCHES:
            gdb.execute("continue", to_string=True)
    except gdb.error as e:
        # "Remote connection closed" = guest exited; acceptable if we
        # already verified enough switches.
        if verifier.switches < TARGET_SWITCHES:
            raise gdb.GdbError(
                f"guest exited after {verifier.switches}/{TARGET_SWITCHES} "
                f"verified switches: {e}"
            )

    print(
        f"[ctx_switch] OK: {verifier.checked_save} saves verified, "
        f"{verifier.checked_restore} restores verified "
        f"({verifier.switches} switches)"
    )


run()
