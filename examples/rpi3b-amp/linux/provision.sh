#!/usr/bin/env bash
# Turn a stock Raspberry Pi OS card into a rivet-alongside-Linux system.
#
#   ./provision.sh /media/you/bootfs /media/you/rootfs [rivet.img]
#
# Run on the machine with the card reader, against a freshly imaged Pi OS
# (Lite, 64-bit) card that has been booted at least once so cloud-init has
# finished. Everything here is idempotent and every file it replaces is
# backed up alongside the original.
#
# This exists instead of shipping only a disk image because a disk image
# does not explain itself. Each step below is a decision with a reason,
# and several of them are not obvious:
#
#   - Core 3 is removed from the device tree outright. `maxcpus=3` is NOT
#     sufficient: arm64 still runs cpu_prepare for every CPU it
#     enumerates, and the spin-table implementation of that writes
#     Linux's own holding-pen address into the core's mailbox. The core
#     leaves the firmware's pen, jumps into Linux's, and waits there
#     forever for a release maxcpus has guaranteed will never come.
#
#   - Memory is carved out with a /reserved-memory node marked `no-map`
#     rather than with `mem=`. Same protection, but `mem=` makes Linux
#     ignore *everything* above the line, which threw away 157 MiB.
#
#   - force_turbo pins the ARM clock. BCM2837 has one clock domain for
#     the whole cluster and Linux owns the cpufreq driver, so otherwise
#     the real-time core's speed is decided by what the other three are
#     doing. It matters for latency more than throughput: frequency
#     transitions stall the core while the PLL relocks, and those were
#     the dominant source of worst-case interrupt latency.
set -euo pipefail

BOOT="${1:?usage: provision.sh <bootfs> <rootfs> [rivet.img]}"
ROOT="${2:?usage: provision.sh <bootfs> <rootfs> [rivet.img]}"
IMG="${3:-}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Physical layout. Must match RIVET_RPI3B_LOAD_ADDR and shmem::SHARED_BASE.
RIVET_BASE=0x30000000
RIVET_SIZE=0x1200000     # 16 MiB image window + 2 MiB shared rings

say() { printf '  %s\n' "$*"; }
backup() { [ -f "$1.stock" ] || sudo cp -p "$1" "$1.stock"; }

command -v fdtput >/dev/null || {
    echo "need device-tree-compiler (fdtput)" >&2
    exit 1
}

echo "== device tree =="
SRC="$BOOT/bcm2710-rpi-3-b.dtb"
OUT="$BOOT/bcm2710-rpi-3-b-rivet.dtb"
backup "$SRC"
TMP=$(mktemp /tmp/rivet-dtb.XXXXXX)
cp "$SRC.stock" "$TMP"
chmod u+w "$TMP"
# arm64 enumerates CPUs by walking /cpus and does not consult `status`,
# so disabling the node is not enough; it has to be gone.
fdtput -r "$TMP" /cpus/cpu@3 2>/dev/null || say "cpu@3 already absent"
fdtput -p -t x "$TMP" /reserved-memory/rivet@30000000 reg $RIVET_BASE $RIVET_SIZE
fdtput -t s "$TMP" /reserved-memory/rivet@30000000 no-map ""
# The layout, in the one place both halves of the system can read it.
# These numbers used to be declared independently in the board crate, the
# cargo config, the Linux loader and this script, with nothing checking
# they agreed, so a change to one produced silent corruption rather than
# an error. rivet-amp reads them from here at run time.
fdtput -t s "$TMP" /reserved-memory/rivet@30000000 compatible "rivet,amp-core"
fdtput -t u "$TMP" /reserved-memory/rivet@30000000 rivet,core 3
# Decimal, deliberately. fdtput -t u accepts a 0x literal without
# complaint and stores zero, which produced a shared window pointing at
# the image's own base and a loader that sat waiting for a ring that was
# never going to appear there.
fdtput -t u "$TMP" /reserved-memory/rivet@30000000 rivet,shared-offset 16777216
fdtput -t u "$TMP" /reserved-memory/rivet@30000000 rivet,tick-hz 10000
fdtput -t u "$TMP" /reserved-memory/rivet@30000000 rivet,abi 1
sudo cp "$TMP" "$OUT"
rm -f "$TMP"
say "cpus now: $(fdtget -l "$OUT" /cpus | tr '\n' ' ')"
say "reserved: $(fdtget -l "$OUT" /reserved-memory | tr '\n' ' ')"

echo "== config.txt =="
backup "$BOOT/config.txt"
add_cfg() { grep -qxF "$1" "$BOOT/config.txt" || echo "$1" | sudo tee -a "$BOOT/config.txt" >/dev/null; }
grep -q "rivet: begin" "$BOOT/config.txt" || sudo tee -a "$BOOT/config.txt" >/dev/null <<'CFG'

# rivet: begin
CFG
add_cfg "device_tree=bcm2710-rpi-3-b-rivet.dtb"
add_cfg "enable_uart=1"
add_cfg "dtoverlay=disable-bt"
# No over_voltage, so this does not set the OTP warranty bit.
add_cfg "force_turbo=1"
say "$(grep -c . "$BOOT/config.txt") lines, rivet settings applied"

echo "== cmdline.txt =="
backup "$BOOT/cmdline.txt"
LINE=$(tr -d '\n' < "$BOOT/cmdline.txt")
# mem= is replaced by the reserved-memory node; strip it if an older
# provisioning run left one behind.
LINE=$(echo "$LINE" | sed 's/ *\bmem=[0-9]*[MG]\b//g')
grep -qw "maxcpus=3" <<<"$LINE" || LINE="$LINE maxcpus=3"
echo "$LINE" | sudo tee "$BOOT/cmdline.txt" >/dev/null
say "$(cat "$BOOT/cmdline.txt")"

echo "== rivet payload and services =="
sudo mkdir -p "$ROOT/usr/local/lib/rivet" "$ROOT/usr/local/bin"
sudo cp "$HERE/rivet-amp.c" "$ROOT/usr/local/lib/rivet/"
sudo install -m755 "$HERE/rivet-select" "$ROOT/usr/local/bin/rivet-select"
sudo install -m755 "$HERE/rivet" "$ROOT/usr/local/bin/rivet"
sudo install -m755 "$HERE/rivet-identity.sh" "$ROOT/usr/local/lib/rivet/"
sudo mkdir -p "$ROOT/usr/local/share/doc/rivet"
for d in "$HERE/../README.md" "$HERE/../../../docs/rpi3b-benchmarks.md" \
         "$HERE/../../../docs/rpi3b-guarantees.md"; do
    [ -f "$d" ] && sudo cp "$d" "$ROOT/usr/local/share/doc/rivet/"
done

# Images are installed under their own names and the one that boots is a
# symlink, because a released core cannot be restarted in place: switching
# images means rebooting, so the boot service has to be told which one to
# take. See rivet-select.
if [ -n "$IMG" ]; then
    if [ -d "$IMG" ]; then
        sudo cp "$IMG"/*.img "$ROOT/usr/local/lib/rivet/"
        say "images: $(ls "$IMG"/*.img | wc -l) installed"
        DEFAULT=$(ls "$IMG"/channel_demo.img 2>/dev/null || ls "$IMG"/*.img | head -1)
    else
        sudo cp "$IMG" "$ROOT/usr/local/lib/rivet/"
        DEFAULT="$IMG"
    fi
    sudo ln -sfn "/usr/local/lib/rivet/$(basename "$DEFAULT")" \
                 "$ROOT/usr/local/lib/rivet/rivet.img"
    say "boot image: $(basename "$DEFAULT" .img)"
fi

for u in "$HERE"/units/*; do
    sudo install -m644 "$u" "$ROOT/etc/systemd/system/$(basename "$u")"
done

# Enable without systemctl, since the target root is not running.
W="$ROOT/etc/systemd/system/multi-user.target.wants"
sudo mkdir -p "$W"
for u in rivet-build rivet rivet-console rivet-health rivet-perf rivet.target; do
    case "$u" in
      *.target) sudo ln -sf "/etc/systemd/system/$u" "$W/$u" ;;
      *)        sudo ln -sf "/etc/systemd/system/$u.service" "$W/$u.service" ;;
    esac
done
say "services enabled: rivet-build, rivet, rivet-console, rivet-health, rivet-perf"

echo "== identity =="
sudo "$HERE/rivet-identity.sh" "$ROOT" "${RIVET_SYSTEM_VERSION:-0.3.0}"

echo
echo "Done. Boot it, then:"
echo "  journalctl -b -u rivet -u rivet-console   # what the RTOS said"
echo "  sudo rivet-amp send ping                  # talk to it"
echo "  sudo rivet-amp trace /tmp/rivet.ptrace    # capture frames"
