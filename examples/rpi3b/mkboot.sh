#!/usr/bin/env bash
# Stage a bootable Raspberry Pi 3B boot partition for a bare-metal rivet
# image, into a plain directory. Copy the result onto a FAT32 partition
# and the board will boot it.
#
#   ./mkboot.sh [binary] [outdir]
#
# Defaults to the `bringup` binary and ./boot-staging.
#
# The firmware blobs come from the raspberrypi/firmware repository, all
# pinned to one commit: start.elf and fixup.dat are a matched pair and
# mixing versions across them is a documented way to end up with a board
# that does not boot at all.
set -euo pipefail

BIN="${1:-bringup}"
OUT="${2:-boot-staging}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
ELF="$ROOT/target/aarch64-unknown-none/release/$BIN"

# Pin to a single firmware commit. Bump deliberately, never implicitly.
FW_REF="${FW_REF:-master}"
FW_RAW="https://raw.githubusercontent.com/raspberrypi/firmware"

echo "==> building $BIN"
(cd "$HERE" && cargo build --release --bin "$BIN")

echo "==> resolving firmware ref '$FW_REF'"
FW_SHA="$(curl -sSL "https://api.github.com/repos/raspberrypi/firmware/commits/$FW_REF" \
          | sed -n 's/.*"sha": "\([0-9a-f]\{40\}\)".*/\1/p' | head -1)"
if [ -z "$FW_SHA" ]; then
    echo "could not resolve firmware commit for '$FW_REF'" >&2
    exit 1
fi
echo "    $FW_SHA"

mkdir -p "$OUT/overlays"

echo "==> fetching firmware"
for f in bootcode.bin start.elf fixup.dat bcm2710-rpi-3-b.dtb; do
    printf '    %-24s' "$f"
    curl -sSL --fail -o "$OUT/$f" "$FW_RAW/$FW_SHA/boot/$f"
    printf '%s bytes\n' "$(wc -c < "$OUT/$f")"
done
printf '    %-24s' "overlays/disable-bt.dtbo"
curl -sSL --fail -o "$OUT/overlays/disable-bt.dtbo" \
    "$FW_RAW/$FW_SHA/boot/overlays/disable-bt.dtbo"
printf '%s bytes\n' "$(wc -c < "$OUT/overlays/disable-bt.dtbo")"

echo "==> objcopy -> kernel8.img"
rust-objcopy -O binary "$ELF" "$OUT/kernel8.img"

cat > "$OUT/config.txt" <<'EOF'
# Raspberry Pi 3B (BCM2837), bare-metal AArch64.

arm_64bit=1
kernel=kernel8.img
# The firmware picks a load address by sniffing for the arm64 Linux Image
# header, which a flat binary does not have. Rather than depend on which
# default that lands on, pin it to match the linker script.
kernel_address=0x80000

# Move the PL011 off the Bluetooth modem and onto GPIO14/15. Overlays are
# applied by start.elf before any kernel is loaded, so this works exactly
# the same for a bare-metal image as it does for Linux -- but it needs
# the .dtb and the .dtbo present on the partition, which is why both are
# staged above.
dtoverlay=disable-bt
enable_uart=1
init_uart_clock=48000000
init_uart_baud=115200

# Make start.elf log its own boot over the same pins, before our image
# runs. This is the difference between "silence, cause unknown" and
# "firmware spoke, then our kernel went quiet".
uart_2ndstage=1

# Pin the VPU core clock. The PL011 does not care, but the mini UART
# derives its baud rate from this, so pinning it keeps the fallback path
# valid too.
core_freq=250
core_freq_min=250
force_turbo=0

disable_splash=1
boot_delay=0
EOF

echo
echo "staged in $OUT:"
ls -l "$OUT"
echo
echo "Copy the contents onto a FAT32 (MBR, type 0x0C) first partition."
