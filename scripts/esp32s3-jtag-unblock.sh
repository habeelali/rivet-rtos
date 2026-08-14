#!/usr/bin/env bash
# Unblocks ESP32-S3 boot on boards where the mask ROM's own main()
# reads the CPU's PRID special register as 0xabab (the value ROM uses
# to mean "running under a simulator") and, as a result, spins forever
# in main() waiting for a debugger to write the app entry point to a
# fixed hardware register (0x600c0004) instead of loading firmware from
# SPI flash normally.
#
# Confirmed on real hardware (N16R8 devkit, chip revision v0.2) this
# session: reproduces identically after a full USB power cycle with no
# debugger ever attached, so it is a genuine, persistent condition of
# this board/chip combination, not a JTAG-session artifact. Root cause
# is inside Espressif's mask ROM, before any of our code runs — nothing
# in this repo can fix it; this script works around it every boot.
#
# Usage: scripts/esp32s3-jtag-unblock.sh <path-to-elf>
#
# Requires: openocd (mainline 0.12+, esp_usb_jtag driver), and the esp
# rustup toolchain's xtensa-esp32s3-elf-readelf on PATH (`source
# ~/export-esp.sh`).

set -euo pipefail

ELF="${1:?usage: $0 <path-to-elf>}"
OCD_CFG="$(dirname "$0")/esp32s3-builtin.cfg"

ENTRY=$(xtensa-esp32s3-elf-readelf -h "$ELF" | awk '/Entry point address/ {print $NF}')
echo "Entry point: $ENTRY"

openocd -f "$OCD_CFG" -c "gdb_port 3334" -c "telnet_port 4445" -c "tcl_port 6667" \
  -c "init" -c "reset run" -c "sleep 400" -c "halt" \
  -c "mww 0x600c0004 $ENTRY" \
  -c "resume" -c "shutdown"
