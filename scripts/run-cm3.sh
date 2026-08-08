#!/usr/bin/env bash
# Build and run the Cortex-M3 QEMU example.
# Requires: rustup target add thumbv7m-none-eabi
#           apt install qemu-system-arm

set -e
cd "$(dirname "$0")/.."

if ! command -v qemu-system-arm &>/dev/null; then
  echo "qemu-system-arm not found. Install with: sudo apt install qemu-system-arm" >&2
  exit 1
fi

echo "Building qemu-cm3 for thumbv7m-none-eabi..."
cargo build --package qemu-cm3 --target thumbv7m-none-eabi --release

ELF="target/thumbv7m-none-eabi/release/qemu-cm3"

echo "Running in QEMU (lm3s6965evb, Cortex-M3)..."
echo "Expected: priority inheritance proof, interleaved A/B preemption, async producer/consumer sum=15, SUCCESS, exit 0"
echo "---"

exec qemu-system-arm \
  -machine lm3s6965evb \
  -kernel "$ELF" \
  -nographic \
  -semihosting
