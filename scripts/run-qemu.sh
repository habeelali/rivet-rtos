#!/usr/bin/env bash
# Build and run the RISC-V QEMU example.
# Requires: rustup target add riscv32imac-unknown-none-elf
#           apt install qemu-system-misc

set -e
cd "$(dirname "$0")/.."

if ! command -v qemu-system-riscv32 &>/dev/null; then
  echo "qemu-system-riscv32 not found. Install with: sudo apt install qemu-system-misc" >&2
  exit 1
fi

echo "Building qemu-riscv for riscv32imac-unknown-none-elf..."
cargo build --package qemu-riscv --target riscv32imac-unknown-none-elf --release

ELF="target/riscv32imac-unknown-none-elf/release/qemu-riscv"

echo "Running in QEMU (virt, RISC-V)..."
echo "Expected: priority inheritance proof, interleaved A/B preemption, async producer/consumer sum=15, SUCCESS, exit 0"
echo "---"

exec qemu-system-riscv32 \
  -machine virt \
  -cpu rv32 \
  -bios none \
  -kernel "$ELF" \
  -nographic \
  -serial mon:stdio \
  -semihosting
