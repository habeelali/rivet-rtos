#!/usr/bin/env bash
# Build the QEMU RISC-V example and run it in QEMU (virt machine, semihosting).
# Requires: rustup target add riscv32imc-unknown-none-elf
#           qemu-system-riscv32 (e.g. apt install qemu-system-misc)

set -e
cd "$(dirname "$0")/.."

if ! command -v qemu-system-riscv32 &>/dev/null; then
  echo "qemu-system-riscv32 not found. Install with: apt install qemu-system-misc" >&2
  exit 1
fi

echo "Building qemu_riscv for riscv32imc-unknown-none-elf..."
cargo build --example qemu_riscv --target riscv32imc-unknown-none-elf --release

ELF="target/riscv32imc-unknown-none-elf/release/examples/qemu_riscv"

echo "Running in QEMU (virt; UART -> terminal, semihosting for exit)..."
exec qemu-system-riscv32 \
  -machine virt \
  -cpu rv32 \
  -bios none \
  -kernel "$ELF" \
  -nographic \
  -serial mon:stdio \
  -semihosting
