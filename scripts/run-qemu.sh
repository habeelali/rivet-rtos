#!/usr/bin/env bash
# Build the QEMU RISC-V example and run it in QEMU (virt machine, semihosting).
# Usage: ./scripts/run-qemu.sh [demo|full|semaphore|preempt]
#   demo (default): qemu_riscv_demo — comprehensive visual demonstration (for presentation)
#   full:           qemu_riscv_full — full-depth test (priority, round-robin, semaphores, tick, 4 tasks)
#   semaphore:      qemu_riscv — semaphore block/signal only
#   preempt:        qemu_riscv_preempt — two tasks alternating via tick()
# Requires: rustup target add riscv32imc-unknown-none-elf
#           qemu-system-riscv32 (e.g. apt install qemu-system-misc)

set -e
cd "$(dirname "$0")/.."

if ! command -v qemu-system-riscv32 &>/dev/null; then
  echo "qemu-system-riscv32 not found. Install with: apt install qemu-system-misc" >&2
  exit 1
fi

case "${1:-demo}" in
  demo)       EXAMPLE=qemu_riscv_demo ;;
  full)       EXAMPLE=qemu_riscv_full ;;
  semaphore)  EXAMPLE=qemu_riscv ;;
  preempt)    EXAMPLE=qemu_riscv_preempt ;;
  *)
    echo "Usage: $0 [demo|full|semaphore|preempt]" >&2
    exit 1
    ;;
esac

echo "Building $EXAMPLE for riscv32imc-unknown-none-elf..."
cargo build --example "$EXAMPLE" --target riscv32imc-unknown-none-elf --release

ELF="target/riscv32imc-unknown-none-elf/release/examples/$EXAMPLE"

echo "Running in QEMU (virt; UART -> terminal, semihosting for exit)..."
exec qemu-system-riscv32 \
  -machine virt \
  -cpu rv32 \
  -bios none \
  -kernel "$ELF" \
  -nographic \
  -serial mon:stdio \
  -semihosting
