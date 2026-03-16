#!/usr/bin/env bash
# Build ESP32-C3 example (UART logs). Ready to flash; does not flash.
# Requires: rustup target add riscv32imc-unknown-none-elf
#
# To flash and monitor (device on /dev/ttyACM0):
#   espflash flash --monitor target/riscv32imc-unknown-none-elf/release/examples/esp32c3_uart --port /dev/ttyACM0
#
# Or: cargo install espflash && ./scripts/build-esp32c3.sh && espflash flash --monitor target/.../esp32c3_uart --port /dev/ttyACM0

set -e
cd "$(dirname "$0")/.."

echo "Building esp32c3_uart for ESP32-C3 (riscv32imc-unknown-none-elf + esp32c3 feature)..."
cargo build --example esp32c3_uart --target riscv32imc-unknown-none-elf --release --features esp32c3

ELF="target/riscv32imc-unknown-none-elf/release/examples/esp32c3_uart"
echo "Done. ELF: $ELF"
echo "To flash and open serial monitor (e.g. /dev/ttyACM0):"
echo "  espflash flash --monitor $ELF --port /dev/ttyACM0"
