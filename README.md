# Rivet RTOS

A minimal real-time operating system for RISC-V microcontrollers, written in Rust (`no_std`). Currently targets ESP32-C3.

## Features

- **Kernel:** Round-robin scheduler, static tasks (up to 4), block/unblock, binary semaphore.
- **Arch:** RISC-V context switch (save/restore callee-saved + `sp`), critical section (disable/restore MIE).
- **Board:** ESP32-C3 port stub (install context switch, tick placeholder).

## Build

- **Host (for tests):** `cargo build`
- **RISC-V (e.g. ESP32-C3):**  
  `rustup target add riscv32imc-unknown-none-elf`  
  `cargo build --target riscv32imc-unknown-none-elf`

## Tests

Run on host (uses context-switch stub; no real preemption):

```bash
cargo test -- --test-threads=1
```

Use `--test-threads=1` because tests share kernel static state.

## QEMU (RISC-V)

Run the kernel on a RISC-V core in QEMU (two tasks, semaphore ping-pong, then exit via semihosting):

```bash
rustup target add riscv32imc-unknown-none-elf
# Install QEMU if needed: apt install qemu-system-misc
./scripts/run-qemu.sh
```

Or manually:

```bash
cargo build --example qemu_riscv --target riscv32imc-unknown-none-elf --release
qemu-system-riscv32 -machine virt -cpu rv32 -bios none -kernel target/riscv32imc-unknown-none-elf/release/examples/qemu_riscv -nographic -semihosting
```

QEMU should exit with code 0 if the test passes.

## ESP32-C3 (real hardware, UART logs)

**Note:** This RTOS is RISC-V. ESP32-**C3** is RISC-V; ESP32-**S3** is Xtensa and would need a separate port.

Build the example (ready to flash; does not flash):

```bash
rustup target add riscv32imc-unknown-none-elf
./scripts/build-esp32c3.sh
```

Or manually:

```bash
cargo build --example esp32c3_uart --target riscv32imc-unknown-none-elf --release --features esp32c3
```

Flash and open serial monitor (device on `/dev/ttyACM0`, 115200 8N1):

```bash
cargo install espflash   # once
espflash flash --monitor target/riscv32imc-unknown-none-elf/release/examples/esp32c3_uart --port /dev/ttyACM0
```

You should see UART logs (e.g. `Rivet RTOS ESP32-C3`, `Task 0 running`, `Task 1 running`). Panics are printed to UART.

## License

Licensed under the [MIT License](LICENSE).
