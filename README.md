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

## License

Licensed under the [MIT License](LICENSE).
