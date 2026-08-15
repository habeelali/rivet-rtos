//! Generates this board's linker script and exposes its path to whatever
//! final binary depends on this crate (see rivet-bsp-qemu-virt/build.rs
//! for why this can't just call `cargo:rustc-link-arg` itself, and
//! docs/porting.md for the consuming snippet).
//!
//! The load address is a build parameter rather than a constant, because
//! where this image lives depends on who else is in memory:
//!
//! - Alone on the board, the firmware loads it at `0x80000`, which is
//!   what `config.txt`'s `kernel_address` pins.
//! - Alongside Linux, it has to sit in memory Linux was told not to
//!   touch, well above its own kernel image.
//!
//! Override with `RIVET_RPI3B_LOAD_ADDR` and `RIVET_RPI3B_RAM_LEN`.
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let template = manifest_dir.join("link-rpi3b.ld.in");

    let load_addr =
        std::env::var("RIVET_RPI3B_LOAD_ADDR").unwrap_or_else(|_| "0x80000".to_string());
    let ram_len = std::env::var("RIVET_RPI3B_RAM_LEN").unwrap_or_else(|_| "16M".to_string());

    let script_text = std::fs::read_to_string(&template)
        .expect("link-rpi3b.ld.in")
        .replace("@LOAD_ADDR@", &load_addr)
        .replace("@RAM_LEN@", &ram_len);

    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("link-rpi3b.ld");
    std::fs::write(&out, script_text).expect("write linker script");

    println!("cargo:linker-script={}", out.display());
    println!("cargo:rerun-if-changed={}", template.display());
    println!("cargo:rerun-if-env-changed=RIVET_RPI3B_LOAD_ADDR");
    println!("cargo:rerun-if-env-changed=RIVET_RPI3B_RAM_LEN");
    // Let the code see where it was linked, so it can sanity-check that
    // against where it actually ended up running.
    println!("cargo:rustc-env=RIVET_RPI3B_LOAD_ADDR={load_addr}");
}
