//! Exposes this board's linker script path to whatever final binary
//! depends on this crate (see rivet-bsp-qemu-virt/build.rs for why this
//! can't just call `cargo:rustc-link-arg` itself, and docs/porting.md for
//! the consuming snippet).
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("link-stm32f401re.ld");
    println!("cargo:linker-script={}", script.display());
    println!("cargo:rerun-if-changed={}", script.display());
}
