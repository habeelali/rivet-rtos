//! Exposes this board's linker script path to whatever final binary
//! depends on this crate. Same mechanism as every other rivet-bsp-*
//! crate — see rivet-bsp-qemu-virt/build.rs / docs/porting.md.
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("link-esp32c6.ld");
    println!("cargo:linker-script={}", script.display());
    println!("cargo:rerun-if-changed={}", script.display());
}
