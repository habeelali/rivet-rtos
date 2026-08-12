//! Exposes this board's linker script path to whatever final binary
//! depends on this crate — identical pattern to
//! rivet-bsp-stm32f401re/build.rs.
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("link-rp2040.ld");
    println!("cargo:linker-script={}", script.display());
    println!("cargo:rerun-if-changed={}", script.display());
}
