//! Exposes this board's linker script path to whatever final binary
//! depends on this crate.
//!
//! Cargo's `cargo:rustc-link-arg` is only honored when emitted by the
//! *final artifact's own* build script — it does not propagate up from a
//! library dependency's build script (a real Cargo limitation, not a
//! design choice here). So this crate can't select the linker script by
//! itself; instead it publishes the path via the standard `links` +
//! `cargo:KEY=VALUE` mechanism (this crate's `Cargo.toml` sets `links =
//! "rivet_bsp_qemu_virt"`), which Cargo exposes to *direct* dependents'
//! build scripts as `DEP_RIVET_BSP_QEMU_VIRT_LINKER_SCRIPT`. See
//! `docs/porting.md` for the five-line build.rs a binary crate needs to
//! consume it (the same snippet works for any board — only the `links`
//! name changes).
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let script = manifest_dir.join("link-qemu-virt.ld");
    println!("cargo:linker-script={}", script.display());
    println!("cargo:rerun-if-changed={}", script.display());
}
