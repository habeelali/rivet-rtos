//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-qemu-virt`). This ~5-line snippet is the one bit of
//! per-binary wiring Cargo's `links`/`DEP_*` mechanism requires (a
//! library's own build script can't set linker args for a crate that
//! merely depends on it) — see rivet-bsp-qemu-virt/build.rs and
//! docs/porting.md. Swapping to a different board means changing the
//! `DEP_*` variable name here to match that board's `links` key, nothing
//! else.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_QEMU_VIRT_LINKER_SCRIPT")
        .expect("rivet-bsp-qemu-virt must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
