//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-mps2-an385`). See examples/qemu-riscv/build.rs and
//! docs/porting.md.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_MPS2_AN385_LINKER_SCRIPT")
        .expect("rivet-bsp-mps2-an385 must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
