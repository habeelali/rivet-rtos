//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-lm3s6965`). See examples/qemu-riscv/build.rs and
//! docs/porting.md.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_LM3S6965_LINKER_SCRIPT")
        .expect("rivet-bsp-lm3s6965 must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
