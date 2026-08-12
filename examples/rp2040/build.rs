//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-rp2040`). See examples/stm32f401re/build.rs.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_RP2040_LINKER_SCRIPT")
        .expect("rivet-bsp-rp2040 must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
