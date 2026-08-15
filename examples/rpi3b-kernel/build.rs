//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-rpi3b`). See examples/rpi3b/build.rs.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_RPI3B_LINKER_SCRIPT")
        .expect("rivet-bsp-rpi3b must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
