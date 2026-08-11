//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-stm32f401re`). See examples/qemu-cm3/build.rs.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_STM32F401RE_LINKER_SCRIPT")
        .expect("rivet-bsp-stm32f401re must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
