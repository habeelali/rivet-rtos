//! Selects the linker script published by this binary's board dependency
//! (`rivet-bsp-esp32c6`). Same ~5-line snippet as every other board's
//! example — see rivet-bsp-esp32c6/build.rs and docs/porting.md.
fn main() {
    let script = std::env::var("DEP_RIVET_BSP_ESP32C6_LINKER_SCRIPT")
        .expect("rivet-bsp-esp32c6 must be a dependency (missing `links` metadata)");
    println!("cargo:rustc-link-arg-bins=-T{script}");
    println!("cargo:rerun-if-changed={script}");
}
