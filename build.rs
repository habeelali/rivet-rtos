// Select linker script: esp32c3 feature -> link-esp32c3.ld, else -> link-qemu.ld
fn main() {
    let link_script = if std::env::var("CARGO_FEATURE_ESP32C3").is_ok() {
        "link-esp32c3.ld"
    } else {
        "link-qemu.ld"
    };
    println!("cargo:rustc-link-arg=-T{}", link_script);
}
