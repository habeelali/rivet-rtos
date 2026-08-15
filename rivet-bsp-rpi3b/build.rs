//! Generates this board's linker script and exposes its path to whatever
//! final binary depends on this crate (see rivet-bsp-qemu-virt/build.rs
//! for why this can't just call `cargo:rustc-link-arg` itself, and
//! docs/porting.md for the consuming snippet).
//!
//! The load address is a build parameter rather than a constant, because
//! where this image lives depends on who else is in memory:
//!
//! - Alone on the board, the firmware loads it at `0x80000`, which is
//!   what `config.txt`'s `kernel_address` pins.
//! - Alongside Linux, it has to sit in memory Linux was told not to
//!   touch, well above its own kernel image.
//!
//! Override with `RIVET_RPI3B_LOAD_ADDR` and `RIVET_RPI3B_RAM_LEN`.
use std::path::PathBuf;

/// Accepts plain, hex, and the K/M/G suffixes a linker script uses.
fn parse_num(s: &str) -> u64 {
    let s = s.trim();
    let (digits, mult) = match s.chars().last() {
        Some('K') | Some('k') => (&s[..s.len() - 1], 1024),
        Some('M') | Some('m') => (&s[..s.len() - 1], 1024 * 1024),
        Some('G') | Some('g') => (&s[..s.len() - 1], 1024 * 1024 * 1024),
        _ => (s, 1),
    };
    let digits = digits.trim();
    let v = if let Some(hex) = digits
        .strip_prefix("0x")
        .or_else(|| digits.strip_prefix("0X"))
    {
        u64::from_str_radix(hex, 16)
    } else {
        digits.parse::<u64>()
    };
    v.unwrap_or_else(|_| panic!("cannot parse number: {s}")) * mult
}

fn main() {
    let manifest_dir = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap());
    let template = manifest_dir.join("link-rpi3b.ld.in");

    let load_addr =
        std::env::var("RIVET_RPI3B_LOAD_ADDR").unwrap_or_else(|_| "0x80000".to_string());
    let ram_len = std::env::var("RIVET_RPI3B_RAM_LEN").unwrap_or_else(|_| "16M".to_string());

    let script_text = std::fs::read_to_string(&template)
        .expect("link-rpi3b.ld.in")
        .replace("@LOAD_ADDR@", &load_addr)
        .replace("@RAM_LEN@", &ram_len);

    let out = PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("link-rpi3b.ld");
    std::fs::write(&out, script_text).expect("write linker script");

    println!("cargo:linker-script={}", out.display());
    println!("cargo:rerun-if-changed={}", template.display());
    println!("cargo:rerun-if-env-changed=RIVET_RPI3B_LOAD_ADDR");
    println!("cargo:rerun-if-env-changed=RIVET_RPI3B_RAM_LEN");
    // Let the code see the window it was linked into, so the MMU can map
    // that and nothing else. Emitted as a generated source file rather
    // than an env var, because these are needed as integer constants and
    // parsing "0x30000000"/"16M" in a const context is not worth it.
    let addr_num = parse_num(&load_addr);
    let len_num = parse_num(&ram_len);
    std::fs::write(
        PathBuf::from(std::env::var("OUT_DIR").unwrap()).join("layout.rs"),
        format!(
            "/// Base of the RAM window this image was linked into.\n\
             pub const OWNED_BASE: usize = {addr_num:#x};\n\
             /// Length of that window.\n\
             pub const OWNED_LEN: usize = {len_num:#x};\n"
        ),
    )
    .expect("write layout.rs");
    println!("cargo:rustc-env=RIVET_RPI3B_LOAD_ADDR={load_addr}");
}
