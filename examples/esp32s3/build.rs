fn main() {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={manifest_dir}");
    println!("cargo:rerun-if-changed={manifest_dir}/memory.x");
    println!("cargo:rerun-if-changed={manifest_dir}/rivet-esp32s3-pre.x");
    println!("cargo:rerun-if-changed={manifest_dir}/rivet-esp32s3-post.x");

    // `rivet-esp32s3-pre.x` comes *first*: its `.rodata` rule (app
    // descriptor content) must establish the section's starting content
    // before `link.x`'s own `.rodata` rule appends the rest.
    //
    // `link.x` (`xtensa-lx-rt`) does `INCLUDE memory.x` itself, so it
    // pulls this crate's `memory.x` in via the search path above;
    // `device.x` (`esp32s3` PAC) provides PROVIDE-default stub symbols
    // for every peripheral interrupt vector name.
    //
    // `rivet-esp32s3-post.x` comes *last*: its `.data`/`AT>` override
    // must be the one that wins (the region/`AT>` assignment itself is
    // governed by whichever processed script's explicit clause is last,
    // unlike section *content*, which accumulates in first-encountered
    // order — see that file's own comment), and its `.rivet_tasks`/
    // `.task_stacks`/`.stack` sections need `link.x`'s own sections
    // (`.bss` in particular) already defined first.
    println!("cargo:rustc-link-arg=-Trivet-esp32s3-pre.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdevice.x");
    println!("cargo:rustc-link-arg=-Trivet-esp32s3-post.x");
}
