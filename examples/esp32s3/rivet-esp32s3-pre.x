/* Processed BEFORE `link.x` (see build.rs) — establishes `.rodata`'s
 * starting content, so `esp_bootloader_esp_idf::esp_app_desc!()`'s
 * `.flash.appdesc` output lands first within it. The ESP-IDF bootloader
 * reads the app descriptor structure starting at the DROM segment's base
 * address directly, with no indirection — anything placed before it there
 * reads back as garbage struct fields (found for real: "Image requires
 * efuse blk rev >= v133.65", nonsense values from a misaligned read, not
 * a plausible real requirement). `link.x`'s own later `.rodata : {
 * *(.rodata .rodata.*) } > RODATA` rule for the same output-section name
 * appends its matches after what's already here, not before — content
 * ordering across multiple same-named SECTIONS commands accumulates in
 * first-encountered order (unlike the region/`AT>` assignment itself,
 * which the *last* processed explicit clause governs — see
 * `rivet-esp32s3-post.x` for why `.data`'s fix has to live there instead
 * of here). */
SECTIONS
{
  .rodata : {
    KEEP(*(.flash.appdesc))
  } > RODATA
}
