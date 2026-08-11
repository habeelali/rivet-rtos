/* Rivet RTOS — ESP32-S3 memory map for `xtensa-lx-rt`'s `link.x`
 * (plan.md Phase 22). `link.x` provides the complete SECTIONS block
 * (.text/.rodata/.data/.bss/.rwtext) against these four region names —
 * ROTEXT/RODATA/RWDATA/RWTEXT are `xtensa-lx-rt`'s own generic naming,
 * not chip-specific; carved here out of the real, chip-specific IRAM/
 * DRAM ranges (cross-checked against `esp-hal`'s own `ld/esp32s3/
 * memory.x`, not guessed):
 *   40378000 <- IRAM/Icache -> 403E0000   (I-bus view)
 *   3FC88000 <- D/IRAM (D)  -> 3FD00000   (D-bus view)
 *
 * CRITICAL: the D/IRAM block is the SAME PHYSICAL SRAM as the IRAM block,
 * just addressed through two different CPU buses — `I_addr = D_addr +
 * 0x6F0000` (confirmed against ESP-IDF's own `memory.ld.in`:
 * `I_D_SRAM_OFFSET = 0x6F0000`; `0x3FC88000 + 0x6F0000 == 0x40378000`
 * exactly). Root-caused on real hardware the hard way: an earlier
 * version of this file put RWDATA at `0x3FC88000` while ROTEXT/RWTEXT/
 * vectors_seg occupied the ALIASED I-bus range starting at its exact
 * D-bus mirror — two independently-loaded ("load"-type) image segments
 * silently overwriting each other's physical bytes, producing a
 * non-deterministic crash (varying PC/EXCCAUSE between otherwise
 * identical boots) right at/before `Reset`'s own first instruction.
 * RWDATA below is deliberately placed to start exactly where ROTEXT's
 * end (translated to its D-bus alias) leaves off, so no D-bus region
 * ever aliases a byte of I-bus content.
 *
 * Deliberately simple (plan.md Phase 22): everything RAM-resident, no
 * flash-cache-mapped XIP the way `esp-hal`'s own linker script optimizes
 * for — a valid, standard ESP32 image layout, just less flash-efficient.
 * `.data`'s LMA (ROTEXT-region, per `link.x`'s own `AT > RODATA`) and VMA
 * (RWDATA) are both real RAM addresses here, so `Reset`'s data-copy step
 * is a harmless RAM-to-RAM copy rather than a real flash-to-RAM one.
 */

MEMORY
{
  /* The Xtensa vector table (exception.x, generated from this chip's
   * config) needs its own named region — real hardware requirement, not
   * a Rivet-specific addition. */
  /* IRAM 0x403B9000-0x403E0000 is reserved by the boot ROM's own startup
   * code at hand-off time (documented in esp-hal's own `memory.x`: "not
   * available for static memory, but can only be used after app starts")
   * — ROTEXT+RWTEXT together must stay safely below that, not just below
   * the full 0x403E0000 IRAM ceiling. Missing this caused a real,
   * reproducible bootloader-side `abort()` right at hand-off (the loaded
   * `.rwtext` overlapped and corrupted the ROM code still executing the
   * load), not a guess. */
  /* RESERVE_ICACHE: the first 32KB of the 0x40370000 IRAM range is the
   * instruction cache's own physical backing storage once
   * `rom_config_instruction_cache_mode` configures a 32KB icache
   * (`__pre_init`, matching `esp-hal`'s own `RESERVE_ICACHE = 0x8000` in
   * its `memory.x`) — that reconfiguration call physically repurposes
   * this address range as cache lines, so no code/data can live there.
   * Root-caused on real hardware: placing our own vectors/code at
   * 0x40374000 (inside this range) produced a real, reproducible
   * `IllegalInstruction` fault immediately after `__pre_init`'s cache
   * calls returned — the CPU was fetching from memory that had just
   * been cannibalized by the cache it was executing out of. */
  vectors_seg (RX) : ORIGIN = 0x40378000, LENGTH = 1K
  RWTEXT (RX)  : ORIGIN = 0x40378400, LENGTH = 32K
  ROTEXT (RX)  : ORIGIN = 0x40380400, LENGTH = 128K
  /* `esp_bootloader_esp_idf::esp_app_desc!()`'s `.flash.appdesc` output
   * must live inside a real flash-cache-mapped (XIP) address range —
   * `espflash`'s image builder specifically looks for it there (a real,
   * observed requirement: `unreachable: appdesc segment not found`
   * otherwise, not a guess) — so RODATA is DROM here, not DRAM. The
   * 2nd-stage bootloader maps it correctly on load, the same way it
   * already does for esp-hal-based binaries (confirmed via this board's
   * own boot log during Phase 20's smoke test). */
  RODATA (R)   : ORIGIN = 0x3C000020, LENGTH = 256K
  /* Starts exactly at ROTEXT's end (0x40380400 + 128K = 0x403A0400)
   * translated to its D-bus alias (0x403A0400 - 0x6F0000 = 0x3FCB0400)
   * — see the big comment above: this guarantees zero physical overlap
   * with vectors_seg/RWTEXT/ROTEXT's I-bus content. */
  RWDATA (RW)  : ORIGIN = 0x3FCB0400, LENGTH = 0x2B300
  /* The boot ROM's cache/MMU bring-up unconditionally expects at least
   * one IROM (flash-cache-mapped, executable) segment, which this
   * board's otherwise-all-RAM layout would otherwise have none of —
   * confirmed on real hardware (a hard bootloader-side `abort()` at
   * hand-off with this segment entirely absent).
   *
   * The flash-cache MMU's entry index is `(vaddr & 0x1FFFFFF) >> 16`
   * (one 64KB page per entry) and — critically — is SHARED between the
   * I-bus and D-bus mapping tables on this chip, not two independent
   * tables. RODATA (`0x3C000020`, 256K) occupies entries 0-3
   * (`0x3C000000`-`0x3C040000`). An IROM placed at `0x42000020` maps to
   * that exact same low-bits index (`0x20 >> 16 == 0`), so the
   * bootloader's *second* MMU-mapping call (IROM, mapped after DROM)
   * silently overwrites entry 0's mapping — meaning `.rodata`
   * (including our own app descriptor and every string literal) was
   * being read back through IROM's mapping instead of DROM's. Root-
   * caused on real hardware: every DROM read failed (long `0xFF`/junk
   * floods on the wire) even after `__pre_init` correctly reconfigured
   * the cache — because the cache was reading the *wrong physical flash
   * page* for entry 0, not because the cache itself was unconfigured.
   * Placed here at entry 4 (`0x42040020`, past RODATA's 4 entries) so
   * IROM's own mapping call can never collide with DROM's. */
  IROM (RX)    : ORIGIN = 0x42040020, LENGTH = 4K
}
