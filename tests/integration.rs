//! Integration tests (run on host with `cargo test`).
//! Add embedded / hardware-in-the-loop tests under `tests/` as needed.

use rivet_rtos::VERSION;

#[test]
fn crate_links_and_version() {
    assert!(!VERSION.is_empty());
}
