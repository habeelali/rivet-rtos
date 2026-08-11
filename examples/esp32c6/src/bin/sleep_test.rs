//! Minimal single-core isolation test (plan.md Phase 24): does
//! `rivet::time::Sleep` / the async executor's timer-wake path work at all
//! on this arch, independent of any dual-core scheduling question?

#![no_std]
#![no_main]

use rivet_bsp_esp32c6 as _;
use rivet_rt as _;

#[rivet::task(priority = 0, stack = 512)]
async fn sleeper() {
    rivet::console::write_str("before sleep\n");
    rivet::time::Sleep::<1_000>::new().await;
    rivet::console::write_str("after sleep\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet esp32c6 sleep_test\n");
    rivet::run();
}
