//! End-to-end proof of `rivet_bsp_support::pl022::Pl022`'s async
//! `embedded_hal_async::spi::SpiBus` impl (embedded-hal-plan.md Phase C):
//! an 8-byte transfer looped back internally by the PL022 controller
//! (`CR1.LBM` — no external SPI device needed), completed via a real
//! `RXIM` interrupt through `rivet::sync::Signal`, not polling. 8 bytes
//! specifically because QEMU's PL022 model only asserts `RXIM` once the
//! RX FIFO holds >= 4 bytes (confirmed against the model source, not
//! assumed) — this is also a real exercise of the chunking path in
//! `transfer_async`, since `FIFO_DEPTH` is exactly 8.

#![no_std]
#![no_main]

use rivet_bsp_lm3s6965 as _;
use rivet_rt as _;

use embedded_hal_async::spi::SpiBus;
use rivet_bsp_support::pl022::Pl022;

rivet_bsp_support::pl022_instance!(SSI0_SIG, ssi0_isr, base = 0x4000_8000);

#[rivet::task(priority = 1, stack = 1024)]
async fn spi_task() {
    rivet::irq::register(rivet_bsp_lm3s6965::irq::SSI0, ssi0_isr).unwrap();
    rivet::irq::enable(rivet_bsp_lm3s6965::irq::SSI0);

    // SAFETY: fixed SSI0 register block (0x4000_8000), exclusively owned
    // by this task; SSI0_SIG is the exact Signal ssi0_isr (registered
    // via pl022_instance! above) calls `signal()` on.
    let mut spi = unsafe { Pl022::new(0x4000_8000, &SSI0_SIG) };
    spi.init(true); // loopback — TX FIFO feeds RX FIFO, no external device

    let tx: [u8; 8] = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    let mut buf = tx;
    spi.transfer_in_place(&mut buf).await.unwrap();

    if buf == tx {
        rivet::console::write_str("SPI_LOOPBACK_OK\n");
        rivet::console::write_str("PL022_TEST_OK\n");
        rivet::exit_success();
    } else {
        rivet::console::write_str("SPI_LOOPBACK_MISMATCH\n");
        rivet::exit_failure(1);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet pl022_test\n");
    rivet::run();
}
