//! End-to-end proof of `rivet_bsp_support::pl022::Pl022`'s async
//! `embedded_hal_async::spi::SpiBus` impl (embedded-hal-plan.md Phase C):
//! an 8-byte transfer looped back internally by the PL022 controller
//! (`CR1.LBM` — no external SPI device needed), completed via a real
//! `RXIM` interrupt through `rivet::sync::Signal`, not polling. Same
//! driver, same protocol as qemu-cm3's `pl022_test` — proves the driver
//! genuinely is board-agnostic (only the base address/IRQ differ),
//! against the "APB" PL022 instance this board's QEMU model provides.

#![no_std]
#![no_main]

use rivet_bsp_mps2_an385 as _;
use rivet_rt as _;

use embedded_hal_async::spi::SpiBus;
use rivet_bsp_support::pl022::Pl022;

rivet_bsp_support::pl022_instance!(SPI0_SIG, spi0_isr, base = 0x4002_0000);

#[rivet::task(priority = 1, stack = 1024)]
async fn spi_task() {
    rivet::irq::register(rivet_bsp_mps2_an385::irq::SPI0, spi0_isr).unwrap();
    rivet::irq::enable(rivet_bsp_mps2_an385::irq::SPI0);

    // SAFETY: fixed PL022 register block (0x4002_0000), exclusively
    // owned by this task; SPI0_SIG is the exact Signal spi0_isr
    // (registered via pl022_instance! above) calls `signal()` on.
    let mut spi = unsafe { Pl022::new(0x4002_0000, &SPI0_SIG) };
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
