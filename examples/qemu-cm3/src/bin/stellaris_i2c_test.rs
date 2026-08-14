//! End-to-end proof of `rivet_bsp_support::stellaris_i2c::StellarisI2c`'s
//! async `embedded_hal_async::i2c::I2c` impl: writes a byte to a real
//! QEMU `at24c-eeprom` device attached
//! at I2C address 0x50 (see `xtask`'s `extra_qemu_args` for this test —
//! `-device at24c-eeprom,address=0x50,rom-size=256`), then reads it back
//! through a `write_read` (address-pointer write, then STOP/START —
//! genuine repeated START is broken in this model, see
//! `stellaris_i2c`'s own module docs) and confirms the round trip.

#![no_std]
#![no_main]

use rivet_bsp_lm3s6965 as _;
use rivet_rt as _;

use embedded_hal_async::i2c::I2c;
use rivet_bsp_support::stellaris_i2c::StellarisI2c;

rivet_bsp_support::stellaris_i2c_instance!(I2C0_SIG, i2c0_isr, base = 0x4002_0000);

const EEPROM_ADDR: u8 = 0x50;

#[rivet::task(priority = 1, stack = 1024)]
async fn i2c_task() {
    rivet::irq::register(rivet_bsp_lm3s6965::irq::I2C0, i2c0_isr).unwrap();
    rivet::irq::enable(rivet_bsp_lm3s6965::irq::I2C0);

    // SAFETY: fixed Stellaris I2C register block (0x4002_0000),
    // exclusively owned by this task; I2C0_SIG is the exact Signal
    // i2c0_isr (registered via stellaris_i2c_instance! above) calls
    // `signal()` on.
    let mut i2c = unsafe { StellarisI2c::new(0x4002_0000, &I2C0_SIG) };
    i2c.init();

    // Write byte 0xAB at EEPROM address 0x00.
    if i2c.write(EEPROM_ADDR, &[0x00, 0xAB]).await.is_err() {
        rivet::console::write_str("I2C_WRITE_FAIL\n");
        rivet::exit_failure(1);
    }
    rivet::console::write_str("I2C_WRITE_OK\n");

    // Set the address pointer back to 0x00, then read the byte back.
    let mut buf = [0u8; 1];
    if i2c.write_read(EEPROM_ADDR, &[0x00], &mut buf).await.is_err() {
        rivet::console::write_str("I2C_READ_FAIL\n");
        rivet::exit_failure(2);
    }

    if buf[0] == 0xAB {
        rivet::console::write_str("I2C_ROUNDTRIP_OK\n");
        rivet::console::write_str("I2C_TEST_OK\n");
        rivet::exit_success();
    } else {
        rivet::console::write_str("I2C_ROUNDTRIP_MISMATCH\n");
        rivet::exit_failure(3);
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet stellaris_i2c_test\n");
    rivet::run();
}
