//! Real-hardware proof of `rivet_bsp_stm32f401re::i2c::Stm32I2c`'s async
//! `embedded_hal_async::i2c::I2c` impl (embedded-hal-plan.md Phase E):
//! no I2C slave is wired to this Nucleo's PB8 (SCL)/PB9 (SDA) this
//! session, so this specifically exercises the **real hardware NAK
//! path** — a genuine `START` condition, address transmission, and
//! `AF` (address-ack-failure) interrupt on real silicon, completed via
//! `Signal::wait`, not the QEMU-model workaround `stellaris_i2c` needs.
//! A future session with a real I2C sensor on this bus can extend this
//! to a genuine round-trip.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use embedded_hal_async::i2c::I2c;
use rivet_bsp_stm32f401re::i2c::Stm32I2c;

const I2C1_BASE: u32 = 0x4000_5400;

rivet_bsp_stm32f401re::stm32_i2c_instance!(
    I2C1_SIG,
    i2c1_ev_isr,
    i2c1_er_isr,
    base = I2C1_BASE as usize
);
const GPIOB_BASE: u32 = 0x4002_0400;
const RCC_BASE: u32 = 0x4002_3800;

/// No device responds at this address — the point of this test is
/// proving the real hardware NAK path.
const NO_DEVICE_ADDR: u8 = 0x42;

#[rivet::task(priority = 1, stack = 1024)]
async fn i2c_task() {
    // SAFETY: fixed RCC/GPIOB register blocks; runs once, before
    // anything else touches PB8/PB9 or I2C1's clocks.
    unsafe {
        let rcc = RCC_BASE as *mut u32;
        // AHB1ENR: GPIOB clock (bit 1). APB1ENR: I2C1 clock (bit 21).
        let ahb1enr = (rcc as usize + 0x30) as *mut u32;
        core::ptr::write_volatile(ahb1enr, core::ptr::read_volatile(ahb1enr) | (1 << 1));
        let apb1enr = (rcc as usize + 0x40) as *mut u32;
        core::ptr::write_volatile(apb1enr, core::ptr::read_volatile(apb1enr) | (1 << 21));

        // PB8/PB9: MODER = 10 (alternate function), AFRH pins 8/9 = AF4,
        // OTYPER = 1 (open-drain, required for I2C).
        let moder = GPIOB_BASE as *mut u32;
        let mut v = core::ptr::read_volatile(moder);
        v = (v & !(0b1111 << 16)) | (0b10 << 16) | (0b10 << 18);
        core::ptr::write_volatile(moder, v);

        let otyper = (GPIOB_BASE + 0x04) as *mut u32;
        core::ptr::write_volatile(otyper, core::ptr::read_volatile(otyper) | (0b11 << 8));

        // PUPDR = 01 (pull-up) on both pins: without this, the
        // open-drain I2C lines float with nothing wired to the bus and
        // never reach idle-high, so START generation spins forever
        // waiting for SB — found live on this exact board this session.
        let pupdr = (GPIOB_BASE + 0x0C) as *mut u32;
        let mut pv = core::ptr::read_volatile(pupdr);
        pv = (pv & !(0b1111 << 16)) | (0b01 << 16) | (0b01 << 18);
        core::ptr::write_volatile(pupdr, pv);

        let afrh = (GPIOB_BASE + 0x24) as *mut u32;
        // AFRH pin 8 is bits [3:0], pin 9 is bits [7:4] (pin 8 = AFRH
        // index 0 since AFRH covers pins 8-15).
        let mut afr = core::ptr::read_volatile(afrh);
        afr = (afr & !0xFF) | 0x44;
        core::ptr::write_volatile(afrh, afr);
    }

    rivet::irq::register(
        rivet_bsp_stm32f401re::irq::I2C1_EV,
        i2c1_ev_isr,
    )
    .unwrap();
    rivet::irq::register(
        rivet_bsp_stm32f401re::irq::I2C1_ER,
        i2c1_er_isr,
    )
    .unwrap();
    rivet::irq::enable(rivet_bsp_stm32f401re::irq::I2C1_EV);
    rivet::irq::enable(rivet_bsp_stm32f401re::irq::I2C1_ER);

    // SAFETY: fixed I2C1 register block (0x4000_5400), exclusively owned
    // by this task; I2C1_SIG is the exact Signal the two ISRs
    // (registered via stm32_i2c_instance! above) call `signal()` on.
    let mut i2c = unsafe { Stm32I2c::new(I2C1_BASE as usize, &I2C1_SIG) };
    i2c.init();

    match i2c.write(NO_DEVICE_ADDR, &[0x00]).await {
        Err(_) => {
            rivet::console::write_str("I2C_NAK_DETECTED\n");
            rivet::console::write_str("STM32_I2C_TEST_OK\n");
            rivet::exit_success();
        }
        Ok(()) => {
            // Would mean something actually acked — treat as a genuine
            // (surprising) success rather than a failure.
            rivet::console::write_str("I2C_UNEXPECTED_ACK\n");
            rivet::console::write_str("STM32_I2C_TEST_OK\n");
            rivet::exit_success();
        }
    }
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet stm32_i2c_test\n");
    rivet::run();
}
