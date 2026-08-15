//! BCM2837 SPI0, polled.
//!
//! Broadcom's own controller, not a PL022, so none of
//! `rivet-bsp-support`'s PL022 code applies here.
//!
//! # Loopback needs a wire
//!
//! This controller has no internal loopback bit, unlike the PL011 next
//! to it. Testing a real transfer therefore means shorting MOSI to MISO,
//! header pins 19 and 21, so the shift register receives what it sent.
//! Without that jumper a transfer still completes and still exercises
//! the clock, chip select, FIFO and DONE handshake, but every received
//! byte reads back as whatever the floating input settles to. The test
//! reports the two cases separately rather than calling a floating read
//! a pass.

use core::ptr::{read_volatile, write_volatile};

use crate::gpio;
use crate::mmio::PERIPHERAL_BASE;

const SPI0_BASE: usize = PERIPHERAL_BASE + 0x0020_4000;
const CS: usize = 0x00;
const FIFO: usize = 0x04;
const CLK: usize = 0x08;

// Control and status bits.
const CS_CLEAR_RX: u32 = 1 << 5;
const CS_CLEAR_TX: u32 = 1 << 4;
const CS_TA: u32 = 1 << 7;
const CS_DONE: u32 = 1 << 16;
const CS_RXD: u32 = 1 << 17;
const CS_TXD: u32 = 1 << 18;

/// Pins for SPI0, all ALT0.
const PIN_CE1: u8 = 7;
const PIN_CE0: u8 = 8;
const PIN_MISO: u8 = 9;
const PIN_MOSI: u8 = 10;
const PIN_SCLK: u8 = 11;

/// Why a transfer did not complete.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// The controller never drained, filled or reported completion.
    ///
    /// Every wait here is bounded. An unbounded spin on a peripheral
    /// register is not acceptable on a core whose whole purpose is
    /// meeting deadlines: absent or wedged hardware would take the
    /// system down rather than return an error.
    Timeout,
}

/// Iterations before a wait gives up. Generous next to any real transfer
/// at any supported clock, and still bounded.
const SPIN_LIMIT: u32 = 1_000_000;

pub struct Spi0;

impl Spi0 {
    /// Mux the pins and bring the controller up.
    ///
    /// `divider` sets the clock: SCLK is the 250 MHz core clock divided
    /// by it, and it must be a power of two. 0 means 65536.
    ///
    /// # Safety
    /// Takes over GPIO 7-11 and the SPI0 registers.
    pub unsafe fn init(&self, divider: u16) {
        // SAFETY: caller has given us these pins.
        unsafe {
            for p in [PIN_CE1, PIN_CE0, PIN_MISO, PIN_MOSI, PIN_SCLK] {
                gpio::set_function(p, gpio::func::ALT0);
            }
            write_volatile((SPI0_BASE + CLK) as *mut u32, divider as u32);
            write_volatile((SPI0_BASE + CS) as *mut u32, CS_CLEAR_RX | CS_CLEAR_TX);
        }
    }

    /// Exchange a buffer in place: each byte is shifted out and the byte
    /// clocked in at the same time replaces it.
    ///
    /// # Safety
    /// Requires [`init`](Self::init) first.
    pub unsafe fn transfer(&self, buf: &mut [u8]) -> Result<(), Error> {
        let cs = (SPI0_BASE + CS) as *mut u32;
        let fifo = (SPI0_BASE + FIFO) as *mut u32;
        // SAFETY: the controller is initialised and these are its
        // registers.
        unsafe {
            write_volatile(cs, CS_CLEAR_RX | CS_CLEAR_TX | CS_TA);

            let mut sent = 0;
            let mut recvd = 0;
            let mut spins = 0u32;
            while recvd < buf.len() {
                // Keep the transmit FIFO fed while draining the receive
                // side, rather than sending everything first: the FIFO is
                // 64 bytes and a longer transfer would otherwise overrun.
                while sent < buf.len() && read_volatile(cs) & CS_TXD != 0 {
                    write_volatile(fifo, buf[sent] as u32);
                    sent += 1;
                }
                while recvd < sent && read_volatile(cs) & CS_RXD != 0 {
                    buf[recvd] = read_volatile(fifo) as u8;
                    recvd += 1;
                }
                spins += 1;
                if spins > SPIN_LIMIT {
                    write_volatile(cs, CS_CLEAR_RX | CS_CLEAR_TX);
                    return Err(Error::Timeout);
                }
            }

            // DONE only means the transfer finished; TA has to be dropped
            // by software to release chip select.
            spins = 0;
            while read_volatile(cs) & CS_DONE == 0 {
                spins += 1;
                if spins > SPIN_LIMIT {
                    write_volatile(cs, CS_CLEAR_RX | CS_CLEAR_TX);
                    return Err(Error::Timeout);
                }
            }
            write_volatile(cs, CS_CLEAR_RX | CS_CLEAR_TX);
        }
        Ok(())
    }
}
