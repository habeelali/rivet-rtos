//! BCM2837 BSC1, the I2C master on GPIO2/3.
//!
//! Broadcom call it the Broadcom Serial Controller; it is I2C with the
//! usual caveats and a documented clock-stretch bug in older silicon.
//!
//! # Testing without a device attached
//!
//! I2C has no loopback mode, and a Pi 3B has nothing on the bus unless a
//! HAT is fitted, so there is nothing to talk to. That turns out not to
//! matter: a scan is a genuine test of the driver. Addressing an absent
//! device produces a NAK, which sets `S.ERR`, and observing that for
//! every address proves the controller was enabled, the clock ran, the
//! transfer started and completed, and the error path reports rather
//! than hangs. A device that *is* present shows up as the one address
//! that does not error.

use core::ptr::{read_volatile, write_volatile};

use crate::gpio;
use crate::mmio::PERIPHERAL_BASE;

const BSC1_BASE: usize = PERIPHERAL_BASE + 0x0080_4000;
const C: usize = 0x00;
const S: usize = 0x04;
const DLEN: usize = 0x08;
const A: usize = 0x0C;
const FIFO: usize = 0x10;
const DIV: usize = 0x14;

const C_READ: u32 = 1 << 0;
const C_CLEAR: u32 = 1 << 4;
const C_ST: u32 = 1 << 7;
const C_I2CEN: u32 = 1 << 15;

const S_TA: u32 = 1 << 0;
const S_DONE: u32 = 1 << 1;
const S_TXD: u32 = 1 << 4;
const S_RXD: u32 = 1 << 5;
const S_ERR: u32 = 1 << 8;
const S_CLKT: u32 = 1 << 9;

const PIN_SDA: u8 = 2;
const PIN_SCL: u8 = 3;

/// What a transfer did.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Error {
    /// The device did not acknowledge its address.
    Nak,
    /// The device stretched the clock past the timeout.
    ClockTimeout,
    /// The controller never reported completion.
    Timeout,
}

/// Bound on every wait, so a wedged bus returns an error rather than
/// hanging a core whose job is meeting deadlines.
const SPIN_LIMIT: u32 = 1_000_000;

pub struct I2c1;

impl I2c1 {
    /// Mux the pins and enable the controller.
    ///
    /// `divider` divides the 150 MHz core clock to produce SCL, so 1500
    /// gives the standard 100 kHz.
    ///
    /// # Safety
    /// Takes over GPIO 2 and 3 and the BSC1 registers.
    pub unsafe fn init(&self, divider: u16) {
        // SAFETY: caller has given us these pins.
        unsafe {
            gpio::set_function(PIN_SDA, gpio::func::ALT0);
            gpio::set_function(PIN_SCL, gpio::func::ALT0);
            write_volatile((BSC1_BASE + DIV) as *mut u32, divider as u32);
            write_volatile((BSC1_BASE + C) as *mut u32, C_I2CEN);
            // Status bits are write-1-to-clear.
            write_volatile((BSC1_BASE + S) as *mut u32, S_DONE | S_ERR | S_CLKT);
        }
    }

    /// Put the controller back in a known state before a transfer.
    ///
    /// Waiting for the bus to go idle is the part that matters. A NAK
    /// returns as soon as `ERR` is seen, which is while the controller is
    /// still winding the transfer up, so its `DONE` arrives a moment
    /// later. Clearing sticky status at the start of the *next* transfer
    /// happens before that stray `DONE` lands, and the next wait then
    /// sees it and reports instant success.
    ///
    /// On a bare bus that produced a perfect alternating pattern, every
    /// other address looking like a device, which is how it was found.
    ///
    /// # Safety
    /// Requires [`init`](Self::init) first.
    unsafe fn prepare(&self) {
        // SAFETY: registers of an initialised controller.
        unsafe {
            let mut spins = 0u32;
            while read_volatile((BSC1_BASE + S) as *const u32) & S_TA != 0 {
                spins += 1;
                if spins > SPIN_LIMIT {
                    break;
                }
            }
            // Drop ST and empty the FIFO, then clear the sticky bits now
            // that nothing further can set them.
            write_volatile((BSC1_BASE + C) as *mut u32, C_I2CEN | C_CLEAR);
            write_volatile((BSC1_BASE + S) as *mut u32, S_DONE | S_ERR | S_CLKT);
        }
    }

    /// Read `buf.len()` bytes from `addr`.
    ///
    /// # Safety
    /// Requires [`init`](Self::init) first.
    pub unsafe fn read(&self, addr: u8, buf: &mut [u8]) -> Result<(), Error> {
        // SAFETY: the controller is initialised and these are its registers.
        unsafe {
            self.prepare();
            write_volatile((BSC1_BASE + A) as *mut u32, addr as u32);
            write_volatile((BSC1_BASE + DLEN) as *mut u32, buf.len() as u32);
            write_volatile(
                (BSC1_BASE + C) as *mut u32,
                C_I2CEN | C_ST | C_CLEAR | C_READ,
            );

            let mut got = 0;
            // Bounded rather than a bare spin: a NAK on this controller
            // still raises DONE, but a wedged bus should not hang a
            // real-time core forever.
            let mut spins = 0u32;
            loop {
                let st = read_volatile((BSC1_BASE + S) as *const u32);
                if st & S_ERR != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_ERR);
                    return Err(Error::Nak);
                }
                if st & S_CLKT != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_CLKT);
                    return Err(Error::ClockTimeout);
                }
                if st & S_RXD != 0 && got < buf.len() {
                    buf[got] = read_volatile((BSC1_BASE + FIFO) as *const u32) as u8;
                    got += 1;
                }
                if st & S_DONE != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_DONE);
                    return Ok(());
                }
                spins += 1;
                if spins > SPIN_LIMIT {
                    return Err(Error::Timeout);
                }
            }
        }
    }

    /// Write bytes to `addr`.
    ///
    /// # Safety
    /// Requires [`init`](Self::init) first.
    pub unsafe fn write(&self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // SAFETY: as above.
        unsafe {
            self.prepare();
            write_volatile((BSC1_BASE + A) as *mut u32, addr as u32);
            write_volatile((BSC1_BASE + DLEN) as *mut u32, bytes.len() as u32);
            write_volatile((BSC1_BASE + C) as *mut u32, C_I2CEN | C_ST | C_CLEAR);

            let mut sent = 0;
            let mut spins = 0u32;
            loop {
                let st = read_volatile((BSC1_BASE + S) as *const u32);
                if st & S_ERR != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_ERR);
                    return Err(Error::Nak);
                }
                if st & S_CLKT != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_CLKT);
                    return Err(Error::ClockTimeout);
                }
                if st & S_TXD != 0 && sent < bytes.len() {
                    write_volatile((BSC1_BASE + FIFO) as *mut u32, bytes[sent] as u32);
                    sent += 1;
                }
                if st & S_DONE != 0 {
                    write_volatile((BSC1_BASE + S) as *mut u32, S_DONE);
                    return Ok(());
                }
                spins += 1;
                if spins > SPIN_LIMIT {
                    return Err(Error::Timeout);
                }
            }
        }
    }

    /// Probe one address by attempting a one-byte read.
    ///
    /// A read rather than a write, because reading from a device that
    /// happens to be there is harmless while writing to an unknown
    /// register is not.
    ///
    /// # Safety
    /// Requires [`init`](Self::init) first.
    pub unsafe fn probe(&self, addr: u8) -> bool {
        let mut b = [0u8; 1];
        // SAFETY: forwarded from this function's contract.
        unsafe { self.read(addr, &mut b).is_ok() }
    }
}
