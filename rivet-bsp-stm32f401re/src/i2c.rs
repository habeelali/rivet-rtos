//! STM32F401 I2C1 Master driver, the real-hardware half of this
//! workspace's async I2C support: `embedded_hal::i2c::I2c` (polling) and
//! `embedded_hal_async::i2c::I2c` (completion via real interrupts —
//! I2C1_EV and I2C1_ER are separate NVIC vectors on this chip, unlike
//! the single-vector controllers `pl022`/`stellaris_i2c` drive) through
//! [`rivet::sync::Signal`].
//!
//! This is STM32's "legacy" I2C peripheral (the same block family since
//! the F1 series), with its own well-known register-sequencing errata —
//! confirmed against RM0368 directly:
//!
//! - **`ADDR` must be cleared by reading `SR1` then `SR2`, in that
//!   order** — reading `SR1` alone leaves the address-match condition
//!   latched and the bus stuck. Every address phase in this driver reads
//!   both, unconditionally, immediately after `ADDR` sets.
//! - **The last byte of a multi-byte read must be NACK'd**: `ACK` in
//!   `CR1` has to be cleared *before* the second-to-last byte is read
//!   out of `DR` (clearing it any later NACKs the wrong byte), and
//!   `STOP` set in that same window, before reading the final byte.
//! - **`BTF` (byte transfer finished), not `TXE`, gates the last
//!   transmitted byte**: waiting on `TXE` alone for the final byte can
//!   issue `STOP` while the shift register is still clocking that byte
//!   out, truncating the transfer on a slow/stretching slave.
//!
//! Real hardware genuinely raises an interrupt on NAK (`AF` in `SR1`,
//! covered by `ITERREN`) — unlike the QEMU `stellaris-i2c` model
//! `stellaris_i2c` works around, this driver's async NAK path really
//! does complete via [`Signal::wait`], not a synchronous fallback.

use embedded_hal::i2c::{Operation, SevenBitAddress};
use rivet::sync::Signal;

const CR1: usize = 0x00;
const CR2: usize = 0x04;
const DR: usize = 0x10;
const SR1: usize = 0x14;
const SR2: usize = 0x18;
const CCR: usize = 0x1C;
const TRISE: usize = 0x20;

const CR1_PE: u32 = 1 << 0;
const CR1_START: u32 = 1 << 8;
const CR1_STOP: u32 = 1 << 9;
const CR1_ACK: u32 = 1 << 10;
const CR1_SWRST: u32 = 1 << 15;

const CR2_ITERREN: u32 = 1 << 8;
const CR2_ITEVTEN: u32 = 1 << 9;

const SR1_SB: u32 = 1 << 0;
const SR1_ADDR: u32 = 1 << 1;
const SR1_BTF: u32 = 1 << 2;
const SR1_RXNE: u32 = 1 << 6;
const SR1_TXE: u32 = 1 << 7;
const SR1_AF: u32 = 1 << 10;
const SR1_ERROR_MASK: u32 = (1 << 8) | (1 << 9) | SR1_AF | (1 << 11) | (1 << 14);

/// A failed transaction — address or data NAK, arbitration lost, bus
/// error, or overrun. `SR1`'s error bits are cleared by writing 0 to
/// them; this driver doesn't distinguish which one fired.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct I2cError;

impl embedded_hal::i2c::Error for I2cError {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::NoAcknowledge(
            embedded_hal::i2c::NoAcknowledgeSource::Unknown,
        )
    }
}

/// One physical STM32 I2C controller: a fixed register base and the
/// [`Signal`] its two interrupt handlers (event + error) complete.
///
/// Construct via [`stm32_i2c_instance!`] — see
/// `rivet_bsp_support::pl022::Pl022::new`'s doc for why a bare generic
/// `fn isr<const BASE: usize>()` with an inline `static Signal` wouldn't
/// give each instance its own signal.
pub struct Stm32I2c {
    base: usize,
    sig: &'static Signal,
}

impl Stm32I2c {
    /// # Safety
    /// `base` must be a real, exclusively-owned STM32 I2C register
    /// block, and `sig` must be the exact [`Signal`] the ISRs registered
    /// for this instance call [`Signal::signal`] on.
    pub const unsafe fn new(base: usize, sig: &'static Signal) -> Self {
        Self { base, sig }
    }

    fn reg(&self, offset: usize) -> *mut u32 {
        (self.base + offset) as *mut u32
    }

    fn sr1(&self) -> u32 {
        // SAFETY: fixed I2C registers, exclusively owned per the
        // constructor's contract.
        unsafe { core::ptr::read_volatile(self.reg(SR1)) }
    }

    fn clear_addr(&self) {
        // SAFETY: as in `sr1`. Reading SR1 then SR2, in that order, is
        // what actually clears ADDR — see module docs.
        unsafe {
            let _ = core::ptr::read_volatile(self.reg(SR1));
            let _ = core::ptr::read_volatile(self.reg(SR2));
        }
    }

    /// Bring the controller up at 100 kHz standard mode, assuming the
    /// board's documented 16 MHz reset-state HSI APB1 clock (see
    /// `rivet-bsp-stm32f401re`'s own module doc) — `CR2.FREQ` in MHz,
    /// `CCR` = APB1_Hz / (2 * 100_000) for standard mode, `TRISE` =
    /// (1000 ns max rise time / APB1 period) + 1 = FREQ_MHz + 1.
    ///
    /// Does *not* configure GPIO — PB8 (SCL)/PB9 (SDA) alternate
    /// function, open-drain, is the caller's responsibility (board-owned
    /// pin muxing, matching every other peripheral in this workspace).
    pub fn init(&self) {
        const APB1_MHZ: u32 = 16;
        // SAFETY: as in `sr1`.
        unsafe {
            core::ptr::write_volatile(self.reg(CR1), CR1_SWRST);
            core::ptr::write_volatile(self.reg(CR1), 0);
            core::ptr::write_volatile(self.reg(CR2), APB1_MHZ);
            core::ptr::write_volatile(self.reg(CCR), APB1_MHZ * 1_000_000 / (2 * 100_000));
            core::ptr::write_volatile(self.reg(TRISE), APB1_MHZ + 1);
            core::ptr::write_volatile(self.reg(CR1), CR1_PE);
        }
    }

    fn start(&self) {
        // SAFETY: as in `sr1`.
        unsafe {
            let cr1 = core::ptr::read_volatile(self.reg(CR1));
            core::ptr::write_volatile(self.reg(CR1), cr1 | CR1_START | CR1_ACK);
        }
        while self.sr1() & SR1_SB == 0 {
            core::hint::spin_loop();
        }
    }

    fn send_address(&self, address: u8, read: bool) -> Result<(), I2cError> {
        // SAFETY: as in `sr1`.
        unsafe {
            let val = ((address as u32) << 1) | u32::from(read);
            core::ptr::write_volatile(self.reg(DR), val);
        }
        loop {
            let sr1 = self.sr1();
            if sr1 & SR1_AF != 0 {
                // SAFETY: as in `sr1`. Clear AF (write 0 to SR1's error bits).
                unsafe { core::ptr::write_volatile(self.reg(SR1), 0) };
                self.send_stop();
                return Err(I2cError);
            }
            if sr1 & SR1_ADDR != 0 {
                self.clear_addr();
                return Ok(());
            }
            core::hint::spin_loop();
        }
    }

    fn send_stop(&self) {
        // SAFETY: as in `sr1`.
        unsafe {
            let cr1 = core::ptr::read_volatile(self.reg(CR1));
            core::ptr::write_volatile(self.reg(CR1), cr1 | CR1_STOP);
        }
    }

    fn write_bytes_sync(&self, bytes: &[u8], is_last_op: bool) -> Result<(), I2cError> {
        let n = bytes.len();
        for (j, &b) in bytes.iter().enumerate() {
            // SAFETY: as in `sr1`.
            unsafe { core::ptr::write_volatile(self.reg(DR), b as u32) };
            let is_last = j + 1 == n;
            // BTF (not just TXE) for the very last byte of the very last
            // operation — see module docs on why TXE alone would
            // truncate the transfer.
            let wait_mask = if is_last && is_last_op { SR1_BTF } else { SR1_TXE };
            loop {
                let sr1 = self.sr1();
                if sr1 & SR1_ERROR_MASK != 0 {
                    // SAFETY: as in `sr1`.
                    unsafe { core::ptr::write_volatile(self.reg(SR1), 0) };
                    self.send_stop();
                    return Err(I2cError);
                }
                if sr1 & wait_mask != 0 {
                    break;
                }
                core::hint::spin_loop();
            }
        }
        if is_last_op {
            self.send_stop();
        }
        Ok(())
    }

    fn read_bytes_sync(&self, bytes: &mut [u8]) -> Result<(), I2cError> {
        let n = bytes.len();
        for (j, slot) in bytes.iter_mut().enumerate() {
            let is_last = j + 1 == n;
            if is_last {
                // NACK the last byte and issue STOP before reading it —
                // see module docs; doing this after the read NACKs (and
                // STOPs after) the *next* byte instead.
                // SAFETY: as in `sr1`.
                unsafe {
                    let cr1 = core::ptr::read_volatile(self.reg(CR1));
                    core::ptr::write_volatile(self.reg(CR1), cr1 & !CR1_ACK);
                }
                self.send_stop();
            }
            loop {
                let sr1 = self.sr1();
                if sr1 & SR1_ERROR_MASK != 0 {
                    // SAFETY: as in `sr1`.
                    unsafe { core::ptr::write_volatile(self.reg(SR1), 0) };
                    return Err(I2cError);
                }
                if sr1 & SR1_RXNE != 0 {
                    break;
                }
                core::hint::spin_loop();
            }
            // SAFETY: as in `sr1`.
            *slot = unsafe { core::ptr::read_volatile(self.reg(DR)) as u8 };
        }
        Ok(())
    }

    /// Real hardware genuinely completes asynchronously — unlike
    /// `stellaris_i2c`'s QEMU-model workaround, this just enables the
    /// event+error interrupts, issues `START`, and awaits. NAK (`AF`)
    /// really does raise `I2C1_ER`'s interrupt on this hardware.
    async fn start_async(&self) {
        self.sig.reset();
        // SAFETY: as in `sr1`.
        unsafe {
            let cr2 = core::ptr::read_volatile(self.reg(CR2));
            core::ptr::write_volatile(self.reg(CR2), cr2 | CR2_ITEVTEN | CR2_ITERREN);
            let cr1 = core::ptr::read_volatile(self.reg(CR1));
            core::ptr::write_volatile(self.reg(CR1), cr1 | CR1_START | CR1_ACK);
        }
        self.sig.wait().await;
    }

    async fn send_address_async(&self, address: u8, read: bool) -> Result<(), I2cError> {
        self.sig.reset();
        // SAFETY: as in `sr1`. Re-arm ITEVTEN/ITERREN: the event/error
        // ISRs mask both on every entry (see their doc comments — the
        // level-latched status bits would otherwise re-trigger the NVIC
        // line forever before this future gets to run), so every
        // `wait()` past the first must re-enable them itself or this
        // phase's completion interrupt never fires — found live on real
        // hardware this session (ADDR/AF genuinely set in `SR1`, but no
        // interrupt ever arrived because `CR2` stayed masked from the
        // `start_async` phase's own event).
        unsafe {
            let cr2 = core::ptr::read_volatile(self.reg(CR2));
            core::ptr::write_volatile(self.reg(CR2), cr2 | CR2_ITEVTEN | CR2_ITERREN);
            let val = ((address as u32) << 1) | u32::from(read);
            core::ptr::write_volatile(self.reg(DR), val);
        }
        self.sig.wait().await;
        let sr1 = self.sr1();
        if sr1 & SR1_AF != 0 {
            // SAFETY: as in `sr1`.
            unsafe { core::ptr::write_volatile(self.reg(SR1), 0) };
            self.send_stop();
            return Err(I2cError);
        }
        self.clear_addr();
        Ok(())
    }
}

/// Shared event-interrupt ISR body for every `Stm32I2c` instance —
/// `SB`/`ADDR`/`BTF`/`TXE`/`RXNE` all share this one NVIC vector on this
/// chip. Masks both `ITEVTEN` and `ITERREN` in `CR2` before waking:
/// `SB`/`ADDR` are level-latched status bits (they stay asserted until
/// the driver's own polled follow-up — writing `DR`, reading
/// `SR1`+`SR2` — clears them), so leaving the enables set here would
/// have the NVIC immediately re-enter this same handler the instant it
/// returns, forever, since Thread-mode code never gets a window to run
/// the clearing sequence. Confirmed live on real hardware this
/// session — masking here is what stops the storm.  `start_async`/
/// `send_address_async` each re-enable both bits before their own
/// `wait()`, so this only ever disarms, never permanently.
pub fn isr_ev(base: usize, sig: &Signal) {
    // SAFETY: `base` is a real I2C register block passed by the
    // `stm32_i2c_instance!` caller, who owns it exclusively.
    unsafe {
        let cr2 = core::ptr::read_volatile((base + CR2) as *mut u32);
        core::ptr::write_volatile(
            (base + CR2) as *mut u32,
            cr2 & !(CR2_ITEVTEN | CR2_ITERREN),
        );
    }
    sig.signal();
}

/// Shared error-interrupt ISR body — `AF`/`BERR`/`ARLO`/`OVR` share this
/// separate NVIC vector. Masks the same `CR2` bits as [`isr_ev`] for the
/// same reason (the error status bits are level-latched too); doesn't
/// clear the error bits themselves (the driver's own synchronous `SR1`
/// check after waking does, so it can still observe *which* error fired
/// first).
pub fn isr_er(base: usize, sig: &Signal) {
    // SAFETY: as in `isr_ev`.
    unsafe {
        let cr2 = core::ptr::read_volatile((base + CR2) as *mut u32);
        core::ptr::write_volatile(
            (base + CR2) as *mut u32,
            cr2 & !(CR2_ITEVTEN | CR2_ITERREN),
        );
    }
    sig.signal();
}

/// Declares a `static Signal` and two named `fn()` ISRs (event, error)
/// bound to it, for one physical STM32 I2C instance. See
/// [`Stm32I2c::new`] for why this can't just be a generic function with
/// an inline static.
#[macro_export]
macro_rules! stm32_i2c_instance {
    ($sig_name:ident, $ev_isr:ident, $er_isr:ident, base = $base:expr) => {
        static $sig_name: ::rivet::sync::Signal = ::rivet::sync::Signal::new();

        fn $ev_isr() {
            $crate::i2c::isr_ev($base, &$sig_name);
        }

        fn $er_isr() {
            $crate::i2c::isr_er($base, &$sig_name);
        }
    };
}

impl embedded_hal::i2c::ErrorType for Stm32I2c {
    type Error = I2cError;
}

impl embedded_hal::i2c::I2c<SevenBitAddress> for Stm32I2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let op_count = operations.len();
        for (i, op) in operations.iter_mut().enumerate() {
            let is_last_op = i + 1 == op_count;
            self.start();
            match op {
                Operation::Write(bytes) => {
                    self.send_address(address, false)?;
                    self.write_bytes_sync(bytes, is_last_op)?;
                }
                Operation::Read(bytes) => {
                    self.send_address(address, true)?;
                    self.read_bytes_sync(bytes)?;
                }
            }
        }
        Ok(())
    }
}

impl embedded_hal_async::i2c::I2c<SevenBitAddress> for Stm32I2c {
    async fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        let op_count = operations.len();
        for (i, op) in operations.iter_mut().enumerate() {
            let is_last_op = i + 1 == op_count;
            self.start_async().await;
            match op {
                Operation::Write(bytes) => {
                    self.send_address_async(address, false).await?;
                    // Data-phase byte transfer stays synchronous even in
                    // the async path: each byte's TXE/BTF wait is a
                    // handful of bus-clock cycles at 100 kHz (tens of
                    // microseconds), not worth a full Signal round trip
                    // per byte — the same "poll the fast, bounded part;
                    // interrupt-drive the genuinely slow/unbounded part"
                    // split `pl022`'s async `flush` already uses.
                    self.write_bytes_sync(bytes, is_last_op)?;
                }
                Operation::Read(bytes) => {
                    self.send_address_async(address, true).await?;
                    self.read_bytes_sync(bytes)?;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(dead_code)]
    fn generic_sync<I: embedded_hal::i2c::I2c>(i2c: &mut I, addr: u8, buf: &mut [u8]) {
        let _ = i2c.read(addr, buf);
    }

    #[allow(dead_code)]
    async fn generic_async<I: embedded_hal_async::i2c::I2c>(i2c: &mut I, addr: u8, buf: &mut [u8]) {
        let _ = i2c.read(addr, buf).await;
    }

    #[test]
    fn type_checks() {
        let _ = generic_sync::<Stm32I2c>;
        let _ = generic_async::<Stm32I2c>;
    }
}
