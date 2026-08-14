//! Stellaris/LM3S6965 I2C Master driver: `embedded_hal::i2c::I2c`
//! (polling) and `embedded_hal_async::i2c::I2c`
//! (completion via a real interrupt through [`rivet::sync::Signal`]) on
//! the same struct — only `transaction()` is a required method on
//! either trait, `read`/`write`/`write_read` come free as default
//! methods that call it.
//!
//! This is the CI reference for async I2C, run against a real
//! `at24c-eeprom` QEMU device on `lm3s6965evb` (`stellaris-i2c` at
//! `0x4002_0000`, NVIC IRQ 8). Three real quirks in QEMU's model of this
//! peripheral (confirmed directly against `hw/i2c/bitbang.c`/
//! `hw/arm/stellaris.c`'s `stellaris_i2c` device on this machine's QEMU
//! 8.2.2, not assumed from the datasheet) shape this driver:
//!
//! 1. **Address-NAK raises no interrupt.** The model sets `MCS.ERROR`
//!    (and halts the transfer) *before* it would set `MRIS` — so a
//!    failed transaction never fires the ISR. An await-only design would
//!    hang forever on a NAK. This driver checks `MCS` synchronously
//!    immediately after every command, before ever calling
//!    [`Signal::wait`] — since this model also completes successful
//!    transfers instantaneously inside the MMIO store (the ISR has
//!    *already* fired by the time the next instruction runs), that same
//!    synchronous check doubles as the fast path for success too; `wait`
//!    is the fallback for real hardware, where the transfer is still
//!    genuinely in flight at that point.
//! 2. **`MIMR` can never be re-masked once set** — writing *any* value
//!    enables the interrupt permanently in this model. The ISR
//!    ([`isr_ack`]) clears the condition via `MICR` (which the real
//!    hardware protocol requires anyway), never by touching `MIMR`.
//! 3. **Repeated START is broken** — `start_transfer` is skipped when
//!    `BUSBSY` is already set, so a write-then-repeated-start-read never
//!    changes bus direction. This driver issues a real STOP after a
//!    `Write` operation that isn't the transaction's last, then a fresh
//!    START for the next operation, instead of a true repeated start.
//!    The AT24C EEPROM's internal address pointer persists across a
//!    STOP/START the same way it would across a repeated START, so the
//!    read-back this is tested against still succeeds — this is a real,
//!    documented divergence from the I2C spec's repeated-start contract,
//!    not something to rely on against a stricter real slave device.

use embedded_hal::i2c::{Operation, SevenBitAddress};
use rivet::sync::Signal;

const I2CMSA: usize = 0x000;
const I2CMCS: usize = 0x004;
const I2CMDR: usize = 0x008;
const I2CMTPR: usize = 0x00C;
const I2CMIMR: usize = 0x010;
const I2CMICR: usize = 0x01C;
const I2CMCR: usize = 0x020;

const CS_RUN: u32 = 1 << 0;
const CS_START: u32 = 1 << 1;
const CS_STOP: u32 = 1 << 2;
const CS_ACK: u32 = 1 << 3;
const CS_BUSY: u32 = 1 << 0;
const CS_ERROR: u32 = 1 << 1;

const CR_MFE: u32 = 1 << 4;
const IMR_IM: u32 = 1 << 0;
const ICR_IC: u32 = 1 << 0;

/// A single failed transaction: the slave NAK'd an address or data byte,
/// or the controller lost arbitration. This driver doesn't distinguish
/// which — every `MCS.ERROR` case maps here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Nak;

impl embedded_hal::i2c::Error for Nak {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        embedded_hal::i2c::ErrorKind::NoAcknowledge(
            embedded_hal::i2c::NoAcknowledgeSource::Unknown,
        )
    }
}

/// One physical Stellaris I2C Master controller: a fixed register base
/// and the [`Signal`] its interrupt handler completes.
///
/// Construct via [`stellaris_i2c_instance!`], not directly — see
/// `rivet_bsp_support::pl022::Pl022::new`'s doc for why a bare `fn
/// isr<const BASE: usize>()` with an inline `static Signal` wouldn't work
/// (statics inside a generic function aren't monomorphized per
/// instantiation).
pub struct StellarisI2c {
    base: usize,
    sig: &'static Signal,
}

impl StellarisI2c {
    /// # Safety
    /// `base` must be a real, exclusively-owned Stellaris I2C Master
    /// register block, and `sig` must be the exact [`Signal`] the ISR
    /// registered for this instance calls [`Signal::signal`] on.
    pub const unsafe fn new(base: usize, sig: &'static Signal) -> Self {
        Self { base, sig }
    }

    fn reg(&self, offset: usize) -> *mut u32 {
        (self.base + offset) as *mut u32
    }

    /// Bring the controller up: master function enable, a conservative
    /// timer-period divisor (untuned — this model's transfers are
    /// instantaneous regardless; real hardware picks its own for the
    /// target bus speed), and unmask the interrupt once up front (see
    /// module docs — this model can never re-mask it anyway).
    pub fn init(&self) {
        // SAFETY: fixed Stellaris I2C registers, exclusively owned per
        // the constructor's contract.
        unsafe {
            core::ptr::write_volatile(self.reg(I2CMCR), CR_MFE);
            core::ptr::write_volatile(self.reg(I2CMTPR), 7);
            core::ptr::write_volatile(self.reg(I2CMIMR), IMR_IM);
        }
    }

    fn set_address(&self, address: u8, read: bool) {
        // SAFETY: as in `init`.
        unsafe {
            let val = ((address as u32) << 1) | u32::from(read);
            core::ptr::write_volatile(self.reg(I2CMSA), val);
        }
    }

    fn mcs(&self) -> u32 {
        // SAFETY: as in `init`.
        unsafe { core::ptr::read_volatile(self.reg(I2CMCS)) }
    }

    fn write_dr(&self, byte: u8) {
        // SAFETY: as in `init`.
        unsafe { core::ptr::write_volatile(self.reg(I2CMDR), byte as u32) };
    }

    fn read_dr(&self) -> u8 {
        // SAFETY: as in `init`.
        unsafe { core::ptr::read_volatile(self.reg(I2CMDR)) as u8 }
    }

    fn run_sync(&self, cmd: u32) -> Result<(), Nak> {
        // SAFETY: as in `init`. Real hardware genuinely needs this poll
        // (the model just resolves it instantaneously).
        unsafe { core::ptr::write_volatile(self.reg(I2CMCS), cmd) };
        while self.mcs() & CS_BUSY != 0 {
            core::hint::spin_loop();
        }
        if self.mcs() & CS_ERROR != 0 {
            Err(Nak)
        } else {
            Ok(())
        }
    }

    /// See module docs item 1: this model resolves both success and
    /// failure synchronously inside the `MCS` write, so the immediate
    /// post-write check below is what actually observes the outcome in
    /// practice; `Signal::wait` is the correct fallback for real
    /// hardware, where the transfer may still be genuinely in flight.
    async fn run_async(&self, cmd: u32) -> Result<(), Nak> {
        self.sig.reset();
        // SAFETY: as in `init`.
        unsafe { core::ptr::write_volatile(self.reg(I2CMCS), cmd) };
        if self.mcs() & CS_BUSY == 0 {
            let status = self.mcs();
            // The ISR may have already fired (success case) with
            // nothing left registered to wake — consume the latch so it
            // doesn't leak into the next command.
            self.sig.try_take();
            return if status & CS_ERROR != 0 { Err(Nak) } else { Ok(()) };
        }
        self.sig.wait().await;
        if self.mcs() & CS_ERROR != 0 {
            Err(Nak)
        } else {
            Ok(())
        }
    }
}

/// Shared ISR body for every Stellaris I2C instance, called from the
/// `fn()` [`stellaris_i2c_instance!`] generates. Clears the interrupt
/// condition via `MICR` (module docs item 2 — `MIMR` can't be
/// re-masked in this model) and hands off via `Signal`.
pub fn isr_ack(base: usize, sig: &Signal) {
    // SAFETY: `base` is a real Stellaris I2C register block passed by
    // the `stellaris_i2c_instance!` caller, who owns it exclusively.
    unsafe {
        core::ptr::write_volatile((base + I2CMICR) as *mut u32, ICR_IC);
    }
    sig.signal();
}

/// Declares a `static Signal` and a named `fn()` ISR bound to it, for
/// one physical Stellaris I2C instance's completion interrupt. See
/// [`StellarisI2c::new`] for why this can't just be a generic function
/// with an inline static.
#[macro_export]
macro_rules! stellaris_i2c_instance {
    ($sig_name:ident, $isr_name:ident, base = $base:expr) => {
        static $sig_name: ::rivet::sync::Signal = ::rivet::sync::Signal::new();

        fn $isr_name() {
            $crate::stellaris_i2c::isr_ack($base, &$sig_name);
        }
    };
}

/// Runs `operations` against `address`, generic over the sync/async byte
/// transfer primitive (`run_sync`/`run_async`, threaded through as a
/// plain function pointer's worth of inlined logic via the two thin
/// trait impls below — kept as one shared function so the STOP/START
/// bookkeeping (module docs item 3) exists in exactly one place).
macro_rules! impl_transaction {
    ($self:ident, $address:ident, $operations:ident, $run:ident $(. $await_kw:ident)?) => {{
        let op_count = $operations.len();
        for (i, op) in $operations.iter_mut().enumerate() {
            let is_last_op = i + 1 == op_count;
            match op {
                Operation::Write(bytes) => {
                    $self.set_address($address, false);
                    let n = bytes.len();
                    for (j, &b) in bytes.iter().enumerate() {
                        $self.write_dr(b);
                        let is_first = j == 0;
                        let is_last = j + 1 == n;
                        let mut cmd = CS_RUN;
                        if is_first {
                            cmd |= CS_START;
                        }
                        if is_last {
                            // STOP after every Write's last byte — even
                            // when another operation follows — since
                            // repeated START is broken in this model
                            // (item 3); the next operation issues a
                            // fresh START instead of relying on one.
                            cmd |= CS_STOP;
                        }
                        $self.$run(cmd)$(.$await_kw)??;
                    }
                }
                Operation::Read(bytes) => {
                    $self.set_address($address, true);
                    let n = bytes.len();
                    for (j, slot) in bytes.iter_mut().enumerate() {
                        let is_first = j == 0;
                        let is_last = j + 1 == n;
                        let mut cmd = CS_RUN;
                        if is_first {
                            cmd |= CS_START;
                        }
                        if is_last {
                            cmd |= CS_STOP;
                        } else {
                            cmd |= CS_ACK;
                        }
                        $self.$run(cmd)$(.$await_kw)??;
                        *slot = $self.read_dr();
                    }
                }
            }
            let _ = is_last_op;
        }
        Ok(())
    }};
}

impl embedded_hal::i2c::ErrorType for StellarisI2c {
    type Error = Nak;
}

impl embedded_hal::i2c::I2c<SevenBitAddress> for StellarisI2c {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        impl_transaction!(self, address, operations, run_sync)
    }
}

impl embedded_hal_async::i2c::I2c<SevenBitAddress> for StellarisI2c {
    async fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        impl_transaction!(self, address, operations, run_async.await)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Compile-only: proves `StellarisI2c` is usable through generic
    // embedded-hal/-async code, not just directly, for both trait
    // families on the same struct.
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
        let _ = generic_sync::<StellarisI2c>;
        let _ = generic_async::<StellarisI2c>;
    }
}
