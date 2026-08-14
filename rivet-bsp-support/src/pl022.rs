//! PL022 SSP/SPI driver (embedded-hal-plan.md Phase C, the reference
//! async peripheral): `embedded_hal::spi::SpiBus` (polling, safe from a
//! preemptive task) and `embedded_hal_async::spi::SpiBus` (RX-ready
//! completion via [`rivet::sync::Signal`], driven by a real hardware
//! interrupt) on the same struct — same "sync trait for the preemptive
//! tier, async trait for the cooperative tier" split `RivetDelay`
//! resolves for `DelayNs`, just for a peripheral whose async completion
//! is a genuine ISR instead of a timer.
//!
//! Not board-agnostic the way `ns16550`/`serial` are: PL022 is the SPI
//! block on lm3s6965evb and mps2-an385 specifically (both QEMU-modeled,
//! both support `CR1.LBM` loopback — TX FIFO copied straight to RX FIFO
//! in the emulator, no external device needed, which is what makes it a
//! real, deterministic, CI-testable interrupt path). A board owns the
//! base address and IRQ number; this module owns the register protocol.
//!
//! Two QEMU model quirks shape the async design (confirmed against this
//! machine's QEMU 8.2.2 PL022 model directly, not just the datasheet):
//! - `RXIM` only asserts once the RX FIFO holds >= 4 bytes — shorter
//!   transfers never raise it under emulation. Callers doing real async
//!   transfers should use >= 4-byte chunks; this driver still completes
//!   shorter reads/writes correctly on real PL022 hardware, where RTIM
//!   (never modelled in QEMU) covers the short case.
//! - `TXIM` is essentially always asserted (transfers are instantaneous
//!   in the model), so enabling it without immediately masking it is an
//!   interrupt storm. This driver only ever unmasks `RXIM`.

use core::convert::Infallible;

use rivet::sync::Signal;

const CR0: usize = 0x000;
const CR1: usize = 0x004;
const DR: usize = 0x008;
const SR: usize = 0x00C;
const CPSR: usize = 0x010;
const IMSC: usize = 0x014;

const SR_TNF: u32 = 1 << 1; // TX FIFO not full
const SR_RNE: u32 = 1 << 2; // RX FIFO not empty
const SR_BSY: u32 = 1 << 4; // controller busy

const CR1_LBM: u32 = 1 << 0; // loopback: TX FIFO feeds RX FIFO directly
const CR1_SSE: u32 = 1 << 1; // synchronous serial enable

const IMSC_RXIM: u32 = 1 << 2;

/// Real PL022 hardware FIFO depth (ARM PL022 TRM) — the unit this
/// driver's async path completes in per hardware round trip; longer
/// buffers are chunked automatically.
const FIFO_DEPTH: usize = 8;

/// One physical PL022 controller: a fixed register base and the
/// [`Signal`] its interrupt handler completes.
///
/// Construct via [`pl022_instance!`], not `new` directly at a call site
/// that doesn't also declare the matching ISR — the macro is what
/// guarantees the `Signal` a given base's ISR touches is the same one
/// this handle reads. A hand-written `fn isr<const BASE: usize>()` with
/// an inline `static Signal` would *not* get a separate static per
/// const-generic instantiation; every base address's ISR would silently
/// share one `Signal`.
pub struct Pl022 {
    base: usize,
    sig: &'static Signal,
}

impl Pl022 {
    /// # Safety
    /// `base` must be a real, exclusively-owned PL022 register block,
    /// and `sig` must be the exact [`Signal`] the ISR registered for
    /// this instance (see [`pl022_instance!`]) calls
    /// [`Signal::signal`] on.
    pub const unsafe fn new(base: usize, sig: &'static Signal) -> Self {
        Self { base, sig }
    }

    fn reg(&self, offset: usize) -> *mut u32 {
        (self.base + offset) as *mut u32
    }

    /// Bring the controller up: master mode (`CR1.MS` = 0), 8-bit
    /// Motorola SPI frames, optionally looped back internally. `CPSR`/
    /// `CR0.SCR` set the bit rate (`SSPCLK / (CPSR * (1 + SCR))`); the
    /// fixed divisor here is untuned since QEMU's model transfers
    /// instantaneously regardless — a real-hardware board picks its own.
    pub fn init(&self, loopback: bool) {
        // SAFETY: fixed PL022 registers, exclusively owned per the
        // constructor's contract.
        unsafe {
            core::ptr::write_volatile(self.reg(CR1), 0); // SSE=0 while configuring
            core::ptr::write_volatile(self.reg(CPSR), 2); // even divisor >= 2, per the TRM
            core::ptr::write_volatile(self.reg(CR0), 0x07); // DSS=0b0111 -> 8-bit data size
            let mut cr1 = CR1_SSE;
            if loopback {
                cr1 |= CR1_LBM;
            }
            core::ptr::write_volatile(self.reg(CR1), cr1);
        }
    }

    fn tx_byte(&self, b: u8) {
        // SAFETY: see `init`.
        unsafe {
            while core::ptr::read_volatile(self.reg(SR)) & SR_TNF == 0 {
                core::hint::spin_loop();
            }
            core::ptr::write_volatile(self.reg(DR), b as u32);
        }
    }

    fn rx_byte_poll(&self) -> u8 {
        // SAFETY: see `init`.
        unsafe {
            while core::ptr::read_volatile(self.reg(SR)) & SR_RNE == 0 {
                core::hint::spin_loop();
            }
            core::ptr::read_volatile(self.reg(DR)) as u8
        }
    }

    fn transfer_word_sync(&self, tx: u8) -> u8 {
        self.tx_byte(tx);
        self.rx_byte_poll()
    }

    /// One hardware round trip, up to [`FIFO_DEPTH`] bytes: unmask
    /// `RXIM`, push `tx`, await the real interrupt, then drain exactly
    /// `rx.len()` bytes. The ISR ([`isr_ack`]) only masks `RXIM` and
    /// signals — it doesn't drain the FIFO itself, so the awaiting task
    /// does that here, after `wait()` returns. Keeping the ISR body to
    /// "mask + signal" is deliberate: see `docs/driver-authoring.md`'s
    /// guidance on keeping ISR bodies minimal.
    async fn transfer_chunk_async(&self, tx: &[u8], rx: &mut [u8]) {
        debug_assert!(tx.len() <= FIFO_DEPTH && rx.len() <= FIFO_DEPTH);
        self.sig.reset();
        // SAFETY: see `init`.
        unsafe {
            core::ptr::write_volatile(self.reg(IMSC), IMSC_RXIM);
        }
        for &b in tx {
            self.tx_byte(b);
        }
        self.sig.wait().await;
        for slot in rx.iter_mut() {
            *slot = self.rx_byte_poll();
        }
    }

    /// Async transfer of `read`/`write` of possibly different lengths
    /// (`embedded_hal_async::spi::SpiBus::transfer` semantics: write
    /// past `read`'s length is sent and its response discarded; read
    /// past `write`'s length sends `0`), chunked into
    /// [`FIFO_DEPTH`]-sized hardware round trips.
    async fn transfer_async(&self, read: &mut [u8], write: &[u8]) {
        let n = read.len().max(write.len());
        let mut i = 0;
        while i < n {
            let end = (i + FIFO_DEPTH).min(n);
            let chunk_len = end - i;
            let mut tx_buf = [0u8; FIFO_DEPTH];
            for (k, slot) in tx_buf[..chunk_len].iter_mut().enumerate() {
                *slot = write.get(i + k).copied().unwrap_or(0);
            }
            let mut rx_buf = [0u8; FIFO_DEPTH];
            self.transfer_chunk_async(&tx_buf[..chunk_len], &mut rx_buf[..chunk_len])
                .await;
            for (k, &b) in rx_buf[..chunk_len].iter().enumerate() {
                if let Some(slot) = read.get_mut(i + k) {
                    *slot = b;
                }
            }
            i = end;
        }
    }
}

/// Shared ISR body for every PL022 instance, called from the `fn()`
/// [`pl022_instance!`] generates. Masks `RXIM` (level-triggered — it
/// stays asserted until the FIFO drains below threshold, so masking
/// rather than draining here is what stops it re-firing before the
/// awaiting task gets a chance to run) and hands off via `Signal`.
pub fn isr_ack(base: usize, sig: &Signal) {
    // SAFETY: `base` is a real PL022 register block passed by the
    // `pl022_instance!` caller, who owns it exclusively.
    unsafe {
        core::ptr::write_volatile((base + IMSC) as *mut u32, 0);
    }
    sig.signal();
}

/// Declares a `static Signal` and a named `fn()` ISR bound to it, for
/// one physical PL022 instance's completion interrupt. See [`Pl022::new`]
/// for why this can't just be a generic function with an inline static.
///
/// ```ignore
/// rivet_bsp_support::pl022_instance!(SPI0_SIG, spi0_isr, base = 0x4000_8000);
/// let spi0 = unsafe { rivet_bsp_support::pl022::Pl022::new(0x4000_8000, &SPI0_SIG) };
/// rivet::irq::register(IRQ_SPI0, spi0_isr).unwrap();
/// rivet::irq::enable(IRQ_SPI0);
/// ```
#[macro_export]
macro_rules! pl022_instance {
    ($sig_name:ident, $isr_name:ident, base = $base:expr) => {
        static $sig_name: ::rivet::sync::Signal = ::rivet::sync::Signal::new();

        fn $isr_name() {
            $crate::pl022::isr_ack($base, &$sig_name);
        }
    };
}

impl embedded_hal::spi::ErrorType for Pl022 {
    type Error = Infallible;
}

impl embedded_hal::spi::SpiBus<u8> for Pl022 {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            *w = self.transfer_word_sync(0);
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
        for &w in words {
            self.transfer_word_sync(w);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
        let n = read.len().max(write.len());
        for i in 0..n {
            let tx = write.get(i).copied().unwrap_or(0);
            let rx = self.transfer_word_sync(tx);
            if let Some(slot) = read.get_mut(i) {
                *slot = rx;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        for w in words.iter_mut() {
            *w = self.transfer_word_sync(*w);
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        // SAFETY: see `init`.
        unsafe {
            while core::ptr::read_volatile(self.reg(SR)) & SR_BSY != 0 {
                core::hint::spin_loop();
            }
        }
        Ok(())
    }
}

// `embedded_hal_async::spi::ErrorType` is the same trait re-exported from
// `embedded_hal::spi` (see that crate's spi.rs) — one `ErrorType` impl
// above covers both the sync and async `SpiBus` impls below.

impl embedded_hal_async::spi::SpiBus<u8> for Pl022 {
    async fn read(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        self.transfer_async(words, &[]).await;
        Ok(())
    }

    async fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
        self.transfer_async(&mut [], words).await;
        Ok(())
    }

    async fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
        self.transfer_async(read, write).await;
        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        let n = words.len();
        let mut i = 0;
        while i < n {
            let end = (i + FIFO_DEPTH).min(n);
            let chunk_len = end - i;
            let mut tx_buf = [0u8; FIFO_DEPTH];
            tx_buf[..chunk_len].copy_from_slice(&words[i..end]);
            let mut rx_buf = [0u8; FIFO_DEPTH];
            self.transfer_chunk_async(&tx_buf[..chunk_len], &mut rx_buf[..chunk_len])
                .await;
            words[i..end].copy_from_slice(&rx_buf[..chunk_len]);
            i = end;
        }
        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Infallible> {
        // SAFETY: see `init`. Not routed through `Signal`/an interrupt —
        // `BSY` is a fast, bounded poll (bus-idle check), not a
        // multi-byte transfer completion.
        unsafe {
            while core::ptr::read_volatile(self.reg(SR)) & SR_BSY != 0 {
                core::hint::spin_loop();
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Compile-only: proves `Pl022` is usable through generic
    // embedded-hal/-async code, not just directly, for both trait
    // families on the same struct.
    #[allow(dead_code)]
    fn generic_sync<S: embedded_hal::spi::SpiBus<u8>>(s: &mut S, buf: &mut [u8]) {
        let _ = s.transfer_in_place(buf);
    }

    #[allow(dead_code)]
    async fn generic_async<S: embedded_hal_async::spi::SpiBus<u8>>(s: &mut S, buf: &mut [u8]) {
        let _ = s.transfer_in_place(buf).await;
    }

    #[test]
    fn type_checks() {
        let _ = generic_sync::<Pl022>;
        let _ = generic_async::<Pl022>;
    }
}
