#![no_std]
//! Raspberry Pi 3 Model B (BCM2837, quad Cortex-A53) board support.
//!
//! This is an early bring-up port: it provides the MMIO map, GPIO pin
//! muxing and the board's two UARTs, and nothing else. It deliberately
//! does not depend on `rivet` and implements none of the `__rivet_board_*`
//! symbols yet.
//!
//! The reason is specific to this SoC rather than a matter of taste.
//! With the MMU off, AArch64 treats every access as Device-nGnRnE
//! memory, and the load/store-exclusive instructions that back
//! `AtomicUsize::fetch_add`, `compare_exchange` and friends take a
//! synchronous data abort there (ESR data fault status code 0x35,
//! "unsupported exclusive or atomic access"). The kernel uses atomic
//! read-modify-write in a few dozen places, so it cannot run on this
//! board until an identity map exists with RAM described as Normal
//! Inner-Shareable Write-Back memory. Plain atomic loads and stores are
//! fine; it is only the exclusive-monitor forms that fault.
//!
//! Note that the `atomics-polyfill` approach used for the RP2040 is not
//! a way around this. That works by masking interrupts, which buys
//! atomicity against preemption on a single core and nothing at all
//! against the other three coherent A53s on this chip.
//!
//! Everything below is therefore plain volatile register access.

use core::ptr::{read_volatile, write_volatile};

pub mod boot;
pub use boot::drop_to_el1;

/// Physical addresses of the peripherals this crate touches.
pub mod mmio {
    /// ARM-side physical base of the peripheral window.
    ///
    /// BCM2837 only. It is 0x2000_0000 on BCM2835 (Pi 1, Zero) and
    /// 0xFE00_0000 on BCM2711 (Pi 4), so this constant is the single
    /// thing most likely to be wrong if this crate is ever pointed at a
    /// different board.
    pub const PERIPHERAL_BASE: usize = 0x3F00_0000;

    pub const GPIO_BASE: usize = PERIPHERAL_BASE + 0x0020_0000;
    /// PL011, the "real" UART. Reachable on GPIO14/15 only once the
    /// Bluetooth modem has been moved off them (`dtoverlay=disable-bt`).
    pub const PL011_BASE: usize = PERIPHERAL_BASE + 0x0020_1000;
    /// Auxiliary peripheral block, containing the mini UART.
    pub const AUX_BASE: usize = PERIPHERAL_BASE + 0x0021_5000;

    // GPIO register offsets.
    pub const GPFSEL0: usize = 0x00;
    /// Function select for GPIO 10-19, three bits per pin.
    pub const GPFSEL1: usize = 0x04;
    pub const GPSET0: usize = 0x1C;
    pub const GPCLR0: usize = 0x28;
    pub const GPLEV0: usize = 0x34;
    /// Pull-up/down control. BCM2837 uses this BCM2835-style handshake;
    /// the GPIO_PUP_PDN_CNTRL_REGn scheme at 0xE4 is BCM2711-only and
    /// does not exist on this chip.
    pub const GPPUD: usize = 0x94;
    pub const GPPUDCLK0: usize = 0x98;

    // PL011 register offsets.
    pub const PL011_DR: usize = 0x00;
    pub const PL011_FR: usize = 0x18;
    pub const PL011_IBRD: usize = 0x24;
    pub const PL011_FBRD: usize = 0x28;
    pub const PL011_LCRH: usize = 0x2C;
    pub const PL011_CR: usize = 0x30;
    pub const PL011_IMSC: usize = 0x38;
    pub const PL011_ICR: usize = 0x44;

    // PL011 flag register bits.
    /// Transmitter busy: set while a character is still being shifted out.
    pub const PL011_FR_BUSY: u32 = 1 << 3;
    pub const PL011_FR_RXFE: u32 = 1 << 4;
    pub const PL011_FR_TXFF: u32 = 1 << 5;

    // AUX / mini UART register offsets.
    /// Gates the whole mini UART. Every other AUX register reads as zero
    /// and ignores writes until bit 0 here is set, so this must be the
    /// first AUX access.
    pub const AUX_ENABLES: usize = 0x04;
    pub const AUX_MU_IO: usize = 0x40;
    pub const AUX_MU_IER: usize = 0x44;
    pub const AUX_MU_IIR: usize = 0x48;
    pub const AUX_MU_LCR: usize = 0x4C;
    pub const AUX_MU_MCR: usize = 0x50;
    pub const AUX_MU_LSR: usize = 0x54;
    pub const AUX_MU_CNTL: usize = 0x60;
    pub const AUX_MU_BAUD: usize = 0x68;

    /// Mini UART line status: transmit holding register empty.
    pub const AUX_MU_LSR_TX_EMPTY: u32 = 1 << 5;
}

use mmio::*;

/// GPFSEL function codes. Note that the alternate-function encodings are
/// not in numeric order: ALT0 is 0b100 but ALT5 is 0b010.
pub mod fsel {
    pub const INPUT: u32 = 0b000;
    pub const OUTPUT: u32 = 0b001;
    /// GPIO14/15 as PL011 TXD0/RXD0.
    pub const ALT0: u32 = 0b100;
    /// GPIO14/15 as mini UART TXD1/RXD1.
    pub const ALT5: u32 = 0b010;
}

/// Busy-wait for roughly `iterations` NOPs.
///
/// The GPPUD handshake below needs a real delay that survives
/// optimisation, so this cannot be a plain empty loop.
#[inline(never)]
pub fn delay(iterations: u32) {
    for _ in 0..iterations {
        // SAFETY: a NOP touches nothing.
        unsafe { core::arch::asm!("nop", options(nomem, nostack, preserves_flags)) };
    }
}

/// Point GPIO14 and GPIO15 (header pins 8 and 10) at function `f`, then
/// release their pull-up/pull-down.
///
/// # Safety
/// Writes GPIO registers directly; the caller must not be racing another
/// agent that is also reconfiguring these pins.
pub unsafe fn mux_uart_pins(f: u32) {
    let gpfsel1 = (GPIO_BASE + GPFSEL1) as *mut u32;
    let mut v = read_volatile(gpfsel1);
    // FSEL14 is bits 14:12, FSEL15 is bits 17:15.
    v &= !((0b111u32 << 12) | (0b111u32 << 15));
    v |= (f << 12) | (f << 15);
    write_volatile(gpfsel1, v);

    // The BCM2835 datasheet's pull-up/down sequence (section 6.1): set
    // the direction, wait 150 cycles for it to register, clock it into
    // the target pads, wait again, then remove both the value and the
    // clock. The waits are required, not defensive.
    let gppud = (GPIO_BASE + GPPUD) as *mut u32;
    let gppudclk0 = (GPIO_BASE + GPPUDCLK0) as *mut u32;
    write_volatile(gppud, 0); // 0b00 = disable pull-up/down
    delay(150);
    write_volatile(gppudclk0, (1 << 14) | (1 << 15));
    delay(150);
    write_volatile(gppud, 0);
    write_volatile(gppudclk0, 0);
}

/// The PL011 UART, on GPIO14/15 via ALT0.
///
/// Preferred over the mini UART because its clock is the fixed 48 MHz
/// `init_uart_clock` rather than the VPU core clock, which moves with
/// DVFS and takes the mini UART's baud rate with it.
pub struct Pl011;

impl Pl011 {
    /// Configure the pins and the UART for 8N1 at the given baud.
    ///
    /// `uart_clk_hz` is whatever `init_uart_clock` was set to in
    /// config.txt, 48 MHz by default.
    ///
    /// # Safety
    /// Reconfigures shared GPIO and UART registers.
    pub unsafe fn init(&self, uart_clk_hz: u32, baud: u32) {
        let base = PL011_BASE;

        // Disable before touching anything else, so a half-applied
        // configuration never drives the line.
        write_volatile((base + PL011_CR) as *mut u32, 0);

        mux_uart_pins(fsel::ALT0);

        write_volatile((base + PL011_ICR) as *mut u32, 0x7FF); // clear all interrupts

        // 16x oversampling: divisor = clk / (16 * baud), split into a
        // 16-bit integer part and a 6-bit fraction. Computed here as a
        // single 64ths-scaled value so the fraction is rounded rather
        // than truncated: at 48 MHz/115200 that is 26 + 3/64 (115177
        // baud, 0.02% off) where truncating would give 26 + 2/64.
        let scaled = uart_clk_hz as u64 * 4; // = 64 * clk / 16
        let div64 = (scaled + baud as u64 / 2) / (baud as u64);
        let ibrd = (div64 / 64) as u32;
        let fbrd = (div64 % 64) as u32;
        write_volatile((base + PL011_IBRD) as *mut u32, ibrd);
        write_volatile((base + PL011_FBRD) as *mut u32, fbrd);

        // Writing LCRH is what actually commits IBRD/FBRD, so it has to
        // come after them.
        // FEN (bit 4) enables the FIFOs, WLEN 0b11 (bits 6:5) is 8 bits.
        write_volatile((base + PL011_LCRH) as *mut u32, (1 << 4) | (0b11 << 5));

        write_volatile((base + PL011_IMSC) as *mut u32, 0); // polled, no interrupts

        // UARTEN | TXE | RXE
        write_volatile((base + PL011_CR) as *mut u32, 1 | (1 << 8) | (1 << 9));
    }

    /// Read back the divisors the UART is currently programmed with.
    ///
    /// Sampling this before `init` reports what the firmware left
    /// behind, which is the only way to observe the real UARTCLK from
    /// software: a wrong `init_uart_clock` shows up here as divisors
    /// that do not match the ones computed above.
    ///
    /// # Safety
    /// Reads PL011 registers directly.
    pub unsafe fn divisors(&self) -> (u32, u32) {
        (
            read_volatile((PL011_BASE + PL011_IBRD) as *const u32),
            read_volatile((PL011_BASE + PL011_FBRD) as *const u32),
        )
    }

    /// Read back the control register.
    ///
    /// # Safety
    /// Reads a PL011 register directly.
    pub unsafe fn control(&self) -> u32 {
        read_volatile((PL011_BASE + PL011_CR) as *const u32)
    }

    /// Write one byte, blocking while the transmit FIFO is full.
    ///
    /// # Safety
    /// Writes PL011 registers directly, and spins forever if the UART is
    /// wedged with a full FIFO.
    pub unsafe fn put_byte(&self, b: u8) {
        while read_volatile((PL011_BASE + PL011_FR) as *const u32) & PL011_FR_TXFF != 0 {}
        write_volatile((PL011_BASE + PL011_DR) as *mut u32, b as u32);
    }

    /// Block until the transmitter has fully drained.
    ///
    /// Required before halting or before re-muxing the pins, otherwise
    /// the last FIFO-full of characters is lost and a working boot looks
    /// like a failed one.
    ///
    /// # Safety
    /// Reads a PL011 register directly, and spins until the transmitter
    /// reports idle.
    pub unsafe fn flush(&self) {
        while read_volatile((PL011_BASE + PL011_FR) as *const u32) & PL011_FR_BUSY != 0 {}
    }
}

impl core::fmt::Write for Pl011 {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                // SAFETY: MMIO write to a UART configured by init().
                unsafe { self.put_byte(b'\r') };
            }
            unsafe { self.put_byte(b) };
        }
        Ok(())
    }
}

/// The mini UART (UART1), on GPIO14/15 via ALT5.
///
/// This is what GPIO14/15 default to on a Pi 3, because the PL011 is
/// wired to the Bluetooth modem out of reset. Kept here as a fallback
/// so a single boot can prove which of the two actually reaches the
/// header pins.
pub struct MiniUart;

impl MiniUart {
    /// Configure the pins and the mini UART for 8N1 at the given baud.
    ///
    /// `core_freq_hz` must match whatever `core_freq` is pinned to in
    /// config.txt, since this UART derives its baud rate from the VPU
    /// core clock rather than a fixed reference.
    ///
    /// # Safety
    /// Reconfigures shared GPIO and AUX registers.
    pub unsafe fn init(&self, core_freq_hz: u32, baud: u32) {
        let base = AUX_BASE;

        // Must come first: the rest of the AUX block is inert until the
        // mini UART is enabled here.
        let en = read_volatile((base + AUX_ENABLES) as *const u32);
        write_volatile((base + AUX_ENABLES) as *mut u32, en | 1);

        write_volatile((base + AUX_MU_CNTL) as *mut u32, 0); // rx/tx off while configuring
        write_volatile((base + AUX_MU_IER) as *mut u32, 0);

        // The datasheet describes this register as selecting 8-bit mode
        // with bit 0 alone, which is wrong: both low bits are needed.
        write_volatile((base + AUX_MU_LCR) as *mut u32, 0b11);
        write_volatile((base + AUX_MU_MCR) as *mut u32, 0);
        write_volatile((base + AUX_MU_IIR) as *mut u32, 0xC6); // clear and enable both FIFOs

        mux_uart_pins(fsel::ALT5);

        // baud = core_freq / (8 * (reg + 1))
        let reg = (core_freq_hz / (8 * baud)).saturating_sub(1);
        write_volatile((base + AUX_MU_BAUD) as *mut u32, reg);

        write_volatile((base + AUX_MU_CNTL) as *mut u32, 0b11); // tx + rx enable
    }

    /// Write one byte, blocking until the holding register is empty.
    ///
    /// # Safety
    /// Writes AUX registers directly, and spins forever if the UART is
    /// wedged.
    pub unsafe fn put_byte(&self, b: u8) {
        while read_volatile((AUX_BASE + AUX_MU_LSR) as *const u32) & AUX_MU_LSR_TX_EMPTY == 0 {}
        write_volatile((AUX_BASE + AUX_MU_IO) as *mut u32, b as u32);
    }

    /// Block until the transmit holding register has drained.
    ///
    /// # Safety
    /// Reads an AUX register directly, and spins until it reports empty.
    pub unsafe fn flush(&self) {
        while read_volatile((AUX_BASE + AUX_MU_LSR) as *const u32) & AUX_MU_LSR_TX_EMPTY == 0 {}
    }
}

impl core::fmt::Write for MiniUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                // SAFETY: MMIO write to a UART configured by init().
                unsafe { self.put_byte(b'\r') };
            }
            unsafe { self.put_byte(b) };
        }
        Ok(())
    }
}
