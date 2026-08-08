//! Debug console — the board's UART/semihosting/whatever, reached through
//! [`crate::port::board`]. Replaces the old `rivet::arch::debug_print`;
//! application code should use this module (or [`crate::print!`] /
//! [`crate::println!`]) instead of talking to the port directly.

use core::fmt::{self, Write};

pub fn write_str(s: &str) {
    crate::port::board::console_write(s.as_bytes());
}

pub fn write_bytes(bytes: &[u8]) {
    crate::port::board::console_write(bytes);
}

struct Console;

impl Write for Console {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        write_str(s);
        Ok(())
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments) {
    // A formatting error here would mean a `fmt::Write` impl returned
    // `Err` for a plain UART byte write, which never fails.
    let _ = Console.write_fmt(args);
}

/// Write formatted text to the debug console. See [`println!`] for a
/// version that appends a newline.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {{
        $crate::console::_print(core::format_args!($($arg)*));
    }};
}

/// Write formatted text to the debug console, followed by a newline.
#[macro_export]
macro_rules! println {
    () => { $crate::print!("\n") };
    ($($arg:tt)*) => {{
        $crate::console::_print(core::format_args!($($arg)*));
        $crate::print!("\n");
    }};
}
