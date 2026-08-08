//! Hardware abstraction layer. Currently: typestate GPIO for Cortex-M
//! targets (LM3S6965). Extending to other peripherals/architectures is
//! future work — see the crate-level "not implemented" list.

#[cfg(any(target_arch = "arm", test))]
pub mod gpio;
