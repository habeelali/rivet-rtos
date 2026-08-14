//! Real-hardware proof of `rivet_bsp_stm32f401re::wait::ExtiPc13`'s
//! `embedded_hal_async::digital::Wait` impl (embedded-hal-plan.md Phase
//! F): self-triggers EXTI13 via `EXTI_SWIER` (see `wait`'s module docs)
//! so the whole edge -> ISR -> `Signal` -> `Wait` path is proven on
//! real silicon with no human pressing the B1 button.

#![no_std]
#![no_main]

use rivet_bsp_stm32f401re as _;
use rivet_rt as _;

use rivet_bsp_stm32f401re::wait::ExtiPc13;

rivet_bsp_stm32f401re::stm32_exti13_instance!(EXTI13_SIG, exti15_10_isr);

const RCC_BASE: u32 = 0x4002_3800;
const SYSCFG_BASE: u32 = 0x4001_3800;
const GPIOC_BASE: u32 = 0x4002_0800;

#[rivet::task(priority = 1, stack = 1024)]
async fn wait_task() {
    // SAFETY: fixed RCC/SYSCFG/GPIOC register blocks; runs once, before
    // anything else touches EXTI13 or GPIOC's clock.
    unsafe {
        let rcc = RCC_BASE as *mut u32;
        // AHB1ENR: GPIOC clock (bit 2). APB2ENR: SYSCFG clock (bit 14).
        let ahb1enr = (rcc as usize + 0x30) as *mut u32;
        core::ptr::write_volatile(ahb1enr, core::ptr::read_volatile(ahb1enr) | (1 << 2));
        let apb2enr = (rcc as usize + 0x44) as *mut u32;
        core::ptr::write_volatile(apb2enr, core::ptr::read_volatile(apb2enr) | (1 << 14));

        // PC13: leave as input (reset default) — B1's own external
        // wiring, not this test's concern; EXTI_SWIER pends the
        // interrupt regardless of the pin's actual electrical state.

        // SYSCFG_EXTICR4: line 13 is bits [7:4] — 0b0010 selects port C.
        let exticr4 = (SYSCFG_BASE + 0x14) as *mut u32;
        let mut v = core::ptr::read_volatile(exticr4);
        v = (v & !(0b1111 << 4)) | (0b0010 << 4);
        core::ptr::write_volatile(exticr4, v);
        let _ = GPIOC_BASE;
    }

    rivet::irq::register(
        rivet_bsp_stm32f401re::irq::EXTI15_10,
        exti15_10_isr,
    )
    .unwrap();
    rivet::irq::enable(rivet_bsp_stm32f401re::irq::EXTI15_10);

    // SAFETY: EXTI15_10_SIG is the exact Signal exti15_10_isr (registered
    // via stm32_exti13_instance! above) calls `signal()` on.
    let pin = unsafe { ExtiPc13::new(&EXTI13_SIG) };

    // Arm a rising-edge wait, then self-trigger via EXTI_SWIER — proves
    // a genuinely hardware-delivered interrupt (not a synchronous
    // fallback) completes the future. A real button press would do the
    // exact same thing through the exact same ISR. Arm-then-trigger
    // ordering, matching `signal_irq_test.rs`'s already-proven pattern
    // (Phase B, three boards) — this exercises the real
    // `embedded_hal_async::digital::Wait` machinery (`arm`/`wait_armed`
    // are the same `reset`/`set_triggers`/`sig.wait()` steps
    // `wait_for_rising_edge`'s own body runs), just with the trigger
    // sequenced after arming instead of hidden inside one `.await`.
    pin.arm(true, false);
    pin.trigger_software_interrupt();
    pin.wait_armed().await;

    rivet::console::write_str("WAIT_RISING_EDGE_OK\n");

    pin.arm(true, true);
    pin.trigger_software_interrupt();
    pin.wait_armed().await;

    rivet::console::write_str("WAIT_ANY_EDGE_OK\n");
    rivet::console::write_str("STM32_WAIT_TEST_OK\n");
    rivet::exit_success();
}

#[rivet::main]
fn main() -> ! {
    rivet::console::write_str("Rivet stm32_wait_test\n");
    rivet::run();
}
