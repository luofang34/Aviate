//! STM32H743 chip backend for Aviate bootloader

#![no_std]
// Production-grade lints (when software-dfu is disabled)
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::panic))]
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::unwrap_used))]
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::expect_used))]

mod app_backend;
mod crash_backend;
mod delay_backend;
mod led_backend;
pub mod memory;
mod update_backend;

pub use app_backend::Stm32h743AppBackend;
pub use crash_backend::Stm32h743CrashBackend;
pub use delay_backend::Stm32h743DelayBackend;
pub use led_backend::Stm32h743LedBackend;
pub use update_backend::Stm32h743UpdateBackend;

use aviate_boot_core::CombinedBackend;
use stm32h7xx_hal::pac;

/// GPIO port identifier (STM32H7 has ports A-K)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Port {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
}

/// Board metadata for STM32H7 LED initialization
#[derive(Debug, Clone, Copy)]
pub struct Stm32LedMetadata {
    pub red: (Port, u8), // e.g., (Port::E, 3)
    pub green: (Port, u8),
    pub blue: (Port, u8),
}

/// Type alias for STM32H743 backend
pub type Stm32Backend = CombinedBackend<
    Stm32h743CrashBackend,
    Stm32h743LedBackend,
    Stm32h743DelayBackend,
    Stm32h743UpdateBackend,
    Stm32h743AppBackend,
>;

/// Chip-specific main function called by bootloader entry point
///
/// This function:
/// 1. Initializes chip peripherals
/// 2. Creates individual backends
/// 3. Combines them using CombinedBackend from boot-core
/// 4. Calls the MCU-agnostic boot_sequence()
pub fn chip_main(led_metadata: Stm32LedMetadata) -> ! {
    // Take peripherals - safe because this is the entry point
    let dp = match pac::Peripherals::take() {
        Some(p) => p,
        None => loop { cortex_m::asm::wfi(); }, // Should never happen
    };

    // Configure flash wait states for boot clock (4 MHz CSI needs 0 wait states)
    // After reset, FLASH_ACR.LATENCY defaults to 7 (7 wait states)
    // This causes ~8x slowdown for low-speed boot code
    // Use PAC for register access instead of hardcoded address
    dp.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(0) });
    cortex_m::asm::dsb();

    // Enable peripheral clocks using PAC (spec: use PAC/HAL, not hardcoded addresses!)
    dp.RCC
        .apb4enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 4)) }); // PWREN bit 4
    dp.RCC
        .ahb4enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 4)) }); // GPIOEEN bit 4

    // Enable backup domain access and RTC APB access
    dp.RCC
        .apb4enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 16)) }); // RTCAPBEN bit 16
    cortex_m::asm::dsb();

    // Small delay for clock to stabilize
    for _ in 0..100 {
        cortex_m::asm::nop();
    }

    // Create individual backends
    let crash = Stm32h743CrashBackend::new(dp.PWR, dp.RTC, dp.RCC);
    let leds = Stm32h743LedBackend::new(dp.GPIOE, led_metadata);
    let delay = Stm32h743DelayBackend::new();
    let update = Stm32h743UpdateBackend::new(dp.OTG2_HS_GLOBAL);
    let app = Stm32h743AppBackend::new();

    // Combine backends using generic CombinedBackend from boot-core
    let backend = Stm32Backend::new(crash, leds, delay, update, app);

    // Call the protocol layer state machine (MCU-agnostic!)
    // This consumes backend and never returns
    aviate_boot_core::boot_sequence(backend, memory::APP_START)
}
