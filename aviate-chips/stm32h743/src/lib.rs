//! STM32H743 chip backend for Aviate bootloader

#![no_std]
// Production-grade lints (when software-dfu is disabled)
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::panic))]
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::unwrap_used))]
#![cfg_attr(not(feature = "software-dfu"), forbid(clippy::expect_used))]

mod app_backend;
mod crash_backend;
mod delay_backend;
mod led_backend;
pub mod memory;
mod rtc_crash_backend;
mod update_backend;

pub use app_backend::Stm32h743AppBackend;
pub use crash_backend::Stm32h743CrashBackend;
pub use delay_backend::Stm32h743DelayBackend;
pub use led_backend::Stm32h743LedBackend;
pub use rtc_crash_backend::RtcCrashBackend;
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
///
/// Uses RtcCrashBackend for boot flags storage in RTC backup registers.
pub type Stm32Backend = CombinedBackend<
    RtcCrashBackend,
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
        None => loop {
            cortex_m::asm::wfi();
        }, // Should never happen
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

    // Enable RTC APB clock (RTCAPBEN bit 16)
    dp.RCC
        .apb4enr
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 16)) }); // RTCAPBEN bit 16
    cortex_m::asm::dsb();

    // Enable backup domain write access (PWR.CR1.DBP bit 8)
    // Required for reading/writing RTC backup registers
    dp.PWR
        .cr1
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 8)) }); // DBP bit 8

    // Wait for DBP to take effect
    for _ in 0..1000 {
        cortex_m::asm::nop();
    }
    cortex_m::asm::dsb();

    // Software DFU: Check boot magic and enter DFU mode if requested
    // Simplified approach matching old working bootloader:
    // - Single magic value 0xB007_B007 in RTC_BK0R
    #[cfg(feature = "software-dfu")]
    {
        const RTC_BK0R: u32 = 0x5800_4050;
        const BOOT_MAGIC: u32 = 0xB007_B007;

        // Read boot magic from RTC backup register
        let magic = unsafe { core::ptr::read_volatile(RTC_BK0R as *const u32) };

        if magic == BOOT_MAGIC {
            // Clear the magic before entering DFU
            unsafe {
                core::ptr::write_volatile(RTC_BK0R as *mut u32, 0);
                cortex_m::asm::dsb();
            }

            // Enter DFU mode
            use aviate_boot_core::UpdateBackend;
            let mut update = Stm32h743UpdateBackend::new(dp.OTG2_HS_GLOBAL);
            update.enter_update_mode();
        }
    }

    // Create individual backends
    let crash = RtcCrashBackend::new();
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
