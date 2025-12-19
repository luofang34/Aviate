//! RTC backup register CrashBackend implementation for STM32H743
//!
//! Uses RTC backup registers (BKP0R-BKP3R) for boot flags.
//! Boot flags are stored at 0x5800_4050 (RTC_BKP0R, first backup register).
//!
//! **Important**: Boot flags only survive software reset when VBAT is connected.
//! On boards where VBAT is floating, use ROM DFU (BOOT+RESET) for firmware updates.

use crate::memory::{BOOT_FLAGS_ADDR, BOOT_FLAGS_MAGIC};
use aviate_boot_core::{magic, BootFlags, BootReason, CrashBackend};

/// Boot flags structure in RTC backup registers (16 bytes = 4 registers)
#[repr(C)]
struct RtcBootFlags {
    magic: u32,
    want_bootloader: u32,
    crash_detected: u32,
    firmware_ok: u32,
}

/// RTC backup register based crash backend
pub struct RtcCrashBackend;

impl RtcCrashBackend {
    /// Create a new RTC crash backend
    pub fn new() -> Self {
        Self
    }

    /// Get pointer to boot flags in RTC backup registers
    fn flags_ptr() -> *mut RtcBootFlags {
        BOOT_FLAGS_ADDR as *mut RtcBootFlags
    }
}

impl Default for RtcCrashBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl CrashBackend for RtcCrashBackend {
    fn load_flags(&self) -> BootFlags {
        unsafe {
            let ptr = Self::flags_ptr();

            // Check magic - if invalid, return empty flags (power-on reset)
            if core::ptr::read_volatile(&(*ptr).magic) != BOOT_FLAGS_MAGIC {
                return BootFlags {
                    want_bootloader: false,
                    crash_detected: false,
                    firmware_ok: false,
                };
            }

            BootFlags {
                want_bootloader: core::ptr::read_volatile(&(*ptr).want_bootloader)
                    == magic::BOOT_TO_BOOTLOADER,
                crash_detected: core::ptr::read_volatile(&(*ptr).crash_detected)
                    == magic::CRASH_DETECTED,
                firmware_ok: core::ptr::read_volatile(&(*ptr).firmware_ok) == magic::FIRMWARE_OK,
            }
        }
    }

    fn store_flags(&mut self, flags: BootFlags) {
        unsafe {
            let ptr = Self::flags_ptr();

            // Always write magic first to validate structure
            core::ptr::write_volatile(&mut (*ptr).magic, BOOT_FLAGS_MAGIC);
            core::ptr::write_volatile(
                &mut (*ptr).want_bootloader,
                if flags.want_bootloader {
                    magic::BOOT_TO_BOOTLOADER
                } else {
                    0
                },
            );
            core::ptr::write_volatile(
                &mut (*ptr).crash_detected,
                if flags.crash_detected {
                    magic::CRASH_DETECTED
                } else {
                    0
                },
            );
            core::ptr::write_volatile(
                &mut (*ptr).firmware_ok,
                if flags.firmware_ok {
                    magic::FIRMWARE_OK
                } else {
                    0
                },
            );

            // Memory barrier to ensure writes complete
            cortex_m::asm::dsb();
        }
    }

    fn boot_reason(&self) -> BootReason {
        // RTC backup registers don't track reset reason - use RCC_RSR if needed
        // For now, return Unknown (crash detection uses flags, not reset reason)
        BootReason::Unknown
    }
}
