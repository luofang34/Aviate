//! CrashBackend implementation for STM32H743
//!
//! Uses RTC backup registers (battery-backed, survive resets)
//! - BK0R: Bootloader request magic
//! - BK1R: Crash detected magic
//! - BK2R: Firmware OK magic
//!
//! Addresses verified from PAC: pac::RTC::ptr() = 0x5800_4000

use aviate_boot_core::{magic, BootFlags, BootReason, CrashBackend};
use stm32h7xx_hal::pac;

pub struct Stm32h743CrashBackend {
    pwr: pac::PWR,
    rtc: pac::RTC,
    rcc: pac::RCC,
}

impl Stm32h743CrashBackend {
    pub fn new(pwr: pac::PWR, rtc: pac::RTC, rcc: pac::RCC) -> Self {
        // Enable backup domain access immediately (needed for reading RTC backup registers)
        pwr.cr1.modify(|_, w| w.dbp().set_bit());
        cortex_m::asm::dsb();

        Self { pwr, rtc, rcc }
    }

    /// Enable write access to RTC backup domain
    fn enable_backup_access(&mut self) {
        self.pwr.cr1.modify(|_, w| w.dbp().set_bit());
        cortex_m::asm::dsb();
    }
}

impl CrashBackend for Stm32h743CrashBackend {
    fn load_flags(&self) -> BootFlags {
        // Decode magic values from RTC backup registers to logical booleans
        let bk0 = self.rtc.bkpr[0].read().bits();
        let bk1 = self.rtc.bkpr[1].read().bits();
        let bk2 = self.rtc.bkpr[2].read().bits();

        BootFlags {
            want_bootloader: bk0 == magic::BOOT_TO_BOOTLOADER,
            crash_detected: bk1 == magic::CRASH_DETECTED,
            firmware_ok: bk2 == magic::FIRMWARE_OK,
        }
    }

    fn store_flags(&mut self, flags: BootFlags) {
        self.enable_backup_access();

        // Encode logical booleans to magic values
        let bk0 = if flags.want_bootloader {
            magic::BOOT_TO_BOOTLOADER
        } else {
            0
        };
        let bk1 = if flags.crash_detected {
            magic::CRASH_DETECTED
        } else {
            0
        };
        let bk2 = if flags.firmware_ok {
            magic::FIRMWARE_OK
        } else {
            0
        };

        self.rtc.bkpr[0].write(|w| w.bits(bk0));
        self.rtc.bkpr[1].write(|w| w.bits(bk1));
        self.rtc.bkpr[2].write(|w| w.bits(bk2));
        cortex_m::asm::dsb();
    }

    fn boot_reason(&self) -> BootReason {
        let rsr = self.rcc.rsr.read();
        if rsr.iwdg1rstf().bit_is_set() {
            BootReason::Watchdog
        } else if rsr.sftrstf().bit_is_set() {
            BootReason::Software
        } else if rsr.porrstf().bit_is_set() {
            BootReason::PowerOn
        } else {
            BootReason::Unknown
        }
    }

    // Default helpers (set_crash_detected, clear_crash_detected, etc.)
    // are provided by the trait using load_flags/store_flags
}
