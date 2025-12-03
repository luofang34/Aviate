//! Bootloader control for STM32H7
//!
//! This module provides functionality to trigger a software reboot into the
//! bootloader without requiring the boot button to be pressed.
//!
//! ## How it works
//!
//! The STM32H7 has RTC backup registers that persist across resets (powered by VBAT).
//! We use RTC backup register 0 (BK0R) to communicate with the bootloader:
//!
//! 1. **Application writes magic word**: Before reset, write `0xb007b007` to BK0R
//! 2. **Trigger software reset**: Call `cortex_m::peripheral::SCB::sys_reset()`
//! 3. **Bootloader checks BK0R**: If magic is present, enter DFU mode
//! 4. **Bootloader clears magic**: So next reset boots normally
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_board_micoair_h743_v2::bootloader;
//!
//! // Reboot into bootloader for firmware update
//! bootloader::reboot_to_bootloader();
//! // This function never returns
//! ```
//!
//! ## Register addresses (STM32H743)
//!
//! - PWR_CR1: 0x5802_4800 (Power control register 1)
//! - RTC_BK0R: 0x5800_4050 (RTC backup register 0)

/// Magic word that signals the bootloader to stay in DFU mode
/// Compatible with PX4 bootloader
pub const BOOT_RTC_SIGNATURE: u32 = 0xb007_b007;

/// Alternative signatures for different boot modes (for future use)
pub mod signatures {
    /// Boot to bootloader (DFU mode)
    pub const BOOT_TO_BOOTLOADER: u32 = super::BOOT_RTC_SIGNATURE;
    /// Boot to valid application (skip bootloader wait)
    pub const BOOT_TO_APP: u32 = 0xb007_0002;
    /// Power down signature
    pub const POWER_DOWN: u32 = 0xdead_beef;
    /// Firmware OK signature (set by app on successful boot)
    pub const FIRMWARE_OK: u32 = 0xb009_3a26;
}

/// STM32H743 register addresses
mod regs {
    /// Power control register 1
    pub const PWR_CR1: u32 = 0x5802_4800;
    /// RTC backup register 0
    pub const RTC_BK0R: u32 = 0x5800_4050;
    /// Bit to enable backup domain access in PWR_CR1
    pub const PWR_CR1_DBP: u32 = 1 << 8;
}

/// Enable write access to RTC backup domain
///
/// # Safety
/// This function directly writes to hardware registers.
#[inline]
fn enable_backup_access() {
    // Safety: Writing to PWR_CR1 to enable backup domain access
    // This is a standard operation for STM32 backup domain access
    unsafe {
        let pwr_cr1 = regs::PWR_CR1 as *mut u32;
        let current = core::ptr::read_volatile(pwr_cr1);
        core::ptr::write_volatile(pwr_cr1, current | regs::PWR_CR1_DBP);
    }
}

/// Disable write access to RTC backup domain
///
/// # Safety
/// This function directly writes to hardware registers.
#[inline]
fn disable_backup_access() {
    // Safety: Writing to PWR_CR1 to disable backup domain access
    unsafe {
        let pwr_cr1 = regs::PWR_CR1 as *mut u32;
        let current = core::ptr::read_volatile(pwr_cr1);
        core::ptr::write_volatile(pwr_cr1, current & !regs::PWR_CR1_DBP);
    }
}

/// Write a value to RTC backup register 0
///
/// # Safety
/// Caller must ensure backup access is enabled.
#[inline]
fn write_rtc_bk0r(value: u32) {
    // Safety: Writing to RTC backup register after enabling backup access
    unsafe {
        let rtc_bk0r = regs::RTC_BK0R as *mut u32;
        core::ptr::write_volatile(rtc_bk0r, value);
    }
}

/// Read the value from RTC backup register 0
///
/// # Safety
/// Caller must ensure backup access is enabled.
#[inline]
fn read_rtc_bk0r() -> u32 {
    // Safety: Reading from RTC backup register after enabling backup access
    unsafe {
        let rtc_bk0r = regs::RTC_BK0R as *const u32;
        core::ptr::read_volatile(rtc_bk0r)
    }
}

/// Set the bootloader signature in RTC backup register
///
/// This writes the magic word that tells the bootloader to stay in DFU mode.
/// Call `trigger_reset()` after this to actually enter the bootloader.
pub fn set_bootloader_signature() {
    enable_backup_access();
    write_rtc_bk0r(BOOT_RTC_SIGNATURE);
    disable_backup_access();
}

/// Clear the bootloader signature
///
/// This clears the magic word so the next reset will boot normally.
pub fn clear_bootloader_signature() {
    enable_backup_access();
    write_rtc_bk0r(0);
    disable_backup_access();
}

/// Check if the bootloader signature is set
pub fn is_bootloader_signature_set() -> bool {
    enable_backup_access();
    let value = read_rtc_bk0r();
    disable_backup_access();
    value == BOOT_RTC_SIGNATURE
}

/// Get the current RTC backup register 0 value
pub fn get_boot_signature() -> u32 {
    enable_backup_access();
    let value = read_rtc_bk0r();
    disable_backup_access();
    value
}

/// Set custom boot signature
///
/// Use this to set signatures like `FIRMWARE_OK` after successful boot.
pub fn set_boot_signature(signature: u32) {
    enable_backup_access();
    write_rtc_bk0r(signature);
    disable_backup_access();
}

/// Trigger a software system reset
///
/// # Safety
/// This function never returns. All peripheral state is lost.
pub fn trigger_reset() -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

/// Reboot into the bootloader for firmware update
///
/// This function:
/// 1. Sets the bootloader magic word in RTC backup register
/// 2. Triggers a software reset
/// 3. Never returns
///
/// After reset, the bootloader will see the magic word and enter DFU mode,
/// allowing firmware updates via USB without pressing the boot button.
///
/// # Example
///
/// ```ignore
/// // Handle MAVLink COMMAND_PREFLIGHT_REBOOT_SHUTDOWN with param1 = 3
/// if cmd.param1 == 3.0 {
///     bootloader::reboot_to_bootloader();
/// }
/// ```
pub fn reboot_to_bootloader() -> ! {
    set_bootloader_signature();
    trigger_reset()
}

/// Mark firmware as successfully booted
///
/// Call this after the application has successfully initialized.
/// The bootloader can use this to detect if the last firmware failed to boot.
pub fn mark_firmware_ok() {
    set_boot_signature(signatures::FIRMWARE_OK);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_boot_signature_values() {
        assert_eq!(BOOT_RTC_SIGNATURE, 0xb007_b007);
        assert_eq!(signatures::BOOT_TO_BOOTLOADER, BOOT_RTC_SIGNATURE);
        assert_eq!(signatures::BOOT_TO_APP, 0xb007_0002);
        assert_eq!(signatures::FIRMWARE_OK, 0xb009_3a26);
    }

    #[test]
    fn test_register_addresses() {
        assert_eq!(regs::PWR_CR1, 0x5802_4800);
        assert_eq!(regs::RTC_BK0R, 0x5800_4050);
        assert_eq!(regs::PWR_CR1_DBP, 0x100);
    }
}
