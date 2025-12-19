//! STM32H743 Memory Layout Configuration
//!
//! This module defines the memory layout for STM32H743-based boards with Aviate bootloader.
//!
//! # Design Philosophy
//!
//! We derive as much as possible from HAL/PAC to avoid hardcoding addresses:
//! - **Flash base**: Use `FlashExt::address()` from HAL at runtime
//! - **Flash size**: Use `FlashSize::bytes()` which reads from chip signature
//! - **Sector size**: Defined by HAL based on chip family (rm0433 vs rm0455)
//! - **Unlock keys**: Standard ARM Cortex-M flash keys (RM0433 section 4.3.10)
//!
//! Constants that MUST be compile-time (for linker script compatibility):
//! - Bootloader/app partition boundary
//! - RAM regions (memory-mapped, not peripheral addresses)
//!
//! # Memory Map (with 2MB Flash, 128K bootloader reserved)
//!
//! ```text
//! Flash Bank 1 (1MB):
//!   0x0800_0000 - 0x0801_FFFF  (128KB) Bootloader (Sector 0)
//!   0x0802_0000 - 0x080F_FFFF  (896KB) Application (Sectors 1-7)
//!
//! Flash Bank 2 (1MB):
//!   0x0810_0000 - 0x081F_FFFF  (1MB)   Application continued
//!
//! RAM (AXI SRAM - D1 domain):
//!   0x2400_0000 - 0x2407_FFFF  (512KB) Main RAM (stack placed here by HAL)
//! ```

use stm32h7xx_hal::flash::FlashExt;
use stm32h7xx_hal::signature::FlashSize;

// =============================================================================
// Compile-time constants (required for linker script compatibility)
// =============================================================================

/// Flash sector size in bytes (128KB for STM32H743, from RM0433)
///
/// This matches `SECTOR_SIZE` in stm32h7xx-hal flash module.
/// STM32H7 family (rm0433/rm0399/rm0468) uses 128KB sectors.
/// Only rm0455 subfamily uses 8KB sectors.
pub const SECTOR_SIZE: u32 = 128 * 1024;

/// Bootloader size (one sector = 128KB)
///
/// The bootloader occupies the first flash sector to ensure
/// it can never be accidentally erased during app updates.
pub const BOOTLOADER_SIZE: u32 = SECTOR_SIZE;

/// DTCM RAM start address (D1 domain, RM0433 Table 7)
///
/// DTCM (128KB) is the fastest RAM for stack - zero wait states.
/// Applications typically place stack here for best performance.
pub const DTCM_START: u32 = 0x2000_0000;

/// DTCM RAM end address (exclusive, 128KB)
pub const DTCM_END: u32 = 0x2002_0000;

/// AXI SRAM start address (D1 domain, RM0433 Table 7)
///
/// AXI SRAM (512KB) is used for heap and large buffers.
/// Some applications may place stack here instead of DTCM.
pub const AXI_START: u32 = 0x2400_0000;

/// AXI SRAM end address (exclusive, 512KB)
pub const AXI_END: u32 = 0x2408_0000;

// =============================================================================
// Boot flags (RTC backup registers)
// =============================================================================
//
// Two systems share RTC backup registers with non-conflicting magic values:
//
// 1. Software DFU (checked first by bootloader):
//    - BK0R = 0xB007_B007: Request DFU mode, cleared immediately
//    - Simple single-word check, no structure
//
// 2. Crash backend (used if no DFU request):
//    - BK0R = 0x5241_4D42 (BOOT_FLAGS_MAGIC): Structure header
//    - BK1R-BK3R: Boot flags data (want_bootloader, crash_detected, firmware_ok)
//
// These don't conflict since 0xB007_B007 != 0x5241_4D42.

/// Boot flags address (RTC backup registers, used by crash backend)
///
/// RTC_BKP0R is at RTC base (0x5800_4000) + 0x50
/// Requires PWR.CR1.DBP to be set for write access.
pub const BOOT_FLAGS_ADDR: u32 = 0x5800_4050;

/// Boot flags magic value for crash backend ("RAMB" = RAM Boot)
///
/// Used to validate that crash backend flags structure is initialized.
/// Note: Software DFU uses 0xB007_B007 which is checked separately.
pub const BOOT_FLAGS_MAGIC: u32 = 0x5241_4D42;

/// Flash unlock key 1 (RM0433 section 4.3.10)
///
/// Standard ARM Cortex-M flash unlock sequence.
/// Same as `UNLOCK_KEY1` in stm32h7xx-hal flash module.
pub const FLASH_KEY1: u32 = 0x4567_0123;

/// Flash unlock key 2 (RM0433 section 4.3.10)
///
/// Standard ARM Cortex-M flash unlock sequence.
/// Same as `UNLOCK_KEY2` in stm32h7xx-hal flash module.
pub const FLASH_KEY2: u32 = 0xCDEF_89AB;

// =============================================================================
// Runtime functions (use HAL where available)
// =============================================================================

/// Get flash base address from HAL
///
/// Uses `FlashExt::address()` which returns `0x0800_0000` for STM32H7.
#[inline]
pub fn flash_base(flash: &stm32h7xx_hal::pac::FLASH) -> u32 {
    flash.address() as u32
}

/// Get flash size from chip signature (reads actual hardware value)
///
/// Uses `FlashSize::bytes()` which reads from the device signature area.
/// This returns the actual flash size programmed at the factory.
#[inline]
pub fn flash_size() -> u32 {
    FlashSize::bytes() as u32
}

/// Get flash end address (base + size)
#[inline]
pub fn flash_end(flash: &stm32h7xx_hal::pac::FLASH) -> u32 {
    flash_base(flash) + flash_size()
}

/// Get application start address (base + bootloader size)
#[inline]
pub fn app_start(flash: &stm32h7xx_hal::pac::FLASH) -> u32 {
    flash_base(flash) + BOOTLOADER_SIZE
}

/// Get application end address (flash_end - 1)
#[inline]
pub fn app_end(flash: &stm32h7xx_hal::pac::FLASH) -> u32 {
    flash_end(flash) - 1
}

// =============================================================================
// Compile-time constants for DFU and validation (must match runtime values)
// =============================================================================

/// Flash memory base address (compile-time constant)
///
/// This MUST match `flash_base()` at runtime. Used for:
/// - Linker script compatibility
/// - DFU descriptor string generation
/// - Compile-time address validation
pub const FLASH_BASE: u32 = 0x0800_0000;

/// Application start address (compile-time constant)
///
/// This MUST match `app_start()` at runtime.
/// Computed as: FLASH_BASE + BOOTLOADER_SIZE
pub const APP_START: u32 = FLASH_BASE + BOOTLOADER_SIZE;

/// Flash end address for 2MB flash (compile-time constant)
///
/// Note: This assumes 2MB flash. For other variants, use `flash_end()` at runtime.
pub const FLASH_END: u32 = FLASH_BASE + (2 * 1024 * 1024);

/// Application end address (compile-time constant)
///
/// Last valid address in application flash region.
pub const APP_END: u32 = FLASH_END - 1;

/// Number of sectors available for application
///
/// Total sectors (16 for 2MB) minus bootloader sector (1).
pub const APP_SECTOR_COUNT: u8 = 15;

/// DFU memory info string for USB descriptor
///
/// Format: @<name>/<base_addr>/<sectors>*<size><unit><access>
/// - @Flash: Memory region name
/// - 0x08020000: Application start address (APP_START)
/// - 15*128Ke: 15 sectors of 128KB each, erasable
pub const DFU_MEM_INFO: &str = "@Flash/0x08020000/15*128Ke";

// =============================================================================
// Compile-time validation
// =============================================================================

// Ensure our compile-time constants are consistent
const _: () = {
    assert!(APP_START == FLASH_BASE + BOOTLOADER_SIZE);
    assert!(APP_END == FLASH_END - 1);
    assert!(FLASH_END - FLASH_BASE == 2 * 1024 * 1024);
    assert!(DTCM_END - DTCM_START == 128 * 1024); // DTCM is 128KB
    assert!(AXI_END - AXI_START == 512 * 1024); // AXI SRAM is 512KB
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn memory_layout_consistency() {
        // Verify bootloader fits in first sector
        assert_eq!(APP_START - FLASH_BASE, BOOTLOADER_SIZE);

        // Verify total flash size constant is 2MB
        assert_eq!(FLASH_END - FLASH_BASE, 2 * 1024 * 1024);

        // Verify application region spans from sector 1 to end
        assert_eq!(APP_END + 1, FLASH_END);

        // Verify DTCM size is 128KB
        assert_eq!(DTCM_END - DTCM_START, 128 * 1024);

        // Verify AXI SRAM size is 512KB
        assert_eq!(AXI_END - AXI_START, 512 * 1024);

        // Verify sector size matches HAL expectation (128KB for non-rm0455)
        assert_eq!(SECTOR_SIZE, 0x2_0000);
    }

    #[test]
    fn compile_time_matches_computed() {
        // These would ideally test against runtime values, but we can't
        // access hardware in unit tests. The const assertions above
        // ensure internal consistency.
        assert_eq!(APP_START, FLASH_BASE + BOOTLOADER_SIZE);
    }
}
