//! Aviate Multi-MCU Bootloader
//!
//! This bootloader supports multiple MCU families through a trait-based architecture:
//! - STM32H743: USB DFU bootloader
//! - RP2040/RP2350: USB MSC ROM bootloader (future)
//! - ESP32-S3: ROM bootloader (future)
//!
//! Memory Layout (STM32H743):
//! - 0x0800_0000 - 0x0801_FFFF: Bootloader (128KB)
//! - 0x0802_0000 - 0x081F_FFFF: Application (1920KB)
//!
//! Boot Flow:
//! 1. Check boot flags (crash detected / software bootloader request)
//! 2. Enter DFU mode if requested, or validate and jump to application

#![no_std]
#![no_main]
// Production-grade lints (when software-dfu is disabled)
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::panic))]
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::unwrap_used))]
#![cfg_attr(not(feature = "software-dfu"), deny(clippy::expect_used))]

use panic_halt as _;

mod chip_select;
mod board_pins;

use chip_select::SelectedChip;
use board_pins::SELECTED_BOARD_PINS;

// Architecture-specific entry points (feature-gated)
// Each calls the selected chip's chip_main() function with board-specific LED pins

#[cfg(feature = "arch-cortex-m")]
#[cortex_m_rt::entry]
fn main() -> ! {
    SelectedChip::chip_main(SELECTED_BOARD_PINS)
}

// Compile-time check: exactly one architecture must be selected
#[cfg(not(any(
    feature = "arch-cortex-m",
)))]
compile_error!("No architecture selected! Enable exactly one arch-* feature.");
