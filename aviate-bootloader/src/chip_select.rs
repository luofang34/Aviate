//! Chip selection based on feature flags
//!
//! This module provides a unified interface to the selected chip's chip_main() function.
//! Each chip crate must provide:
//! - A chip_main(led_metadata) -> ! function
//! - The appropriate LedMetadata type

#[cfg(feature = "chip-stm32h743")]
pub use aviate_chip_stm32h743::{chip_main, Stm32LedMetadata as LedMetadata};

#[cfg(feature = "chip-rp2350")]
pub use aviate_chip_rp2350::{chip_main, Rp2350LedMetadata as LedMetadata};

// Wrapper struct for cleaner usage in main.rs
pub struct SelectedChip;

impl SelectedChip {
    pub fn chip_main(led_metadata: LedMetadata) -> ! {
        chip_main(led_metadata)
    }
}

// Compile-time check: exactly one chip must be selected
#[cfg(not(any(feature = "chip-stm32h743", feature = "chip-rp2350",)))]
compile_error!("No chip selected! Enable exactly one chip-* feature.");
