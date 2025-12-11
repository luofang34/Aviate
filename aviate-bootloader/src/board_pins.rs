//! Board pin selection based on feature flags
//!
//! This module provides the LED pin metadata for the selected board.
//! Each board crate must export its LED pin definitions.

#[cfg(feature = "board-micoair-h743-v2")]
pub use aviate_board_micoair_h743_v2::leds as board_leds;

// Import the chip's LedMetadata type from chip_select
// NOTE: Use crate::, not super::, since both modules are siblings in src/
use crate::chip_select::LedMetadata;

/// Board-specific LED pins (converted to chip's LedMetadata format)
#[cfg(feature = "board-micoair-h743-v2")]
pub const SELECTED_BOARD_PINS: LedMetadata = LedMetadata {
    red: board_leds::RED,
    green: board_leds::GREEN,
    blue: board_leds::BLUE,
};

// Compile-time check: exactly one board must be selected
#[cfg(not(any(
    feature = "board-micoair-h743-v2",
)))]
compile_error!("No board selected! Enable exactly one board-* feature.");
