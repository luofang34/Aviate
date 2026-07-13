//! Dependency-free hardware metadata for the MicoAir H743-V2 board.

#![no_std]
#![deny(missing_docs)]
#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

mod gpio;
mod status_leds;

pub use gpio::{GpioPin, GpioPort};
pub use status_leds::{ActiveLowRgbLedPins, STATUS_LEDS};
