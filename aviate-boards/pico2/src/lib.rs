//! Raspberry Pi Pico 2 (RP2350) board support

#![no_std]

/// LED pin definitions for Pico 2
/// Pico 2 has a single onboard LED on GPIO25
pub mod leds {
    /// Onboard LED on GPIO25 (active high)
    pub const LED: u8 = 25;
}
