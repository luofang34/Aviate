use crate::{GpioPin, GpioPort};

/// Active-low red, green, and blue status LED identities.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActiveLowRgbLedPins {
    /// Red status LED.
    pub red: GpioPin,
    /// Green status LED.
    pub green: GpioPin,
    /// Blue status LED.
    pub blue: GpioPin,
}

/// MicoAir H743-V2 active-low status LEDs.
pub const STATUS_LEDS: ActiveLowRgbLedPins = ActiveLowRgbLedPins {
    red: GpioPin::new(GpioPort::E, 3),
    green: GpioPin::new(GpioPort::E, 2),
    blue: GpioPin::new(GpioPort::E, 4),
};
