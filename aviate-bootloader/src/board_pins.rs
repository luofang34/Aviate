//! LED metadata selection for bootloader-supported boards.

use crate::chip_select::LedMetadata;

#[cfg(feature = "board-micoair-h743-v2")]
use aviate_board_micoair_h743_v2_metadata::{GpioPin, GpioPort, STATUS_LEDS};
#[cfg(feature = "board-micoair-h743-v2")]
use aviate_chip_stm32h743::Port;

#[cfg(feature = "board-micoair-h743-v2")]
const fn stm32_port(port: GpioPort) -> Port {
    match port {
        GpioPort::A => Port::A,
        GpioPort::B => Port::B,
        GpioPort::C => Port::C,
        GpioPort::D => Port::D,
        GpioPort::E => Port::E,
        GpioPort::F => Port::F,
        GpioPort::G => Port::G,
        GpioPort::H => Port::H,
        GpioPort::I => Port::I,
        GpioPort::J => Port::J,
        GpioPort::K => Port::K,
    }
}

#[cfg(feature = "board-micoair-h743-v2")]
const fn stm32_pin(pin: GpioPin) -> (Port, u8) {
    (stm32_port(pin.port), pin.number)
}

/// LED pins for the selected MicoAir board.
#[cfg(feature = "board-micoair-h743-v2")]
pub const SELECTED_BOARD_PINS: LedMetadata = LedMetadata {
    red: stm32_pin(STATUS_LEDS.red),
    green: stm32_pin(STATUS_LEDS.green),
    blue: stm32_pin(STATUS_LEDS.blue),
};

/// LED pins for Pico 2.
#[cfg(feature = "pico2")]
pub const SELECTED_BOARD_PINS: LedMetadata = LedMetadata {
    red: Some(aviate_chip_rp2350::GpioPin(25)),
    green: None,
    blue: None,
};

#[cfg(not(any(feature = "board-micoair-h743-v2", feature = "pico2",)))]
compile_error!("No board selected! Enable exactly one board-* feature.");
