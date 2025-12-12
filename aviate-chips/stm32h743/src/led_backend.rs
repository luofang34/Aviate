//! LED backend implementation for STM32H743
//!
//! Controls RGB LEDs using GPIO HAL

use crate::{Port, Stm32LedMetadata};
use aviate_boot_core::StatusLeds;
use stm32h7xx_hal::pac;

pub struct Stm32h743LedBackend {
    gpio: pac::GPIOE,
    pins: Stm32LedMetadata,
}

impl Stm32h743LedBackend {
    pub fn new(gpio: pac::GPIOE, pins: Stm32LedMetadata) -> Self {
        let mut backend = Self { gpio, pins };
        backend.init();
        backend
    }

    /// Initialize LED pins as outputs
    fn init(&mut self) {
        // Configure pins as outputs (MODER = 01)
        self.gpio.moder.modify(|r, w| {
            let mut moder = r.bits();

            // Clear and set red pin
            moder &= !(0b11 << (self.pins.red.1 * 2));
            moder |= 0b01 << (self.pins.red.1 * 2);

            // Clear and set green pin
            moder &= !(0b11 << (self.pins.green.1 * 2));
            moder |= 0b01 << (self.pins.green.1 * 2);

            // Clear and set blue pin
            moder &= !(0b11 << (self.pins.blue.1 * 2));
            moder |= 0b01 << (self.pins.blue.1 * 2);

            unsafe { w.bits(moder) }
        });

        // Turn off all LEDs initially (active low)
        self.set_red(false);
        self.set_green(false);
        self.set_blue(false);
    }

    /// Set a specific pin state (active low LEDs)
    fn set_pin(&mut self, pin: u8, on: bool) {
        if on {
            // Reset bit (turn on, active low)
            self.gpio.bsrr.write(|w| unsafe { w.bits(1 << (pin + 16)) });
        } else {
            // Set bit (turn off)
            self.gpio.bsrr.write(|w| unsafe { w.bits(1 << pin) });
        }
    }
}

impl StatusLeds for Stm32h743LedBackend {
    fn set_red(&mut self, on: bool) {
        self.set_pin(self.pins.red.1, on);
    }

    fn set_green(&mut self, on: bool) {
        self.set_pin(self.pins.green.1, on);
    }

    fn set_blue(&mut self, on: bool) {
        self.set_pin(self.pins.blue.1, on);
    }
}
