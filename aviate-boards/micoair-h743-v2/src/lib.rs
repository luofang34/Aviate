//! MicoAir H743-V2 Flight Controller Board Family
//!
//! Hardware configuration for the MicoAir H743-V2 board family based on STM32H743.
//! This crate supports multiple board variants that share the same flight controller
//! hardware but differ in integrated peripherals (ESC, BEC, form factor).
//!
//! ## Board Variants
//!
//! | Variant | Description | Feature Flag |
//! |---------|-------------|--------------|
//! | MicoAir743v2 | Standalone flight controller | (default) |
//! | MicoAir743v2-AIO | All-in-one with 4x45A ESC | `aio` |
//!
//! All variants share:
//! - MCU: STM32H743VIH6 @ 480MHz, 2MB Flash
//! - IMU: BMI088 + BMI270 (dual redundant)
//! - Baro: SPL06 @ I2C2 address 0x77
//! - 7x UART, 1x I2C (external), 1x SWD, 2x ADC
//!
//! ## Feature Flags
//!
//! | Feature | Description |
//! |---------|-------------|
//! | `software-bootloader` | Enable software-triggered bootloader entry (dev only) |
//! | `aio` | AIO variant with integrated ESC (45A x4, AM32 firmware) |
//!
//! ## Sensor Configuration
//!
//! | Sensor | Model | Interface | Bus | CS Pin | DRDY Pin |
//! |--------|-------|-----------|-----|--------|----------|
//! | IMU 1  | BMI088 | SPI | SPI2 | Gyro: PD5, Accel: PD4 | Gyro: PC15, Accel: PC14 |
//! | IMU 2  | BMI270 | SPI | SPI3 | PA15 | PB7 |
//! | Baro   | SPL06 | I2C | I2C2 | - | PD0 |
//! | Mag    | QMC5883L | I2C | I2C1 (external) | - | - |
//!
//! ## Motor Outputs (PWM/DShot300/DShot600)
//!
//! | Motor | Timer | Channel | GPIO |
//! |-------|-------|---------|------|
//! | M1 | TIM1 | CH4 | PE14 |
//! | M2 | TIM1 | CH3 | PE13 |
//! | M3 | TIM1 | CH2 | PE11 |
//! | M4 | TIM1 | CH1 | PE9 |
//! | M5 | TIM3 | CH4 | PB1 |
//! | M6 | TIM3 | CH3 | PB0 |
//! | M7 | TIM4 | CH1 | PD12 |
//! | M8 | TIM4 | CH2 | PD13 |
//! | M9 | TIM15 | CH1 | PE5 |
//! | M10 | TIM15 | CH2 | PE6 |
//!
//! ## Status LEDs
//!
//! | LED | GPIO | Active |
//! |-----|------|--------|
//! | Red | PE3 | Low |
//! | Green | PE2 | Low |
//! | Blue | PE4 | Low |

#![no_std]
// Production builds forbid unsafe code - bootloader entry requires physical button
#![cfg_attr(not(feature = "software-bootloader"), forbid(unsafe_code))]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

/// Software-triggered bootloader entry (development/testing only)
///
/// This module is only available when the `software-bootloader` feature is enabled.
/// It allows rebooting into the bootloader via software without pressing the boot button.
///
/// **WARNING**: This feature uses unsafe code to write to hardware registers.
/// For production builds, disable this feature to enforce physical boot button requirement.
///
/// Enable with: `cargo build --features software-bootloader`
#[cfg(feature = "software-bootloader")]
pub mod bootloader;

/// LED pin assignments for bootloader
///
/// These pins are used by the bootloader for status indication.
/// LEDs are active-low on this board.
#[cfg(feature = "aviate-chip-stm32h743")]
pub mod leds {
    use aviate_chip_stm32h743::Port;

    /// Red LED pin (PE3, active low)
    pub const RED: (Port, u8) = (Port::E, 3);

    /// Green LED pin (PE2, active low)
    pub const GREEN: (Port, u8) = (Port::E, 2);

    /// Blue LED pin (PE4, active low)
    pub const BLUE: (Port, u8) = (Port::E, 4);
}

/// Board identification
#[cfg(not(feature = "aio"))]
pub const BOARD_ID: &str = "micoair-h743-v2";
#[cfg(feature = "aio")]
pub const BOARD_ID: &str = "micoair-h743-v2-aio";

/// Board information
#[cfg(not(feature = "aio"))]
pub const BOARD_INFO: BoardInfo = BoardInfo {
    name: "micoair-h743-v2",
    description: "MicoAir H743-V2 flight controller",
    mcu: "STM32H743VIH6",
    variant: BoardVariant::Standalone,
};

#[cfg(feature = "aio")]
pub const BOARD_INFO: BoardInfo = BoardInfo {
    name: "micoair-h743-v2-aio",
    description: "MicoAir H743-V2 AIO (4x45A ESC)",
    mcu: "STM32H743VIH6",
    variant: BoardVariant::Aio,
};

/// Board variant
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoardVariant {
    /// Standalone flight controller (no integrated ESC)
    Standalone,
    /// All-in-one with integrated 4x45A ESC
    Aio,
}

/// Board information structure
#[derive(Clone, Debug)]
pub struct BoardInfo {
    pub name: &'static str,
    pub description: &'static str,
    pub mcu: &'static str,
    pub variant: BoardVariant,
}

/// AIO variant configuration (only available with `aio` feature)
#[cfg(feature = "aio")]
pub mod aio {
    /// Integrated ESC configuration
    pub mod esc {
        /// ESC firmware type
        pub const FIRMWARE: &str = "AM32_F4A_4IN1_F421";
        /// Firmware version
        pub const VERSION: &str = "2.17";
        /// Number of motor outputs
        pub const MOTOR_COUNT: u8 = 4;
        /// Continuous current per motor (amps)
        pub const CONTINUOUS_CURRENT_A: u8 = 45;
        /// PWM frequency (Hz)
        pub const PWM_FREQ_HZ: u32 = 48_000;
        /// Supported voltage range
        pub const VOLTAGE_MIN_V: f32 = 5.6;  // 2S
        pub const VOLTAGE_MAX_V: f32 = 27.0; // 6S
        /// Supported protocols
        pub const SUPPORTS_PWM: bool = true;
        pub const SUPPORTS_DSHOT300: bool = true;
        pub const SUPPORTS_DSHOT600: bool = true;
        pub const SUPPORTS_BDSHOT: bool = true;
    }

    /// BEC (Battery Eliminator Circuit) outputs
    pub mod bec {
        /// 5V BEC for controller, receiver, GPS, etc.
        pub const BEC_5V_AMPS: f32 = 2.0;
        /// 12V BEC for DJI O3/O4
        pub const BEC_12V_AMPS: f32 = 2.0;
    }
}

/// SPI bus assignments
pub mod spi {
    /// SPI2 for BMI088 IMU
    pub const BMI088_SPI: u8 = 2;
    /// SPI3 for BMI270 IMU
    pub const BMI270_SPI: u8 = 3;
}

/// I2C bus assignments and pin definitions
pub mod i2c {
    /// I2C1 for external devices (magnetometer, GPS, etc.)
    pub const EXTERNAL: u8 = 1;
    /// I2C2 for internal devices (barometer)
    pub const INTERNAL: u8 = 2;

    /// I2C1 pins (PB8=SCL, PB9=SDA)
    pub mod i2c1 {
        pub const SCL: (char, u8) = ('B', 8);
        pub const SDA: (char, u8) = ('B', 9);
    }

    /// I2C2 pins (PB10=SCL, PB11=SDA)
    pub mod i2c2 {
        pub const SCL: (char, u8) = ('B', 10);
        pub const SDA: (char, u8) = ('B', 11);
    }
}

/// Pin definitions for the board
///
/// These are abstract pin identifiers that map to the STM32H743 GPIO.
/// The actual HAL implementation will use these to configure the pins.
pub mod pins {
    /// BMI088 IMU pins (SPI2)
    pub mod bmi088 {
        /// Gyroscope chip select (PD5)
        pub const GYRO_CS: (char, u8) = ('D', 5);
        /// Accelerometer chip select (PD4)
        pub const ACCEL_CS: (char, u8) = ('D', 4);
        /// Gyroscope data ready interrupt (PC15)
        pub const GYRO_DRDY: (char, u8) = ('C', 15);
        /// Accelerometer data ready interrupt (PC14)
        pub const ACCEL_DRDY: (char, u8) = ('C', 14);
    }

    /// BMI270 IMU pins (SPI3)
    pub mod bmi270 {
        /// Chip select (PA15)
        pub const CS: (char, u8) = ('A', 15);
        /// Data ready interrupt (PB7)
        pub const DRDY: (char, u8) = ('B', 7);
    }

    /// SPL06 barometer pins (I2C2)
    pub mod spl06 {
        /// Data ready interrupt (PD0)
        pub const DRDY: (char, u8) = ('D', 0);
        /// I2C address (0x77 per PX4 board config)
        pub const I2C_ADDR: u8 = 0x77;
    }

    /// QMC5883L magnetometer pins (I2C1)
    pub mod qmc5883l {
        /// I2C address
        pub const I2C_ADDR: u8 = 0x0D;
    }

    /// Motor output pins (PWM/DShot)
    pub mod motors {
        /// Motor 1: TIM1_CH4 (PE14)
        pub const M1: (char, u8) = ('E', 14);
        /// Motor 2: TIM1_CH3 (PE13)
        pub const M2: (char, u8) = ('E', 13);
        /// Motor 3: TIM1_CH2 (PE11)
        pub const M3: (char, u8) = ('E', 11);
        /// Motor 4: TIM1_CH1 (PE9)
        pub const M4: (char, u8) = ('E', 9);
        /// Motor 5: TIM3_CH4 (PB1)
        pub const M5: (char, u8) = ('B', 1);
        /// Motor 6: TIM3_CH3 (PB0)
        pub const M6: (char, u8) = ('B', 0);
        /// Motor 7: TIM4_CH1 (PD12)
        pub const M7: (char, u8) = ('D', 12);
        /// Motor 8: TIM4_CH2 (PD13)
        pub const M8: (char, u8) = ('D', 13);
        /// Motor 9: TIM15_CH1 (PE5)
        pub const M9: (char, u8) = ('E', 5);
        /// Motor 10: TIM15_CH2 (PE6)
        pub const M10: (char, u8) = ('E', 6);

        /// Number of motor outputs
        pub const COUNT: usize = 10;
    }

    /// Status LED pins
    pub mod leds {
        /// Red LED (PE3, active low)
        pub const RED: (char, u8) = ('E', 3);
        /// Green LED (PE2, active low)
        pub const GREEN: (char, u8) = ('E', 2);
        /// Blue LED (PE4, active low)
        pub const BLUE: (char, u8) = ('E', 4);
    }

    /// UART assignments
    pub mod uart {
        /// RC input (USART6 / ttyS5)
        pub const RC: u8 = 6;
        /// GPS1 (USART3 / ttyS2)
        pub const GPS1: u8 = 3;
        /// GPS2 (USART2 / ttyS1)
        pub const GPS2: u8 = 2;
        /// Telemetry 1 (USART1 / ttyS0)
        pub const TEL1: u8 = 1;
        /// Telemetry 2 (UART4 / ttyS3)
        pub const TEL2: u8 = 4;
        /// Telemetry 3 (UART5 / ttyS4)
        pub const TEL3: u8 = 5;
        /// Telemetry 4 (UART8 / ttyS7)
        pub const TEL4: u8 = 8;
    }

    /// ADC channels
    pub mod adc {
        /// Battery voltage sensing (PC0, ADC1_IN10)
        pub const BATTERY_VOLTAGE: (char, u8) = ('C', 0);
        /// Battery current sensing (PC1, ADC1_IN11)
        pub const BATTERY_CURRENT: (char, u8) = ('C', 1);
    }
}

/// Timer configuration for motor outputs
pub mod timers {
    /// Timer 1 configuration (motors 1-4)
    pub const TIM1_CHANNELS: [u8; 4] = [4, 3, 2, 1];
    /// Timer 3 configuration (motors 5-6)
    pub const TIM3_CHANNELS: [u8; 2] = [4, 3];
    /// Timer 4 configuration (motors 7-8)
    pub const TIM4_CHANNELS: [u8; 2] = [1, 2];
    /// Timer 15 configuration (motors 9-10)
    pub const TIM15_CHANNELS: [u8; 2] = [1, 2];

    /// HRT (high resolution timer) uses TIM2
    pub const HRT_TIMER: u8 = 2;
    pub const HRT_CHANNEL: u8 = 1;
}

/// Sensor rotation from PX4 ROTATION enum
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Rotation {
    #[default]
    None = 0,
    Yaw45 = 1,
    Yaw90 = 2,
    Yaw135 = 3,
    Yaw180 = 4,
    Yaw225 = 5,
    Yaw270 = 6,
    Yaw315 = 7,
    Roll180 = 8,
    Roll180Yaw45 = 9,
    Roll180Yaw90 = 10,
    Roll180Yaw135 = 11,
    Pitch180 = 12,
    Roll180Yaw225 = 13,
    Roll180Yaw270 = 14,
    Roll180Yaw315 = 15,
}

/// Default sensor configurations for this board
pub mod sensors {
    use super::Rotation;

    /// BMI088 IMU configuration
    pub struct Bmi088Config {
        /// Sensor rotation relative to board frame
        pub rotation: Rotation,
        /// Accelerometer range in g (default: 24g)
        pub accel_range_g: u8,
        /// Gyroscope range in dps (default: 2000)
        pub gyro_range_dps: u16,
    }

    impl Default for Bmi088Config {
        fn default() -> Self {
            Self {
                rotation: Rotation::None,
                accel_range_g: 24,
                gyro_range_dps: 2000,
            }
        }
    }

    /// BMI270 IMU configuration
    pub struct Bmi270Config {
        /// Sensor rotation relative to board frame
        pub rotation: Rotation,
        /// Accelerometer range in g (default: 16g)
        pub accel_range_g: u8,
        /// Gyroscope range in dps (default: 2000)
        pub gyro_range_dps: u16,
    }

    impl Default for Bmi270Config {
        fn default() -> Self {
            Self {
                rotation: Rotation::None,
                accel_range_g: 16,
                gyro_range_dps: 2000,
            }
        }
    }

    /// SPL06 barometer configuration
    pub struct Spl06Config {
        /// Pressure oversampling rate
        pub pressure_oversample: u8,
        /// Temperature oversampling rate
        pub temp_oversample: u8,
    }

    impl Default for Spl06Config {
        fn default() -> Self {
            Self {
                pressure_oversample: 64,
                temp_oversample: 8,
            }
        }
    }

    /// QMC5883L magnetometer configuration
    pub struct Qmc5883lConfig {
        /// Sensor rotation relative to board frame
        pub rotation: Rotation,
        /// Output data rate (10, 50, 100, or 200 Hz)
        pub output_rate_hz: u8,
        /// Field range (2 or 8 gauss)
        pub range_gauss: u8,
    }

    impl Default for Qmc5883lConfig {
        fn default() -> Self {
            Self {
                rotation: Rotation::None,
                output_rate_hz: 200,
                range_gauss: 8,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_board_info() {
        assert_eq!(BOARD_INFO.name, "micoair-h743-v2");
        assert_eq!(BOARD_INFO.mcu, "STM32H743VIH6");
    }

    #[test]
    fn test_motor_count() {
        assert_eq!(pins::motors::COUNT, 10);
    }

    #[test]
    fn test_default_configs() {
        let bmi088 = sensors::Bmi088Config::default();
        assert_eq!(bmi088.accel_range_g, 24);
        assert_eq!(bmi088.gyro_range_dps, 2000);

        let bmi270 = sensors::Bmi270Config::default();
        assert_eq!(bmi270.accel_range_g, 16);
        assert_eq!(bmi270.gyro_range_dps, 2000);

        let spl06 = sensors::Spl06Config::default();
        assert_eq!(spl06.pressure_oversample, 64);

        let qmc = sensors::Qmc5883lConfig::default();
        assert_eq!(qmc.output_rate_hz, 200);
    }
}
