//! STM32H7 PWM Motor Output
//!
//! PWM driver for motor ESCs on STM32H7 family.
//!
//! ## Architecture
//!
//! ```text
//! BoardHal<..., PwmMotors<TIM1>> → write(&cmd) → TIM1 PWM channels → ESC → Motors
//! ```
//!
//! ## Supported Timers
//!
//! - TIM1: Advanced timer, 4 channels (typical for motors 1-4)
//! - TIM3/TIM4: General purpose timers (motors 5-8)
//! - TIM15: 2-channel timer (motors 9-10)
//!
//! ## PWM Configuration
//!
//! | Parameter | Value | Notes |
//! |-----------|-------|-------|
//! | Frequency | 400 Hz | Standard ESC PWM |
//! | Period | 2.5 ms | 1/400 Hz |
//! | Min pulse | 1000 µs | 0% throttle |
//! | Max pulse | 2000 µs | 100% throttle |
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_hal_stm32h7::pwm::PwmMotors;
//!
//! // Create PWM motor driver from configured timer
//! let pwm = PwmMotors::new(tim1_pwm, 4);  // 4 motors on TIM1
//!
//! // Use via ActuatorDriver trait
//! pwm.arm();
//! pwm.write(&cmd)?;
//! ```
//!
//! ## Safety
//!
//! - Disarmed state outputs 0% duty cycle (motors off)
//! - Armed state allows write() to control motors
//! - Disarm immediately sets all outputs to safe value

use aviate_hal_io::error::ActuatorResult;
use aviate_hal_io::traits::{ActuatorDriver, ActuatorStatus, RawActuatorCmd, MAX_ACTUATOR_OUTPUTS};

/// PWM configuration for ESC signals
#[derive(Clone, Copy, Debug)]
pub struct PwmConfig {
    /// PWM frequency in Hz (typically 400 for standard ESCs)
    pub freq_hz: u32,
    /// Minimum pulse width in microseconds (typically 1000)
    pub min_pulse_us: u32,
    /// Maximum pulse width in microseconds (typically 2000)
    pub max_pulse_us: u32,
}

impl Default for PwmConfig {
    fn default() -> Self {
        Self {
            freq_hz: 400,
            min_pulse_us: 1000,
            max_pulse_us: 2000,
        }
    }
}

impl PwmConfig {
    /// Standard PWM configuration for ESCs (400Hz, 1000-2000µs)
    pub const fn standard() -> Self {
        Self {
            freq_hz: 400,
            min_pulse_us: 1000,
            max_pulse_us: 2000,
        }
    }

    /// OneShot125 configuration (faster ESC protocol)
    pub const fn oneshot125() -> Self {
        Self {
            freq_hz: 2000, // Higher update rate
            min_pulse_us: 125,
            max_pulse_us: 250,
        }
    }
}

/// PWM motor driver for STM32H7
///
/// Generic driver that works with any configured PWM timer.
/// The actual timer configuration is done at board initialization.
///
/// ## State Machine
///
/// ```text
/// Disarmed (default)
///     │
///     │ arm()
///     ▼
///   Armed ←──────────┐
///     │              │
///     │ disarm()     │ write()
///     ▼              │
/// Disarmed ──────────┘
/// ```
#[derive(Debug)]
pub struct PwmMotors {
    /// Number of motor outputs (channels)
    motor_count: u8,
    /// PWM configuration
    config: PwmConfig,
    /// Armed state
    armed: bool,
    /// Cached output values (for telemetry/debug)
    outputs: [f32; MAX_ACTUATOR_OUTPUTS],
    /// Timer period in clock ticks (calculated from freq_hz and timer clock)
    #[allow(dead_code)]
    period_ticks: u32,
}

impl PwmMotors {
    /// Create a new PWM motor driver
    ///
    /// # Arguments
    ///
    /// * `motor_count` - Number of motor outputs (1-16)
    /// * `timer_clock_hz` - Timer input clock frequency in Hz
    /// * `config` - PWM configuration (frequency, pulse widths)
    ///
    /// # Example
    ///
    /// ```ignore
    /// let pwm = PwmMotors::new(4, 240_000_000, PwmConfig::standard());
    /// ```
    pub fn new(motor_count: u8, timer_clock_hz: u32, config: PwmConfig) -> Self {
        // Calculate period in timer ticks
        let period_ticks = timer_clock_hz / config.freq_hz;

        Self {
            motor_count: motor_count.min(MAX_ACTUATOR_OUTPUTS as u8),
            config,
            armed: false,
            outputs: [0.0; MAX_ACTUATOR_OUTPUTS],
            period_ticks,
        }
    }

    /// Create with default standard ESC configuration
    ///
    /// # Arguments
    ///
    /// * `motor_count` - Number of motor outputs
    /// * `timer_clock_hz` - Timer input clock frequency
    pub fn with_defaults(motor_count: u8, timer_clock_hz: u32) -> Self {
        Self::new(motor_count, timer_clock_hz, PwmConfig::standard())
    }

    /// Get PWM configuration
    pub fn config(&self) -> &PwmConfig {
        &self.config
    }

    /// Get number of motor outputs
    pub fn motor_count(&self) -> u8 {
        self.motor_count
    }

    /// Convert normalized throttle [0.0, 1.0] to PWM duty cycle (ticks)
    ///
    /// Maps 0.0 → min_pulse_us, 1.0 → max_pulse_us
    fn throttle_to_duty(&self, throttle: f32) -> u32 {
        let clamped = throttle.clamp(0.0, 1.0);
        let pulse_range = self.config.max_pulse_us - self.config.min_pulse_us;
        let pulse_us = self.config.min_pulse_us + (clamped * pulse_range as f32) as u32;

        // Convert pulse_us to duty cycle (ticks)
        // duty_ticks = pulse_us * period_ticks / period_us
        // period_us = 1_000_000 / freq_hz
        // So: duty_ticks = pulse_us * period_ticks * freq_hz / 1_000_000
        //
        // To avoid overflow with large period_ticks, calculate as:
        // duty_ticks = period_ticks * pulse_us / period_us
        // where period_us = 1_000_000 / freq_hz
        let period_us = 1_000_000 / self.config.freq_hz;
        (self.period_ticks as u64 * pulse_us as u64 / period_us as u64) as u32
    }

    /// Write to hardware PWM channels
    ///
    /// This is the board-specific implementation hook.
    /// Boards override this to set actual PWM duty cycles.
    ///
    /// Default: no-op (for testing without hardware)
    #[allow(unused_variables)]
    fn write_hardware(&mut self, duties: &[u32]) {
        // COV:EXCL_START(STUB) - Hardware-only code
        // Board-specific implementation goes here.
        // This is a stub that does nothing.
        //
        // Real implementation would:
        // 1. Set TIM1->CCR1 = duties[0] for motor 1
        // 2. Set TIM1->CCR2 = duties[1] for motor 2
        // etc.
        // COV:EXCL_STOP
    }

    /// Set all outputs to disarmed state (0% throttle)
    fn set_disarmed_outputs(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only code
        // Set all duty cycles to minimum (motors off)
        let duties = [self.throttle_to_duty(0.0); MAX_ACTUATOR_OUTPUTS];
        self.write_hardware(&duties[..self.motor_count as usize]);
        self.outputs = [0.0; MAX_ACTUATOR_OUTPUTS];
        // COV:EXCL_STOP
    }
}

impl ActuatorDriver for PwmMotors {
    fn write(&mut self, cmd: &RawActuatorCmd) -> ActuatorResult<()> {
        // COV:EXCL_START(STUB) - Hardware-only code

        // If not armed, ignore command (safety)
        if !self.armed {
            return Ok(());
        }

        // Calculate duty cycles for each motor
        let mut duties = [0u32; MAX_ACTUATOR_OUTPUTS];
        let count = (cmd.count as usize).min(self.motor_count as usize);

        for (i, (duty, &output)) in duties.iter_mut().zip(cmd.outputs.iter()).enumerate().take(count) {
            *duty = self.throttle_to_duty(output);
            self.outputs[i] = output;
        }

        // Write to hardware
        self.write_hardware(&duties[..count]);

        Ok(())
        // COV:EXCL_STOP
    }

    fn read_status(&mut self) -> Option<ActuatorStatus> {
        // PWM doesn't support telemetry
        None
    }

    fn status_ready(&mut self) -> bool {
        false
    }

    fn arm(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only code
        self.armed = true;
        // COV:EXCL_STOP
    }

    fn disarm(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only code
        self.armed = false;
        self.set_disarmed_outputs();
        // COV:EXCL_STOP
    }

    fn is_armed(&self) -> bool {
        self.armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pwm_config_default() {
        let config = PwmConfig::default();
        assert_eq!(config.freq_hz, 400);
        assert_eq!(config.min_pulse_us, 1000);
        assert_eq!(config.max_pulse_us, 2000);
    }

    #[test]
    fn test_pwm_config_oneshot125() {
        let config = PwmConfig::oneshot125();
        assert_eq!(config.freq_hz, 2000);
        assert_eq!(config.min_pulse_us, 125);
        assert_eq!(config.max_pulse_us, 250);
    }

    #[test]
    fn test_pwm_motors_creation() {
        let pwm = PwmMotors::new(4, 240_000_000, PwmConfig::standard());
        assert_eq!(pwm.motor_count(), 4);
        assert!(!pwm.is_armed());
    }

    #[test]
    fn test_pwm_motors_arm_disarm() {
        let mut pwm = PwmMotors::with_defaults(4, 240_000_000);

        assert!(!pwm.is_armed());

        pwm.arm();
        assert!(pwm.is_armed());

        pwm.disarm();
        assert!(!pwm.is_armed());
    }

    #[test]
    fn test_pwm_motors_write_ignored_when_disarmed() {
        let mut pwm = PwmMotors::with_defaults(4, 240_000_000);

        // Write should succeed but be ignored when disarmed
        let cmd = RawActuatorCmd {
            outputs: [0.5; MAX_ACTUATOR_OUTPUTS],
            count: 4,
        };

        let result = pwm.write(&cmd);
        assert!(result.is_ok());
        // Outputs should remain at 0 (disarmed)
        assert_eq!(pwm.outputs[0], 0.0);
    }

    #[test]
    fn test_pwm_motors_write_when_armed() {
        let mut pwm = PwmMotors::with_defaults(4, 240_000_000);
        pwm.arm();

        let cmd = RawActuatorCmd {
            outputs: [0.5, 0.6, 0.7, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            count: 4,
        };

        let result = pwm.write(&cmd);
        assert!(result.is_ok());

        // Outputs should be cached
        assert!((pwm.outputs[0] - 0.5).abs() < 0.01);
        assert!((pwm.outputs[1] - 0.6).abs() < 0.01);
        assert!((pwm.outputs[2] - 0.7).abs() < 0.01);
        assert!((pwm.outputs[3] - 0.8).abs() < 0.01);
    }

    #[test]
    fn test_pwm_motors_no_telemetry() {
        let mut pwm = PwmMotors::with_defaults(4, 240_000_000);
        assert!(!pwm.status_ready());
        assert!(pwm.read_status().is_none());
    }

    #[test]
    fn test_throttle_to_duty_bounds() {
        let pwm = PwmMotors::with_defaults(4, 240_000_000);

        // 0% throttle should give min pulse
        let duty_0 = pwm.throttle_to_duty(0.0);
        // 100% throttle should give max pulse
        let duty_100 = pwm.throttle_to_duty(1.0);

        // Duty at 100% should be greater than at 0%
        assert!(duty_100 > duty_0);

        // Out of bounds should clamp
        let duty_neg = pwm.throttle_to_duty(-0.5);
        let duty_over = pwm.throttle_to_duty(1.5);

        assert_eq!(duty_neg, duty_0);
        assert_eq!(duty_over, duty_100);
    }

    #[test]
    fn test_disarm_clears_outputs() {
        let mut pwm = PwmMotors::with_defaults(4, 240_000_000);
        pwm.arm();

        // Write some outputs
        let cmd = RawActuatorCmd {
            outputs: [0.5; MAX_ACTUATOR_OUTPUTS],
            count: 4,
        };
        let _ = pwm.write(&cmd);

        // Disarm should clear outputs
        pwm.disarm();
        assert_eq!(pwm.outputs[0], 0.0);
        assert_eq!(pwm.outputs[1], 0.0);
    }
}
