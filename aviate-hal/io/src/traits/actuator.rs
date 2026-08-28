//! Actuator commands, status, and driver interface.

use crate::error::ActuatorResult;

/// Maximum number of actuator outputs.
pub const MAX_ACTUATOR_OUTPUTS: usize = 16;

/// One normalized actuator command group.
#[derive(Debug, Clone, Copy)]
pub struct RawActuatorCmd {
    /// Normalized actuator outputs.
    pub outputs: [f32; MAX_ACTUATOR_OUTPUTS],
    /// Number of active outputs.
    pub count: u8,
}

impl Default for RawActuatorCmd {
    fn default() -> Self {
        Self {
            outputs: [0.0; MAX_ACTUATOR_OUTPUTS],
            count: 0,
        }
    }
}

/// Error flags for one actuator channel.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ActuatorErrorFlags(pub u8);

impl ActuatorErrorFlags {
    /// No error.
    pub const NONE: Self = Self(0);
    /// Current is above its limit.
    pub const OVERCURRENT: Self = Self(1 << 0);
    /// Temperature is above its limit.
    pub const OVERTEMPERATURE: Self = Self(1 << 1);
    /// The actuator cannot move.
    pub const STALL: Self = Self(1 << 2);
    /// Supply voltage is below its limit.
    pub const VOLTAGE_LOW: Self = Self(1 << 3);
    /// Supply voltage is above its limit.
    pub const VOLTAGE_HIGH: Self = Self(1 << 4);
    /// Communication failed.
    pub const COMM_ERROR: Self = Self(1 << 5);
    /// The actuator has an internal fault.
    pub const HARDWARE_FAULT: Self = Self(1 << 6);

    /// Return true when one or more error flags are set.
    #[must_use]
    pub fn has_error(self) -> bool {
        self.0 != 0
    }

    /// Return true when the selected flag is set.
    #[must_use]
    pub fn contains(self, flag: Self) -> bool {
        self.0 & flag.0 != 0
    }
}

/// Optional feedback from one actuator channel.
#[derive(Debug, Clone, Copy, Default)]
pub struct ActuatorTelemetry {
    /// Motor speed or normalized servo position.
    pub speed_or_position: Option<f32>,
    /// Current in amperes.
    pub current_a: Option<f32>,
    /// Temperature in degrees Celsius.
    pub temperature_c: Option<f32>,
    /// Voltage in volts.
    pub voltage_v: Option<f32>,
    /// Channel error flags.
    pub errors: ActuatorErrorFlags,
}

impl ActuatorTelemetry {
    /// Return true when the channel supplies telemetry.
    #[must_use]
    pub fn has_data(&self) -> bool {
        self.speed_or_position.is_some()
            || self.current_a.is_some()
            || self.temperature_c.is_some()
            || self.voltage_v.is_some()
    }

    /// Return true when the channel reports an error.
    #[must_use]
    pub fn has_error(&self) -> bool {
        self.errors.has_error()
    }
}

/// Aggregate status for all actuator channels.
#[derive(Debug, Clone, Copy, Default)]
pub struct ActuatorStatus {
    /// Per-channel telemetry.
    pub channels: [ActuatorTelemetry; MAX_ACTUATOR_OUTPUTS],
    /// Number of channels with valid telemetry.
    pub channel_count: u8,
    /// Actuator bus voltage in volts.
    pub bus_voltage_v: Option<f32>,
    /// Total actuator current in amperes.
    pub total_current_a: Option<f32>,
}

impl ActuatorStatus {
    /// Return true when an active channel reports an error.
    #[must_use]
    pub fn has_errors(&self) -> bool {
        self.active_channels()
            .iter()
            .any(ActuatorTelemetry::has_error)
    }

    /// Get the maximum reported channel temperature.
    #[must_use]
    pub fn max_temperature_c(&self) -> Option<f32> {
        self.active_channels()
            .iter()
            .filter_map(|channel| channel.temperature_c)
            .fold(None, |maximum, temperature| {
                Some(maximum.map_or(temperature, |value: f32| value.max(temperature)))
            })
    }

    fn active_channels(&self) -> &[ActuatorTelemetry] {
        let count = usize::from(self.channel_count).min(MAX_ACTUATOR_OUTPUTS);
        &self.channels[..count]
    }
}

/// Bidirectional driver interface for one actuator group.
pub trait ActuatorDriver {
    /// Write normalized actuator outputs.
    fn write(&mut self, command: &RawActuatorCmd) -> ActuatorResult<()>;

    /// Read available actuator feedback.
    fn read_status(&mut self) -> Option<ActuatorStatus> {
        None
    }

    /// Return true when new actuator feedback is available.
    fn status_ready(&mut self) -> bool {
        false
    }

    /// Enable actuator outputs.
    fn arm(&mut self);

    /// Disable actuator outputs.
    fn disarm(&mut self);

    /// Return true when actuator outputs are enabled.
    fn is_armed(&self) -> bool;
}
