//! Command and estimator values in one tuning observation.

use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_hal_xil::sim_types::SimImuData;
use aviate_hal_xil::{MavlinkCommandFamily, MavlinkCommandProvenance};
use serde::{Deserialize, Serialize};

/// Control-mode names used by trace commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningControlMode {
    /// Direct body-rate control.
    Rate,
    /// Attitude control.
    Attitude,
    /// Altitude hold.
    AltitudeHold,
    /// Position hold.
    PositionHold,
    /// Velocity control.
    VelocityControl,
    /// Path-deviation tracking.
    DeviationTracking,
}

/// Command-source names used by trace commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningCommandSource {
    /// Pilot input.
    Pilot,
    /// Onboard automation.
    Autopilot,
    /// Ground control station.
    Gcs,
    /// Failsafe command.
    Failsafe,
}

/// MAVLink setpoint-family names used by raw command provenance.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningCommandFamily {
    /// MAVLink `SET_ATTITUDE_TARGET`.
    AttitudeTarget,
    /// MAVLink `SET_POSITION_TARGET_LOCAL_NED`.
    PositionTargetLocalNed,
}

/// Exact producer-side identity of one raw MAVLink setpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningCommandProvenance {
    /// UDP source endpoint observed by Aviate.
    pub source_endpoint: std::net::SocketAddr,
    /// Nonzero source epoch for this process and sender incarnation.
    pub source_epoch: u64,
    /// MAVLink header system identifier.
    pub mavlink_system_id: u8,
    /// MAVLink header component identifier.
    pub mavlink_component_id: u8,
    /// Full MAVLink header sequence field.
    pub mavlink_frame_sequence: u8,
    /// MAVLink setpoint boot time.
    pub time_boot_ms: u32,
    /// MAVLink setpoint family.
    pub command_family: TuningCommandFamily,
    /// SHA-256 digest of the exact received frame.
    pub frame_digest: [u8; 32],
}

/// Requested configuration mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningConfigMode {
    /// Hover configuration.
    Hover,
    /// Cruise configuration.
    Cruise,
    /// Transition configuration.
    Transition,
    /// Degraded configuration.
    Degraded,
}

/// One command setpoint in SI units and normalized force.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningSetpoint {
    /// Attitude quaternion in `[w, x, y, z]` order.
    pub attitude_wxyz: Option<[f32; 4]>,
    /// Body angular rates in radians per second.
    pub angular_rate_rad_s: Option<[f32; 3]>,
    /// Altitude in meters.
    pub altitude_m: Option<f32>,
    /// Vertical speed in meters per second.
    pub vertical_speed_m_s: Option<f32>,
    /// Heading in radians.
    pub heading_rad: Option<f32>,
    /// Position in NED meters.
    pub position_ned_m: Option<[f32; 3]>,
    /// Velocity in NED meters per second.
    pub velocity_ned_m_s: Option<[f32; 3]>,
    /// Lateral path deviation in meters.
    pub lateral_deviation_m: Option<f32>,
    /// Vertical path deviation in meters.
    pub vertical_deviation_m: Option<f32>,
    /// Collective in normalized force domain.
    pub collective_force: f32,
}

/// One requested or effective flight command.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningCommand {
    /// Source command sequence.
    pub sequence: u32,
    /// Command source.
    pub source: TuningCommandSource,
    /// Requested control mode.
    pub control_mode: TuningControlMode,
    /// Optional configuration-mode request.
    pub config_mode_request: Option<TuningConfigMode>,
    /// Requested setpoint.
    pub setpoint: TuningSetpoint,
}

/// Estimator quality names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningEstimateQuality {
    /// All required estimator checks pass.
    Good,
    /// The estimate is usable with reduced quality.
    Degraded,
    /// The estimate is not usable for control.
    Unusable,
}

/// Decoded estimator-validity flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningEstimateValidity {
    /// Attitude is valid.
    pub attitude: bool,
    /// Angular rate is valid.
    pub angular_rate: bool,
    /// Position is valid.
    pub position: bool,
    /// Velocity is valid.
    pub velocity: bool,
}

/// One estimator readback after the packet's kernel step.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningEstimate {
    /// Raw flags: attitude=1, angular-rate=2, position=4, velocity=8.
    pub valid_flags: u8,
    /// Decoded validity flags.
    pub validity: TuningEstimateValidity,
    /// Estimator quality.
    pub quality: TuningEstimateQuality,
    /// Attitude quaternion in `[w, x, y, z]` order.
    pub attitude_wxyz: [f32; 4],
    /// Body angular rates in radians per second.
    pub angular_rate_rad_s: [f32; 3],
    /// Position in NED meters.
    pub position_ned_m: [f32; 3],
    /// Velocity in NED meters per second.
    pub velocity_ned_m_s: [f32; 3],
}

/// IMU values in one simulator packet.
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningImu {
    /// Acceleration in meters per second squared.
    pub acceleration_m_s2: [f32; 3],
    /// Body angular rates in radians per second.
    pub angular_rate_rad_s: [f32; 3],
    /// Sensor temperature in degrees Celsius.
    pub temperature_c: Option<f32>,
}

impl From<MavlinkCommandProvenance> for TuningCommandProvenance {
    fn from(value: MavlinkCommandProvenance) -> Self {
        Self {
            source_endpoint: value.source_endpoint,
            source_epoch: value.source_epoch,
            mavlink_system_id: value.mavlink_system_id,
            mavlink_component_id: value.mavlink_component_id,
            mavlink_frame_sequence: value.mavlink_frame_sequence,
            time_boot_ms: value.time_boot_ms,
            command_family: value.command_family.into(),
            frame_digest: value.frame_digest,
        }
    }
}

impl From<&Command> for TuningCommand {
    fn from(value: &Command) -> Self {
        Self {
            sequence: value.sequence,
            source: value.source.into(),
            control_mode: value.mode.into(),
            config_mode_request: value.config_mode_request.map(Into::into),
            setpoint: TuningSetpoint::from(&value.setpoint),
        }
    }
}

impl From<&Setpoint> for TuningSetpoint {
    fn from(value: &Setpoint) -> Self {
        Self {
            attitude_wxyz: value.attitude.map(|q| [q.w, q.x, q.y, q.z]),
            angular_rate_rad_s: value.angular_rate.map(|axis| axis.map(|rate| rate.0)),
            altitude_m: value.altitude.map(|item| item.0),
            vertical_speed_m_s: value.vertical_speed.map(|item| item.0),
            heading_rad: value.heading.map(|item| item.0),
            position_ned_m: value.position.map(|axis| axis.map(|item| item.0)),
            velocity_ned_m_s: value.velocity.map(|axis| axis.map(|item| item.0)),
            lateral_deviation_m: value.lateral_deviation.map(|item| item.0),
            vertical_deviation_m: value.vertical_deviation.map(|item| item.0),
            collective_force: value.collective_thrust.0,
        }
    }
}

impl From<&StateEstimate> for TuningEstimate {
    fn from(value: &StateEstimate) -> Self {
        let flags = value.valid_flags;
        Self {
            valid_flags: flags.bits(),
            validity: TuningEstimateValidity {
                attitude: flags.contains(StateValidFlags::ATTITUDE),
                angular_rate: flags.contains(StateValidFlags::ANGULAR_RATE),
                position: flags.contains(StateValidFlags::POSITION),
                velocity: flags.contains(StateValidFlags::VELOCITY),
            },
            quality: value.quality.into(),
            attitude_wxyz: [
                value.attitude.w,
                value.attitude.x,
                value.attitude.y,
                value.attitude.z,
            ],
            angular_rate_rad_s: value.angular_velocity.map(|item| item.0),
            position_ned_m: value.position_ned.map(|item| item.0),
            velocity_ned_m_s: value.velocity_ned.map(|item| item.0),
        }
    }
}

impl From<SimImuData> for TuningImu {
    fn from(value: SimImuData) -> Self {
        Self {
            acceleration_m_s2: value.accel,
            angular_rate_rad_s: value.gyro,
            temperature_c: value.temperature,
        }
    }
}

macro_rules! enum_map {
    ($source:ty => $target:ty, $($variant:ident),+ $(,)?) => {
        impl From<$source> for $target {
            fn from(value: $source) -> Self {
                match value { $(<$source>::$variant => Self::$variant,)+ }
            }
        }
    };
}

enum_map!(ControlMode => TuningControlMode,
    Rate, Attitude, AltitudeHold, PositionHold, VelocityControl, DeviationTracking);
enum_map!(CommandSource => TuningCommandSource, Pilot, Autopilot, Gcs, Failsafe);
enum_map!(MavlinkCommandFamily => TuningCommandFamily, AttitudeTarget, PositionTargetLocalNed);
enum_map!(ConfigMode => TuningConfigMode, Hover, Cruise, Transition, Degraded);
enum_map!(EstimateQuality => TuningEstimateQuality, Good, Degraded, Unusable);
