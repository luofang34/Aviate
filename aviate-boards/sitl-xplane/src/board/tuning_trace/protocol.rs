//! Versioned wire values for the simulator tuning trace.

use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_hal_xil::sim_types::SimImuData;
use serde::{Deserialize, Serialize};

use super::super::{XPlaneConstraintFlags, XPlaneControlObservation};

/// Frame names in the tuning trace protocol.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningFrameType {
    /// Aviate identity sent before observations.
    AviateTuningHandshake,
    /// Runner acceptance of the Aviate identity.
    AviateTuningReady,
    /// One causal simulator-packet observation.
    AviateControlObservation,
    /// Runner acceptance of one observation sequence.
    AviateTuningObservationAck,
}

/// One exact run identity sent before the trace starts.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningHandshake {
    /// Must be `aviate-tuning-handshake`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the exact run manifest text.
    pub run_manifest_digest: String,
    /// SHA-256 of the running executable.
    pub build_identity: String,
    /// SHA-256 of the embedded source bundle.
    pub source_identity: String,
    /// SHA-256 of the dependency lock file.
    pub lock_identity: String,
    /// SHA-256 of the simulator model.
    pub simulator_model_digest: String,
    /// SHA-256 of the consumed runtime handshake.
    pub runtime_handshake_digest: String,
    /// SHA-256 of the candidate document. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub candidate_digest: Option<String>,
    /// SHA-256 of the resolved overlay lineage. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub candidate_lineage_digest: Option<String>,
    /// SHA-256 of the plant artifact. Omitted for a base run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plant_artifact_digest: Option<String>,
    /// Flight algorithm identity as 16 lowercase hexadecimal digits.
    pub algorithm_identity_hash: String,
    /// Resolved kernel identity as 16 lowercase hexadecimal digits.
    pub kernel_config_hash: String,
}

/// Runner acceptance of one handshake.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningReady {
    /// Must be `aviate-tuning-ready`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the accepted run manifest text.
    pub run_manifest_digest: String,
}

/// Runner acceptance of one observation.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningObservationAck {
    /// Must be `aviate-tuning-observation-ack`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// SHA-256 of the active run manifest text.
    pub run_manifest_digest: String,
    /// Exact observation sequence accepted by the runner.
    pub sequence: u64,
}

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

/// Plant-boundary constraints applied to one packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningConstraintFlags {
    /// Injection required a force clamp.
    pub injection_clamp: bool,
    /// Actuator lane count was invalid.
    pub invalid_actuator_count: bool,
    /// The kernel did not provide an actuator answer.
    pub missing_actuator_answer: bool,
    /// Collective rate limiting changed the command.
    pub collective_rate: bool,
    /// Collective mean limiting changed the command.
    pub mean_ceiling: bool,
    /// A lane ceiling changed the command.
    pub lane_ceiling: bool,
    /// The on-ground authority limit changed the command.
    pub ground_squeeze: bool,
    /// The external trace did not accept this packet.
    pub tuning_trace_failure: bool,
}

/// One causal high-rate observation.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningControlObservation {
    /// Must be `aviate-control-observation`.
    #[serde(rename = "type")]
    pub frame_type: TuningFrameType,
    /// Protocol schema version.
    pub schema_version: u16,
    /// Gap-free sequence assigned by Aviate.
    pub sequence: u64,
    /// Simulator sample time in microseconds.
    pub simulator_timestamp_us: u64,
    /// Command retained from an external or experiment request.
    pub requested_command: Option<TuningCommand>,
    /// Exact command supplied to the kernel update.
    pub effective_command: TuningCommand,
    /// Effective control mode, repeated for streaming gates.
    pub control_mode: TuningControlMode,
    /// Estimator readback after the kernel update.
    pub estimate: TuningEstimate,
    /// IMU values that caused this update.
    pub imu: Option<TuningImu>,
    /// Mixer-order force lanes before wire constraints.
    pub pre_wire_force_lanes: [f32; 4],
    /// Mixer-order force lanes after wire constraints and before the curve.
    pub applied_force_lanes: [f32; 4],
    /// Constraint flags for this packet.
    pub constraint_flags: TuningConstraintFlags,
    /// Kernel armed state after the update.
    pub armed: bool,
}

impl TuningControlObservation {
    pub(super) fn from_packet(
        sequence: u64,
        observation: XPlaneControlObservation,
        requested: Option<&Command>,
        effective: &Command,
        estimate: &StateEstimate,
        armed: bool,
    ) -> Self {
        Self {
            frame_type: TuningFrameType::AviateControlObservation,
            schema_version: super::TUNING_TRACE_SCHEMA_VERSION,
            sequence,
            simulator_timestamp_us: observation.timestamp_us,
            requested_command: requested.map(TuningCommand::from),
            effective_command: TuningCommand::from(effective),
            control_mode: effective.mode.into(),
            estimate: TuningEstimate::from(estimate),
            imu: observation.imu.map(TuningImu::from),
            pre_wire_force_lanes: observation.pre_wire_force_lanes,
            applied_force_lanes: observation.applied_force_lanes,
            constraint_flags: observation.constraint_flags.into(),
            armed,
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

impl From<XPlaneConstraintFlags> for TuningConstraintFlags {
    fn from(value: XPlaneConstraintFlags) -> Self {
        Self {
            injection_clamp: value.injection_clamp,
            invalid_actuator_count: value.invalid_actuator_count,
            missing_actuator_answer: value.missing_actuator_answer,
            collective_rate: value.collective_rate,
            mean_ceiling: value.mean_ceiling,
            lane_ceiling: value.lane_ceiling,
            ground_squeeze: value.ground_squeeze,
            tuning_trace_failure: value.tuning_trace_failure,
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
enum_map!(ConfigMode => TuningConfigMode, Hover, Cruise, Transition, Degraded);
enum_map!(EstimateQuality => TuningEstimateQuality, Good, Degraded, Unusable);
