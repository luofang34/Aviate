//! Exact causal observation sent for one simulator packet.

use aviate_core::control::Command;
use aviate_core::state::StateEstimate;
use aviate_hal_xil::MavlinkCommandProvenance;
use serde::{Deserialize, Serialize};

use super::command::{
    TuningCommand, TuningCommandProvenance, TuningControlMode, TuningEstimate, TuningImu,
};
use super::perturbation::{
    TuningActuatorApplication, TuningHoverInitialization, TuningSendEvidence,
    TuningSensorApplication,
};
use super::TuningFrameType;
use crate::board::{XPlaneConstraintFlags, XPlaneControlObservation};

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
    /// Gap-free sequence assigned by the trace publisher.
    pub sequence: u64,
    /// Simulator sample time in microseconds.
    pub simulator_timestamp_us: u64,
    /// Global sample sequence used by deterministic perturbations.
    pub global_sample_sequence: u64,
    /// Command retained from an external or experiment request.
    pub requested_command: Option<TuningCommand>,
    /// Exact raw-frame identity for a retained GCS command.
    pub command_provenance: Option<TuningCommandProvenance>,
    /// Exact command supplied to the kernel update.
    pub effective_command: TuningCommand,
    /// Effective control mode, repeated for streaming gates.
    pub control_mode: TuningControlMode,
    /// Estimator readback after the kernel update.
    pub estimate: TuningEstimate,
    /// IMU values that caused this update.
    pub imu: Option<TuningImu>,
    /// Sensor perturbation evidence for this sample.
    pub sensor_application: Option<TuningSensorApplication>,
    /// Actuator perturbation evidence for this sample.
    pub actuator_application: Option<TuningActuatorApplication>,
    /// Exact force-domain lane-injection bits.
    pub lane_injection_bits: [u32; 4],
    /// Exact altitude bits supplied to the plant boundary.
    pub fix_altitude_m_bits: Option<u32>,
    /// Exact sample-duration bits supplied to the plant boundary.
    pub sample_dt_sec_bits: u32,
    /// Mixer-order force bits before wire constraints.
    pub pre_wire_force_lane_bits: [u32; 4],
    /// Mixer-order force bits after wire constraints.
    pub applied_force_lane_bits: [u32; 4],
    /// Final reordered actuator lane bits supplied to the send attempt.
    pub sent_lane_bits: [u32; 4],
    /// Evidence for the only actuator send for this sample.
    pub send: TuningSendEvidence,
    /// Repeated immutable controller initialization.
    pub hover_initialization: TuningHoverInitialization,
    /// Constraint flags for this packet.
    pub constraint_flags: TuningConstraintFlags,
    /// Kernel armed state after the update.
    pub armed: bool,
}

impl TuningControlObservation {
    pub(in crate::board::tuning_trace) fn from_packet(
        sequence: u64,
        observation: XPlaneControlObservation,
        requested: Option<&Command>,
        command_provenance: Option<MavlinkCommandProvenance>,
        effective: &Command,
        estimate: &StateEstimate,
        armed: bool,
    ) -> Self {
        Self {
            frame_type: TuningFrameType::AviateControlObservation,
            schema_version: super::super::TUNING_TRACE_SCHEMA_VERSION,
            sequence,
            simulator_timestamp_us: observation.timestamp_us,
            global_sample_sequence: observation.sample_sequence,
            requested_command: requested.map(TuningCommand::from),
            command_provenance: command_provenance.map(Into::into),
            effective_command: TuningCommand::from(effective),
            control_mode: effective.mode.into(),
            estimate: TuningEstimate::from(estimate),
            imu: observation.imu.map(TuningImu::from),
            sensor_application: observation.sensor_application.map(Into::into),
            actuator_application: observation.actuator_application.map(Into::into),
            lane_injection_bits: observation.lane_injection.map(f32::to_bits),
            fix_altitude_m_bits: observation.fix_altitude_m.map(f32::to_bits),
            sample_dt_sec_bits: observation.sample_dt_sec.to_bits(),
            pre_wire_force_lane_bits: observation.pre_wire_force_lanes.map(f32::to_bits),
            applied_force_lane_bits: observation.applied_force_lanes.map(f32::to_bits),
            sent_lane_bits: observation.sent_lanes.map(f32::to_bits),
            send: observation.send.into(),
            hover_initialization: observation.hover_initialization.into(),
            constraint_flags: observation.constraint_flags.into(),
            armed,
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
