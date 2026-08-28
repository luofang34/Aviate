//! Condition-execution values in one tuning observation.

use aviate_hal_xil::perturbation::{
    ActuatorApplication, ActuatorBypassReason, ActuatorEligibility, SensorApplication,
};
use serde::{Deserialize, Serialize};

use super::super::super::{XPlaneHoverInitialization, XPlaneSendEvidence};

/// One Aviate-owned perturbation capability.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TuningPerturbationCapability {
    /// Scale eligible actuator commands.
    ActuatorAuthority,
    /// Apply deterministic command hold.
    CommandHold,
    /// Scale the controller hover-force initialization.
    HoverTrimUncertainty,
    /// Apply deterministic bounded sensor perturbations.
    SensorPerturbation,
}

impl TuningPerturbationCapability {
    pub(in crate::board::tuning_trace) const fn as_str(self) -> &'static str {
        match self {
            Self::ActuatorAuthority => "actuator_authority",
            Self::CommandHold => "command_hold",
            Self::HoverTrimUncertainty => "hover_trim_uncertainty",
            Self::SensorPerturbation => "sensor_perturbation",
        }
    }
}

/// Online hover estimator state for one run.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TuningHoverEstimatorMode {
    /// The estimator can update the hover-force value.
    Online,
    /// The estimator is disabled.
    Disabled,
    /// The estimator keeps one fixed value.
    Frozen,
}

/// Sensor values before and after deterministic perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningSensorApplication {
    /// Digest of the converted input before perturbation.
    pub raw_digest: [u8; 32],
    /// Digest of the input supplied to the controller.
    pub effective_digest: [u8; 32],
    /// Presence bits in stable sensor-lane order.
    pub presence_mask: u16,
    /// Bits for lanes whose values changed.
    pub changed_mask: u16,
    /// Global update bucket for each configured and present lane.
    pub update_buckets: [Option<u64>; 12],
    /// Converted input bits in stable sensor-lane order.
    pub raw_value_bits: [Option<u32>; 12],
    /// Controller input bits in stable sensor-lane order.
    pub effective_value_bits: [Option<u32>; 12],
}

/// A safety reason that prevents actuator perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum TuningActuatorBypassReason {
    /// The kernel did not produce an answer.
    MissingAnswer,
    /// The actuator lane count is invalid.
    InvalidActuatorCount,
    /// The command comes from a backup path.
    Backup,
    /// The command comes from a direct path.
    Direct,
    /// The command comes from a failsafe path.
    Failsafe,
    /// A fallback mask is active.
    FallbackMask,
    /// The arm state changed on this sample.
    ArmTransition,
    /// The system is disarmed.
    Disarmed,
    /// Emergency termination is active.
    EmergencyTermination,
}

/// Whether one actuator answer can use perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(tag = "kind", content = "reason", rename_all = "kebab-case")]
pub enum TuningActuatorEligibility {
    /// Apply configured actuator factors.
    Eligible,
    /// Preserve one safety command without a factor.
    Bypass(TuningActuatorBypassReason),
}

/// Actuator values before plant constraints and transforms.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningActuatorApplication {
    /// Kernel-requested force-domain lane bits.
    pub requested_lane_bits: [u32; 16],
    /// Authority-scaled force-domain lane bits.
    pub authority_scaled_lane_bits: [u32; 16],
    /// Current or held force-domain lane bits.
    pub effective_lane_bits: [u32; 16],
    /// Active actuator lane count.
    pub lane_count: u8,
    /// Armed state in the actuator answer.
    pub actuator_answer_armed: bool,
    /// Kernel fallback lanes used for eligibility.
    pub kernel_fallback_mask: u8,
    /// Eligibility result for this sample.
    pub eligibility: TuningActuatorEligibility,
    /// True for the safe history prime outside interval zero.
    pub prime: bool,
    /// Current interval epoch.
    pub interval_epoch: Option<u64>,
    /// Current interval index.
    pub interval_index: Option<u64>,
    /// Current zero-based interval position.
    pub interval_position: Option<u32>,
    /// Digest of the fixed-width interval identity.
    pub interval_identity: Option<[u8; 32]>,
    /// True when the schedule selected hold.
    pub selected_hold: bool,
    /// True when the engine applied hold.
    pub applied_hold: bool,
    /// True when this sample completed the interval.
    pub interval_complete: bool,
}

/// Evidence for the only actuator send for one sample.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningSendEvidence {
    /// True when the send call completed.
    pub reply_attempted: bool,
    /// True when the transport accepted the full reply.
    pub reply_succeeded: bool,
    /// Simulator timestamp in the reply.
    pub echoed_timestamp_us: u64,
    /// True when the reply declared lockstep operation.
    pub lockstep: bool,
}

/// Immutable hover initialization for one run.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TuningHoverInitialization {
    /// Baseline hover-force bits.
    pub baseline_force_bits: u32,
    /// Effective hover-force bits.
    pub effective_force_bits: u32,
    /// Baseline scale in basis points.
    pub scale_basis_points: u16,
    /// Effective kernel configuration hash.
    pub kernel_config_hash: u64,
}

impl From<SensorApplication> for TuningSensorApplication {
    fn from(value: SensorApplication) -> Self {
        Self {
            raw_digest: value.raw_digest,
            effective_digest: value.effective_digest,
            presence_mask: value.presence_mask,
            changed_mask: value.changed_mask,
            update_buckets: value.update_buckets,
            raw_value_bits: value.raw_value_bits,
            effective_value_bits: value.effective_value_bits,
        }
    }
}

impl From<ActuatorApplication> for TuningActuatorApplication {
    fn from(value: ActuatorApplication) -> Self {
        Self {
            requested_lane_bits: value.requested_lanes.map(f32::to_bits),
            authority_scaled_lane_bits: value.authority_scaled_lanes.map(f32::to_bits),
            effective_lane_bits: value.effective_lanes.map(f32::to_bits),
            lane_count: value.lane_count,
            actuator_answer_armed: value.requested_armed,
            kernel_fallback_mask: value.kernel_fallback_mask,
            eligibility: value.eligibility.into(),
            prime: value.prime,
            interval_epoch: value.interval_epoch,
            interval_index: value.interval_index,
            interval_position: value.interval_position,
            interval_identity: value.interval_identity,
            selected_hold: value.selected_hold,
            applied_hold: value.applied_hold,
            interval_complete: value.interval_complete,
        }
    }
}

impl From<ActuatorEligibility> for TuningActuatorEligibility {
    fn from(value: ActuatorEligibility) -> Self {
        match value {
            ActuatorEligibility::Eligible => Self::Eligible,
            ActuatorEligibility::Bypass(reason) => Self::Bypass(reason.into()),
        }
    }
}

impl From<ActuatorBypassReason> for TuningActuatorBypassReason {
    fn from(value: ActuatorBypassReason) -> Self {
        match value {
            ActuatorBypassReason::MissingAnswer => Self::MissingAnswer,
            ActuatorBypassReason::InvalidActuatorCount => Self::InvalidActuatorCount,
            ActuatorBypassReason::Backup => Self::Backup,
            ActuatorBypassReason::Direct => Self::Direct,
            ActuatorBypassReason::Failsafe => Self::Failsafe,
            ActuatorBypassReason::FallbackMask => Self::FallbackMask,
            ActuatorBypassReason::ArmTransition => Self::ArmTransition,
            ActuatorBypassReason::Disarmed => Self::Disarmed,
            ActuatorBypassReason::EmergencyTermination => Self::EmergencyTermination,
        }
    }
}

impl From<XPlaneSendEvidence> for TuningSendEvidence {
    fn from(value: XPlaneSendEvidence) -> Self {
        Self {
            reply_attempted: value.reply_attempted,
            reply_succeeded: value.reply_succeeded,
            echoed_timestamp_us: value.echoed_timestamp_us,
            lockstep: value.lockstep,
        }
    }
}

impl From<XPlaneHoverInitialization> for TuningHoverInitialization {
    fn from(value: XPlaneHoverInitialization) -> Self {
        Self {
            baseline_force_bits: value.baseline_force_bits,
            effective_force_bits: value.effective_force_bits,
            scale_basis_points: value.scale_basis_points,
            kernel_config_hash: value.kernel_config_hash,
        }
    }
}
