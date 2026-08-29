//! Actuator authority and exact zero-order-hold execution.

use sha2::{Digest as _, Sha256};

use crate::sim_types::SimActuatorCmd;

use super::{PerturbationError, PerturbationIdentity};

pub(super) const NOMINAL_BASIS_POINTS: u16 = 10_000;
const COMMAND_HOLD_DOMAIN: &[u8] = b"pilotage-command-hold-v1";

/// One exact command-hold request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandHoldPerturbation {
    /// Held fraction in basis points.
    pub fraction_basis_points: u16,
    /// Eligible commands in one complete decision interval.
    pub decision_interval_samples: u32,
}

/// Actuator factors for one run.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActuatorPerturbation {
    /// Force-domain authority scale in basis points.
    pub authority_scale_basis_points: u16,
    /// Exact command-hold request, if present.
    pub command_hold: Option<CommandHoldPerturbation>,
}

impl Default for ActuatorPerturbation {
    fn default() -> Self {
        Self {
            authority_scale_basis_points: NOMINAL_BASIS_POINTS,
            command_hold: None,
        }
    }
}

/// A safety reason that prevents actuator perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActuatorBypassReason {
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

/// Whether one command can use actuator perturbation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActuatorEligibility {
    /// Apply configured actuator factors.
    Eligible,
    /// Preserve the safety command without a factor.
    Bypass(ActuatorBypassReason),
}

/// Actuator evidence before plant constraints and transforms.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorApplication {
    /// Kernel-requested force-domain lanes.
    pub requested_lanes: [f32; 16],
    /// Authority-scaled force-domain lanes.
    pub authority_scaled_lanes: [f32; 16],
    /// Current or replayed force-domain lanes supplied to constraints.
    pub effective_lanes: [f32; 16],
    /// Active lane count.
    pub lane_count: u8,
    /// Armed state of the kernel answer before perturbation.
    pub requested_armed: bool,
    /// Kernel fallback lanes that contributed to the eligibility decision.
    pub kernel_fallback_mask: u8,
    /// Eligibility decision for this sample.
    pub eligibility: ActuatorEligibility,
    /// True for the safe history prime outside interval zero.
    pub prime: bool,
    /// Current interval epoch for an eligible interval command.
    pub interval_epoch: Option<u64>,
    /// Current interval index for an eligible interval command.
    pub interval_index: Option<u64>,
    /// Current zero-based position in the interval.
    pub interval_position: Option<u32>,
    /// SHA-256 of the fixed-width interval identity preimage.
    pub interval_identity: Option<[u8; 32]>,
    /// Whether the seeded schedule selected this position.
    pub selected_hold: bool,
    /// Whether the engine replayed the last accepted command.
    pub applied_hold: bool,
    /// Whether this sample completed its exact decision interval.
    pub interval_complete: bool,
}

pub(super) struct ActuatorEngine {
    identity: PerturbationIdentity,
    config: ActuatorPerturbation,
    state: HoldState,
}

struct HoldState {
    interval_epoch: u64,
    interval_index: u64,
    interval_position: u32,
    interval_first_sequence: Option<u64>,
    schedule: Vec<bool>,
    last_accepted: Option<[f32; 16]>,
}

#[derive(Clone, Copy)]
struct ApplicationInputs {
    requested_lanes: [f32; 16],
    requested_armed: bool,
    kernel_fallback_mask: u8,
    authority_scaled_lanes: [f32; 16],
}

#[derive(Clone, Copy)]
struct HoldDecision {
    selected: bool,
    applied: bool,
    complete: bool,
}

impl ActuatorEngine {
    pub(super) fn new(
        identity: PerturbationIdentity,
        config: ActuatorPerturbation,
    ) -> Result<Self, PerturbationError> {
        validate_config(config)?;
        Ok(Self {
            identity,
            config,
            state: HoldState::new(),
        })
    }

    pub(super) fn apply(
        &mut self,
        sequence: u64,
        command: &mut SimActuatorCmd,
        eligibility: ActuatorEligibility,
        kernel_fallback_mask: u8,
    ) -> Result<ActuatorApplication, PerturbationError> {
        let mut inputs = ApplicationInputs {
            requested_lanes: command.outputs,
            requested_armed: command.armed,
            kernel_fallback_mask,
            authority_scaled_lanes: command.outputs,
        };
        if let ActuatorEligibility::Bypass(reason) = eligibility {
            self.state.safety_reset();
            return Ok(bypass_application(command, inputs, reason));
        }
        if usize::from(command.count) > command.outputs.len() {
            return Err(PerturbationError::InvalidActuatorCount(command.count));
        }
        scale_authority(command, self.config.authority_scale_basis_points);
        inputs.authority_scaled_lanes = command.outputs;
        let Some(hold) = self.config.command_hold else {
            return Ok(eligible_application(command, inputs));
        };
        self.apply_hold(sequence, command, inputs, hold)
    }

    fn apply_hold(
        &mut self,
        sequence: u64,
        command: &mut SimActuatorCmd,
        inputs: ApplicationInputs,
        hold: CommandHoldPerturbation,
    ) -> Result<ActuatorApplication, PerturbationError> {
        if self.state.last_accepted.is_none() {
            self.state.last_accepted = Some(inputs.authority_scaled_lanes);
            return Ok(prime_application(command, inputs));
        }
        self.state.ensure_schedule(self.identity, sequence, hold)?;
        let position = self.state.interval_position;
        let selected = self
            .state
            .schedule
            .get(position as usize)
            .copied()
            .ok_or(PerturbationError::InvalidCommandHold)?;
        let applied = if selected {
            if let Some(accepted) = self.state.last_accepted {
                command.outputs = accepted;
                true
            } else {
                false
            }
        } else {
            self.state.last_accepted = Some(inputs.authority_scaled_lanes);
            false
        };
        let application = interval_application(
            self.identity,
            command,
            inputs,
            &self.state,
            HoldDecision {
                selected,
                applied,
                complete: position.wrapping_add(1) == hold.decision_interval_samples,
            },
        )?;
        self.state.advance(hold.decision_interval_samples);
        Ok(application)
    }
}

impl HoldState {
    fn new() -> Self {
        Self {
            interval_epoch: 0,
            interval_index: 0,
            interval_position: 0,
            interval_first_sequence: None,
            schedule: Vec::new(),
            last_accepted: None,
        }
    }

    fn safety_reset(&mut self) {
        self.interval_epoch = self.interval_epoch.wrapping_add(1);
        self.interval_index = 0;
        self.interval_position = 0;
        self.interval_first_sequence = None;
        self.schedule.clear();
        self.last_accepted = None;
    }

    fn ensure_schedule(
        &mut self,
        identity: PerturbationIdentity,
        sequence: u64,
        hold: CommandHoldPerturbation,
    ) -> Result<(), PerturbationError> {
        if !self.schedule.is_empty() {
            return Ok(());
        }
        self.interval_first_sequence = Some(sequence);
        self.schedule = schedule(
            identity,
            self.interval_epoch,
            self.interval_index,
            sequence,
            hold,
        )?;
        Ok(())
    }

    fn advance(&mut self, interval_size: u32) {
        self.interval_position = self.interval_position.wrapping_add(1);
        if self.interval_position == interval_size {
            self.interval_position = 0;
            self.interval_index = self.interval_index.wrapping_add(1);
            self.interval_first_sequence = None;
            self.schedule.clear();
        }
    }
}

fn validate_config(config: ActuatorPerturbation) -> Result<(), PerturbationError> {
    if !(5_000..=15_000).contains(&config.authority_scale_basis_points) {
        return Err(PerturbationError::InvalidAuthorityScale(
            config.authority_scale_basis_points,
        ));
    }
    let Some(hold) = config.command_hold else {
        return Ok(());
    };
    let exact_product =
        u64::from(hold.fraction_basis_points) * u64::from(hold.decision_interval_samples);
    if !(1..=1_000).contains(&hold.fraction_basis_points)
        || !(1..=10_000).contains(&hold.decision_interval_samples)
        || exact_product % u64::from(NOMINAL_BASIS_POINTS) != 0
    {
        Err(PerturbationError::InvalidCommandHold)
    } else {
        Ok(())
    }
}

fn scale_authority(command: &mut SimActuatorCmd, basis_points: u16) {
    for lane in command.outputs.iter_mut().take(usize::from(command.count)) {
        *lane =
            ((f64::from(*lane) * f64::from(basis_points)) / f64::from(NOMINAL_BASIS_POINTS)) as f32;
    }
}

pub(super) fn schedule(
    identity: PerturbationIdentity,
    epoch: u64,
    index: u64,
    first_sequence: u64,
    hold: CommandHoldPerturbation,
) -> Result<Vec<bool>, PerturbationError> {
    let size = usize::try_from(hold.decision_interval_samples)
        .map_err(|_| PerturbationError::InvalidCommandHold)?;
    let mut positions = (0..size).collect::<Vec<_>>();
    for cursor in (1..positions.len()).rev() {
        let encoded = u64::try_from(cursor).map_err(|_| PerturbationError::InvalidCommandHold)?;
        let value = permutation_value(identity, epoch, index, first_sequence, encoded);
        let bound = encoded.wrapping_add(1);
        let swap =
            usize::try_from(value % bound).map_err(|_| PerturbationError::InvalidCommandHold)?;
        positions.swap(cursor, swap);
    }
    let count = u64::from(hold.fraction_basis_points) * u64::from(hold.decision_interval_samples)
        / u64::from(NOMINAL_BASIS_POINTS);
    let count = usize::try_from(count).map_err(|_| PerturbationError::InvalidCommandHold)?;
    let mut decisions = vec![false; size];
    for position in positions.into_iter().take(count) {
        decisions[position] = true;
    }
    Ok(decisions)
}

pub(super) fn permutation_value(
    identity: PerturbationIdentity,
    epoch: u64,
    index: u64,
    first_sequence: u64,
    cursor: u64,
) -> u64 {
    let mut hasher = interval_hasher(identity, epoch, index, first_sequence);
    hasher.update(cursor.to_le_bytes());
    let bytes = hasher.finalize();
    u64::from_le_bytes([
        bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7],
    ])
}

pub(super) fn interval_digest(
    identity: PerturbationIdentity,
    epoch: u64,
    index: u64,
    first_sequence: u64,
) -> [u8; 32] {
    interval_hasher(identity, epoch, index, first_sequence)
        .finalize()
        .into()
}

fn interval_hasher(
    identity: PerturbationIdentity,
    epoch: u64,
    index: u64,
    first_sequence: u64,
) -> Sha256 {
    let mut hasher = Sha256::new();
    hasher.update(COMMAND_HOLD_DOMAIN);
    hasher.update(identity.condition_digest);
    hasher.update(identity.run_seed.to_le_bytes());
    hasher.update(epoch.to_le_bytes());
    hasher.update(index.to_le_bytes());
    hasher.update(first_sequence.to_le_bytes());
    hasher
}

fn bypass_application(
    command: &SimActuatorCmd,
    inputs: ApplicationInputs,
    reason: ActuatorBypassReason,
) -> ActuatorApplication {
    ActuatorApplication {
        requested_lanes: inputs.requested_lanes,
        authority_scaled_lanes: inputs.requested_lanes,
        effective_lanes: command.outputs,
        lane_count: command.count,
        requested_armed: inputs.requested_armed,
        kernel_fallback_mask: inputs.kernel_fallback_mask,
        eligibility: ActuatorEligibility::Bypass(reason),
        prime: false,
        interval_epoch: None,
        interval_index: None,
        interval_position: None,
        interval_identity: None,
        selected_hold: false,
        applied_hold: false,
        interval_complete: false,
    }
}

fn eligible_application(
    command: &SimActuatorCmd,
    inputs: ApplicationInputs,
) -> ActuatorApplication {
    ActuatorApplication {
        requested_lanes: inputs.requested_lanes,
        authority_scaled_lanes: inputs.authority_scaled_lanes,
        effective_lanes: command.outputs,
        lane_count: command.count,
        requested_armed: inputs.requested_armed,
        kernel_fallback_mask: inputs.kernel_fallback_mask,
        eligibility: ActuatorEligibility::Eligible,
        prime: false,
        interval_epoch: None,
        interval_index: None,
        interval_position: None,
        interval_identity: None,
        selected_hold: false,
        applied_hold: false,
        interval_complete: false,
    }
}

fn prime_application(command: &SimActuatorCmd, inputs: ApplicationInputs) -> ActuatorApplication {
    ActuatorApplication {
        prime: true,
        ..eligible_application(command, inputs)
    }
}

fn interval_application(
    identity: PerturbationIdentity,
    command: &SimActuatorCmd,
    inputs: ApplicationInputs,
    state: &HoldState,
    decision: HoldDecision,
) -> Result<ActuatorApplication, PerturbationError> {
    let first_sequence = state
        .interval_first_sequence
        .ok_or(PerturbationError::InvalidCommandHold)?;
    Ok(ActuatorApplication {
        requested_lanes: inputs.requested_lanes,
        authority_scaled_lanes: inputs.authority_scaled_lanes,
        effective_lanes: command.outputs,
        lane_count: command.count,
        requested_armed: inputs.requested_armed,
        kernel_fallback_mask: inputs.kernel_fallback_mask,
        eligibility: ActuatorEligibility::Eligible,
        prime: false,
        interval_epoch: Some(state.interval_epoch),
        interval_index: Some(state.interval_index),
        interval_position: Some(state.interval_position),
        interval_identity: Some(interval_digest(
            identity,
            state.interval_epoch,
            state.interval_index,
            first_sequence,
        )),
        selected_hold: decision.selected,
        applied_hold: decision.applied,
        interval_complete: decision.complete,
    })
}
