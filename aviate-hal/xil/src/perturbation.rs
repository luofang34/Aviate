//! Deterministic, simulator-neutral calibration perturbations.

mod actuator;
mod artifact;
mod sensor;

pub use actuator::{
    ActuatorApplication, ActuatorBypassReason, ActuatorEligibility, ActuatorPerturbation,
    CommandHoldPerturbation,
};
pub use artifact::{
    ArtifactError, LiveArtifactGuard, LoadedPerturbationArtifact, PerturbationArtifactIdentity,
    PerturbationCapability,
};
pub use sensor::{SensorApplication, SensorLane, SensorNoise};

use core::fmt;

use crate::sim_types::{SimActuatorCmd, SimSensorPacket};
use actuator::NOMINAL_BASIS_POINTS;

/// Stable inputs that separate one perturbation run from all other runs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PerturbationIdentity {
    /// SHA-256 of the canonical condition artifact.
    pub condition_digest: [u8; 32],
    /// Run seed supplied by the harness.
    pub run_seed: u64,
}

/// Validated factors for one calibration run.
#[derive(Clone, Debug, PartialEq)]
pub struct PerturbationConfig {
    /// Stable run identity.
    pub identity: PerturbationIdentity,
    /// Sensor noise requests in physical SI units.
    pub sensor_noise: Vec<SensorNoise>,
    /// Actuator authority and hold requests.
    pub actuator: ActuatorPerturbation,
}

impl PerturbationConfig {
    /// The exact capability set this run executes.
    ///
    /// The hover-force scale is applied at kernel construction, so the caller
    /// supplies the value the kernel was built with. The order matches the
    /// condition capability order, so the result compares directly with the
    /// required set in an artifact identity.
    #[must_use]
    pub fn executed_capabilities(
        &self,
        hover_scale_basis_points: u16,
    ) -> Vec<PerturbationCapability> {
        let mut executed = Vec::new();
        if !self.sensor_noise.is_empty() {
            executed.push(PerturbationCapability::SensorPerturbation);
        }
        if self.actuator.authority_scale_basis_points != NOMINAL_BASIS_POINTS {
            executed.push(PerturbationCapability::ActuatorAuthority);
        }
        if self.actuator.command_hold.is_some() {
            executed.push(PerturbationCapability::CommandHold);
        }
        if hover_scale_basis_points != NOMINAL_BASIS_POINTS {
            executed.push(PerturbationCapability::HoverTrimUncertainty);
        }
        executed
    }
}

/// A deterministic perturbation refusal.
#[derive(Clone, Debug, PartialEq)]
pub enum PerturbationError {
    /// The condition digest is the all-zero sentinel.
    ZeroConditionDigest,
    /// Two requests select the same sensor lane.
    DuplicateSensorLane(SensorLane),
    /// A sensor amplitude is not finite or is outside its physical bound.
    InvalidSensorAmplitude(SensorLane),
    /// A sensor update interval is outside the contract range.
    InvalidSensorInterval(SensorLane),
    /// An actuator authority scale is outside the contract range.
    InvalidAuthorityScale(u16),
    /// A command-hold request is not exact or is outside its bounds.
    InvalidCommandHold,
    /// A sample sequence has a gap, duplicate, or rewind.
    SampleSequence {
        /// The next valid global sample sequence.
        expected: u64,
        /// The received global sample sequence.
        received: u64,
    },
    /// The global sample sequence is at its maximum value.
    SampleSequenceExhausted,
    /// An actuator decision does not match the current sensor sample.
    ActuatorSequence {
        /// The current sensor sample sequence.
        current: u64,
        /// The received actuator sample sequence.
        received: u64,
    },
    /// One sample received more than one actuator decision.
    DuplicateActuatorDecision(u64),
    /// One sample received more than one send completion.
    DuplicateActuatorSend(u64),
    /// A new sensor sample arrived before the prior reply completed.
    MissingActuatorSend(u64),
    /// The only actuator send for a sample failed.
    ActuatorSendFailed(u64),
    /// An eligible actuator command has an invalid lane count.
    InvalidActuatorCount(u8),
    /// A sensor value is not finite.
    NonFiniteSensor(SensorLane),
    /// A sensor presence bit does not match its decoded value.
    SensorPresenceMismatch(&'static str),
    /// An earlier contract failure quarantined this engine.
    Quarantined,
}

impl fmt::Display for PerturbationError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ZeroConditionDigest => formatter.write_str("condition digest is zero"),
            Self::DuplicateSensorLane(lane) => write!(formatter, "sensor lane {lane:?} repeats"),
            Self::InvalidSensorAmplitude(lane) => {
                write!(formatter, "sensor lane {lane:?} has an invalid amplitude")
            }
            Self::InvalidSensorInterval(lane) => {
                write!(
                    formatter,
                    "sensor lane {lane:?} has an invalid update interval"
                )
            }
            Self::InvalidAuthorityScale(value) => {
                write!(formatter, "actuator authority scale {value} is invalid")
            }
            Self::InvalidCommandHold => formatter.write_str("command hold is invalid"),
            Self::SampleSequence { expected, received } => write!(
                formatter,
                "sample sequence {received} does not match expected sequence {expected}"
            ),
            Self::SampleSequenceExhausted => {
                formatter.write_str("global sample sequence is exhausted")
            }
            Self::ActuatorSequence { current, received } => write!(
                formatter,
                "actuator sequence {received} does not match sensor sequence {current}"
            ),
            Self::DuplicateActuatorDecision(sequence) => {
                write!(formatter, "sample {sequence} has two actuator decisions")
            }
            Self::DuplicateActuatorSend(sequence) => {
                write!(formatter, "sample {sequence} has two actuator send results")
            }
            Self::MissingActuatorSend(sequence) => {
                write!(
                    formatter,
                    "sample {sequence} has no completed actuator send"
                )
            }
            Self::ActuatorSendFailed(sequence) => {
                write!(formatter, "sample {sequence} actuator send failed")
            }
            Self::InvalidActuatorCount(count) => {
                write!(formatter, "actuator lane count {count} is invalid")
            }
            Self::NonFiniteSensor(lane) => {
                write!(formatter, "sensor lane {lane:?} is not finite")
            }
            Self::SensorPresenceMismatch(group) => {
                write!(formatter, "sensor presence does not match {group} values")
            }
            Self::Quarantined => formatter.write_str("perturbation engine is quarantined"),
        }
    }
}

impl std::error::Error for PerturbationError {}

/// Stateful executor for one identity-bound run.
pub struct PerturbationEngine {
    sensor: sensor::SensorEngine,
    actuator: actuator::ActuatorEngine,
    next_sequence: Option<u64>,
    current_sequence: Option<u64>,
    sequence_exhausted: bool,
    actuator_applied: bool,
    send_completed: bool,
    quarantined: bool,
}

impl PerturbationEngine {
    /// Validate a factor set and create one fresh executor.
    pub fn new(config: PerturbationConfig) -> Result<Self, PerturbationError> {
        validate_identity(config.identity)?;
        let sensor = sensor::SensorEngine::new(config.identity, config.sensor_noise)?;
        let actuator = actuator::ActuatorEngine::new(config.identity, config.actuator)?;
        Ok(Self {
            sensor,
            actuator,
            next_sequence: None,
            current_sequence: None,
            sequence_exhausted: false,
            actuator_applied: false,
            send_completed: false,
            quarantined: false,
        })
    }

    /// Apply sensor factors after backend conversion.
    pub fn apply_sensor(
        &mut self,
        sequence: u64,
        packet: &mut SimSensorPacket,
    ) -> Result<SensorApplication, PerturbationError> {
        self.check_sensor_sequence(sequence)?;
        match self.sensor.apply(sequence, packet) {
            Ok(application) => Ok(application),
            Err(error) => {
                self.quarantined = true;
                Err(error)
            }
        }
    }

    /// Apply actuator factors before plant constraints and transforms.
    pub fn apply_actuator(
        &mut self,
        sequence: u64,
        command: &mut SimActuatorCmd,
        eligibility: ActuatorEligibility,
        kernel_fallback_mask: u8,
    ) -> Result<ActuatorApplication, PerturbationError> {
        self.check_actuator_sequence(sequence)?;
        match self
            .actuator
            .apply(sequence, command, eligibility, kernel_fallback_mask)
        {
            Ok(application) => {
                self.actuator_applied = true;
                Ok(application)
            }
            Err(error) => {
                self.quarantined = true;
                Err(error)
            }
        }
    }

    /// Record completion of the only actuator send for the current sample.
    pub fn complete_actuator_send(
        &mut self,
        sequence: u64,
        succeeded: bool,
    ) -> Result<(), PerturbationError> {
        self.check_send_sequence(sequence)?;
        self.send_completed = true;
        if succeeded {
            Ok(())
        } else {
            self.quarantined = true;
            Err(PerturbationError::ActuatorSendFailed(sequence))
        }
    }

    fn check_sensor_sequence(&mut self, sequence: u64) -> Result<(), PerturbationError> {
        if self.quarantined {
            return Err(PerturbationError::Quarantined);
        }
        if let Some(current) = self.current_sequence {
            if !self.send_completed {
                self.quarantined = true;
                return Err(PerturbationError::MissingActuatorSend(current));
            }
        }
        if self.sequence_exhausted {
            self.quarantined = true;
            return Err(PerturbationError::SampleSequenceExhausted);
        }
        if self
            .next_sequence
            .is_some_and(|expected| sequence != expected)
        {
            self.quarantined = true;
            return Err(PerturbationError::SampleSequence {
                expected: self.next_sequence.unwrap_or(sequence),
                received: sequence,
            });
        }
        self.next_sequence = Some(sequence.wrapping_add(1));
        self.current_sequence = Some(sequence);
        self.sequence_exhausted = sequence == u64::MAX;
        self.actuator_applied = false;
        self.send_completed = false;
        Ok(())
    }

    fn check_actuator_sequence(&mut self, sequence: u64) -> Result<(), PerturbationError> {
        if self.quarantined {
            return Err(PerturbationError::Quarantined);
        }
        let current = self.current_sequence.unwrap_or(u64::MAX);
        if current != sequence {
            self.quarantined = true;
            Err(PerturbationError::ActuatorSequence {
                current,
                received: sequence,
            })
        } else if self.actuator_applied {
            self.quarantined = true;
            Err(PerturbationError::DuplicateActuatorDecision(sequence))
        } else {
            Ok(())
        }
    }

    fn check_send_sequence(&mut self, sequence: u64) -> Result<(), PerturbationError> {
        if self.quarantined {
            return Err(PerturbationError::Quarantined);
        }
        let current = self.current_sequence.unwrap_or(u64::MAX);
        if current != sequence || !self.actuator_applied {
            self.quarantined = true;
            Err(PerturbationError::ActuatorSequence {
                current,
                received: sequence,
            })
        } else if self.send_completed {
            self.quarantined = true;
            Err(PerturbationError::DuplicateActuatorSend(sequence))
        } else {
            Ok(())
        }
    }
}

fn validate_identity(identity: PerturbationIdentity) -> Result<(), PerturbationError> {
    if identity.condition_digest == [0; 32] {
        Err(PerturbationError::ZeroConditionDigest)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests;
