//! Typed simulator backend contract.
//!
//! A backend accepts one directive at a time. It returns a receipt for
//! the same directive. Frames contain one reset generation, one
//! simulation step, and one authoritative simulation time.

use std::time::Duration;

use aviate_core::control::Command;
use thiserror::Error;

/// A reset generation for one simulator session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResetGeneration(u32);

impl ResetGeneration {
    /// The first generation in a new session.
    pub const INITIAL: Self = Self(1);

    /// Create a generation from a transport value.
    #[must_use]
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    /// Return the transport value.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0
    }

    /// Return the next generation.
    #[must_use]
    pub const fn next(self) -> Self {
        Self(self.0.wrapping_add(1))
    }
}

/// The acknowledged state of a simulator session.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SimulatorLifecycle {
    /// The backend does not advance the simulator.
    Stopped,
    /// The backend clears state for a new generation.
    Resetting,
    /// The flight controller uses fresh samples to converge.
    Converging,
    /// The backend can accept flight directives.
    Ready,
}

/// One coherent view of the backend lifecycle.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BackendStatus {
    /// The current reset generation.
    pub generation: ResetGeneration,
    /// The current lifecycle state.
    pub lifecycle: SimulatorLifecycle,
    /// The last acknowledged simulation step.
    pub step: u64,
    /// The last authoritative simulation time.
    pub simulation_time: Duration,
    /// True when the flight controller is armed.
    pub armed: bool,
}

impl Default for BackendStatus {
    fn default() -> Self {
        Self {
            generation: ResetGeneration::INITIAL,
            lifecycle: SimulatorLifecycle::Stopped,
            step: 0,
            simulation_time: Duration::ZERO,
            armed: false,
        }
    }
}

/// A caller-supplied identity for one directive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DirectiveId(pub u64);

/// One command for the simulator session.
#[derive(Clone, Debug)]
pub enum SimulatorDirectiveKind {
    /// Start sample processing.
    Start,
    /// Stop sample processing and make outputs safe.
    Stop,
    /// Start a new reset generation.
    Reset,
    /// Check that the flight controller can arm.
    CheckArmReadiness,
    /// Arm the flight controller.
    Arm,
    /// Apply one flight setpoint.
    Setpoint(Command),
    /// Disarm the flight controller.
    Disarm,
}

impl SimulatorDirectiveKind {
    /// Return a stable operation name for diagnostics.
    #[must_use]
    pub const fn operation(&self) -> SimulatorOperation {
        match self {
            Self::Start => SimulatorOperation::Start,
            Self::Stop => SimulatorOperation::Stop,
            Self::Reset => SimulatorOperation::Reset,
            Self::CheckArmReadiness => SimulatorOperation::CheckArmReadiness,
            Self::Arm => SimulatorOperation::Arm,
            Self::Setpoint(_) => SimulatorOperation::Setpoint,
            Self::Disarm => SimulatorOperation::Disarm,
        }
    }
}

/// A directive that is valid only for one reset generation.
#[derive(Clone, Debug)]
pub struct SimulatorDirective {
    /// The identity that the receipt must repeat.
    pub id: DirectiveId,
    /// The generation that the caller observed.
    pub generation: ResetGeneration,
    /// The requested operation.
    pub kind: SimulatorDirectiveKind,
}

/// The confirmed effect of a directive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DirectiveOutcome {
    /// Sample processing started.
    Started,
    /// Sample processing stopped.
    Stopped,
    /// The backend accepted a reset for a new generation.
    ResetAccepted,
    /// The flight controller can arm.
    ArmReady,
    /// The flight controller armed.
    Armed,
    /// The backend accepted the setpoint for the current generation.
    SetpointAccepted,
    /// The flight controller disarmed.
    Disarmed,
}

/// An acknowledgement for one directive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DirectiveReceipt {
    /// The identity from the directive.
    pub id: DirectiveId,
    /// The generation that contains the confirmed effect.
    pub generation: ResetGeneration,
    /// The step that contains the confirmed effect.
    pub step: u64,
    /// The authoritative simulation time for the receipt.
    pub simulation_time: Duration,
    /// The confirmed effect.
    pub outcome: DirectiveOutcome,
}

/// Backend-neutral vehicle state in the North-East-Down frame.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct VehicleState {
    /// Position in metres.
    pub position: [f32; 3],
    /// Velocity in metres per second.
    pub velocity: [f32; 3],
    /// Body-to-world quaternion in `[w, x, y, z]` order.
    pub orientation: [f32; 4],
    /// Body angular velocity in radians per second.
    pub angular_velocity: [f32; 3],
    /// True when all required fields are available.
    pub valid: bool,
}

/// One sample-paced simulator frame.
#[derive(Clone, Debug, PartialEq)]
pub struct SimulatorFrame {
    /// The reset generation for all frame fields.
    pub generation: ResetGeneration,
    /// The authoritative simulation step.
    pub step: u64,
    /// The authoritative simulation time.
    pub simulation_time: Duration,
    /// The lifecycle state for this generation.
    pub lifecycle: SimulatorLifecycle,
    /// The vehicle state for this step.
    pub vehicle: VehicleState,
    /// True when the flight controller is armed.
    pub armed: bool,
}

/// The result of one bounded frame request.
#[derive(Clone, Debug, PartialEq)]
pub enum FrameEvent {
    /// The backend supplied one frame.
    Frame(SimulatorFrame),
    /// No frame arrived before the wall-clock transport timeout.
    TimedOut {
        /// The current generation.
        generation: ResetGeneration,
        /// The last acknowledged step.
        last_step: u64,
        /// The requested timeout.
        timeout: Duration,
    },
}

/// A backend operation name.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SimulatorOperation {
    /// Connect the transport.
    Connect,
    /// Start sample processing.
    Start,
    /// Stop sample processing.
    Stop,
    /// Reset the session.
    Reset,
    /// Check arm readiness.
    CheckArmReadiness,
    /// Arm the flight controller.
    Arm,
    /// Apply a setpoint.
    Setpoint,
    /// Disarm the flight controller.
    Disarm,
    /// Read the next frame.
    NextFrame,
}

impl std::fmt::Display for SimulatorOperation {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

/// A typed simulator backend failure.
#[derive(Debug, Error)]
pub enum SimulatorError {
    /// The backend cannot connect to the simulator.
    #[error("{backend} connection failed: {detail}")]
    ConnectionFailed {
        /// Backend name.
        backend: String,
        /// Failure detail.
        detail: String,
    },
    /// The backend does not provide an operation.
    #[error("{operation} is not available: {detail}")]
    NotAvailable {
        /// Requested operation.
        operation: SimulatorOperation,
        /// Refusal detail.
        detail: String,
    },
    /// A transport operation failed.
    #[error("{operation} input/output failed: {source}")]
    Io {
        /// Failed operation.
        operation: SimulatorOperation,
        /// Source error.
        #[source]
        source: std::io::Error,
    },
    /// A directive did not complete before its timeout.
    #[error("{operation} timed out in generation {generation:?} after {timeout:?}")]
    Timeout {
        /// Timed-out operation.
        operation: SimulatorOperation,
        /// Active generation.
        generation: ResetGeneration,
        /// Requested timeout.
        timeout: Duration,
    },
    /// The sample-paced bridge disconnected.
    #[error("bridge lost in generation {generation:?} after step {last_step}")]
    BridgeLost {
        /// Active generation.
        generation: ResetGeneration,
        /// Last acknowledged step.
        last_step: u64,
    },
    /// The backend cannot reset the simulator state.
    #[error("reset failed in generation {generation:?}: {detail}")]
    ResetFailed {
        /// Active generation.
        generation: ResetGeneration,
        /// Failure detail.
        detail: String,
    },
    /// The flight controller did not become ready.
    #[error("readiness failed in generation {generation:?}: {detail}")]
    ReadinessFailed {
        /// Active generation.
        generation: ResetGeneration,
        /// Failure detail.
        detail: String,
    },
    /// The flight controller refused to arm.
    #[error("arm refused in generation {generation:?}: {detail}")]
    ArmRefused {
        /// Active generation.
        generation: ResetGeneration,
        /// Failure detail.
        detail: String,
    },
    /// A directive uses an inactive generation.
    #[error("directive generation {received:?} does not match {expected:?}")]
    StaleGeneration {
        /// Active generation.
        expected: ResetGeneration,
        /// Directive generation.
        received: ResetGeneration,
    },
    /// The lifecycle does not permit an operation.
    #[error("{operation} is invalid while the backend is {lifecycle:?}")]
    InvalidLifecycle {
        /// Requested operation.
        operation: SimulatorOperation,
        /// Current lifecycle state.
        lifecycle: SimulatorLifecycle,
    },
}

/// A sample-paced simulator backend.
pub trait SimulatorBackend: Send {
    /// Return the backend name.
    fn name(&self) -> &str;

    /// Connect one simulator instance and return its status.
    fn connect(&mut self, instance: u8, timeout: Duration)
        -> Result<BackendStatus, SimulatorError>;

    /// Return one coherent backend status.
    fn status(&self) -> BackendStatus;

    /// Execute one directive and return its acknowledgement.
    fn execute(
        &mut self,
        directive: SimulatorDirective,
        timeout: Duration,
    ) -> Result<DirectiveReceipt, SimulatorError>;

    /// Request the next sample-paced frame.
    fn next_frame(&mut self, timeout: Duration) -> Result<FrameEvent, SimulatorError>;

    /// Return the connected instance.
    fn instance(&self) -> u8;
}

#[cfg(test)]
mod tests {
    use super::ResetGeneration;

    #[test]
    fn reset_generation_wraps() {
        assert_eq!(ResetGeneration::new(u32::MAX).next().get(), 0);
    }
}
