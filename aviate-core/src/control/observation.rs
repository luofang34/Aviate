//! Controller-only diagnostics for causal tuning attribution.

use crate::math::Quaternion;
use crate::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Scalar};

use super::{AxisCommand, ControlMode, EffectiveControlTopology};

/// The action that one control cycle applied to an integrator.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum IntegratorAction {
    /// The loop did not run and retained the stored value.
    #[default]
    FrozenInactive,
    /// The loop integrated the current error.
    Integrated,
    /// Output saturation prevented an integrating update.
    FrozenSaturation,
    /// A named controller reset cleared the integrator.
    Reset,
}

/// Exact terms and state actions for one three-axis loop.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerLoopObservation {
    /// Proportional contribution for each axis.
    pub p: [Scalar; 3],
    /// Integral contribution for each axis.
    pub i: [Scalar; 3],
    /// Derivative contribution for each axis.
    pub d: [Scalar; 3],
    /// Feedforward contribution for each axis.
    pub feedforward: [Scalar; 3],
    /// Integrator state before this cycle.
    pub integrator_before: [Scalar; 3],
    /// Integrator state after this cycle.
    pub integrator_after: [Scalar; 3],
    /// Integrator action for each axis.
    pub integrator_action: [IntegratorAction; 3],
    /// Controller-local saturation for each axis.
    pub saturated: [bool; 3],
}

/// Effective setpoints produced inside the multirotor cascade.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EffectiveSetpointObservation {
    /// Position target selected by the active outer loop.
    pub position_ned: Option<[Meters; 3]>,
    /// Velocity target supplied to the velocity loop.
    pub velocity_ned: Option<[MetersPerSecond; 3]>,
    /// Acceleration feedforward supplied to the velocity loop.
    pub acceleration_ned: [MetersPerSecondSquared; 3],
    /// Attitude target supplied to the attitude loop.
    pub attitude: Quaternion,
    /// Angular-rate target supplied to the rate loop.
    pub angular_rate: [RadiansPerSecond; 3],
    /// Collective target supplied to the mixer.
    pub collective: crate::types::NormalizedThrust,
}

/// One multirotor controller cycle without simulator truth.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultirotorControllerObservation {
    /// Effective mode stored by the preceding controller cycle.
    pub previous_mode: Option<ControlMode>,
    /// Effective mode for this controller cycle.
    pub current_mode: ControlMode,
    /// Effective topology stored by the preceding controller cycle.
    pub previous_topology: Option<EffectiveControlTopology>,
    /// Effective topology for this controller cycle.
    pub current_topology: EffectiveControlTopology,
    /// Effective cascade setpoints.
    pub setpoints: EffectiveSetpointObservation,
    /// Velocity-loop terms and integrator actions.
    pub velocity_loop: ControllerLoopObservation,
    /// Rate-loop terms and integrator actions.
    pub rate_loop: ControllerLoopObservation,
    /// Exact controller output supplied to the mixer.
    pub axis_command: AxisCommand,
}

/// Diagnostic output from one controller cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerStepObservation {
    /// Multirotor cascade diagnostics, when the active controller supplies them.
    pub multirotor: Option<MultirotorControllerObservation>,
}

impl ControllerStepObservation {
    /// Construct one multirotor diagnostic observation.
    pub const fn from_multirotor(value: MultirotorControllerObservation) -> Self {
        Self {
            multirotor: Some(value),
        }
    }
}

/// Controller output and its non-authoritative diagnostic witness.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ControllerStep {
    /// Exact controller output supplied to the mixer.
    pub axis_command: AxisCommand,
    /// Diagnostic witness that cannot become a control input.
    pub observation: ControllerStepObservation,
}
