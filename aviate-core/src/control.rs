//! Flight-control surface: enums, setpoints, commands, limits, and the
//! `VehicleController` trait the kernel drives every cycle.
//!
//! Center-coded enums (ControlMode / ControlLawV1 / SafetyLevelV1 /
//! CommandSource / ConfigMode) live in the [`enums`] submodule to keep
//! this file under the 500-line cap; everything else (Setpoint, Command,
//! Limits, AxisCommand, VehicleController, …) stays here.

use crate::math::Quaternion;
use crate::sensor::GnssHealth;
use crate::state::StateEstimate;
use crate::types::{
    Meters, MetersPerSecond, Normalized, NormalizedSigned, Radians, RadiansPerSecond,
};

// Re-exporting for submodules
pub use crate::types::Scalar;

pub mod enums;
pub mod runtime;
pub use enums::{CommandSource, ConfigMode, ControlLawV1, ControlMode, SafetyLevelV1};

#[derive(Clone, Debug)]
pub struct Setpoint {
    pub attitude: Option<Quaternion>,
    pub angular_rate: Option<[RadiansPerSecond; 3]>,
    pub altitude: Option<Meters>,
    pub vertical_speed: Option<MetersPerSecond>,
    pub heading: Option<Radians>,
    pub position: Option<[Meters; 3]>,
    pub velocity: Option<[MetersPerSecond; 3]>,
    pub lateral_deviation: Option<Meters>,
    pub vertical_deviation: Option<Meters>,
    pub collective_thrust: Normalized,
}

impl Default for Setpoint {
    fn default() -> Self {
        Self {
            attitude: None,
            angular_rate: None,
            altitude: None,
            vertical_speed: None,
            heading: None,
            position: None,
            velocity: None,
            lateral_deviation: None,
            vertical_deviation: None,
            collective_thrust: Normalized(0.0),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Timestamp {
    pub ticks: u64,
    pub source: TimeSource,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeSource {
    Internal,
    Gps,
    Ptp,
}

#[derive(Copy, Clone, Debug)]
pub struct SensorOverrides {
    pub gnss_force_state: Option<GnssHealth>, // None = no override
}

#[derive(Clone, Debug)]
pub struct Command {
    pub mode: ControlMode,
    pub setpoint: Setpoint,
    pub config_mode_request: Option<ConfigMode>,
    pub sensor_overrides: Option<SensorOverrides>,
    // pub timestamp: Timestamp, // Removed for now as Timestamp is not fully defined in this file context
    pub sequence: u32,
    pub source: CommandSource,
}

impl Command {
    /// Validate all enum fields for SEU resilience (Spec §15.3)
    ///
    /// Checks that enum discriminants are within valid ranges.
    /// Returns true if all fields are valid, false if any corruption detected.
    /// This is a fast O(1) operation that checks discriminant values.
    #[inline]
    pub fn validate_enums(&self) -> bool {
        // COV:EXCL_START(DEFENSIVE: SEU/memory corruption detection - cannot trigger in unit tests)
        // Check ControlMode discriminant (0-5)
        if !self.mode.is_valid_discriminant() {
            return false;
        }

        // Check CommandSource discriminant (0-3)
        if !self.source.is_valid_discriminant() {
            return false;
        }

        // Check ConfigMode if present (0-3)
        if let Some(config_mode) = self.config_mode_request {
            if !config_mode.is_valid_discriminant() {
                return false;
            }
        }
        // COV:EXCL_STOP

        true
    }
}

#[derive(Clone, Debug)]
pub struct Limits {
    pub max_roll: Radians,
    pub max_pitch: Radians,
    pub max_roll_rate: RadiansPerSecond,
    pub max_pitch_rate: RadiansPerSecond,
    pub max_yaw_rate: RadiansPerSecond,
    pub max_horizontal_speed: MetersPerSecond,
    pub max_climb_rate: MetersPerSecond,
    pub max_descent_rate: MetersPerSecond,
    pub max_altitude: Meters,
    pub min_altitude: Meters,
    pub min_airspeed: Option<MetersPerSecond>,
    pub max_airspeed: Option<MetersPerSecond>,
    pub max_load_factor: Scalar,
    pub min_load_factor: Scalar,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AuthorityProfile {
    HardEnvelope,
    SoftEnvelope,
}

// COV:EXCL_START(phantom DA from enums.rs re-export; LawProfile decl
// has no executable code under grcov instrumentation)
#[derive(Clone, Debug)]
pub struct LawProfile {
    pub authority: AuthorityProfile,
    pub chain: &'static [ControlLawV1],
    // capabilities...
}
// COV:EXCL_STOP

#[derive(Clone, Debug)]
pub struct AxisCommand {
    pub roll: NormalizedSigned,
    pub pitch: NormalizedSigned,
    pub yaw: NormalizedSigned,
    pub collective: Normalized,
}

// COV:EXCL_START(phantom DA: rustc's coverage attribution places
// phantom DA entries on the VehicleController trait's doc comment
// — same artifact class as the kernel_trait.rs DELEGATE block and
// mixer.rs Sanitizer declaration. No executable code on these lines.)
/// Vehicle-level controller — maps a state estimate + command into
/// an `AxisCommand` (roll/pitch/yaw/collective normalized control
/// inputs).
///
/// **Algorithm/state split (LLR-CTL-102)**: every implementor exposes
/// two halves:
///
/// - `&self` — algorithm identity and tuning gains (e.g. P-gain
///   arrays, sub-controller objects). Read-only during the flight
///   loop; mutated only at construction. Lives inside
///   `KernelPipeline`.
/// - `&mut Self::RuntimeState` — persistent runtime state
///   (integrators, anti-windup, filter memories, mode latches,
///   transition-blend state). Lives inside `KernelState.control` so
///   the kernel's "exactly one safety-relevant-state owner" rule
///   covers controller state too — making it amenable to the same
///   snapshot / hash / vote / hot-spare-takeover machinery as
///   `EkfState` and `ActuatorFallbackState`.
///
/// Today's gains-only placeholders set `type RuntimeState =
/// NoControllerState` (a zero-size unit-struct); a controller that
/// grows persistent state swaps in its own
/// `ControllerRuntimeState`-implementing struct without a second
/// trait refactor.
///
/// `reset` returns the runtime state to its baseline; the default
/// implementation delegates to `runtime.reset()`. Override only if
/// the controller needs to reset additional state beyond the
/// runtime struct itself (rare).
pub trait VehicleController {
    /// Persistent runtime state owned by `KernelState.control`.
    type RuntimeState: runtime::ControllerRuntimeState;

    /// 64-bit algorithm-identity constant, fixed at the impl site.
    /// See `Estimator::ALGORITHM_ID` for the contract — same scope
    /// (controller-class identity) and same lockstep gating role.
    const ALGORITHM_ID: u64;

    fn step(
        &self,
        runtime: &mut Self::RuntimeState,
        state: &StateEstimate,
        command: &Command,
        mode: ConfigMode,
        limits: &Limits, // COV:EXCL(phantom DA from enums.rs re-export; param decl)
    ) -> AxisCommand; // COV:EXCL(phantom DA from enums.rs re-export; return type)

    /// Return controller runtime state to its post-power-on baseline.
    /// Default impl delegates to `runtime.reset()`. The kernel calls
    /// this from `ground_reset` and on `disarm`.
    fn reset(&self, runtime: &mut Self::RuntimeState) {
        <Self::RuntimeState as runtime::ControllerRuntimeState>::reset(runtime);
    }
}
// COV:EXCL_STOP // COV:EXCL(phantom DA from enums.rs re-export; real line has no code)
// COV:EXCL_START(phantom DA from enums.rs re-export: these mod decls carry
//   coverage attributions from enums.rs items re-exported via `pub use`.
//   Includes this COV:EXCL_START line and the blank line separation.)
pub mod attitude;
pub mod envelope;
pub mod position;
pub mod rate;
pub mod velocity;
// COV:EXCL_STOP

#[cfg(feature = "mc")]
pub mod multirotor;

#[cfg(feature = "fw")]
pub mod fixed_wing;

#[cfg(feature = "vtol")]
pub mod vtol;
