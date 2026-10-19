//! Gazebo model state → Aviate truth, and motor commands → gz rotor speeds.
//!
//! The `AviateGzPlugin` publishes the model's pose in gz-sim's
//! conventions (ENU world, FLU body). This module converts it into the
//! board-independent [`TruthSample`] that
//! `aviate_hal_xil::synthesis::StepSynthesizer` turns into sensor
//! samples. A sign error here would produce a vehicle that flies but
//! with a subtly wrong attitude estimate the kernel cannot tell from
//! real physics, so the frame conversions are unit-tested.

use aviate_backend_gz::{enu_to_ned_f32, AviateModelState};
use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_core::types::NormalizedThrust;
use aviate_hal_xil::synthesis::TruthSample;

/// X500 rotor `maxRotVelocity` from the PX4-gazebo-models SDF
/// (`motorConstant` 8.55e-6 N/(rad/s)² × 1000² × 4 motors ≈ 34 N max
/// thrust against ≈20 N weight, so a thrust-to-weight ratio of ~1.7
/// at full motor output). Maps `Normalized([0.0, 1.0])` actuator
/// output linearly to rotor speed.
pub const MOTOR_MAX_RPS: f64 = 1000.0;

/// The truth the shared synthesizer consumes, from one plugin read.
pub fn truth(state: &AviateModelState) -> TruthSample {
    TruthSample {
        time_us: state.time_us,
        reset_generation: state.reset_generation,
        step: state.sim_step,
        position_ned_m: enu_to_ned_f32(state.pos),
        attitude: enu_quat_to_ned([
            state.quat[0] as f32,
            state.quat[1] as f32,
            state.quat[2] as f32,
            state.quat[3] as f32,
        ]),
        // The gz plugin states velocity and angular velocity, but the
        // synthesizer derives both from successive poses so the IMU has
        // the shape of a real one whatever the plugin publishes.
        measured: None,
    }
}

/// Convert an attitude quaternion produced by gz-sim (ENU world,
/// FLU body: forward-left-up) to Aviate's convention (NED world,
/// FRD body: forward-right-down). Both encode the rotation from
/// body to world; world AND body frames differ.
///
/// Two basis swaps are needed:
///
/// * **World ENU → NED**: change-of-basis `[[0,1,0],[1,0,0],
///   [0,0,-1]]` — det +1, a 180° rotation about `(1,1,0)/√2`,
///   rotor `q_ENU→NED = (0, 1/√2, 1/√2, 0)`.
/// * **Body FRD → FLU**: 180° rotation about the forward (X)
///   axis (negates Y and Z), rotor `q_FRD→FLU = (0, 1, 0, 0)`.
///
/// Composition for the same physical attitude:
///   `q_NED_FRD = q_ENU→NED · q_ENU_FLU · q_FRD→FLU`
///
/// Expanding the Hamilton product (`s = 1/√2`):
/// ```text
///   w_out = s·(w_in + z_in)
///   x_out = s·(x_in + y_in)
///   y_out = s·(x_in − y_in)
///   z_out = s·(w_in − z_in)
/// ```
///
/// For `q_ENU_FLU = identity` (vehicle in FLU body aligned with
/// ENU world, the gz-sim default), the result is `(s, 0, 0, s)` —
/// a 90° yaw about NED-Down, i.e. body-X points East in NED. That
/// matches the gz model's default pose (model frame X = world X =
/// East in ENU = +Y = East in NED). See `tests::enu_quat_to_ned_*`.
pub fn enu_quat_to_ned(q_enu_flu: [f32; 4]) -> [f32; 4] {
    let [w, x, y, z] = q_enu_flu;
    let s = core::f32::consts::FRAC_1_SQRT_2;
    [s * (w + z), s * (x + y), s * (x - y), s * (w - z)]
}

/// Convert a body-frame vector from gz-sim's FLU convention
/// (forward, left, up) to Aviate's FRD (forward, right, down).
/// `Y` and `Z` flip; `X` (forward) is unchanged.
///
/// Currently unused in the production synthesize path — gz's
/// `WorldAngularVelocity` returns ENU-world ang vel, so the
/// gyro pipeline goes through the full world→NED→body rotation
/// rather than a simple body-frame sign swap. Kept for the
/// rotation unit-test suite and for the day a different gz
/// component provides body-frame data directly.
#[allow(dead_code)]
pub fn flu_to_frd_body(v_flu: [f32; 3]) -> [f32; 3] {
    [v_flu[0], -v_flu[1], -v_flu[2]]
}

/// Convert a mixer output — normalized per-motor THRUST, force
/// domain — into the quad rotor angular-velocity setpoint
/// (rad/s) for the X500's `MulticopterMotorModelSystem`.
///
/// This is the one place the resolved actuator curve is applied:
/// the controller and mixer reason purely in force, and
/// `ResolvedKernelConfig::actuator_curve` describes the plant. The
/// gz rotor produces thrust `T = motorConstant · ω²` (quadratic),
/// so the X500 configuration resolves
/// `ActuatorCurveKind::QuadraticRotor` and the boundary command is
/// `√thrust`: `ω = √thrust · MAX_RPS`, making physical thrust
/// linear in the kernel's command. At the X500's force-domain
/// hover trim (20.25 N weight / 34.19 N max = 0.5929) this
/// commands `√0.5929 · MAX_RPS = 0.77 · MAX_RPS` — the rotor speed
/// at which the X500 model's total thrust equals its weight.
pub fn cmd_to_omega(curve: ActuatorCurveKind, thrust: f32) -> f64 {
    f64::from(curve.boundary_command(NormalizedThrust(thrust)).0) * MOTOR_MAX_RPS
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests;
