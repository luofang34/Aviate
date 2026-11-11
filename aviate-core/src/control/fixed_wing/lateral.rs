//! Lateral guidance: a course or heading error becomes a bank angle.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::kernel::config::FixedWingParams;
use crate::state::StateEstimate;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::air_data::Airspeed;
use super::targets::{ground_course, wrap_angle, Envelope, Lateral};

/// Memory of the lateral guidance loop.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LateralLoopState {
    /// Turn rate \[rad/s\] that the course integrator holds. It is the
    /// bank that straight flight needs on a vehicle that is not
    /// symmetric: propeller effects and trim.
    pub turn_rate_integrator: Scalar,
}

/// Largest bank angle \[rad\] that keeps the wing below its stall lift.
///
/// A level turn at bank `b` needs `1/cos(b)` times the lift of level
/// flight, and the lift available grows with the square of airspeed. At
/// the stall speed no bank is available; the limit opens as speed grows.
fn stall_bank_limit(air: &Airspeed, params: &FixedWingParams) -> Scalar {
    let ratio = params.stall_airspeed.0 / air.indicated.max(params.stall_airspeed.0);
    libm::acosf(ratio * ratio)
}

/// Inputs of one lateral guidance cycle.
pub(super) struct LateralInputs<'a> {
    pub lateral: Lateral,
    pub state: &'a StateEstimate,
    pub air: &'a Airspeed,
    pub params: &'a FixedWingParams,
    pub envelope: &'a Envelope,
    /// Cycle interval \[s\].
    pub dt: Scalar,
}

/// Bank angle \[rad\] for the lateral target, inside the bank limits.
pub(super) fn bank_command(memory: &mut LateralLoopState, inputs: &LateralInputs<'_>) -> Scalar {
    let params = inputs.params;
    let envelope_limit = inputs
        .envelope
        .bank_limit
        .min(stall_bank_limit(inputs.air, params));
    let (_roll, _pitch, yaw) = inputs.state.attitude.to_euler();
    let direction_error = match inputs.lateral {
        Lateral::WingsLevel => {
            *memory = LateralLoopState::default();
            return 0.0;
        }
        Lateral::Bank(bank) => {
            *memory = LateralLoopState::default();
            return bank.clamp(-envelope_limit, envelope_limit);
        }
        Lateral::Course(course) => wrap_angle(course - ground_course(inputs.state)),
        Lateral::Heading(heading) => wrap_angle(heading - yaw),
    };
    // A turn that guidance asks for uses the route bank; the envelope
    // limit above it is for a direct bank command and for protection.
    let limit = envelope_limit.min(params.route_bank_angle.0);
    // A coordinated turn at bank b turns at g*tan(b)/V.
    let rate_per_tangent = STANDARD_GRAVITY_MPS2 / inputs.air.divisor(params);
    let proportional = direction_error / params.course_time_constant.0;
    let bank_of = |turn_rate: Scalar| libm::atanf(turn_rate / rate_per_tangent);
    let unlimited = bank_of(proportional + memory.turn_rate_integrator);
    let bank = unlimited.clamp(-limit, limit);

    // The integrator trims straight flight. It runs near the target
    // only: in a turn that the proportional term commands with more than
    // half of the route bank it would store the turn, not the trim.
    let near_target = bank_of(proportional).abs() <= 0.5 * limit;
    if near_target && (unlimited - bank) * direction_error <= 0.0 {
        let bound = libm::tanf(limit) * rate_per_tangent;
        memory.turn_rate_integrator = (memory.turn_rate_integrator
            + proportional / params.course_integral_time.0 * inputs.dt)
            .clamp(-bound, bound);
    }
    bank
}
