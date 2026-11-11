//! Lateral guidance: a course or heading error becomes a bank angle.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::kernel::config::FixedWingParams;
use crate::state::StateEstimate;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::air_data::Airspeed;
use super::targets::{ground_course, wrap_angle, Envelope, Lateral};

/// Largest bank angle \[rad\] that keeps the wing below its stall lift.
///
/// A level turn at bank `b` needs `1/cos(b)` times the lift of level
/// flight, and the lift available grows with the square of airspeed. At
/// the stall speed no bank is available; the limit opens as speed grows.
fn stall_bank_limit(air: &Airspeed, params: &FixedWingParams) -> Scalar {
    let ratio = params.stall_airspeed.0 / air.indicated.max(params.stall_airspeed.0);
    libm::acosf(ratio * ratio)
}

/// Bank angle \[rad\] for the lateral target, inside the bank limits.
pub(super) fn bank_command(
    lateral: Lateral,
    state: &StateEstimate,
    air: &Airspeed,
    params: &FixedWingParams,
    envelope: &Envelope,
) -> Scalar {
    let limit = envelope.bank_limit.min(stall_bank_limit(air, params));
    let (_roll, _pitch, yaw) = state.attitude.to_euler();
    let direction_error = match lateral {
        Lateral::WingsLevel => return 0.0,
        Lateral::Bank(bank) => return bank.clamp(-limit, limit),
        Lateral::Course(course) => wrap_angle(course - ground_course(state)),
        Lateral::Heading(heading) => wrap_angle(heading - yaw),
    };
    // A coordinated turn at bank b turns at g*tan(b)/V.
    let turn_rate = direction_error / params.course_time_constant.0;
    libm::atanf(turn_rate * air.divisor(params) / STANDARD_GRAVITY_MPS2).clamp(-limit, limit)
}
