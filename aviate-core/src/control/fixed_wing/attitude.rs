//! Attitude and body-rate loops: bank and pitch targets become aileron,
//! elevator and rudder commands.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::kernel::config::FixedWingParams;
use crate::state::StateEstimate;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::air_data::Airspeed;
use super::targets::Envelope;

/// Memory of the body-rate loops.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RateLoopState {
    /// Surface deflection that each rate integrator holds, in roll,
    /// pitch, yaw order.
    pub integrators: [Scalar; 3],
}

/// Body rates \[rad/s\] that close the attitude error in a coordinated turn.
pub(super) fn rate_demand(
    bank_target: Scalar,
    pitch_target: Scalar,
    state: &StateEstimate,
    air: &Airspeed,
    params: &FixedWingParams,
    envelope: &Envelope,
) -> [Scalar; 3] {
    let (roll, pitch, _yaw) = state.attitude.to_euler();
    let speed = air.divisor(params);
    let roll_rate = (bank_target - roll) / params.roll_time_constant.0;
    let pitch_rate = (pitch_target - pitch) / params.pitch_time_constant.0;
    // Heading rate of a coordinated turn at the present bank. The bank is
    // bounded first: the tangent has no bound near knife edge.
    let turning_bank = roll.clamp(-envelope.bank_limit, envelope.bank_limit);
    let turn_rate = STANDARD_GRAVITY_MPS2 * libm::tanf(turning_bank) / speed;

    let (sin_roll, cos_roll) = (roll.sin(), roll.cos());
    let (sin_pitch, cos_pitch) = (pitch.sin(), pitch.cos());
    let body_pitch_rate = pitch_rate * cos_roll + turn_rate * cos_pitch * sin_roll;
    // The load factor is the lift that holds the present attitude plus
    // the lift that the pitch rate adds: cos(roll)*cos(pitch) + V*q/g.
    let level_lift = cos_roll * cos_pitch;
    let rate_per_load = STANDARD_GRAVITY_MPS2 / speed;
    // The envelope gives a minimum that is not above the maximum, so the
    // range below cannot invert.
    let lowest = (envelope.min_load_factor - level_lift) * rate_per_load;
    let highest = (envelope.max_load_factor - level_lift) * rate_per_load;
    [
        roll_rate - turn_rate * sin_pitch,
        body_pitch_rate.clamp(lowest, highest),
        turn_rate * cos_pitch * cos_roll - pitch_rate * sin_roll,
    ]
}

/// Bound each body-rate demand by its envelope limit.
pub(super) fn limit_rates(demand: [Scalar; 3], envelope: &Envelope) -> [Scalar; 3] {
    let [roll, pitch, yaw] = demand;
    let [roll_limit, pitch_limit, yaw_limit] = envelope.rate_limits;
    [
        roll.clamp(-roll_limit, roll_limit),
        pitch.clamp(-pitch_limit, pitch_limit),
        yaw.clamp(-yaw_limit, yaw_limit),
    ]
}

/// Gains and conditions that all three rate axes share in one cycle.
struct AxisContext {
    /// Gain scale for the present airspeed.
    scale: Scalar,
    /// Whether the integrators run.
    integrating: bool,
    integrator_limit: Scalar,
    dt: Scalar,
}

fn axis(
    integrator: &mut Scalar,
    demand: Scalar,
    rate: Scalar,
    gains: (Scalar, Scalar, Scalar),
    context: &AxisContext,
) -> Scalar {
    let (proportional, integral, feedforward) = gains;
    let error = demand - rate;
    // Surface moment grows with the square of airspeed and the rate that
    // a deflection holds grows with airspeed: feedback scales twice and
    // feedforward once.
    let unlimited = feedforward * demand * context.scale
        + (proportional * error + *integrator) * context.scale * context.scale;
    let output = unlimited.clamp(-1.0, 1.0);
    if context.integrating && (unlimited - output) * error <= 0.0 {
        *integrator = (*integrator + integral * error * context.dt)
            .clamp(-context.integrator_limit, context.integrator_limit);
    }
    output
}

/// Surface commands in `[-1, 1]` for the body-rate demand.
pub(super) fn rate_loop(
    memory: &mut RateLoopState,
    demand: [Scalar; 3],
    state: &StateEstimate,
    air: &Airspeed,
    params: &FixedWingParams,
    dt: Scalar,
) -> [Scalar; 3] {
    let stall = params.stall_airspeed.0;
    let context = AxisContext {
        scale: (params.cruise_airspeed.0 / air.indicated.max(stall))
            .clamp(params.airspeed_scale_min, params.airspeed_scale_max),
        // Below the stall speed the vehicle is on the ground or not
        // flying: an integrator would only store the error of that state.
        integrating: air.indicated >= stall,
        integrator_limit: params.rate_integrator_limit,
        dt,
    };
    let [roll_demand, pitch_demand, yaw_demand] = demand;
    let [roll_rate, pitch_rate, yaw_rate] = state.angular_velocity;
    let [roll_p, pitch_p, yaw_p] = params.rate_p;
    let [roll_i, pitch_i, yaw_i] = params.rate_i;
    let [roll_ff, pitch_ff, yaw_ff] = params.rate_ff;
    let [roll_memory, pitch_memory, yaw_memory] = &mut memory.integrators;
    [
        axis(
            roll_memory,
            roll_demand,
            roll_rate.0,
            (roll_p, roll_i, roll_ff),
            &context,
        ),
        axis(
            pitch_memory,
            pitch_demand,
            pitch_rate.0,
            (pitch_p, pitch_i, pitch_ff),
            &context,
        ),
        axis(
            yaw_memory,
            yaw_demand,
            yaw_rate.0,
            (yaw_p, yaw_i, yaw_ff),
            &context,
        ),
    ]
}
