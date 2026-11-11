//! Total-energy control of airspeed and height.
//!
//! Throttle changes the total energy of the vehicle and pitch moves
//! energy between height and speed. The throttle loop therefore tracks
//! the demanded rate of total energy, and the pitch loop tracks the
//! demanded rate of the difference between height energy and speed
//! energy. Both rates are expressed as a climb rate in m/s: the rate of
//! speed energy is `V * dV/dt / g`.

use crate::control::law_invariants::STANDARD_GRAVITY_MPS2;
use crate::kernel::config::FixedWingParams;
use crate::state::StateEstimate;
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::air_data::Airspeed;
use super::targets::{Envelope, Vertical};

/// Memory of the energy loops.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnergyLoopState {
    /// Throttle that the total-energy integrator holds.
    pub throttle_integrator: Scalar,
    /// Pitch \[rad\] that the energy-balance integrator holds.
    pub pitch_integrator: Scalar,
    /// Whether the airspeed fell below the minimum and has not recovered.
    pub underspeed: bool,
}

/// Inputs of one energy-loop cycle.
pub(super) struct EnergyInputs<'a> {
    pub vertical: Vertical,
    /// Indicated airspeed to hold \[m/s\].
    pub airspeed_target: Scalar,
    /// Throttle for a target that does not command it.
    pub throttle: Scalar,
    pub state: &'a StateEstimate,
    pub air: &'a Airspeed,
    pub params: &'a FixedWingParams,
    pub envelope: &'a Envelope,
    /// Height below the held pressure level \[m\] and climb rate \[m/s\]
    /// from the static port of the probe, when a level is held.
    pub pressure_hold: Option<(Scalar, Scalar)>,
    /// Cycle interval \[s\].
    pub dt: Scalar,
}

/// Pitch \[rad\] and throttle that the energy loops command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct EnergyCommand {
    pub pitch: Scalar,
    pub throttle: Scalar,
}

/// Demanded and measured rates of speed energy \[m/s\].
struct SpeedChannel {
    demand: Scalar,
    actual: Scalar,
}

/// Add `rate * dt` to `integrator` unless the output is against its
/// limit and the error pushes further into it.
fn integrate(integrator: &mut Scalar, rate: Scalar, dt: Scalar, overshoot: Scalar, bound: Scalar) {
    if overshoot * rate <= 0.0 {
        *integrator = (*integrator + rate * dt).clamp(-bound, bound);
    }
}

impl EnergyLoopState {
    /// Latch an airspeed below the minimum.
    ///
    /// The latch gives pitch to the airspeed and opens the throttle. It
    /// releases at the recovery airspeed, or above the minimum once the
    /// vehicle climbs as fast as the command asks: the energy that was
    /// missing is then back. Without a measurement, or without a height
    /// target to give up, there is no protection to latch.
    fn update_underspeed(
        &mut self,
        air: &Airspeed,
        minimum: Scalar,
        recovery: Scalar,
        climb: Option<(Scalar, Scalar)>,
    ) {
        let climb_met = climb.is_some_and(|(demand, rate)| rate >= demand);
        if !air.is_measured()
            || climb.is_none()
            || air.indicated >= recovery
            || (air.indicated >= minimum && climb_met)
        {
            self.underspeed = false;
        } else if air.indicated < minimum {
            self.underspeed = true;
        }
    }
}

fn speed_channel(inputs: &EnergyInputs<'_>, target: Scalar, speed: Scalar) -> SpeedChannel {
    let air = inputs.air;
    if !air.is_measured() {
        // Without a measurement the law does not close a speed loop.
        return SpeedChannel {
            demand: 0.0,
            actual: 0.0,
        };
    }
    // The error is taken in indicated airspeed, the quantity that the
    // wing and the limits are defined in. Its ratio to true airspeed
    // acts on the loop as a gain near one.
    let error = target - air.indicated;
    let acceleration = error / inputs.params.speed_time_constant.0;
    SpeedChannel {
        demand: (speed * acceleration / STANDARD_GRAVITY_MPS2)
            .clamp(-inputs.envelope.sink_limit, inputs.envelope.climb_limit),
        actual: speed * air.rate / STANDARD_GRAVITY_MPS2,
    }
}

/// Throttle for a demanded total-energy rate in steady flight.
fn throttle_feedforward(params: &FixedWingParams, energy_rate: Scalar) -> Scalar {
    let cruise = params.cruise_throttle.0;
    if energy_rate >= 0.0 {
        cruise + energy_rate / params.max_climb_rate.0 * (params.max_throttle.0 - cruise)
    } else {
        cruise + energy_rate / params.max_sink_rate.0 * (cruise - params.min_throttle.0)
    }
}

fn throttle_loop(
    memory: &mut EnergyLoopState,
    inputs: &EnergyInputs<'_>,
    climb: Option<(Scalar, Scalar)>,
    speed: &SpeedChannel,
) -> Scalar {
    let params = inputs.params;
    let (low, high) = (params.min_throttle.0, params.max_throttle.0);
    if memory.underspeed {
        return high;
    }
    let Some((climb_demand, climb_rate)) = climb else {
        // Pitch holds the airspeed and nothing measures the height: the
        // throttle stays where level flight needs it.
        memory.throttle_integrator = 0.0;
        return params.cruise_throttle.0;
    };
    let demand = climb_demand + speed.demand;
    let error = demand - (climb_rate + speed.actual);
    let unlimited = throttle_feedforward(params, demand)
        + params.energy_rate_p * error
        + memory.throttle_integrator;
    let throttle = unlimited.clamp(low, high);
    integrate(
        &mut memory.throttle_integrator,
        params.energy_rate_i * error,
        inputs.dt,
        unlimited - throttle,
        high - low,
    );
    throttle
}

fn pitch_loop(
    memory: &mut EnergyLoopState,
    inputs: &EnergyInputs<'_>,
    climb: Option<(Scalar, Scalar)>,
    speed_channel: &SpeedChannel,
    speed: Scalar,
) -> Scalar {
    let params = inputs.params;
    let limit = inputs.envelope.pitch_limit;
    // Below the minimum airspeed pitch serves the airspeed alone.
    let (climb_demand, climb_rate) = climb.filter(|_| !memory.underspeed).unwrap_or((0.0, 0.0));
    let error =
        ((climb_demand - speed_channel.demand) - (climb_rate - speed_channel.actual)) / speed;
    let path_angle = (climb_demand / speed).clamp(-1.0, 1.0).asin();
    let unlimited =
        params.trim_pitch.0 + path_angle + params.balance_rate_p * error + memory.pitch_integrator;
    let pitch = unlimited.clamp(-limit, limit);
    integrate(
        &mut memory.pitch_integrator,
        params.balance_rate_i * error,
        inputs.dt,
        unlimited - pitch,
        limit,
    );
    pitch
}

/// Run one cycle of the energy loops.
pub(super) fn step(memory: &mut EnergyLoopState, inputs: &EnergyInputs<'_>) -> EnergyCommand {
    let (params, envelope) = (inputs.params, inputs.envelope);
    let [_north_rate, _east_rate, down_rate] = inputs.state.velocity_ned;
    let [_north, _east, down] = inputs.state.position_ned;
    let limit_climb = |rate: Scalar| rate.clamp(-envelope.sink_limit, envelope.climb_limit);
    // Demanded and measured climb rate, when the target holds a height.
    let climb = match inputs.vertical {
        Vertical::Pitch(pitch) => {
            // A direct pitch command leaves no energy error to remember.
            *memory = EnergyLoopState::default();
            return EnergyCommand {
                pitch: pitch.clamp(-envelope.pitch_limit, envelope.pitch_limit),
                throttle: inputs.throttle.clamp(0.0, 1.0),
            };
        }
        Vertical::AirspeedOnPitch => None,
        Vertical::ClimbRate(rate) => Some((limit_climb(rate), -down_rate.0)),
        Vertical::Height(height) => Some((
            limit_climb((height + down.0) / params.height_time_constant.0),
            -down_rate.0,
        )),
    };

    let minimum = envelope.min_airspeed;
    let recovery = minimum.max(0.5 * (minimum + params.cruise_airspeed.0));
    memory.update_underspeed(inputs.air, minimum, recovery, climb);
    let target = if memory.underspeed {
        minimum
    } else {
        inputs.airspeed_target.clamp(minimum, envelope.max_airspeed)
    };

    let speed = inputs.air.divisor(params);
    let speed_channel = speed_channel(inputs, target, speed);
    // With the airspeed on pitch, the throttle holds the pressure level
    // of the probe: that needs no position and no velocity estimate.
    let throttle_climb = climb.or(inputs.pressure_hold.map(|(below, climb_rate)| {
        (
            limit_climb(below / params.height_time_constant.0),
            climb_rate,
        )
    }));
    EnergyCommand {
        pitch: pitch_loop(memory, inputs, climb, &speed_channel, speed),
        throttle: throttle_loop(memory, inputs, throttle_climb, &speed_channel),
    }
}
