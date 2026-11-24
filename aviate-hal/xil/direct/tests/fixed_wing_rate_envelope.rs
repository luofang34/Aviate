//! TST-CTL-213: the C172 stays inside the body-rate limits of its
//! envelope on large attitude steps, with the autopilot engaged, and its
//! rate feedforward is the inverse of the response of the aeroplane.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust};
use aviate_hal_io::MAX_ACTUATOR_OUTPUTS;
use aviate_xil_direct::{DirectActuatorCmd, DirectNoiseTier};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, FixedWingPlant, Flight, PlantModel, DT_S};

/// Steps between two flight commands: 0.1 s.
const COMMAND_STEPS: u64 = 25;

/// What one attitude step showed.
#[derive(Debug)]
struct StepResponse {
    /// Largest body rate toward the target \[rad/s\].
    peak_rate: f32,
    /// Largest body-rate demand of the law toward the target \[rad/s\].
    peak_demand: f32,
    /// Smallest and largest load factor.
    load: (f32, f32),
    /// Time from 10 % to 90 % of the step \[s\].
    rise_time: f32,
    /// Largest angle beyond the target, as a fraction of the step.
    overshoot: f32,
    /// Largest distance from the target in the last second, as a
    /// fraction of the step.
    final_error: f32,
}

/// Body axis of a step.
#[derive(Clone, Copy)]
enum Axis {
    Roll,
    Pitch,
}

impl Axis {
    /// Angle of the plant about this axis \[rad\].
    fn angle(self, flight: &Flight) -> f32 {
        match self {
            Self::Roll => flight.plant.roll,
            Self::Pitch => flight.plant.pitch,
        }
    }

    /// Body rate of the plant about this axis \[rad/s\].
    fn rate(self, flight: &Flight) -> f32 {
        match self {
            Self::Roll => flight.plant.roll_rate,
            Self::Pitch => flight.plant.pitch_rate(),
        }
    }

    /// Body-rate demand of the law about this axis \[rad/s\].
    fn demand(self, flight: &Flight) -> f32 {
        let [roll, pitch, _yaw] = flight.board.kernel().state.controller.targets.rates;
        match self {
            Self::Roll => roll,
            Self::Pitch => pitch,
        }
    }
}

/// Fly `seconds` under `setpoint` and measure the step of the plant
/// about `axis` toward `target`.
fn step_response(
    flight: &mut Flight,
    seconds: f32,
    mode: ControlMode,
    setpoint: Setpoint,
    axis: Axis,
    target: f32,
) -> StepResponse {
    let start = axis.angle(flight);
    let step = target - start;
    let fraction = |flight: &Flight| (axis.angle(flight) - start) / step;
    let steps = (seconds / DT_S).round() as u64;
    let (mut peak_rate, mut overshoot, mut final_error) = (0.0_f32, 0.0_f32, 0.0_f32);
    let (mut at_tenth, mut at_ninety) = (None, None);
    let (mut peak_demand, mut load) = (0.0_f32, (f32::MAX, f32::MIN));
    for index in 0..steps {
        if index.is_multiple_of(COMMAND_STEPS) {
            flight.send(mode, setpoint.clone());
        }
        flight.advance();
        let time = (index + 1) as f32 * DT_S;
        let done = fraction(flight);
        peak_rate = peak_rate.max(axis.rate(flight) * step.signum());
        peak_demand = peak_demand.max(axis.demand(flight) * step.signum());
        load = (
            load.0.min(flight.plant.load_factor),
            load.1.max(flight.plant.load_factor),
        );
        overshoot = overshoot.max(done - 1.0);
        if done >= 0.1 {
            at_tenth.get_or_insert(time);
        }
        if done >= 0.9 {
            at_ninety.get_or_insert(time);
        }
        if seconds - time <= 1.0 {
            final_error = final_error.max((done - 1.0).abs());
        }
    }
    StepResponse {
        peak_rate,
        peak_demand,
        load,
        rise_time: at_ninety.expect("90 % of the step") - at_tenth.expect("10 % of the step"),
        overshoot,
        final_error,
    }
}

/// A bank angle with the height and the airspeed of the cruise.
fn bank(roll: f32) -> Setpoint {
    Setpoint {
        attitude: Some(Quaternion::from_axis_angle(
            Vector3::new(1.0, 0.0, 0.0),
            roll,
        )),
        altitude: Some(Meters(300.0)),
        airspeed: Some(MetersPerSecond(55.0)),
        ..Setpoint::default()
    }
}

/// A pitch angle with wings level and the cruise throttle.
fn pitch(angle: f32) -> Setpoint {
    Setpoint {
        attitude: Some(Quaternion::from_axis_angle(
            Vector3::new(0.0, 1.0, 0.0),
            angle,
        )),
        collective_thrust: NormalizedThrust(c172().params.cruise_throttle.0),
        ..Setpoint::default()
    }
}

/// A C172 in settled cruise.
fn settled_cruise() -> Flight {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _settle = flight.fly(20.0, ControlMode::VelocityControl, |_| {
        course_speed_height(0.0, 55.0, 300.0)
    });
    flight
}

/// Largest roll rate as a fraction of the limit. The roll response of
/// an aeroplane is a first-order lag, so an exact feedforward does not
/// overshoot. The margin is for the rounding of the feedforward and for
/// the roll that sideslip adds.
const ROLL_RATE_MARGIN: f32 = 1.05;
/// Largest pitch rate as a fraction of the demand of the law. The pitch
/// rate leads the angle of attack: with the exact feedforward and the
/// rate feedback of the C172 the linear short-period model peaks at 1.23
/// times a step of the demand.
const PITCH_RATE_LEAD: f32 = 1.30;
/// Largest pitch rate as a fraction of the limit of the envelope.
const PITCH_RATE_MARGIN: f32 = 1.10;

#[test]
fn a_bank_step_of_40_degrees_stays_inside_the_roll_rate_limit() {
    let limit = c172().limits.max_roll_rate.0;
    let mut flight = settled_cruise();
    for target in [0.7_f32, 0.0, -0.7] {
        let seen = step_response(
            &mut flight,
            6.0,
            ControlMode::AltitudeHold,
            bank(target),
            Axis::Roll,
            target,
        );
        println!("bank step to {target}: {seen:?}");
        // The step is large enough for the demand to be at the limit.
        assert!((seen.peak_demand - limit).abs() < 1.0e-6, "{seen:?}");
        assert!(seen.peak_rate <= ROLL_RATE_MARGIN * limit, "{seen:?}");
        assert!(seen.peak_rate >= 0.9 * limit, "{seen:?}");
        // 40 degrees at the rate limit take 1.4 s; 80 % of that is the
        // rise time of a response that holds the limit.
        assert!(seen.rise_time < 1.6, "{seen:?}");
        assert!(seen.overshoot < 0.03, "{seen:?}");
        assert!(seen.final_error < 0.03, "{seen:?}");
    }
}

#[test]
fn a_pitch_step_stays_inside_the_pitch_rate_and_load_limits() {
    let limits = c172().limits;
    let mut flight = settled_cruise();
    for target in [0.22_f32, 0.0] {
        let seen = step_response(
            &mut flight,
            4.0,
            ControlMode::Attitude,
            pitch(target),
            Axis::Pitch,
            target,
        );
        println!("pitch step to {target}: {seen:?}");
        // The load-factor limits bound the pitch-rate demand before the
        // rate limit does: 2.5 g and 0 g are +0.27 and -0.18 rad/s at
        // 55 m/s.
        assert!(seen.peak_demand <= limits.max_pitch_rate.0, "{seen:?}");
        assert!(seen.peak_demand > 0.15, "{seen:?}");
        assert!(
            seen.peak_rate <= PITCH_RATE_LEAD * seen.peak_demand,
            "{seen:?}"
        );
        assert!(
            seen.peak_rate <= PITCH_RATE_MARGIN * limits.max_pitch_rate.0,
            "{seen:?}"
        );
        assert!(seen.peak_rate >= 0.9 * seen.peak_demand, "{seen:?}");
        let (least_load, most_load) = seen.load;
        assert!(least_load >= limits.min_load_factor, "{seen:?}");
        assert!(most_load <= limits.max_load_factor, "{seen:?}");
        assert!(seen.rise_time < 1.5, "{seen:?}");
        assert!(seen.overshoot < 0.03, "{seen:?}");
        assert!(seen.final_error < 0.03, "{seen:?}");
    }
}

/// Fly the bare plant for `seconds` with the trim controls and the
/// given aileron and elevator added, in the pilot's sense. Return the
/// largest roll rate and the last pitch rate.
fn open_loop(seconds: f32, aileron: f32, elevator: f32) -> (f32, f32) {
    let model = PlantModel::C172;
    let mut plant = FixedWingPlant::cruising(model, 0.0, 9.806_65);
    let mut outputs = [0.5; MAX_ACTUATOR_OUTPUTS];
    outputs[0] = model.trim_throttle;
    outputs[1] = 0.5 + 0.5 * aileron;
    outputs[2] = 0.5 + 0.5 * (model.trim_elevator + elevator);
    let controls = DirectActuatorCmd {
        timestamp_us: 0,
        outputs,
        count: 4,
        armed: true,
    };
    let mut peak_roll_rate = 0.0_f32;
    for _ in 0..(seconds / DT_S).round() as u64 {
        plant.step(&controls, DT_S);
        peak_roll_rate = peak_roll_rate.max(plant.roll_rate);
    }
    (peak_roll_rate, plant.pitch_rate())
}

#[test]
fn the_rate_feedforward_is_the_inverse_of_the_response_of_the_aeroplane() {
    let [roll_ff, pitch_ff, _yaw_ff] = c172().params.rate_ff;
    // The plant is at the cruise airspeed of the description, where the
    // gain scale of the law is one.
    assert_eq!(
        PlantModel::C172.reference_speed_mps,
        c172().params.cruise_airspeed.0
    );

    // The roll rate that an aileron step reaches before the sideslip of
    // the turn takes part of it away.
    let aileron = 0.2;
    let (peak_roll_rate, _) = open_loop(1.0, aileron, 0.0);
    let roll_gain = peak_roll_rate / aileron;
    println!("roll rate for each unit of aileron: {roll_gain}");
    assert!((roll_ff * roll_gain - 1.0).abs() < 0.05, "{roll_gain}");

    // A steady pull-up, after the short-period motion and before the
    // airspeed falls: 1.5 s after an elevator step.
    let elevator = 0.1;
    let (_, pull_up_rate) = open_loop(1.5, 0.0, elevator);
    let pitch_gain = pull_up_rate / elevator;
    println!("pitch rate for each unit of elevator: {pitch_gain}");
    assert!((pitch_ff * pitch_gain - 1.0).abs() < 0.08, "{pitch_gain}");
}
