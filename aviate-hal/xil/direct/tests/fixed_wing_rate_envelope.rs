//! TST-CTL-213: the C172 stays inside the body-rate limits of its
//! envelope on large attitude steps, with the autopilot engaged, and its
//! rate feedforward is the inverse of the response of the aeroplane, at
//! sea level, at 2000 m and at 4000 m.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::fixed_wing::{FixedWingController, FixedWingRuntimeState};
use aviate_core::control::runtime::ControllerRuntimeState;
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlMode, Setpoint, VehicleControlMode,
    VehicleController,
};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::sensor::AirData;
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, NormalizedThrust, Pascals, RadiansPerSecond, Seconds,
};
use aviate_hal_io::MAX_ACTUATOR_OUTPUTS;
use aviate_xil_direct::{DirectActuatorCmd, DirectNoiseTier, DirectSensorRates};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, Faults, FixedWingPlant, Flight, PlantModel, DT_S};

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

/// Heights of the flights: sea level, 2000 m and 4000 m in the standard
/// atmosphere. The true airspeed is 1.00, 1.10 and 1.22 times the
/// indicated airspeed.
const HEIGHTS_M: [f32; 3] = [0.0, 2_000.0, 4_000.0];
/// Indicated airspeed of each flight \[m/s\].
const INDICATED_MPS: f32 = 55.0;

/// Density of the standard atmosphere at `height` as a fraction of the
/// sea-level density.
fn density_ratio(height: f32) -> f32 {
    (1.0 - 2.2558e-5 * height).powf(4.2559)
}

/// A bank angle with the airspeed of the cruise at `height` metres.
fn bank(roll: f32, height: f32) -> Setpoint {
    Setpoint {
        attitude: Some(Quaternion::from_axis_angle(
            Vector3::new(1.0, 0.0, 0.0),
            roll,
        )),
        altitude: Some(Meters(height)),
        airspeed: Some(MetersPerSecond(INDICATED_MPS)),
        ..Setpoint::default()
    }
}

/// A pitch angle with wings level and the given throttle.
fn pitch(angle: f32, throttle: f32) -> Setpoint {
    Setpoint {
        attitude: Some(Quaternion::from_axis_angle(
            Vector3::new(0.0, 1.0, 0.0),
            angle,
        )),
        collective_thrust: NormalizedThrust(throttle),
        ..Setpoint::default()
    }
}

/// A C172 in settled cruise at `height` metres and 55 m/s indicated.
fn settled_cruise(height: f32) -> Flight {
    let mut flight = Flight::cruising_with(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Off,
        DirectSensorRates::default().with_airspeed(50_000),
        Faults {
            height_m: Some(height),
            ..Faults::default()
        },
    );
    let _settle = flight.fly(40.0, ControlMode::VelocityControl, |_| {
        course_speed_height(0.0, INDICATED_MPS, height)
    });
    let indicated = flight.plant.indicated_airspeed();
    assert!((indicated - INDICATED_MPS).abs() < 1.0, "{indicated}");
    let ratio = flight.plant.airspeed / indicated;
    assert!((ratio - density_ratio(height).powf(-0.5)).abs() < 0.01);
    flight
}

/// Largest roll rate as a fraction of the limit. The roll response of
/// an aeroplane is a first-order lag, so an exact feedforward does not
/// overshoot. The margin is for the rounding of the feedforward and for
/// the roll that sideslip adds.
const ROLL_RATE_MARGIN: f32 = 1.05;
/// Largest pitch rate as a fraction of the demand of the law. The pitch
/// rate leads the angle of attack, which follows it more slowly in thin
/// air. With the exact feedforward and the rate feedback of the C172 the
/// linear short-period model peaks at 1.23 times a step of the demand at
/// sea level, 1.28 times at 2000 m and 1.35 times at 4000 m.
const PITCH_RATE_LEAD: f32 = 1.40;
/// Largest pitch rate as a fraction of the limit of the envelope.
const PITCH_RATE_MARGIN: f32 = 1.10;

#[test]
fn a_bank_step_of_40_degrees_stays_inside_the_roll_rate_limit_at_each_height() {
    let limit = c172().limits.max_roll_rate.0;
    for height in HEIGHTS_M {
        let mut flight = settled_cruise(height);
        for target in [0.7_f32, 0.0, -0.7] {
            let seen = step_response(
                &mut flight,
                6.0,
                ControlMode::AltitudeHold,
                bank(target, height),
                Axis::Roll,
                target,
            );
            println!("{height} m, bank step to {target}: {seen:?}");
            // The step is large enough for the demand to be at the limit.
            assert!((seen.peak_demand - limit).abs() < 1.0e-6, "{seen:?}");
            assert!(seen.peak_rate <= ROLL_RATE_MARGIN * limit, "{seen:?}");
            assert!(seen.peak_rate >= 0.9 * limit, "{seen:?}");
            // 40 degrees at the rate limit take 1.4 s; 80 % of that is
            // the rise time of a response that holds the limit.
            assert!(seen.rise_time < 1.6, "{seen:?}");
            assert!(seen.overshoot < 0.03, "{seen:?}");
            assert!(seen.final_error < 0.03, "{seen:?}");
        }
    }
}

#[test]
fn a_pitch_step_stays_inside_the_pitch_rate_and_load_limits_at_each_height() {
    let limits = c172().limits;
    for height in HEIGHTS_M {
        let mut flight = settled_cruise(height);
        let throttle = flight.last.outputs[0];
        for target in [0.22_f32, 0.0] {
            let seen = step_response(
                &mut flight,
                4.0,
                ControlMode::Attitude,
                pitch(target, throttle),
                Axis::Pitch,
                target,
            );
            println!("{height} m, pitch step to {target}: {seen:?}");
            // The load-factor limits bound the pitch-rate demand before
            // the rate limit does: 2.5 g and 0 g are +0.27 and -0.18
            // rad/s at 55 m/s of true airspeed.
            assert!(seen.peak_demand <= limits.max_pitch_rate.0, "{seen:?}");
            assert!(seen.peak_demand > 0.12, "{seen:?}");
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
            // The demand falls with the true airspeed, so the step is
            // slower at height: 0.22 rad at 0.16 rad/s take 1.4 s.
            assert!(seen.rise_time < 2.0, "{seen:?}");
            assert!(seen.overshoot < 0.03, "{seen:?}");
            assert!(seen.final_error < 0.04, "{seen:?}");
        }
    }
}

/// Aileron and elevator that the law gives as feedforward for the body
/// rates `demand` at 55 m/s indicated and `height` metres: the vehicle
/// turns at the demand, so the feedback gives nothing.
fn feedforward_at(height: f32, demand: [f32; 2]) -> [f32; 2] {
    let airframe = c172();
    let controller = FixedWingController::new(airframe.params).expect("valid block");
    let mut runtime = FixedWingRuntimeState::default();
    runtime.set_cycle_interval(Seconds(DT_S));
    let pressure = 101_325.0 * (1.0 - 2.2558e-5 * height).powf(5.2559);
    runtime.set_air_data(Some(&AirData {
        dynamic_pressure: Some(Pascals(0.5 * 1.225 * INDICATED_MPS * INDICATED_MPS)),
        static_pressure: Some(Pascals(pressure)),
        temperature: Some(Celsius(15.0 - 0.0065 * height)),
        ..AirData::default()
    }));
    let [roll_rate, pitch_rate] = demand.map(RadiansPerSecond);
    let rates = [roll_rate, pitch_rate, RadiansPerSecond(0.0)];
    let state = StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: rates,
        position_ned: [Meters(0.0); 3],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    };
    let command = Command {
        mode: ControlMode::Rate,
        setpoint: Setpoint {
            angular_rate: Some(rates),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Autopilot,
    };
    let output = controller.step(
        &mut runtime,
        &state,
        &command,
        &VehicleControlMode::from_control_mode(command.mode),
        ConfigMode::Cruise,
        &airframe.limits,
    );
    [output.roll.0, output.pitch.0]
}

/// Fly the bare plant for `seconds` at 55 m/s indicated and `height`
/// metres with the trim controls and the given aileron and elevator
/// added, in the pilot's sense. Return the largest roll rate and the
/// last pitch rate.
fn open_loop(height: f32, seconds: f32, aileron: f32, elevator: f32) -> (f32, f32) {
    let model = PlantModel::C172;
    let mut plant = FixedWingPlant::cruising(model, height, 9.806_65);
    plant.airspeed = INDICATED_MPS * density_ratio(height).powf(-0.5);
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
fn the_feedforward_of_the_law_holds_the_demand_at_each_height() {
    // The plant is at the cruise airspeed of the description, where the
    // gain scale of the law is one at sea level.
    assert_eq!(
        PlantModel::C172.reference_speed_mps,
        c172().params.cruise_airspeed.0
    );
    let demand = [0.3_f32, 0.13];
    for height in HEIGHTS_M {
        let [aileron, elevator] = feedforward_at(height, demand);
        // The roll rate that the aileron reaches before the sideslip of
        // the turn takes part of it away.
        let (roll_rate, _) = open_loop(height, 1.0, aileron, 0.0);
        // A steady pull-up, after the short-period motion and before
        // the airspeed falls: 1.5 s after the elevator step.
        let (_, pitch_rate) = open_loop(height, 1.5, 0.0, elevator);
        let held = [roll_rate / demand[0], pitch_rate / demand[1]];
        println!("{height} m: aileron {aileron} elevator {elevator} held {held:?}");
        assert!((held[0] - 1.0).abs() < 0.05, "{height} m: {held:?}");
        assert!((held[1] - 1.0).abs() < 0.08, "{height} m: {held:?}");
    }
}
