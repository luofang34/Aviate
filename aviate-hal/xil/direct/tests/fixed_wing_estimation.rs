//! One estimator for a fixed wing with and without GNSS: attitude,
//! heading and height through turns in wind, with sensor biases, on
//! the real kernel and host-built sensor samples.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::state::StateValidFlags;
use aviate_core::types::{Meters, NormalizedThrust};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{
    course_speed_height, heading_speed_height, wrap, Faults, FixedWingPlant, Flight, PlantModel,
    ProbeErrors,
};

const WIND_NE: [f32; 2] = [4.0, -3.0];
const CRUISE_MPS: f32 = 55.0;
const HEIGHT_M: f32 = 300.0;
const WIND_SHIFT: [f32; 2] = [-0.03, 0.03];
const QUARTER: f32 = core::f32::consts::FRAC_PI_2;

fn flight(faults: Faults) -> Flight {
    let mut flight = Flight::cruising_with(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Industrial,
        DirectSensorRates::default().with_airspeed(40_000),
        faults,
    );
    flight.probe_errors = Some(ProbeErrors::new(3));
    flight
}

/// Largest roll, pitch and heading error over a run, in degrees.
#[derive(Debug, Default, Clone, Copy)]
struct Worst([f32; 3]);

impl Worst {
    fn add(&mut self, errors: [f32; 3]) {
        for (worst, error) in self.0.iter_mut().zip(errors) {
            *worst = worst.max(error.abs().to_degrees());
        }
    }

    fn merge(&mut self, other: Worst) {
        for (worst, error) in self.0.iter_mut().zip(other.0) {
            *worst = worst.max(error);
        }
    }

    fn within(&self, tilt_deg: f32, heading_deg: f32) -> bool {
        self.0[0] < tilt_deg && self.0[1] < tilt_deg && self.0[2] < heading_deg
    }
}

/// Fly `seconds` in 1 s pieces under `command` and return the worst error.
fn leg(
    flight: &mut Flight,
    seconds: u32,
    mode: ControlMode,
    command: impl Fn(&FixedWingPlant) -> Setpoint,
) -> Worst {
    let mut worst = Worst::default();
    for _ in 0..seconds {
        let seen = flight.fly(1.0, mode, &command);
        worst.add(seen.max_attitude_error);
    }
    worst
}

/// Heading, height and the cruise airspeed: the command a vehicle with
/// no position fix can fly.
fn heading_hold(heading: f32) -> impl Fn(&FixedWingPlant) -> Setpoint {
    move |_| heading_speed_height(heading, CRUISE_MPS, HEIGHT_M)
}

/// 300 s at three headings joined by two 90 degree turns at the route bank.
fn three_legs(flight: &mut Flight) -> (Worst, f32) {
    let mut worst = Worst::default();
    let mut max_bank = 0.0_f32;
    for heading in [0.0, QUARTER, 2.0 * QUARTER] {
        for _ in 0..100 {
            let seen = flight.fly(1.0, ControlMode::AltitudeHold, heading_hold(heading));
            worst.add(seen.max_attitude_error);
            max_bank = max_bank.max(seen.max_bank.to_degrees());
        }
    }
    (worst, max_bank)
}

#[test]
fn no_gnss_ever_from_an_in_air_start_through_two_turns() {
    let mut flight = flight(Faults {
        no_gnss: true,
        wind_ne: WIND_NE,
        ..Faults::default()
    });
    let (worst, max_bank) = three_legs(&mut flight);
    let estimate = flight.estimate();
    println!(
        "no GNSS: worst {worst:?} deg, bank {max_bank:.1} deg, flags {:?}",
        estimate.valid_flags
    );
    assert!(
        max_bank > 24.0,
        "the turns reach the route bank: {max_bank}"
    );
    assert!(worst.within(2.0, 5.0), "{worst:?}");
    assert!(estimate
        .valid_flags
        .contains(StateValidFlags::HEIGHT | StateValidFlags::AIRSPEED));
    assert!(!estimate.valid_flags.contains(StateValidFlags::POSITION));
    assert!((flight.plant.height() - HEIGHT_M).abs() < 10.0);
}

#[test]
fn no_gnss_ever_with_a_gyro_turn_on_bias() {
    let bias = 0.5_f32.to_radians();
    let mut flight = flight(Faults {
        no_gnss: true,
        wind_ne: WIND_NE,
        gyro_bias_rps: [bias, -bias, bias],
        ..Faults::default()
    });
    let (worst, _) = three_legs(&mut flight);
    let gyro_bias = flight.board.kernel().state.estimator.gyro_bias;
    println!(
        "gyro bias: worst {worst:?} deg, estimated bias ({:.2}, {:.2}, {:.2}) deg/s",
        gyro_bias.x.0.to_degrees(),
        gyro_bias.y.0.to_degrees(),
        gyro_bias.z.0.to_degrees()
    );
    assert!(worst.within(2.0, 5.0), "{worst:?}");
}

/// Position error over the 2-D position covariance: chi-square with two
/// degrees of freedom for an honest covariance.
fn horizontal_nees(flight: &Flight) -> f32 {
    let estimate = flight.estimate();
    let p = &flight.board.kernel().state.estimator.p_cov;
    let truth = flight.truth_in_estimate_frame();
    let north = estimate.position_ned[0].0 - truth[0];
    let east = estimate.position_ned[1].0 - truth[1];
    let (pnn, pne, pee) = (p.get(0, 0), p.get(0, 1), p.get(1, 1));
    let det = pnn * pee - pne * pne;
    (north * north * pee - 2.0 * north * east * pne + east * east * pnn) / det
}

/// Course, height and the cruise airspeed over the ground.
fn course_hold(course: f32) -> impl Fn(&FixedWingPlant) -> Setpoint {
    move |_| course_speed_height(course, CRUISE_MPS, HEIGHT_M)
}

#[test]
fn gnss_outage_with_turns_and_a_wind_shift_then_recovery() {
    let mut flight = flight(Faults {
        wind_ne: WIND_NE,
        ..Faults::default()
    });
    let mut worst = leg(
        &mut flight,
        60,
        ControlMode::VelocityControl,
        course_hold(0.0),
    );

    // 300 s without GNSS: three legs, two 90 degree turns, and a wind
    // that changes by 3 m/s on each axis during the second leg. The
    // flight covers about 16 km, so the frame origin moves about 16 times.
    flight.gnss_silent = true;
    let rebases_before = flight.board.kernel().state.estimator.frame.rebases;
    let mut nees = Vec::new();
    let mut worst_height_m = 0.0_f32;
    for (index, course) in [0.0, QUARTER, 2.0 * QUARTER].into_iter().enumerate() {
        flight.plant.wind_rate_ne = if index == 1 { WIND_SHIFT } else { [0.0; 2] };
        for _ in 0..20 {
            worst.merge(leg(
                &mut flight,
                5,
                ControlMode::VelocityControl,
                course_hold(course),
            ));
            nees.push(horizontal_nees(&flight));
            worst_height_m = worst_height_m.max((flight.plant.height() - HEIGHT_M).abs());
        }
    }
    flight.plant.wind_rate_ne = [0.0; 2];
    let rebases = flight.board.kernel().state.estimator.frame.rebases - rebases_before;
    let fallback = flight.board.last_effective_command().mode;
    let mean_nees = nees.iter().sum::<f32>() / nees.len() as f32;
    let max_nees = nees.iter().fold(0.0_f32, |a, b| a.max(*b));

    // Recovery: the attitude error must not step when the first fixes land.
    flight.gnss_silent = false;
    let before = flight.attitude_error();
    let mut largest_step = 0.0_f32;
    let mut position_back_after_s = None;
    for tenth in 1..=300 {
        worst.add(
            flight
                .fly(
                    0.1,
                    ControlMode::VelocityControl,
                    course_hold(2.0 * QUARTER),
                )
                .max_attitude_error,
        );
        if tenth <= 30 {
            for (a, b) in before.iter().zip(flight.attitude_error()) {
                largest_step = largest_step.max(wrap(b - a).abs().to_degrees());
            }
        }
        if position_back_after_s.is_none()
            && flight
                .estimate()
                .valid_flags
                .contains(StateValidFlags::POSITION)
        {
            position_back_after_s = Some(tenth as f32 / 10.0);
        }
    }
    println!(
        "outage: worst {worst:?} deg, fallback {fallback:?}, NEES mean {mean_nees:.2} max {max_nees:.2}, \
         {rebases} rebases, worst height error {worst_height_m:.2} m, \
         attitude error change in the first 3 s of fixes {largest_step:.2} deg, \
         position valid again after {position_back_after_s:?} s, wind {:?} true {:?}",
        flight.board.kernel().state.estimator.air.wind_ne,
        flight.plant.wind_ne
    );
    assert_eq!(fallback, ControlMode::AltitudeHold);
    assert!(rebases >= 10, "the outage crosses many frames: {rebases}");
    assert!(worst.within(2.0, 5.0), "{worst:?}");
    // A rebase must not carry a height offset into the next frame.
    assert!(worst_height_m < 0.75, "height error {worst_height_m}");
    assert!(
        mean_nees < 2.0,
        "position covariance is overconfident: {mean_nees}"
    );
    assert!(
        largest_step < 1.0,
        "attitude step at recovery: {largest_step}"
    );
    assert!(position_back_after_s.is_some_and(|s| s <= 5.0));
    assert_eq!(
        flight.board.last_effective_command().mode,
        ControlMode::VelocityControl
    );
}

/// Bank of `degrees` with the height held: a steep turn of the stress case.
fn bank(degrees: f32) -> impl Fn(&FixedWingPlant) -> Setpoint {
    let (sin, cos) = (0.5 * degrees.to_radians()).sin_cos();
    move |_| Setpoint {
        attitude: Some(Quaternion::new(cos, sin, 0.0, 0.0)),
        altitude: Some(Meters(HEIGHT_M)),
        collective_thrust: NormalizedThrust(0.9),
        ..Setpoint::default()
    }
}

fn steep_turn_with_accel_bias(no_gnss: bool) -> (Worst, Worst) {
    let mut flight = flight(Faults {
        no_gnss,
        wind_ne: WIND_NE,
        accel_bias_mps2: [-0.578, -0.224, -0.360],
        ..Faults::default()
    });
    let start = flight.plant.heading;
    let mut turn = Worst::default();
    while wrap(flight.plant.heading - start - 200.0_f32.to_radians()).abs() > 0.05 {
        turn.add(
            flight
                .fly(0.5, ControlMode::AltitudeHold, bank(45.0))
                .max_attitude_error,
        );
        assert!(
            (flight.plant.height() - HEIGHT_M).abs() < 30.0,
            "height {} in the turn",
            flight.plant.height()
        );
    }
    let rolled_out = flight.plant.heading;
    let after = leg(
        &mut flight,
        60,
        ControlMode::AltitudeHold,
        heading_hold(rolled_out),
    );
    let accel_bias = flight.board.kernel().state.estimator.accel_bias;
    println!(
        "accel bias, no_gnss {no_gnss}: turn {turn:?} deg, after {after:?} deg, \
         estimated bias ({:.3}, {:.3}, {:.3})",
        accel_bias.x.0, accel_bias.y.0, accel_bias.z.0
    );
    (turn, after)
}

/// The stress case of an accelerometer turn-on bias and a steep turn
/// right after arming. The forward bias is above the bias limit of the
/// estimator and is nearly unobservable without GNSS: it reads as pitch.
/// The bounds are the measured errors with a margin; the claim is that
/// the estimate stays usable and the vehicle keeps its height.
#[test]
fn accelerometer_bias_and_a_steep_turn_right_after_arming() {
    let (turn, after) = steep_turn_with_accel_bias(false);
    assert!(turn.within(5.0, 5.0), "{turn:?}");
    assert!(after.within(2.0, 5.0), "{after:?}");
    let (turn, after) = steep_turn_with_accel_bias(true);
    assert!(turn.within(6.0, 10.0), "{turn:?}");
    assert!(after.within(3.0, 5.0), "{after:?}");
}
