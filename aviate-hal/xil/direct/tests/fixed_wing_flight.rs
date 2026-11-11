//! The fixed-wing control law in closed loop with a point-mass aeroplane,
//! through the real kernel and host-built sensor samples.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::{c172, rascal};
use aviate_core::control::fixed_wing::AirspeedSource;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::types::{Meters, MetersPerSecond, Radians};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates};

#[path = "plant/flight.rs"]
mod flight;
use flight::{course_speed_height, wrap, FixedWingPlant, Flight, PlantModel};

fn level(course_rad: f32, airspeed: f32, height: f32) -> impl FnMut(&FixedWingPlant) -> Setpoint {
    move |_| course_speed_height(course_rad, airspeed, height)
}

const VELOCITY: ControlMode = ControlMode::VelocityControl;

#[test]
fn holds_cruise_airspeed_height_and_course_with_sensor_noise() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Industrial);
    let _settle = flight.fly(10.0, VELOCITY, level(0.0, 55.0, 300.0));
    let seen = flight.fly(30.0, VELOCITY, level(0.0, 55.0, 300.0));

    // Trimmed flight under industrial-grade sensor noise: the law keeps
    // the vehicle inside 2 m, 1 m/s and 1 degree of bank.
    assert!(
        seen.min_height > 298.0 && seen.max_height < 302.0,
        "{seen:?}"
    );
    assert!(
        seen.min_indicated > 54.0 && seen.max_indicated < 56.0,
        "{seen:?}"
    );
    assert!(seen.max_bank < 1.0_f32.to_radians(), "{seen:?}");
    assert!(wrap(flight.plant.course).abs() < 0.5_f32.to_radians());
    assert_eq!(flight.airspeed_source(), AirspeedSource::Measured);
}

#[test]
fn airspeed_steps_settle_and_keep_the_height() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    for target in [45.0_f32, 60.0, 55.0] {
        let seen = flight.fly(60.0, VELOCITY, level(0.0, target, 300.0));
        let settled = flight.plant.indicated_airspeed();
        // The speed loop has a 5 s time constant, so 60 s is twelve of
        // them. 0.7 m/s covers the slow last part of an acceleration that
        // the thrust limits.
        assert!((settled - target).abs() < 0.7, "{settled} m/s for {target}");
        assert!(
            seen.min_height > 296.0 && seen.max_height < 304.0,
            "{seen:?}"
        );
    }
}

#[test]
fn height_steps_follow_the_climb_and_sink_limits() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let climb = flight.fly(70.0, VELOCITY, level(0.0, 55.0, 400.0));
    assert!(
        (flight.plant.height() - 400.0).abs() < 2.0,
        "{}",
        flight.plant.height()
    );
    // The demand stops at the 2.5 m/s that the airframe description
    // gives; the rate overshoots it by under 1 m/s while pitch settles.
    assert!(climb.max_climb < 3.5, "{climb:?}");
    assert!(
        climb.min_indicated > 54.0 && climb.max_indicated < 56.5,
        "{climb:?}"
    );
    assert!(climb.max_height < 403.0, "{climb:?}");

    let descent = flight.fly(60.0, VELOCITY, level(0.0, 55.0, 300.0));
    assert!(
        (flight.plant.height() - 300.0).abs() < 2.0,
        "{}",
        flight.plant.height()
    );
    // The sink demand stops at 5 m/s; the same overshoot allowance.
    assert!(descent.max_sink < 6.6, "{descent:?}");
    assert!(
        descent.min_indicated > 53.5 && descent.max_indicated < 56.5,
        "{descent:?}"
    );
    assert!(descent.min_height > 297.0, "{descent:?}");
}

#[test]
fn course_step_turns_inside_the_bank_and_load_limits() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let east = core::f32::consts::FRAC_PI_2;
    let entry = flight.fly(4.0, VELOCITY, level(east, 55.0, 300.0));
    // In the established right turn the rudder is into the turn and the
    // elevator holds the nose up.
    assert!(
        flight.plant.roll > 40.0_f32.to_radians(),
        "{}",
        flight.plant.roll
    );
    assert!(
        flight.last.outputs[3] > 0.51,
        "rudder {}",
        flight.last.outputs[3]
    );
    assert!(
        flight.last.outputs[2] > 0.52,
        "elevator {}",
        flight.last.outputs[2]
    );
    let rest = flight.fly(36.0, VELOCITY, level(east, 55.0, 300.0));

    for seen in [entry, rest] {
        // 45 degrees is the configured bank limit; 1 degree is the
        // overshoot of the roll loop. A level 45 degree turn is 1.41 g.
        assert!(seen.max_bank < 46.0_f32.to_radians(), "{seen:?}");
        assert!(seen.max_load < 1.5, "{seen:?}");
        assert!(
            seen.min_height > 296.0 && seen.max_height < 304.0,
            "{seen:?}"
        );
        assert!(seen.min_indicated > 54.0, "{seen:?}");
    }
    assert!(wrap(flight.plant.course - east).abs() < 0.5_f32.to_radians());
    assert!(flight.plant.roll.abs() < 0.5_f32.to_radians());
}

#[test]
fn an_airspeed_below_the_minimum_is_not_flown() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    // 20 m/s is below the 25 m/s stall speed of the airframe.
    let seen = flight.fly(80.0, VELOCITY, level(0.0, 20.0, 300.0));
    assert!(seen.min_indicated > 32.5, "{seen:?}");
    assert!((flight.plant.indicated_airspeed() - 33.0).abs() < 0.5);
    assert!(
        seen.min_height > 296.0 && seen.max_height < 304.0,
        "{seen:?}"
    );
}

#[test]
fn engine_failure_becomes_a_glide_at_the_minimum_airspeed() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _ = flight.fly(5.0, VELOCITY, level(0.0, 55.0, 300.0));
    flight.plant.engine = 0.0;
    let decay = flight.fly(70.0, VELOCITY, level(0.0, 55.0, 300.0));
    let glide = flight.fly(30.0, VELOCITY, level(0.0, 55.0, 300.0));

    // The law gives up the height and keeps the airspeed: 33 m/s is the
    // minimum of the airframe description, 25 m/s is its stall speed.
    for seen in [decay, glide] {
        assert!(seen.min_indicated > 32.0, "{seen:?}");
        assert!(seen.max_bank < 1.0_f32.to_radians(), "{seen:?}");
    }
    assert!(glide.max_indicated < 35.0, "{glide:?}");
    assert!(flight.board.kernel().state.controller.energy.underspeed);
    assert_eq!(flight.last.outputs[0], 1.0, "throttle stays open");
    // 9:1 glide at 33 m/s is 3.7 m/s; the model sinks at about 1.9 m/s.
    assert!(glide.max_sink < 3.0 && glide.max_climb < -1.0, "{glide:?}");
    assert!(wrap(flight.plant.course).abs() < 1.0_f32.to_radians());
}

#[test]
fn track_deviation_is_closed_without_overshoot() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    // The track runs north, 150 m west of the vehicle and 20 m above it.
    const TRACK_EAST_M: f32 = -150.0;
    const PROFILE_HEIGHT_M: f32 = 320.0;
    let deviation = |plant: &FixedWingPlant| Setpoint {
        lateral_deviation: Some(Meters(plant.pos_ned[1] - TRACK_EAST_M)),
        vertical_deviation: Some(Meters(plant.height() - PROFILE_HEIGHT_M)),
        heading: Some(Radians(0.0)),
        ..Setpoint::default()
    };
    let mut most_left = f32::MAX;
    for _ in 0..90 {
        let _ = flight.fly(1.0, ControlMode::DeviationTracking, deviation);
        most_left = most_left.min(flight.plant.pos_ned[1] - TRACK_EAST_M);
    }
    let lateral = flight.plant.pos_ned[1] - TRACK_EAST_M;
    assert!(lateral.abs() < 3.0, "lateral deviation {lateral} m");
    assert!(most_left > -5.0, "overshoot to {most_left} m");
    assert!((flight.plant.height() - PROFILE_HEIGHT_M).abs() < 2.0);
    assert!(wrap(flight.plant.course).abs() < 1.0_f32.to_radians());
}

#[test]
fn without_a_probe_the_law_flies_on_ground_speed() {
    let mut flight = Flight::cruising(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Off,
        DirectSensorRates::default(),
    );
    let seen = flight.fly(70.0, VELOCITY, level(1.0, 55.0, 380.0));
    assert_eq!(flight.airspeed_source(), AirspeedSource::GroundSpeed);
    // Height and course are held. The airspeed is not a loop without a
    // probe: it stays where the throttle of the height loop puts it.
    assert!((flight.plant.height() - 380.0).abs() < 3.0);
    assert!(wrap(flight.plant.course - 1.0).abs() < 1.0_f32.to_radians());
    assert!(
        seen.min_indicated > 48.0 && seen.max_indicated < 58.0,
        "{seen:?}"
    );
}

#[test]
fn gnss_loss_falls_back_to_wings_level_flight_at_the_cruise_airspeed() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    // The receiver goes silent in a turn at 50 m/s.
    let _ = flight.fly(6.0, VELOCITY, level(2.0, 50.0, 300.0));
    assert!(flight.plant.roll > 20.0_f32.to_radians());
    flight.gnss_silent = true;
    // The estimator holds velocity validity for 5 s after the last fix.
    let _ = flight.fly(8.0, VELOCITY, level(2.0, 50.0, 300.0));
    assert_eq!(
        flight.board.last_effective_command().mode,
        ControlMode::Attitude
    );
    let _recover = flight.fly(15.0, VELOCITY, level(2.0, 50.0, 300.0));
    let seen = flight.fly(40.0, VELOCITY, level(2.0, 50.0, 300.0));

    // Attitude and airspeed still work: wings level, cruise throttle and
    // the cruise airspeed on pitch.
    assert_eq!(flight.airspeed_source(), AirspeedSource::Measured);
    assert!(seen.max_bank < 2.0_f32.to_radians(), "{seen:?}");
    assert!(
        seen.min_indicated > 53.0 && seen.max_indicated < 57.0,
        "{seen:?}"
    );
    assert!((flight.last.outputs[0] - 0.65).abs() < 1.0e-6);
}

#[test]
fn gnss_loss_without_a_probe_holds_trim_attitude_and_cruise_throttle() {
    let mut flight = Flight::cruising(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Off,
        DirectSensorRates::default(),
    );
    let _ = flight.fly(5.0, VELOCITY, level(0.0, 55.0, 300.0));
    flight.gnss_silent = true;
    let _ = flight.fly(8.0, VELOCITY, level(0.0, 55.0, 300.0));
    let seen = flight.fly(40.0, VELOCITY, level(0.0, 55.0, 300.0));
    assert_eq!(
        flight.board.last_effective_command().mode,
        ControlMode::Attitude
    );
    assert_eq!(flight.airspeed_source(), AirspeedSource::Assumed);
    // Nothing measures the airspeed now. The trim pitch and the cruise
    // throttle of the airframe description keep it well above the stall.
    assert!(seen.max_bank < 2.0_f32.to_radians(), "{seen:?}");
    assert!(seen.min_indicated > 45.0, "{seen:?}");
    assert!((flight.last.outputs[0] - 0.65).abs() < 1.0e-6);
}

#[test]
fn command_loss_becomes_a_wings_level_descent_at_the_cruise_airspeed() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _ = flight.fly(5.0, VELOCITY, level(0.3, 55.0, 300.0));
    // No command for 10 s: the kernel flies its descent terminal.
    flight.coast(10.0);
    let height_before = flight.plant.height();
    flight.coast(10.0);
    let sink = (height_before - flight.plant.height()) / 10.0;
    // The descent rate is the 5 m/s limit of the airframe.
    assert!((sink - 5.0).abs() < 0.5, "sink {sink} m/s");
    assert!((flight.plant.indicated_airspeed() - 55.0).abs() < 1.5);
    assert!(flight.plant.roll.abs() < 1.0_f32.to_radians());
}

#[test]
fn position_hold_is_refused_with_idle_throttle_and_neutral_surfaces() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _ = flight.fly(1.0, ControlMode::PositionHold, |plant: &FixedWingPlant| {
        Setpoint {
            position: Some([
                Meters(plant.pos_ned[0]),
                Meters(plant.pos_ned[1]),
                Meters(plant.pos_ned[2]),
            ]),
            ..Setpoint::default()
        }
    });
    assert!(flight.last.armed);
    assert_eq!(&flight.last.outputs[..4], &[0.0, 0.5, 0.5, 0.5]);
}

#[test]
fn attitude_and_rate_commands_reach_the_vehicle() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let bank = 20.0_f32.to_radians();
    let pitch = 3.0_f32.to_radians();
    let half = |angle: f32| ((0.5 * angle).cos(), (0.5 * angle).sin());
    let ((cr, sr), (cp, sp)) = (half(bank), half(pitch));
    let attitude = aviate_core::math::Quaternion::new(cr * cp, sr * cp, cr * sp, -sr * sp);
    let _ = flight.fly(8.0, ControlMode::Attitude, |_: &FixedWingPlant| Setpoint {
        attitude: Some(attitude),
        collective_thrust: aviate_core::types::NormalizedThrust(0.7),
        ..Setpoint::default()
    });
    assert!((flight.plant.roll - bank).abs() < 1.0_f32.to_radians());
    assert!((flight.plant.pitch - pitch).abs() < 1.0_f32.to_radians());
    assert_eq!(flight.last.outputs[0], 0.7);

    let _ = flight.fly(1.5, ControlMode::Rate, |_: &FixedWingPlant| Setpoint {
        angular_rate: Some([
            aviate_core::types::RadiansPerSecond(-0.2),
            aviate_core::types::RadiansPerSecond(0.0),
            aviate_core::types::RadiansPerSecond(0.0),
        ]),
        collective_thrust: aviate_core::types::NormalizedThrust(0.6),
        ..Setpoint::default()
    });
    assert!(
        (flight.plant.roll_rate + 0.2).abs() < 0.03,
        "{}",
        flight.plant.roll_rate
    );
    assert_eq!(flight.last.outputs[0], 0.6);
}

#[test]
fn altitude_hold_flies_a_heading_and_a_climb_rate() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let heading = 30.0_f32.to_radians();
    let seen = flight.fly(30.0, ControlMode::AltitudeHold, |_: &FixedWingPlant| {
        Setpoint {
            heading: Some(Radians(heading)),
            // NED: a negative down rate is a climb.
            vertical_speed: Some(MetersPerSecond(-1.0)),
            ..Setpoint::default()
        }
    });
    assert!(wrap(flight.plant.course - heading).abs() < 1.0_f32.to_radians());
    let climb = -flight.plant.velocity_ned()[2];
    assert!((climb - 1.0).abs() < 0.2, "climb {climb} m/s");
    assert!(
        seen.min_indicated > 54.0 && seen.max_indicated < 56.0,
        "{seen:?}"
    );
}

#[test]
fn another_parameter_set_flies_another_airframe() {
    let mut flight = Flight::cruising(
        &rascal(),
        PlantModel::RASCAL,
        DirectNoiseTier::Off,
        DirectSensorRates::default().with_airspeed(50_000),
    );
    let _ = flight.fly(20.0, VELOCITY, level(0.0, 22.0, 300.0));
    let seen = flight.fly(60.0, VELOCITY, level(-1.2, 18.0, 340.0));
    assert!(
        (flight.plant.height() - 340.0).abs() < 2.0,
        "{}",
        flight.plant.height()
    );
    assert!((flight.plant.indicated_airspeed() - 18.0).abs() < 0.7);
    assert!(wrap(flight.plant.course + 1.2).abs() < 1.0_f32.to_radians());
    // 14 m/s is the minimum airspeed of this description.
    assert!(
        seen.min_indicated > 14.0 && seen.max_bank < 46.0_f32.to_radians(),
        "{seen:?}"
    );
}
