//! The fixed-wing control law in closed loop with a point-mass aeroplane,
//! through the real kernel and host-built sensor samples.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::{c172, rascal};
use aviate_core::control::fixed_wing::AirspeedSource;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::types::{Meters, MetersPerSecond, Radians};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, wrap, FixedWingPlant, Flight, PlantModel, ProbeErrors};

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
    // The aeroplane needs 0.12 of up elevator to hold its trim. The
    // pitch-rate integrator finds it: the lane sits at 0.5 + 0.5 * 0.12.
    assert!(
        (flight.last.outputs[2] - 0.56).abs() < 0.01,
        "elevator {}",
        flight.last.outputs[2]
    );
    // The propeller yaws the nose left. Without a sideslip measurement
    // the law leaves that to the fin: a constant half degree of sideslip.
    assert!(seen.max_sideslip < 1.0_f32.to_radians(), "{seen:?}");
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
    // The sink demand stops at 5 m/s. The pitch loop overshoots the
    // path angle while the descent builds: the rate peaks at 7.1 m/s.
    assert!(descent.max_sink < 7.5, "{descent:?}");
    assert!(
        descent.min_indicated > 53.5 && descent.max_indicated < 56.5,
        "{descent:?}"
    );
    assert!(descent.min_height > 297.0, "{descent:?}");
}

#[test]
fn course_step_turns_at_the_route_bank_inside_the_load_limit() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let east = core::f32::consts::FRAC_PI_2;
    // Settle first: the law has to find the elevator trim of the aeroplane.
    let _settle = flight.fly(10.0, VELOCITY, level(0.0, 55.0, 300.0));
    let entry = flight.fly(4.0, VELOCITY, level(east, 55.0, 300.0));
    // The roll into the turn drags the nose outward: adverse yaw from
    // the ailerons, before the fin and the rudder catch up.
    assert!(entry.max_sideslip < 5.0_f32.to_radians(), "{entry:?}");

    // The established turn is at the 25 degree route bank of the
    // airframe, not at its 45 degree envelope limit. The rudder is into
    // the turn, the elevator holds the nose up, the turn is coordinated,
    // and the load factor is that of a level 25 degree turn: 1.10 g.
    let established = flight.fly(3.0, VELOCITY, level(east, 55.0, 300.0));
    let bank = flight.plant.roll.to_degrees();
    assert!((bank - 25.2).abs() < 1.5, "bank {bank} degrees");
    assert!(established.mean_rudder > 0.505, "{established:?}");
    assert!(
        established.max_sideslip < 1.5_f32.to_radians(),
        "{established:?}"
    );
    assert!(
        flight.last.outputs[2] > 0.565,
        "elevator {}",
        flight.last.outputs[2]
    );
    assert!(
        established.max_load > 1.05 && established.max_load < 1.2,
        "{established:?}"
    );
    let rest = flight.fly(33.0, VELOCITY, level(east, 55.0, 300.0));
    assert!(rest.max_sideslip < 1.5_f32.to_radians(), "{rest:?}");

    for seen in [entry, established, rest] {
        assert!(seen.max_bank < 27.0_f32.to_radians(), "{seen:?}");
        assert!(seen.max_load < 1.3, "{seen:?}");
        assert!(
            seen.min_height > 297.0 && seen.max_height < 303.0,
            "{seen:?}"
        );
        assert!(seen.min_indicated > 54.0, "{seen:?}");
    }
    // The course integrator took a little of the turn with it and gives
    // it back over its 20 s integral time.
    assert!(wrap(flight.plant.course - east).abs() < 1.0_f32.to_radians());
    assert!(flight.plant.roll.abs() < 1.0_f32.to_radians());
}

#[test]
fn a_direct_bank_command_may_use_the_envelope_limit() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _settle = flight.fly(10.0, VELOCITY, level(0.0, 55.0, 300.0));
    let (sin, cos) = (0.5_f32 * 1.2).sin_cos();
    let steep = aviate_core::math::Quaternion::new(cos, sin, 0.0, 0.0);
    let seen = flight.fly(8.0, ControlMode::Attitude, |_: &FixedWingPlant| Setpoint {
        attitude: Some(steep),
        collective_thrust: aviate_core::types::NormalizedThrust(0.9),
        ..Setpoint::default()
    });
    // 1.2 rad is asked for; 45 degrees is the configured limit.
    let bank = flight.plant.roll.to_degrees();
    assert!((bank - 45.0).abs() < 1.5, "bank {bank} degrees");
    assert!(seen.max_bank < 46.5_f32.to_radians(), "{seen:?}");
}

#[test]
fn probe_pressure_errors_do_not_shake_the_throttle() {
    // A probe at 25 Hz with 3 Pa of white noise on each port and a slow
    // drift of 10 Pa and 20 Pa.
    let mut flight = Flight::cruising(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Off,
        DirectSensorRates::default().with_airspeed(40_000),
    );
    flight.probe_errors = Some(ProbeErrors::new(3));
    let _settle = flight.fly(20.0, VELOCITY, level(0.0, 55.0, 300.0));
    let seen = flight.fly(40.0, VELOCITY, level(0.0, 55.0, 300.0));
    // With one filter stage on the airspeed rate the throttle moved with
    // a standard deviation of 0.025 and the elevator with 0.007. With
    // two stages they are at 0.003 and 0.001.
    assert!(seen.throttle_deviation < 0.008, "{seen:?}");
    assert!(seen.elevator_deviation < 0.004, "{seen:?}");
    assert!(
        seen.min_height > 298.0 && seen.max_height < 302.0,
        "{seen:?}"
    );
    assert!(
        seen.min_indicated > 54.0 && seen.max_indicated < 56.0,
        "{seen:?}"
    );
}

#[test]
fn straight_flight_holds_the_course_with_a_steady_sideslip() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _ = flight.fly(60.0, VELOCITY, level(0.3, 55.0, 300.0));
    let early = (flight.plant.sideslip(), flight.last.outputs[3]);
    let seen = flight.fly(90.0, VELOCITY, level(0.3, 55.0, 300.0));
    let late = (flight.plant.sideslip(), flight.last.outputs[3]);

    // The propeller needs a constant bank to fly straight. The course
    // integrator supplies it, so the course has no steady error.
    assert!(wrap(flight.plant.course - 0.3).abs() < 0.1_f32.to_radians());
    // Sideslip and rudder do not creep: the yaw loop has no integrator
    // that could store a heading error as rudder.
    assert!(
        (late.0 - early.0).abs() < 0.05_f32.to_radians(),
        "{early:?} {late:?}"
    );
    assert!((late.1 - early.1).abs() < 0.002, "{early:?} {late:?}");
    assert!(seen.max_sideslip < 1.0_f32.to_radians(), "{seen:?}");
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
fn gnss_loss_falls_back_to_heading_airspeed_and_pressure_level() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    // The receiver goes silent in a turn toward course 2.0 rad at 50 m/s.
    let _ = flight.fly(6.0, VELOCITY, level(2.0, 50.0, 300.0));
    assert!(flight.plant.roll > 20.0_f32.to_radians());
    flight.gnss_silent = true;
    // The estimator holds velocity validity for 5 s after the last fix.
    // Then the height stays valid on the barometer, so the mode falls
    // back to a heading and a height.
    let _ = flight.fly(8.0, VELOCITY, level(2.0, 50.0, 300.0));
    assert_eq!(
        flight.board.last_effective_command().mode,
        ControlMode::AltitudeHold
    );
    let level_at_fallback = flight.plant.height();
    let _recover = flight.fly(20.0, VELOCITY, level(2.0, 50.0, 300.0));
    let seen = flight.fly(60.0, VELOCITY, level(2.0, 50.0, 300.0));

    // Attitude, airspeed and static pressure still work. The law flies
    // the commanded course as a heading, holds the commanded airspeed
    // with pitch, and holds the pressure level with the throttle.
    assert_eq!(flight.airspeed_source(), AirspeedSource::Measured);
    assert!(wrap(flight.plant.heading - 2.0).abs() < 1.0_f32.to_radians());
    assert!(seen.max_bank < 2.0_f32.to_radians(), "{seen:?}");
    assert!(
        seen.min_indicated > 49.0 && seen.max_indicated < 51.0,
        "{seen:?}"
    );
    // The level is the one at the moment of the fallback. Over 80 s the
    // vehicle stays inside 5 m of it; with the throttle at a fixed
    // cruise value it would leave it at the trim error of that value.
    assert!(
        (seen.min_height - level_at_fallback).abs() < 5.0
            && (seen.max_height - level_at_fallback).abs() < 5.0,
        "level {level_at_fallback} m, {seen:?}"
    );
}

#[test]
fn gnss_loss_without_a_probe_holds_trim_pitch_and_the_height_on_the_barometer() {
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
        ControlMode::AltitudeHold
    );
    assert_eq!(flight.airspeed_source(), AirspeedSource::Assumed);
    // Nothing measures the airspeed now. The trim pitch of the airframe
    // description keeps it well above the stall, and the throttle holds
    // the commanded height on the barometer.
    assert!(seen.max_bank < 2.0_f32.to_radians(), "{seen:?}");
    assert!(seen.min_indicated > 45.0, "{seen:?}");
    assert!(
        seen.min_height > 298.0 && seen.max_height < 302.0,
        "{seen:?}"
    );
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
