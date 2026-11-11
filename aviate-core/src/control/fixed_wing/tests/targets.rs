//! How each command mode is read as fixed-wing targets.

use super::super::targets::{ground_course, wrap_angle, Envelope, Lateral, Targets, Vertical};
use super::{attitude, command, cruise_state, limits};
use crate::control::{ControlMode, Setpoint, VehicleControlMode};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::state::{StateEstimate, StateValidFlags};
use crate::types::{Meters, MetersPerSecond, NormalizedThrust, Radians, RadiansPerSecond};

fn resolve(mode: ControlMode, setpoint: Setpoint, state: &StateEstimate) -> Targets {
    Targets::resolve(
        &command(mode, setpoint),
        &VehicleControlMode::from_control_mode(mode),
        state,
        &reference_params(),
    )
}

fn flight(targets: Targets) -> (Lateral, Vertical, f32, f32) {
    match targets {
        Targets::Flight {
            lateral,
            vertical,
            airspeed,
            throttle,
        } => (lateral, vertical, airspeed, throttle),
        Targets::Rates { .. } => panic!("flight targets expected"),
    }
}

fn velocity(north: f32, east: f32, down: f32) -> Option<[MetersPerSecond; 3]> {
    Some([
        MetersPerSecond(north),
        MetersPerSecond(east),
        MetersPerSecond(down),
    ])
}

fn assert_course(lateral: Lateral, expected: f32) {
    match lateral {
        Lateral::Course(course) => assert!(
            wrap_angle(course - expected).abs() < 1.0e-4,
            "course {course}, expected {expected}"
        ),
        other => panic!("course expected, got {other:?}"),
    }
}

#[test]
fn envelope_is_the_tighter_of_airframe_and_configuration() {
    let params = reference_params();
    let open = Envelope::resolve(&params, &limits());
    assert_eq!(open.min_airspeed, 33.0);
    assert_eq!(open.max_airspeed, 65.0);
    // 45 degrees of roll limit is tighter than the 66 degrees of 2.5 g.
    assert_eq!(open.bank_limit, 0.785);
    // The airframe climbs at 2.5 m/s; the configuration allows 5.
    assert_eq!(open.climb_limit, 2.5);
    assert_eq!(open.sink_limit, 5.0);

    let mut tight = limits();
    tight.min_airspeed = Some(MetersPerSecond(40.0));
    tight.max_airspeed = Some(MetersPerSecond(50.0));
    tight.max_load_factor = 1.2;
    let envelope = Envelope::resolve(&params, &tight);
    assert_eq!(envelope.min_airspeed, 40.0);
    assert_eq!(envelope.max_airspeed, 50.0);
    // 1.2 g is a 33.6 degree level turn.
    assert!((envelope.bank_limit - 0.586).abs() < 0.001);

    // Limits wider than the airframe do not widen it, and a load limit
    // under 1 g leaves no bank at all.
    let mut wide = limits();
    wide.min_airspeed = Some(MetersPerSecond(10.0));
    wide.max_airspeed = Some(MetersPerSecond(90.0));
    wide.max_load_factor = 0.8;
    let envelope = Envelope::resolve(&params, &wide);
    assert_eq!(envelope.min_airspeed, 33.0);
    assert_eq!(envelope.max_airspeed, 65.0);
    assert_eq!(envelope.bank_limit, 0.0);

    // A load-factor minimum above the maximum is replaced by zero g.
    let mut inverted = limits();
    inverted.min_load_factor = 4.0;
    assert_eq!(Envelope::resolve(&params, &inverted).min_load_factor, 0.0);

    // A minimum above the maximum wins: the range collapses onto it.
    let mut crossed = limits();
    crossed.min_airspeed = Some(MetersPerSecond(60.0));
    crossed.max_airspeed = Some(MetersPerSecond(45.0));
    let envelope = Envelope::resolve(&params, &crossed);
    assert_eq!((envelope.min_airspeed, envelope.max_airspeed), (60.0, 60.0));
}

#[test]
fn course_is_the_track_over_the_ground_or_else_the_heading() {
    let mut state = cruise_state();
    state.attitude = attitude(0.0, 0.0, 1.0);
    state.velocity_ned = [
        MetersPerSecond(0.0),
        MetersPerSecond(-40.0),
        MetersPerSecond(0.0),
    ];
    assert!((ground_course(&state) + core::f32::consts::FRAC_PI_2).abs() < 1.0e-5);

    // At rest the track has no direction.
    state.velocity_ned = [MetersPerSecond(0.0); 3];
    assert!((ground_course(&state) - 1.0).abs() < 1.0e-5);

    // Without a velocity estimate the stored velocity says nothing.
    state.velocity_ned[1].0 = -40.0;
    state.valid_flags.remove(StateValidFlags::VELOCITY);
    assert!((ground_course(&state) - 1.0).abs() < 1.0e-5);
}

#[test]
fn rate_command_passes_rates_and_throttle() {
    let rates = Setpoint {
        angular_rate: Some([
            RadiansPerSecond(0.1),
            RadiansPerSecond(-0.2),
            RadiansPerSecond(0.05),
        ]),
        collective_thrust: NormalizedThrust(0.4),
        ..Setpoint::default()
    };
    assert_eq!(
        resolve(ControlMode::Rate, rates, &cruise_state()),
        Targets::Rates {
            rates: [0.1, -0.2, 0.05],
            throttle: 0.4,
        }
    );
    assert_eq!(
        resolve(ControlMode::Rate, Setpoint::default(), &cruise_state()),
        Targets::Rates {
            rates: [0.0; 3],
            throttle: 0.0,
        }
    );
}

#[test]
fn attitude_command_is_bank_pitch_and_throttle() {
    let commanded = Setpoint {
        attitude: Some(attitude(0.3, 0.1, 2.0)),
        collective_thrust: NormalizedThrust(0.8),
        ..Setpoint::default()
    };
    let (lateral, vertical, _airspeed, throttle) =
        flight(resolve(ControlMode::Attitude, commanded, &cruise_state()));
    match (lateral, vertical) {
        (Lateral::Bank(bank), Vertical::Pitch(pitch)) => {
            assert!((bank - 0.3).abs() < 1.0e-4);
            assert!((pitch - 0.1).abs() < 1.0e-4);
        }
        other => panic!("bank and pitch expected, got {other:?}"),
    }
    assert_eq!(throttle, 0.8);

    // No attitude in the command: wings level at the trim pitch.
    let (lateral, vertical, ..) = flight(resolve(
        ControlMode::Attitude,
        Setpoint::default(),
        &cruise_state(),
    ));
    assert_eq!(lateral, Lateral::Bank(0.0));
    assert_eq!(vertical, Vertical::Pitch(reference_params().trim_pitch.0));
}

#[test]
fn attitude_after_an_automatic_fallback_ignores_the_stale_setpoint() {
    let params = reference_params();
    let guidance = Setpoint {
        velocity: velocity(0.0, 50.0, 0.0),
        attitude: Some(attitude(0.5, 0.2, 0.0)),
        ..Setpoint::default()
    };
    let mut flags = VehicleControlMode::from_control_mode(ControlMode::Attitude);
    flags.automatic_fallback_from = Some(ControlMode::VelocityControl);
    let targets = Targets::resolve(
        &command(ControlMode::Attitude, guidance),
        &flags,
        &cruise_state(),
        &params,
    );
    assert_eq!(
        flight(targets),
        (
            Lateral::WingsLevel,
            Vertical::AirspeedOnPitch,
            params.cruise_airspeed.0,
            params.cruise_throttle.0
        )
    );
}

#[test]
fn height_modes_take_heading_then_bank_then_wings_level() {
    let state = cruise_state();
    let heading = Setpoint {
        heading: Some(Radians(0.7)),
        attitude: Some(attitude(0.2, 0.0, 0.0)),
        altitude: Some(Meters(450.0)),
        ..Setpoint::default()
    };
    let (lateral, vertical, airspeed, _) =
        flight(resolve(ControlMode::AltitudeHold, heading, &state));
    assert_eq!(lateral, Lateral::Heading(0.7));
    assert_eq!(vertical, Vertical::Height(450.0));
    assert_eq!(airspeed, 55.0);

    let banked = Setpoint {
        attitude: Some(attitude(0.2, 0.0, 0.0)),
        vertical_speed: Some(MetersPerSecond(1.5)),
        ..Setpoint::default()
    };
    let (lateral, vertical, ..) = flight(resolve(ControlMode::AltitudeHold, banked, &state));
    match lateral {
        Lateral::Bank(bank) => assert!((bank - 0.2).abs() < 1.0e-4),
        other => panic!("bank expected, got {other:?}"),
    }
    // A positive down rate is a descent.
    assert_eq!(vertical, Vertical::ClimbRate(-1.5));

    // A position-hold command that reaches the law is a height hold.
    let (lateral, vertical, ..) = flight(resolve(
        ControlMode::PositionHold,
        Setpoint::default(),
        &state,
    ));
    assert_eq!(lateral, Lateral::WingsLevel);
    assert_eq!(vertical, Vertical::ClimbRate(0.0));
}

#[test]
fn velocity_command_is_course_airspeed_and_climb() {
    let state = cruise_state();
    let north_east_up = Setpoint {
        velocity: velocity(30.0, 40.0, -2.0),
        ..Setpoint::default()
    };
    let (lateral, vertical, airspeed, throttle) =
        flight(resolve(ControlMode::VelocityControl, north_east_up, &state));
    assert_course(lateral, 0.927_295);
    assert_eq!(vertical, Vertical::ClimbRate(2.0));
    assert_eq!(airspeed, 50.0);
    assert_eq!(throttle, reference_params().cruise_throttle.0);

    // A height in the command takes the place of the vertical rate.
    let with_height = Setpoint {
        velocity: velocity(30.0, 40.0, -2.0),
        altitude: Some(Meters(500.0)),
        ..Setpoint::default()
    };
    let (_, vertical, ..) = flight(resolve(ControlMode::VelocityControl, with_height, &state));
    assert_eq!(vertical, Vertical::Height(500.0));

    // A velocity with no horizontal part has no course: the law holds
    // the commanded heading at the cruise airspeed.
    let vertical_only = Setpoint {
        velocity: velocity(0.0, 0.0, 1.0),
        heading: Some(Radians(-0.4)),
        ..Setpoint::default()
    };
    let (lateral, vertical, airspeed, _) =
        flight(resolve(ControlMode::VelocityControl, vertical_only, &state));
    assert_eq!(lateral, Lateral::Heading(-0.4));
    assert_eq!(vertical, Vertical::ClimbRate(-1.0));
    assert_eq!(airspeed, 55.0);

    let empty = flight(resolve(
        ControlMode::VelocityControl,
        Setpoint::default(),
        &state,
    ));
    assert_eq!(empty.0, Lateral::WingsLevel);
    assert_eq!(empty.1, Vertical::ClimbRate(0.0));
}

#[test]
fn deviation_command_turns_toward_the_track_and_the_profile() {
    let params = reference_params();
    let state = cruise_state();
    // 8 s of look-ahead at 55 m/s is 440 m.
    let lookahead = params.track_lookahead_time.0 * 55.0;
    let right_and_above = Setpoint {
        lateral_deviation: Some(Meters(100.0)),
        vertical_deviation: Some(Meters(30.0)),
        heading: Some(Radians(0.5)),
        velocity: velocity(45.0, 0.0, 0.0),
        ..Setpoint::default()
    };
    let (lateral, vertical, airspeed, _) = flight(resolve(
        ControlMode::DeviationTracking,
        right_and_above,
        &state,
    ));
    // Right of the track: the course turns left of the track course.
    assert_course(lateral, 0.5 - (100.0 / lookahead).atan());
    // 30 m above the profile at 300 m: the profile is at 270 m.
    assert_eq!(vertical, Vertical::Height(270.0));
    assert_eq!(airspeed, 45.0);

    // A large deviation meets the intercept limit, on either side.
    for side in [1.0_f32, -1.0] {
        let far = Setpoint {
            lateral_deviation: Some(Meters(side * 5_000.0)),
            heading: Some(Radians(0.5)),
            ..Setpoint::default()
        };
        let (lateral, vertical, airspeed, _) =
            flight(resolve(ControlMode::DeviationTracking, far, &state));
        assert_course(lateral, 0.5 - side * params.max_intercept_angle.0);
        assert_eq!(vertical, Vertical::ClimbRate(0.0));
        assert_eq!(airspeed, 55.0);
    }

    // No track course and no deviation: hold the present course.
    let mut eastbound = cruise_state();
    eastbound.velocity_ned = [
        MetersPerSecond(0.0),
        MetersPerSecond(55.0),
        MetersPerSecond(0.0),
    ];
    let (lateral, ..) = flight(resolve(
        ControlMode::DeviationTracking,
        Setpoint::default(),
        &eastbound,
    ));
    assert_course(lateral, core::f32::consts::FRAC_PI_2);
}

#[test]
fn angles_wrap_into_half_a_turn() {
    use core::f32::consts::PI;
    assert!((wrap_angle(3.0 * PI / 2.0) + PI / 2.0).abs() < 1.0e-5);
    assert!((wrap_angle(-3.0 * PI / 2.0) - PI / 2.0).abs() < 1.0e-5);
    assert!(wrap_angle(0.25).abs() - 0.25 < 1.0e-6);
}

#[test]
fn vertical_target_falls_back_with_the_validity_of_the_estimate() {
    let hold = Setpoint {
        altitude: Some(Meters(500.0)),
        ..Setpoint::default()
    };
    let vertical_with = |valid: StateValidFlags, setpoint: &Setpoint| {
        let state = StateEstimate {
            valid_flags: valid,
            ..cruise_state()
        };
        flight(resolve(ControlMode::AltitudeHold, setpoint.clone(), &state)).1
    };
    let attitude_only = StateValidFlags::ATTITUDE | StateValidFlags::ANGULAR_RATE;

    assert_eq!(
        vertical_with(StateValidFlags::all(), &hold),
        Vertical::Height(500.0)
    );
    // Without a position the height is unknown: hold level flight.
    assert_eq!(
        vertical_with(attitude_only | StateValidFlags::VELOCITY, &hold),
        Vertical::ClimbRate(0.0)
    );
    // Without a velocity the climb rate is unknown too: hold the
    // airspeed, which needs neither.
    assert_eq!(
        vertical_with(attitude_only, &hold),
        Vertical::AirspeedOnPitch
    );
    let descend = Setpoint {
        vertical_speed: Some(MetersPerSecond(3.0)),
        ..Setpoint::default()
    };
    assert_eq!(
        vertical_with(attitude_only, &descend),
        Vertical::AirspeedOnPitch
    );
    assert_eq!(
        vertical_with(attitude_only | StateValidFlags::VELOCITY, &descend),
        Vertical::ClimbRate(-3.0)
    );

    // A direct pitch command needs no estimate beyond attitude.
    let state = StateEstimate {
        valid_flags: attitude_only,
        ..cruise_state()
    };
    let (_, vertical, ..) = flight(resolve(ControlMode::Attitude, Setpoint::default(), &state));
    assert_eq!(vertical, Vertical::Pitch(reference_params().trim_pitch.0));
}
