//! A long straight leg flies the commanded direction over the ground
//! across many moves of the frame origin.
//!
//! A leg to the east turns the frame at each move: the north of the new
//! origin is not the north of the old one (meridian convergence). The
//! estimate, the retained command and the host's next command must all
//! turn with it, or the aeroplane holds a direction that is off by the
//! accumulated turn. The legs here cross more than 20 moves, are flown
//! both on a course over the ground and on a heading (the magnetometer
//! path), and are compared with the same legs flown in a frame that
//! never moves.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};
use aviate_core::types::{Meters, Radians};
use aviate_xil_direct::{DirectNoiseTier, DirectOrigin, DirectSensorRates, DirectSynthesizer};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, wrap, Faults, Flight, PlantModel};

const CRUISE_MPS: f32 = 55.0;
const HEIGHT_M: f32 = 300.0;
/// Time to turn onto the leg and settle \[s\].
const SETTLE_S: u32 = 60;
/// Time over which the track is measured \[s\]: 23 km at cruise.
const MEASURE_S: u32 = 420;

/// How the host steers the leg.
#[derive(Clone, Copy, Debug)]
enum Steering {
    /// A velocity target: a course over the ground.
    Course,
    /// A height and a heading, as `SET_POSITION_TARGET_LOCAL_NED` with
    /// only `z` and `yaw` active.
    Heading,
}

struct Leg {
    direction_deg: f32,
    steering: Steering,
    moves_origin: bool,
    wind_ne: [f32; 2],
    geomagnetic: Option<GeomagneticReference>,
}

impl Leg {
    fn east(steering: Steering) -> Self {
        Self {
            direction_deg: 90.0,
            steering,
            moves_origin: true,
            wind_ne: [0.0; 2],
            geomagnetic: None,
        }
    }
}

/// What a leg flew: the mean direction of the true velocity over the
/// ground minus the commanded direction \[deg\], and the number of moves.
struct Flown {
    track_error_deg: f32,
    rebases: u32,
}

fn fly(leg: &Leg) -> Flown {
    let mut flight = Flight::cruising_with(
        &c172(),
        PlantModel::C172,
        DirectNoiseTier::Industrial,
        DirectSensorRates::default().with_airspeed(40_000),
        Faults {
            wind_ne: leg.wind_ne,
            ..Faults::default()
        },
    );
    flight.geomagnetic = leg.geomagnetic;
    if !leg.moves_origin {
        flight.board.set_rebase_radius(1.0e9);
    }
    let direction = leg.direction_deg.to_radians();
    let (mode, setpoint) = match leg.steering {
        Steering::Course => (
            ControlMode::VelocityControl,
            course_speed_height(direction, CRUISE_MPS, HEIGHT_M),
        ),
        Steering::Heading => (
            ControlMode::AltitudeHold,
            Setpoint {
                heading: Some(Radians(direction)),
                altitude: Some(Meters(HEIGHT_M)),
                ..Setpoint::default()
            },
        ),
    };
    let _ = flight.fly(SETTLE_S as f32, mode, |_| setpoint.clone());
    let mut sum = 0.0_f32;
    for _ in 0..MEASURE_S {
        let _ = flight.fly(1.0, mode, |_| setpoint.clone());
        let [north, east, _] = flight.plant.ground_velocity_ned();
        sum += wrap(east.atan2(north) - direction);
    }
    Flown {
        track_error_deg: (sum / MEASURE_S as f32).to_degrees(),
        rebases: flight.board.navigation_frame().rebases,
    }
}

fn check(name: &str, leg: &Leg, limit_deg: f32) -> Flown {
    let flown = fly(leg);
    println!(
        "{name}: track error {:.3} deg over {} moves",
        flown.track_error_deg, flown.rebases
    );
    if leg.moves_origin {
        assert!(flown.rebases >= 20, "{name}: {} moves", flown.rebases);
    } else {
        assert_eq!(flown.rebases, 0, "{name}");
    }
    assert!(
        flown.track_error_deg.abs() < limit_deg,
        "{name}: {:.3} deg",
        flown.track_error_deg
    );
    flown
}

#[test]
fn an_east_course_holds_across_moves_of_the_origin() {
    let moved = check("course east", &Leg::east(Steering::Course), 0.5);
    let fixed = check(
        "course east, fixed origin",
        &Leg {
            moves_origin: false,
            ..Leg::east(Steering::Course)
        },
        0.5,
    );
    // The moves change coordinates, not the flight.
    assert!(
        (moved.track_error_deg - fixed.track_error_deg).abs() < 0.3,
        "{} vs {}",
        moved.track_error_deg,
        fixed.track_error_deg
    );
}

#[test]
fn an_east_heading_holds_across_moves_of_the_origin() {
    let moved = check("heading east", &Leg::east(Steering::Heading), 0.5);
    let fixed = check(
        "heading east, fixed origin",
        &Leg {
            moves_origin: false,
            ..Leg::east(Steering::Heading)
        },
        0.5,
    );
    assert!(
        (moved.track_error_deg - fixed.track_error_deg).abs() < 0.3,
        "{} vs {}",
        moved.track_error_deg,
        fixed.track_error_deg
    );
}

#[test]
fn a_north_east_course_in_a_crosswind_holds_across_moves() {
    // 10 m/s from the north-west: across the leg.
    let leg = Leg {
        direction_deg: 45.0,
        wind_ne: [-7.07, 7.07],
        ..Leg::east(Steering::Course)
    };
    let _ = check("course north-east, crosswind", &leg, 0.5);
}

/// The heading is true: the board adds the declination to the magnetic
/// heading. A host that supplies a declination of zero where the field
/// has one turns the leg toward magnetic north. GNSS velocity and the
/// airspeed take back part of the turn through the wind states, so the
/// leg is off by a part of the declination, on its side. This shows that
/// the tests above would see a lost declination.
#[test]
fn a_heading_flown_without_the_declination_is_turned_toward_magnetic_north() {
    let leg = Leg {
        geomagnetic: Some(GeomagneticReference {
            declination: Radians(0.0),
            source: GeomagneticSource::Configured,
        }),
        ..Leg::east(Steering::Heading)
    };
    let flown = fly(&leg);
    let config = *Flight::c172_with_probe(DirectNoiseTier::Off).board.config();
    let declination_deg = DirectSynthesizer::new(
        DirectOrigin::new(config.origin, config.epoch),
        DirectSensorRates::default(),
        DirectNoiseTier::Off,
        config.noise_seed,
    )
    .expect("host sensor models")
    .environment()
    .declination_rad()
    .to_degrees();
    println!(
        "heading east, declination withheld: track error {:.3} deg, declination {declination_deg:.3} deg",
        flown.track_error_deg
    );
    assert!(declination_deg.abs() > 5.0, "{declination_deg}");
    let toward = flown.track_error_deg * declination_deg.signum();
    assert!(
        toward > 2.0 && toward < declination_deg.abs() + 1.0,
        "{} vs {declination_deg}",
        flown.track_error_deg
    );
}
