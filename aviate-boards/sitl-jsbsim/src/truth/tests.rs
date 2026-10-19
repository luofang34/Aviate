//! Truth conversion and world-generation tests.

use super::*;
use world_core::Ned;

const LAT: f64 = 47.397_741_9;
const LON: f64 = 8.545_593_8;
const ALT_M: f64 = 488.0;

fn plane() -> Option<LocalTangentPlane> {
    LocalTangentPlane::new(GeodeticPosition::wgs84(LAT, LON, ALT_M)).ok()
}

fn state(time_usec: u64, position: GeodeticPosition) -> HilStateQuaternion {
    HilStateQuaternion {
        time_usec,
        attitude_quaternion: [0.5, 0.5, -0.5, 0.5],
        lat: (position.latitude_deg * 1e7).round() as i32,
        lon: (position.longitude_deg * 1e7).round() as i32,
        alt: (position.ellipsoidal_height_m * 1e3).round() as i32,
        ..HilStateQuaternion::default()
    }
}

#[test]
fn a_session_keeps_its_generation_and_counts_its_steps() {
    let mut tracker = GenerationTracker::default();
    assert_eq!(tracker.observe(1, 4_000), (0, 0));
    assert_eq!(tracker.observe(1, 8_000), (0, 1));
    // A repeated time is the same world, not a reset.
    assert_eq!(tracker.observe(1, 8_000), (0, 2));
}

#[test]
fn rewound_time_or_a_new_session_starts_a_new_generation() {
    let mut tracker = GenerationTracker::default();
    tracker.observe(1, 4_000);
    tracker.observe(1, 8_000);
    assert_eq!(tracker.observe(1, 4_000), (1, 0), "time ran backwards");
    assert_eq!(tracker.observe(1, 8_000), (1, 1));
    assert_eq!(tracker.observe(2, 12_000), (2, 0), "a new session");
}

#[test]
fn the_state_position_is_its_tangent_plane_offset_from_the_origin() {
    let Some(plane) = plane() else {
        return;
    };
    assert!(plane.origin().is_well_formed());
    let offset = Ned {
        north_m: 120.0,
        east_m: -45.0,
        down_m: -30.0,
    };
    let there = plane.to_geodetic(offset);
    let truth = truth_sample(&state(8_000, there), &plane, 3, 7);
    assert!(truth.is_some());
    let Some(truth) = truth else {
        return;
    };
    // Lat/lon in degE7 and altitude in mm quantize the position to
    // about a centimeter.
    let [n, e, d] = truth.position_ned_m;
    assert!((n - 120.0).abs() < 0.02 && (e + 45.0).abs() < 0.02 && (d + 30.0).abs() < 0.002);
    assert_eq!(truth.time_us, 8_000);
    assert_eq!((truth.reset_generation, truth.step), (3, 7));
    assert_eq!(truth.attitude, [0.5, 0.5, -0.5, 0.5]);
}

#[test]
fn the_origin_state_is_the_local_origin() {
    let Some(plane) = plane() else {
        return;
    };
    let truth = truth_sample(&state(4_000, plane.origin()), &plane, 0, 0);
    let ned = truth.map(|t| t.position_ned_m);
    assert!(ned.is_some_and(|[n, e, d]| n.abs() < 0.01 && e.abs() < 0.01 && d.abs() < 0.001));
}

#[test]
fn the_states_own_kinematics_travel_with_the_sample() {
    let Some(plane) = plane() else {
        return;
    };
    let mut with_kinematics = state(4_000, plane.origin());
    with_kinematics.vx = 150;
    with_kinematics.vy = -20;
    with_kinematics.vz = -35;
    with_kinematics.rollspeed = 0.1;
    with_kinematics.pitchspeed = -0.2;
    with_kinematics.yawspeed = 0.3;
    with_kinematics.xacc = 12;
    with_kinematics.yacc = -4;
    with_kinematics.zacc = -1_000;
    let measured = truth_sample(&with_kinematics, &plane, 0, 0).and_then(|t| t.measured);
    assert!(measured.is_some());
    let Some(measured) = measured else {
        return;
    };
    assert_eq!(measured.velocity_ned_mps, [1.5, -0.2, -0.35]);
    assert_eq!(measured.body_rates_rps, [0.1, -0.2, 0.3]);
    // Milli-g: -1000 mG is one standard gravity, upward in FRD.
    let [fx, fy, fz] = measured.specific_force_body_mps2;
    assert!((fx - 0.117_68).abs() < 1e-4 && (fy + 0.039_23).abs() < 1e-4);
    assert!((fz + 9.806_65).abs() < 1e-4);
}
