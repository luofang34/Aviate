//! Projection behavior: the origin latches once, axes point the right
//! way, and a fix the receiver does not claim never becomes a position.

use aviate_hal_xil::sim_types::SimGnssFix;

use world_core::{GeodeticPosition, LocalTangentPlane};

use super::NedOrigin;

/// A reference near Zurich, matching the SITL world the other backends
/// use, so distances here are comparable to theirs.
const LAT: f64 = 47.397_741_9;
const LON: f64 = 8.545_593_8;
const ALT: f32 = 488.0;

#[test]
fn the_first_usable_fix_becomes_the_origin() {
    let mut origin = NedOrigin::default();
    assert!(!origin.is_latched());
    let ned = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);
    assert!(origin.is_latched());
    assert_eq!(ned, [0.0, 0.0, 0.0], "the origin projects to zero");
}

#[test]
fn north_east_and_down_point_the_right_way() {
    let mut origin = NedOrigin::default();
    let _ = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);

    // One thousandth of a degree north is about 111 m.
    let north = origin.project(LAT + 0.001, LON, ALT, SimGnssFix::ThreeD);
    assert!(north[0] > 100.0 && north[0] < 120.0, "got {north:?}");
    assert!(north[1].abs() < 0.1, "pure north has no east component");

    let east = origin.project(LAT, LON + 0.001, ALT, SimGnssFix::ThreeD);
    assert!(east[1] > 60.0 && east[1] < 90.0, "got {east:?}");

    // Climbing REDUCES the down coordinate.
    let up = origin.project(LAT, LON, ALT + 50.0, SimGnssFix::ThreeD);
    assert!((up[2] + 50.0).abs() < 0.5, "got {up:?}");
}

#[test]
fn the_origin_never_re_latches() {
    let mut origin = NedOrigin::default();
    let _ = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);
    let moved = origin.project(LAT + 0.01, LON, ALT, SimGnssFix::ThreeD);
    assert!(moved[0] > 1000.0, "the vehicle moved away from the origin");
    // Projecting the ORIGIN again must return zero: a re-latch would
    // teleport the estimator to wherever the vehicle happened to be.
    let back = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);
    assert!(back[0].abs() < 0.5 && back[1].abs() < 0.5, "got {back:?}");
}

#[test]
fn a_fix_without_a_lock_never_latches_or_positions() {
    let mut origin = NedOrigin::default();
    for fix in [SimGnssFix::None, SimGnssFix::TwoD] {
        assert_eq!(origin.project(LAT, LON, ALT, fix), [0.0; 3]);
        assert!(
            !origin.is_latched(),
            "a fix the receiver does not claim must not define the frame"
        );
    }
    // The first 3D fix still latches normally afterward.
    let _ = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);
    assert!(origin.is_latched());
}

#[test]
fn a_non_finite_fix_is_refused() {
    let mut origin = NedOrigin::default();
    assert_eq!(
        origin.project(f64::NAN, LON, ALT, SimGnssFix::ThreeD),
        [0.0; 3]
    );
    assert_eq!(
        origin.project(LAT, LON, f32::INFINITY, SimGnssFix::ThreeD),
        [0.0; 3]
    );
    assert!(!origin.is_latched());
}

#[test]
fn rtk_locks_are_usable_fixes() {
    for fix in [SimGnssFix::RtkFloat, SimGnssFix::RtkFixed] {
        let mut origin = NedOrigin::default();
        let _ = origin.project(LAT, LON, ALT, fix);
        assert!(origin.is_latched(), "{fix:?} is a usable lock");
    }
}

#[test]
fn fixes_project_on_the_tangent_plane_at_the_latched_origin() {
    let mut origin = NedOrigin::default();
    let _ = origin.project(LAT, LON, ALT, SimGnssFix::ThreeD);
    let plane = LocalTangentPlane::new(GeodeticPosition::wgs84(LAT, LON, f64::from(ALT)));
    assert!(plane.is_ok(), "a well-formed origin");
    let Ok(plane) = plane else {
        return;
    };
    for (lat, lon, alt) in [
        (LAT + 0.001, LON, ALT),
        (LAT, LON + 0.01, ALT + 30.0),
        (LAT + 0.5, LON + 0.01, ALT),
        (LAT - 0.2, LON - 0.3, ALT - 100.0),
    ] {
        let got = origin.project(lat, lon, alt, SimGnssFix::ThreeD);
        let want = plane
            .to_ned(GeodeticPosition::wgs84(lat, lon, f64::from(alt)))
            .map(|n| [n.north_m as f32, n.east_m as f32, n.down_m as f32]);
        let got = Ok(got);
        assert_eq!(got, want, "{lat} {lon} {alt}");
    }
}
