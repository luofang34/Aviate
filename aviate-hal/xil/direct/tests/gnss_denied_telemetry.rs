//! What a host reads of a fixed wing when GNSS is denied: the airspeed
//! in `VFR_HUD` (launch detection without a position fix), and the
//! height in `LOCAL_POSITION_NED.z`, which the HEIGHT bit of
//! `AVIATE_ESTIMATOR_STATUS` vouches for while POSITION and VELOCITY do
//! not.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_core::control::ControlMode;
use aviate_link::mavlink::protocol::{aviate_state_valid_flags, VfrHud};
use aviate_link::mavlink::{parse_mavlink_frame, MavMessage};
use aviate_xil_direct::DirectNoiseTier;

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, Flight};

/// Geoid height above the ellipsoid at the origin \[m\].
const GEOID_M: f64 = -33.4;
/// The origin is 15 m above the ellipsoid; the vehicle flies 300 m above it.
const MSL_M: f32 = 15.0 + 300.0 + 33.4;

#[derive(Default, Debug)]
struct Seen {
    hud: Option<VfrHud>,
    huds: u32,
    local_z: Vec<f32>,
    valid_flags: Option<u8>,
}

/// Fly `seconds` on course north at 55 m/s and 300 m, reading telemetry.
fn fly(flight: &mut Flight, seconds: f32) -> Seen {
    let mut seen = Seen::default();
    let mut buf = [0_u8; 8192];
    // Only what this segment sends.
    while flight.board.pop_mavlink_tx(&mut buf) > 0 {}
    for _ in 0..(seconds * 10.0).round() as u32 {
        let _ = flight.fly(0.1, ControlMode::VelocityControl, |_| {
            course_speed_height(0.0, 55.0, 300.0)
        });
        let n = flight.board.pop_mavlink_tx(&mut buf);
        let mut cursor = 0;
        while cursor < n {
            let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("frame");
            cursor += parsed.consumed;
            match parsed.message {
                MavMessage::VfrHud(hud) => {
                    seen.hud = Some(hud);
                    seen.huds += 1;
                }
                MavMessage::LocalPositionNed(local) => seen.local_z.push(local.z),
                MavMessage::AviateEstimatorStatus(status) => {
                    seen.valid_flags = Some(status.valid_flags);
                }
                _ => {}
            }
        }
    }
    seen
}

#[test]
fn with_gnss_the_hud_has_airspeed_groundspeed_and_sea_level_altitude() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    flight.board.set_geoid_separation(GEOID_M);
    let seen = fly(&mut flight, 10.0);
    let hud = seen.hud.expect("VFR_HUD");
    println!("{hud:?}, {} frames in 10 s", seen.huds);
    assert!((39..=41).contains(&seen.huds), "4 Hz: {}", seen.huds);
    assert!((hud.airspeed - 55.0).abs() < 1.5, "{hud:?}");
    assert!(
        (hud.groundspeed - flight.plant.airspeed).abs() < 1.5,
        "{hud:?}"
    );
    assert!((hud.alt - MSL_M).abs() < 2.0, "{hud:?}");
    assert!(hud.climb.abs() < 0.5 && (hud.heading == 0 || hud.heading >= 358 || hud.heading <= 2));
    assert!((1..=100).contains(&hud.throttle), "{hud:?}");
}

#[test]
fn with_gnss_denied_the_height_and_the_airspeed_still_go_out() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    flight.board.set_geoid_separation(GEOID_M);
    let _ = fly(&mut flight, 5.0);
    flight.gnss_silent = true;
    // Position and velocity validity lapse 5 s after the last fix.
    let _ = fly(&mut flight, 8.0);
    let seen = fly(&mut flight, 5.0);

    let flags = seen.valid_flags.expect("AVIATE_ESTIMATOR_STATUS");
    assert_ne!(flags & aviate_state_valid_flags::HEIGHT, 0, "{flags:#04x}");
    assert_eq!(
        flags & aviate_state_valid_flags::POSITION,
        0,
        "{flags:#04x}"
    );
    assert_eq!(
        flags & aviate_state_valid_flags::VELOCITY,
        0,
        "{flags:#04x}"
    );
    // LOCAL_POSITION_NED keeps coming at 10 Hz with a meaningful z.
    assert!(seen.local_z.len() >= 45, "{}", seen.local_z.len());
    assert!(
        seen.local_z.iter().all(|z| (z + 300.0).abs() < 3.0),
        "{:?}",
        seen.local_z
    );
    let hud = seen.hud.expect("VFR_HUD");
    assert!((hud.airspeed - 55.0).abs() < 1.5, "{hud:?}");
    // No vouched velocity: the groundspeed convention is 0.
    assert_eq!(hud.groundspeed, 0.0);
    assert!((hud.alt - MSL_M).abs() < 3.0, "{hud:?}");
}

#[test]
fn without_a_geoid_separation_the_hud_altitude_is_not_stated() {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let hud = fly(&mut flight, 2.0).hud.expect("VFR_HUD");
    assert!(hud.alt.is_nan(), "{hud:?}");
    assert!((hud.airspeed - 55.0).abs() < 1.5, "{hud:?}");
}
