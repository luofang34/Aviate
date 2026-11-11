//! Long flights move the navigation-frame origin to the vehicle. The
//! move changes coordinates, not the flight: attitude and height stay
//! right against the local level, the outputs do not step, and the
//! reported geodetic position is continuous.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_airframe_fixed_wing::c172;
use aviate_core::control::ControlMode;
use aviate_link::mavlink::{parse_mavlink_frame, MavMessage};
use aviate_xil_direct::{DirectNoiseTier, DirectSensorRates};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, Faults, Flight, PlantModel};

const CRUISE_MPS: f32 = 55.0;
const HEIGHT_M: f32 = 300.0;

fn c172_flight(noise: DirectNoiseTier) -> Flight {
    Flight::cruising_with(
        &c172(),
        PlantModel::C172,
        noise,
        DirectSensorRates::default().with_airspeed(40_000),
        Faults::default(),
    )
}

/// Worst roll, pitch and heading error \[deg\] and worst height error
/// \[m\] over `seconds` of a straight flight north.
fn fly_north(flight: &mut Flight, seconds: u32) -> ([f32; 3], f32) {
    let mut worst = [0.0_f32; 3];
    let mut height = 0.0_f32;
    for _ in 0..seconds {
        let seen = flight.fly(1.0, ControlMode::VelocityControl, |_| {
            course_speed_height(0.0, CRUISE_MPS, HEIGHT_M)
        });
        for (w, e) in worst.iter_mut().zip(seen.max_attitude_error) {
            *w = w.max(e.abs().to_degrees());
        }
        height = height.max((flight.plant.height() - HEIGHT_M).abs());
    }
    (worst, height)
}

#[test]
fn a_50_km_flight_keeps_attitude_and_height_against_the_local_level() {
    let mut flight = c172_flight(DirectNoiseTier::Industrial);
    let (worst, height) = fly_north(&mut flight, 910);
    let rebases = flight.board.kernel().state.estimator.frame.rebases;

    // The same flight with the origin never moved.
    let mut fixed = c172_flight(DirectNoiseTier::Industrial);
    fixed.board.set_rebase_radius(1.0e9);
    let (fixed_worst, fixed_height) = fly_north(&mut fixed, 910);
    println!(
        "50 km: {rebases} rebases, attitude {worst:?} deg, height {height:.2} m; \
         without rebases: attitude {fixed_worst:?} deg, height {fixed_height:.2} m"
    );
    assert!(flight.plant.pos_ned[0] > 49_000.0);
    assert!(rebases >= 45, "{rebases}");
    assert!(
        worst[0] < 1.0 && worst[1] < 1.0 && worst[2] < 3.0,
        "{worst:?}"
    );
    assert!(height < 2.0, "{height}");
    // Without the moves the plane leaves the local level by 0.45 degrees
    // and the ellipsoid by 196 m at 50 km.
    assert!(fixed_height > 10.0 * height.max(0.1), "{fixed_height}");
}

#[test]
fn outputs_do_not_step_when_the_origin_moves() {
    let mut moved = c172_flight(DirectNoiseTier::Off);
    let mut fixed = c172_flight(DirectNoiseTier::Off);
    fixed.board.set_rebase_radius(1.0e9);
    let command = |_: &flight::FixedWingPlant| course_speed_height(0.0, CRUISE_MPS, HEIGHT_M);
    // Up to the first move both flights are the same flight.
    let mut largest = 0.0_f32;
    let mut moved_at = None;
    for second in 0..40 {
        let _ = moved.fly(1.0, ControlMode::VelocityControl, command);
        let _ = fixed.fly(1.0, ControlMode::VelocityControl, command);
        if moved_at.is_none() && moved.board.kernel().state.estimator.frame.rebases > 0 {
            moved_at = Some(second);
        }
        for (a, b) in moved.last.outputs.iter().zip(fixed.last.outputs) {
            largest = largest.max((a - b).abs());
        }
    }
    println!("first move after {moved_at:?} s, largest output difference {largest:.5}");
    assert!(moved_at.is_some());
    // The plane curvature that the move removes is 0.08 m at 1 km.
    assert!(largest < 0.01, "{largest}");
}

#[test]
fn the_reported_position_is_continuous_across_moves() {
    let mut flight = c172_flight(DirectNoiseTier::Off);
    let mut last: Option<(i32, i32)> = None;
    let mut largest_m = 0.0_f64;
    let mut buf = [0_u8; 8192];
    for _ in 0..(60.0 / flight::DT_S) as u32 {
        flight.advance();
        let n = flight.board.pop_mavlink_tx(&mut buf);
        let mut cursor = 0;
        while cursor < n {
            let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("frame");
            cursor += parsed.consumed;
            if let MavMessage::GlobalPositionInt(position) = parsed.message {
                if let Some((lat, lon)) = last {
                    let north = f64::from(position.lat - lat) * 1e-7 * 111_000.0;
                    let east = f64::from(position.lon - lon) * 1e-7 * 86_000.0;
                    largest_m = largest_m.max(north.hypot(east));
                }
                last = Some((position.lat, position.lon));
            }
        }
    }
    let rebases = flight.board.kernel().state.estimator.frame.rebases;
    println!("{rebases} moves, largest step between position frames {largest_m:.2} m");
    assert!(rebases >= 2);
    // 10 Hz frames at 55 m/s are 5.5 m apart; a frame error would be
    // hundreds of metres.
    assert!(largest_m < 7.0, "{largest_m}");
}

#[test]
fn each_move_announces_the_new_origin() {
    let mut flight = c172_flight(DirectNoiseTier::Off);
    flight.board.set_geoid_separation(-33.4);
    let mut origins = Vec::new();
    let mut buf = [0_u8; 8192];
    for _ in 0..(45.0 / flight::DT_S) as u32 {
        flight.advance();
        let n = flight.board.pop_mavlink_tx(&mut buf);
        let mut cursor = 0;
        while cursor < n {
            let parsed = parse_mavlink_frame(&buf[cursor..n]).expect("frame");
            cursor += parsed.consumed;
            if let MavMessage::GpsGlobalOrigin(origin) = parsed.message {
                origins.push(origin);
            }
        }
    }
    let frame = flight.board.kernel().state.estimator.frame;
    let newest = frame.origin.expect("the origin moved");
    // One at the start and one per move, the last at the present origin.
    assert_eq!(origins.len() as u32, 1 + frame.rebases, "{origins:?}");
    let last = origins.last().expect("an origin");
    assert_eq!(last.latitude, (newest.latitude_deg * 1e7).round() as i32);
    assert_eq!(last.longitude, (newest.longitude_deg * 1e7).round() as i32);
}
