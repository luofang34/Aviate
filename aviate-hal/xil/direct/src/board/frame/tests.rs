//! The frame map and the retained-command move.

use super::{rebase_command, shift_between};
use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::ekf::rebase::NavOrigin;
use aviate_core::types::{Meters, MetersPerSecond, Radians};

const HEIGHT_M: f64 = 15.0;

fn origin(latitude_deg: f64, longitude_deg: f64) -> NavOrigin {
    NavOrigin {
        latitude_deg,
        longitude_deg,
    }
}

#[test]
fn the_new_origin_maps_to_zero_and_the_rotation_is_orthonormal() {
    let from = origin(39.1582, -75.5244);
    let to = origin(39.1672, -75.5144);
    let shift = shift_between(&from, &to, HEIGHT_M).expect("shift");
    let at_origin = shift.position(shift.offset_m);
    assert!(at_origin.iter().all(|c| c.abs() < 1e-9), "{at_origin:?}");
    let r = shift.rotation;
    for i in 0..3 {
        for j in 0..3 {
            let dot: f64 = (0..3).map(|k| r[i][k] * r[j][k]).sum();
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((dot - expected).abs() < 1e-12);
        }
    }
    // About 1 km north-east: the old plane lies 0.08 m above the datum there.
    assert!(
        (shift.offset_m[2] - 0.12).abs() < 0.05,
        "{:?}",
        shift.offset_m
    );
}

#[test]
fn a_retained_command_moves_with_the_frame_and_keeps_its_height() {
    let from = origin(39.1582, -75.5244);
    let to = origin(39.1672, -75.5144);
    let shift = shift_between(&from, &to, HEIGHT_M).expect("shift");
    let target = shift.offset_m;
    let mut command = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            position: Some(target.map(|c| Meters(c as f32))),
            velocity: Some([
                MetersPerSecond(10.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            heading: Some(Radians(0.0)),
            altitude: Some(Meters(300.0)),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Gcs,
    };
    rebase_command(&mut command, &shift);
    let setpoint = command.setpoint;
    // The target at the new origin is the new origin.
    let position = setpoint.position.expect("position");
    assert!(position.iter().all(|c| c.0.abs() < 1e-3), "{position:?}");
    // North turns by the convergence of the meridians: about 0.006 degrees.
    let turn = shift.heading_change();
    assert!(turn.abs() < 1e-3 && turn != 0.0, "{turn}");
    assert!((f64::from(setpoint.heading.expect("heading").0) - turn).abs() < 1e-7);
    let speed = setpoint.velocity.expect("velocity").map(|v| v.0);
    assert!((speed.iter().map(|v| v * v).sum::<f32>().sqrt() - 10.0).abs() < 1e-4);
    assert_eq!(setpoint.altitude, Some(Meters(300.0)));
}
