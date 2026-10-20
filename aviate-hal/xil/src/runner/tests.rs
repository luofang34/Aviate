//! Runner contract tests.
#![allow(clippy::expect_used, clippy::panic)]

use super::*;
use crate::SimulatorError;

mod session;

use super::criteria::velocity_tracks_position;
use super::TraceSample;

fn velocity_sample(elapsed: f32, north: f32, speed: f32) -> TraceSample {
    TraceSample {
        elapsed,
        sim_time_us: (elapsed * 1e6) as u64,
        position: [north, 0.0, -5.0],
        velocity: [speed, 0.0, 0.0],
        attitude: [1.0, 0.0, 0.0, 0.0],
        angular_velocity: [0.0; 3],
    }
}

#[test]
fn velocity_lane_matches_position_derivative() {
    let trace: Vec<_> = (0..20)
        .map(|index| velocity_sample(index as f32 * 0.1, index as f32 * 0.3, 3.0))
        .collect();
    assert!(velocity_tracks_position(&trace, 1.0).passed);
}

#[test]
fn dead_velocity_lane_fails_position_derivative_check() {
    let trace: Vec<_> = (0..20)
        .map(|index| velocity_sample(index as f32 * 0.1, index as f32 * 0.3, 0.0))
        .collect();
    let result = velocity_tracks_position(&trace, 1.0);
    assert!(!result.passed);
    assert!(result.actual_value.contains("3.00m/s"));
}

/// Positions stated to a centimeter, sampled every 4 ms at 2 m/s: a
/// step moves 8 mm, so consecutive samples differ by 0 or 1 cm. The
/// lane that states the true 2 m/s still tracks the position.
#[test]
fn velocity_lane_tracks_positions_quantized_finer_than_a_step() {
    let trace: Vec<_> = (0..500)
        .map(|index| {
            let elapsed = index as f32 * 0.004;
            let north = (elapsed * 2.0 * 100.0).floor() / 100.0;
            velocity_sample(elapsed, north, 2.0)
        })
        .collect();
    let result = velocity_tracks_position(&trace, 0.25);
    assert!(result.passed, "{}", result.actual_value);
    let dead: Vec<_> = trace
        .iter()
        .map(|sample| TraceSample {
            velocity: [0.0; 3],
            ..*sample
        })
        .collect();
    assert!(!velocity_tracks_position(&dead, 0.25).passed);
}

#[test]
fn stationary_trace_cannot_prove_velocity_lane_health() {
    let trace: Vec<_> = (0..20)
        .map(|index| velocity_sample(index as f32 * 0.1, 0.0, 0.0))
        .collect();
    let result = velocity_tracks_position(&trace, 1.0);
    assert!(!result.passed);
    assert!(result.actual_value.contains("never moved"));
}

#[test]
fn test_vehicle_state_default() {
    let state = VehicleState::default();
    assert!(!state.valid);
    assert_eq!(state.position, [0.0; 3]);
}

#[test]
fn test_simulator_error_display() {
    let err = SimulatorError::ConnectionFailed {
        backend: "test".to_owned(),
        detail: "test failure".to_owned(),
    };
    assert!(err.to_string().contains("test"));
}
