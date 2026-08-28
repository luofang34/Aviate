//! Evaluate mission criteria from simulator frames.

use crate::mission::{Criterion, CriterionResult};
use crate::SimulatorBackend;

use super::{MissionRunner, TraceSample};

impl<B: SimulatorBackend> MissionRunner<B> {
    pub(super) fn verify_criterion(
        &self,
        criterion: &Criterion,
        phase_max_altitude: f32,
        trace: &[TraceSample],
    ) -> CriterionResult {
        match criterion {
            Criterion::Armed(expected) => boolean_result("armed", self.armed, *expected),
            Criterion::MinAltitude(minimum) => {
                minimum_result("min_altitude", phase_max_altitude, *minimum)
            }
            Criterion::MaxAltitude(maximum) => {
                maximum_result("max_altitude", -self.current_state.position[2], *maximum)
            }
            Criterion::AltitudeHold { target, tolerance } => {
                altitude_hold(-self.current_state.position[2], *target, *tolerance)
            }
            Criterion::PositionHold { target, tolerance } => position_result(
                "position_hold",
                self.current_state.position,
                *target,
                *tolerance,
            ),
            Criterion::MaxDrift(maximum) => {
                max_drift(self.current_state.position, self.start_position, *maximum)
            }
            Criterion::SensorDataReceived => sensor_data_received(self.last_step),
            Criterion::VelocityTracksPosition { tolerance } => {
                velocity_tracks_position(trace, *tolerance)
            }
            Criterion::ReachedWaypoint { target, tolerance } => {
                reached_waypoint(trace, *target, *tolerance)
            }
            Criterion::StableHover {
                altitude,
                tolerance,
                hold_secs,
            } => stable_hover(trace, *altitude, *tolerance, *hold_secs),
            Criterion::StationKeeping {
                center_ned,
                xy_tolerance,
                z_tolerance,
            } => station_keeping(trace, *center_ned, *xy_tolerance, *z_tolerance),
            Criterion::MaxExcursion {
                center_ned,
                xy_max,
                z_max,
            } => max_excursion(trace, *center_ned, *xy_max, *z_max),
            Criterion::TrajectoryTracking {
                waypoints,
                tolerance,
                max_time_s,
            } => trajectory_tracking(trace, waypoints, *tolerance, *max_time_s),
            Criterion::ReturnedNear {
                target_ned,
                tolerance,
            } => position_result(
                "returned_near",
                self.current_state.position,
                *target_ned,
                *tolerance,
            ),
            Criterion::AttitudeBounded { roll_pitch_max_deg } => {
                attitude_bounded(trace, *roll_pitch_max_deg)
            }
            Criterion::TouchdownVelocity {
                max_descent_mps,
                ground_tolerance,
            } => touchdown_velocity(trace, *max_descent_mps, *ground_tolerance),
        }
    }
}

fn boolean_result(name: &str, actual: bool, expected: bool) -> CriterionResult {
    CriterionResult {
        criterion: name.to_owned(),
        passed: actual == expected,
        actual_value: actual.to_string(),
        expected: expected.to_string(),
    }
}

fn minimum_result(name: &str, actual: f32, minimum: f32) -> CriterionResult {
    CriterionResult {
        criterion: name.to_owned(),
        passed: actual >= minimum,
        actual_value: format!("{actual:.2}m"),
        expected: format!(">= {minimum:.2}m"),
    }
}

fn maximum_result(name: &str, actual: f32, maximum: f32) -> CriterionResult {
    CriterionResult {
        criterion: name.to_owned(),
        passed: actual <= maximum,
        actual_value: format!("{actual:.2}m"),
        expected: format!("<= {maximum:.2}m"),
    }
}

fn altitude_hold(actual: f32, target: f32, tolerance: f32) -> CriterionResult {
    let error = (actual - target).abs();
    CriterionResult {
        criterion: "altitude_hold".to_owned(),
        passed: error <= tolerance,
        actual_value: format!("{actual:.2}m (error: {error:.2}m)"),
        expected: format!("{target:.2}m +/- {tolerance:.2}m"),
    }
}

fn position_result(
    name: &str,
    actual: [f32; 3],
    target: [f32; 3],
    tolerance: f32,
) -> CriterionResult {
    let error = distance(actual, target);
    CriterionResult {
        criterion: name.to_owned(),
        passed: error <= tolerance,
        actual_value: format!("error: {error:.2}m"),
        expected: format!("<= {tolerance:.2}m of {target:?}"),
    }
}

fn max_drift(actual: [f32; 3], start: [f32; 3], maximum: f32) -> CriterionResult {
    let north = actual[0] - start[0];
    let east = actual[1] - start[1];
    maximum_result("max_drift", north.hypot(east), maximum)
}

fn sensor_data_received(last_step: u64) -> CriterionResult {
    CriterionResult {
        criterion: "sensor_data".to_owned(),
        passed: last_step > 0,
        actual_value: format!("{last_step} steps"),
        expected: "> 0 steps".to_owned(),
    }
}

pub(super) fn velocity_tracks_position(trace: &[TraceSample], tolerance: f32) -> CriterionResult {
    const MOVING_MPS: f32 = 0.5;
    let mut error_sum = 0.0_f32;
    let mut derived_sum = 0.0_f32;
    let mut published_sum = 0.0_f32;
    let mut moving = 0_u32;

    for pair in trace.windows(2) {
        let elapsed_us = pair[1].sim_time_us.saturating_sub(pair[0].sim_time_us);
        let elapsed = elapsed_us as f32 * 1e-6;
        if elapsed <= 0.0 {
            continue;
        }
        let derived = (0..3)
            .map(|axis| {
                let speed = (pair[1].position[axis] - pair[0].position[axis]) / elapsed;
                speed * speed
            })
            .sum::<f32>()
            .sqrt();
        if derived < MOVING_MPS {
            continue;
        }
        let published = pair[1]
            .velocity
            .iter()
            .map(|speed| speed * speed)
            .sum::<f32>()
            .sqrt();
        error_sum += (published - derived).abs();
        published_sum += published;
        derived_sum += derived;
        moving = moving.wrapping_add(1);
    }

    if moving == 0 {
        return CriterionResult {
            criterion: "velocity_tracks_position".to_owned(),
            passed: false,
            actual_value: "the vehicle never moved".to_owned(),
            expected: format!("motion above {MOVING_MPS:.2}m/s to compare against"),
        };
    }

    let count = moving as f32;
    let derived = derived_sum / count;
    let published = published_sum / count;
    let error = error_sum / count;
    CriterionResult {
        criterion: "velocity_tracks_position".to_owned(),
        passed: error <= tolerance,
        actual_value: format!(
            "published {published:.2}m/s vs {derived:.2}m/s from position \
             (error {error:.2}m/s over {moving} samples)"
        ),
        expected: format!("<= {tolerance:.2}m/s"),
    }
}

fn reached_waypoint(trace: &[TraceSample], target: [f32; 3], tolerance: f32) -> CriterionResult {
    let minimum_error = trace
        .iter()
        .map(|sample| distance(sample.position, target))
        .fold(f32::INFINITY, f32::min);
    CriterionResult {
        criterion: "reached_waypoint".to_owned(),
        passed: minimum_error <= tolerance,
        actual_value: format!("minimum error: {minimum_error:.2}m"),
        expected: format!("<= {tolerance:.2}m at one sample"),
    }
}

fn stable_hover(
    trace: &[TraceSample],
    altitude: f32,
    tolerance: f32,
    hold_seconds: f32,
) -> CriterionResult {
    let mut best_run = 0.0_f32;
    let mut run_start = None;
    for sample in trace {
        if (-sample.position[2] - altitude).abs() <= tolerance {
            let start = *run_start.get_or_insert(sample.elapsed);
            best_run = best_run.max(sample.elapsed - start);
        } else {
            run_start = None;
        }
    }
    CriterionResult {
        criterion: "stable_hover".to_owned(),
        passed: best_run >= hold_seconds,
        actual_value: format!("best continuous interval: {best_run:.2}s"),
        expected: format!(">= {hold_seconds:.2}s in the altitude band"),
    }
}

fn station_keeping(
    trace: &[TraceSample],
    center: [f32; 3],
    xy_tolerance: f32,
    z_tolerance: f32,
) -> CriterionResult {
    let (worst_xy, worst_z, worst_time) = worst_excursion(trace, center);
    CriterionResult {
        criterion: "station_keeping".to_owned(),
        passed: !trace.is_empty() && worst_xy <= xy_tolerance && worst_z <= z_tolerance,
        actual_value: format!("worst xy={worst_xy:.2}m z={worst_z:.2}m at t={worst_time:.2}s"),
        expected: format!("all samples have xy<={xy_tolerance:.2}m and z<={z_tolerance:.2}m"),
    }
}

fn max_excursion(
    trace: &[TraceSample],
    center: [f32; 3],
    xy_maximum: f32,
    z_maximum: f32,
) -> CriterionResult {
    let (worst_xy, worst_z, _) = worst_excursion(trace, center);
    CriterionResult {
        criterion: "max_excursion".to_owned(),
        passed: worst_xy <= xy_maximum && worst_z <= z_maximum,
        actual_value: format!("xy={worst_xy:.2}m z={worst_z:.2}m"),
        expected: format!("xy<={xy_maximum:.2}m and z<={z_maximum:.2}m"),
    }
}

fn worst_excursion(trace: &[TraceSample], center: [f32; 3]) -> (f32, f32, f32) {
    let mut worst_xy = 0.0_f32;
    let mut worst_z = 0.0_f32;
    let mut worst_time = 0.0_f32;
    for sample in trace {
        let north = sample.position[0] - center[0];
        let east = sample.position[1] - center[1];
        let xy = north.hypot(east);
        if xy > worst_xy {
            worst_xy = xy;
            worst_time = sample.elapsed;
        }
        worst_z = worst_z.max((sample.position[2] - center[2]).abs());
    }
    (worst_xy, worst_z, worst_time)
}

fn trajectory_tracking(
    trace: &[TraceSample],
    waypoints: &[[f32; 3]],
    tolerance: f32,
    max_time: f32,
) -> CriterionResult {
    let mut visited = 0usize;
    let mut visit_time = None;
    for sample in trace.iter().take_while(|sample| sample.elapsed <= max_time) {
        let Some(waypoint) = waypoints.get(visited) else {
            break;
        };
        if distance(sample.position, *waypoint) <= tolerance {
            visited = visited.wrapping_add(1);
            visit_time = Some(sample.elapsed);
        }
    }
    CriterionResult {
        criterion: "trajectory_tracking".to_owned(),
        passed: visited == waypoints.len(),
        actual_value: format!("visited {visited}/{} at {visit_time:?}", waypoints.len()),
        expected: format!("all waypoints within {tolerance:.2}m before {max_time:.2}s"),
    }
}

fn attitude_bounded(trace: &[TraceSample], maximum_degrees: f32) -> CriterionResult {
    let mut worst_roll = 0.0_f32;
    let mut worst_pitch = 0.0_f32;
    for sample in trace {
        let (roll, pitch, _) = quat_to_rpy(sample.attitude);
        if roll.abs() > worst_roll.abs() {
            worst_roll = roll;
        }
        if pitch.abs() > worst_pitch.abs() {
            worst_pitch = pitch;
        }
    }
    let maximum = maximum_degrees.to_radians();
    CriterionResult {
        criterion: "attitude_bounded".to_owned(),
        passed: !trace.is_empty() && worst_roll.abs() <= maximum && worst_pitch.abs() <= maximum,
        actual_value: format!(
            "worst roll={:.1} degrees and pitch={:.1} degrees",
            worst_roll.to_degrees(),
            worst_pitch.to_degrees()
        ),
        expected: format!("absolute roll and pitch <= {maximum_degrees:.1} degrees"),
    }
}

fn touchdown_velocity(
    trace: &[TraceSample],
    max_descent: f32,
    ground_tolerance: f32,
) -> CriterionResult {
    let touchdown = trace
        .iter()
        .position(|sample| sample.position[2] >= -ground_tolerance);
    let Some(index) = touchdown else {
        return touchdown_missing(trace, ground_tolerance);
    };
    if index == 0 {
        return touchdown_at_start(ground_tolerance);
    }
    let current = &trace[index];
    let previous = touchdown_lookback(&trace[..index], current);
    let elapsed = (current.elapsed - previous.elapsed).max(1e-3);
    let descent = (current.position[2] - previous.position[2]) / elapsed;
    CriterionResult {
        criterion: "touchdown_velocity".to_owned(),
        passed: descent <= max_descent,
        actual_value: format!("descent={descent:.2}m/s at t={:.2}s", current.elapsed),
        expected: format!("descent <= {max_descent:.2}m/s near the ground"),
    }
}

fn touchdown_lookback<'a>(trace: &'a [TraceSample], current: &TraceSample) -> &'a TraceSample {
    trace
        .iter()
        .rev()
        .take_while(|sample| current.elapsed - sample.elapsed < 0.1)
        .min_by(|left, right| left.position[2].total_cmp(&right.position[2]))
        .unwrap_or(&trace[trace.len() - 1])
}

fn touchdown_at_start(ground_tolerance: f32) -> CriterionResult {
    CriterionResult {
        criterion: "touchdown_velocity".to_owned(),
        passed: false,
        actual_value: "the vehicle is on the ground at phase start".to_owned(),
        expected: format!("the vehicle reaches {ground_tolerance:.2}m during the phase"),
    }
}

fn touchdown_missing(trace: &[TraceSample], ground_tolerance: f32) -> CriterionResult {
    let minimum_altitude = trace
        .iter()
        .map(|sample| -sample.position[2])
        .fold(f32::INFINITY, f32::min);
    CriterionResult {
        criterion: "touchdown_velocity".to_owned(),
        passed: false,
        actual_value: format!("no touchdown sample; minimum altitude={minimum_altitude:.2}m"),
        expected: format!("the vehicle reaches {ground_tolerance:.2}m"),
    }
}

fn distance(left: [f32; 3], right: [f32; 3]) -> f32 {
    let north = left[0] - right[0];
    let east = left[1] - right[1];
    let down = left[2] - right[2];
    (north * north + east * east + down * down).sqrt()
}

pub(super) fn quat_to_rpy(quaternion: [f32; 4]) -> (f32, f32, f32) {
    let [w, x, y, z] = quaternion;
    let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
    let pitch_term = 2.0 * (w * y - z * x);
    let pitch = if pitch_term.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(pitch_term)
    } else {
        pitch_term.asin()
    };
    let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
    (roll, pitch, yaw)
}
