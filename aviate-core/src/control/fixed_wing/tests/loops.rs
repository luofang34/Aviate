//! The guidance, energy, attitude and rate loops through the controller step.

use super::{attitude, command, cruise_state, Rig, DT};
use crate::control::{Command, ControlMode, Setpoint};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::state::{StateEstimate, StateValidFlags};
use crate::types::{Meters, MetersPerSecond, NormalizedThrust, Radians, RadiansPerSecond};

fn course_command(course: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0 * course.cos()),
                MetersPerSecond(55.0 * course.sin()),
                MetersPerSecond(0.0),
            ]),
            ..Setpoint::default()
        },
    )
}

fn height_command(height: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            altitude: Some(Meters(height)),
            ..Setpoint::default()
        },
    )
}

fn climb_command(climb: f32) -> Command {
    command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(55.0),
                MetersPerSecond(0.0),
                MetersPerSecond(-climb),
            ]),
            ..Setpoint::default()
        },
    )
}

fn attitude_command(roll: f32, pitch: f32, throttle: f32) -> Command {
    command(
        ControlMode::Attitude,
        Setpoint {
            attitude: Some(attitude(roll, pitch, 0.0)),
            collective_thrust: NormalizedThrust(throttle),
            ..Setpoint::default()
        },
    )
}

fn rate_command(roll: f32, pitch: f32, yaw: f32) -> Command {
    command(
        ControlMode::Rate,
        Setpoint {
            angular_rate: Some([
                RadiansPerSecond(roll),
                RadiansPerSecond(pitch),
                RadiansPerSecond(yaw),
            ]),
            collective_thrust: NormalizedThrust(0.5),
            ..Setpoint::default()
        },
    )
}

fn banked(roll: f32) -> StateEstimate {
    StateEstimate {
        attitude: attitude(roll, 0.0, 0.0),
        ..cruise_state()
    }
}

#[test]
fn course_error_becomes_a_coordinated_turn_bank() {
    let mut rig = Rig::new();
    let _ = rig.step(&cruise_state(), &course_command(0.2));
    // 0.2 rad of error over the 5 s time constant is 0.04 rad/s, and
    // a 55 m/s turn at that rate banks atan(55 * 0.04 / 9.81).
    assert!((rig.runtime.targets.bank - 0.2206).abs() < 1.0e-3);

    let _ = rig.step(&cruise_state(), &course_command(-0.2));
    assert!((rig.runtime.targets.bank + 0.2206).abs() < 1.0e-3);

    // A large error stops at the 45 degree bank limit, on either side.
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert_eq!(rig.runtime.targets.bank, 0.785);
    let _ = rig.step(&cruise_state(), &course_command(-2.5));
    assert_eq!(rig.runtime.targets.bank, -0.785);
}

#[test]
fn heading_hold_turns_on_the_attitude_and_wings_level_does_not_turn() {
    let mut rig = Rig::new();
    let heading = command(
        ControlMode::AltitudeHold,
        Setpoint {
            heading: Some(Radians(-0.2)),
            ..Setpoint::default()
        },
    );
    let _ = rig.step(&cruise_state(), &heading);
    assert!((rig.runtime.targets.bank + 0.2206).abs() < 1.0e-3);

    let level = command(ControlMode::AltitudeHold, Setpoint::default());
    let _ = rig.step(&banked(0.3), &level);
    assert_eq!(rig.runtime.targets.bank, 0.0);
}

#[test]
fn bank_is_limited_by_the_envelope_and_by_the_stall_margin() {
    let mut rig = Rig::new();
    let _ = rig.step(&cruise_state(), &attitude_command(1.2, 0.0, 0.5));
    assert_eq!(rig.runtime.targets.bank, 0.785);
    let _ = rig.step(&cruise_state(), &attitude_command(-1.2, 0.0, 0.5));
    assert_eq!(rig.runtime.targets.bank, -0.785);

    // At 26 m/s the wing has (26/25)^2 = 1.08 times the lift of level
    // flight: a level turn can bank acos(1/1.08) = 0.39 rad.
    rig.probe = Some(26.0);
    rig.runtime = Default::default();
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert!((rig.runtime.targets.bank - 0.391).abs() < 2.0e-3);

    // Below the stall speed no bank is available.
    rig.probe = Some(20.0);
    rig.runtime = Default::default();
    let _ = rig.step(&cruise_state(), &course_command(2.5));
    assert!(rig.runtime.targets.bank.abs() < 1.0e-3);
}

#[test]
fn height_above_the_target_lowers_the_nose_and_closes_the_throttle() {
    let mut rig = Rig::new();
    let output = rig.step(&cruise_state(), &height_command(250.0));
    // 50 m high over the 5 s time constant asks for 10 m/s down, which
    // the 5 m/s sink limit halves. The path angle for that is
    // asin(5/55) = 0.091 rad below the trim pitch of 0.02, and the
    // balance error adds half of 5/55 more.
    assert!((rig.runtime.targets.pitch + 0.1165).abs() < 1.0e-3);
    assert_eq!(output.collective.0, 0.0);
}

#[test]
fn climb_demand_stops_at_the_airframe_climb_rate_and_does_not_wind_up() {
    let mut rig = Rig::new();
    let output = rig.run(100, &cruise_state(), &climb_command(10.0));
    // The 2.5 m/s that the airframe can climb needs full throttle.
    assert_eq!(output.collective.0, 1.0);
    assert_eq!(rig.runtime.energy.throttle_integrator, 0.0);
    // Pitch is inside its limit, so its integrator runs: 100 cycles of
    // 0.2 * (2.5 / 55) * 10 ms.
    let expected = 100.0 * 0.2 * (2.5 / 55.0) * DT;
    assert!((rig.runtime.energy.pitch_integrator - expected).abs() < 1.0e-4);
    assert!(rig.runtime.targets.pitch > 0.08);
}

#[test]
fn pitch_at_its_limit_does_not_wind_up() {
    let mut rig = Rig::new();
    rig.limits.max_pitch = Radians(0.05);
    let _ = rig.run(100, &cruise_state(), &climb_command(10.0));
    assert_eq!(rig.runtime.targets.pitch, 0.05);
    assert_eq!(rig.runtime.energy.pitch_integrator, 0.0);

    let _ = rig.run(100, &cruise_state(), &height_command(0.0));
    assert_eq!(rig.runtime.targets.pitch, -0.05);
    assert_eq!(rig.runtime.energy.pitch_integrator, 0.0);
}

#[test]
fn energy_integrators_run_inside_the_limits_and_stop_at_their_bounds() {
    let mut rig = Rig::new();
    // 1 m/s of climb error: both loops are inside their limits.
    let _ = rig.run(50, &cruise_state(), &climb_command(1.0));
    let throttle_memory = 50.0 * 0.02 * 1.0 * DT;
    assert!((rig.runtime.energy.throttle_integrator - throttle_memory).abs() < 1.0e-4);

    // Held for a long time the memories stop at the throttle range and
    // at the pitch limit.
    let mut params = reference_params();
    params.energy_rate_i = 50.0;
    params.balance_rate_i = 500.0;
    params.energy_rate_p = 0.0;
    params.max_climb_rate.0 = 500.0;
    let mut rig = Rig::with(params);
    rig.limits.max_climb_rate = MetersPerSecond(500.0);
    let _ = rig.run(400, &cruise_state(), &climb_command(0.01));
    assert!(rig.runtime.energy.throttle_integrator <= 1.0);
    assert!(rig.runtime.energy.pitch_integrator <= rig.limits.max_pitch.0);
}

#[test]
fn airspeed_below_the_target_lowers_the_nose_and_opens_the_throttle() {
    let mut rig = Rig::new();
    rig.probe = Some(45.0);
    let output = rig.step(&cruise_state(), &climb_command(0.0));
    assert!(rig.runtime.targets.pitch < reference_params().trim_pitch.0);
    assert_eq!(output.collective.0, 1.0);

    // Above the target the nose rises and the throttle closes.
    let mut rig = Rig::new();
    rig.probe = Some(64.0);
    let output = rig.step(&cruise_state(), &climb_command(0.0));
    assert!(rig.runtime.targets.pitch > reference_params().trim_pitch.0);
    assert!(output.collective.0 < reference_params().cruise_throttle.0);
}

#[test]
fn airspeed_target_stays_inside_the_airspeed_limits() {
    let slow = command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(10.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ]),
            ..Setpoint::default()
        },
    );
    // At exactly the minimum airspeed a 10 m/s command leaves no error:
    // the target is the 33 m/s minimum, not the command.
    let mut rig = Rig::new();
    rig.probe = Some(33.0);
    let mut at_minimum = cruise_state();
    at_minimum.velocity_ned[0].0 = 33.0;
    let _ = rig.step(&at_minimum, &slow);
    assert!((rig.runtime.targets.pitch - reference_params().trim_pitch.0).abs() < 1.0e-4);
}

#[test]
fn underspeed_gives_pitch_to_the_airspeed_and_opens_the_throttle() {
    let mut rig = Rig::new();
    rig.probe = Some(30.0);
    // The command asks for a climb; below the minimum airspeed the law
    // does not try to climb.
    let output = rig.step(&cruise_state(), &height_command(400.0));
    assert!(rig.runtime.energy.underspeed);
    assert_eq!(output.collective.0, 1.0);
    assert!(rig.runtime.targets.pitch < reference_params().trim_pitch.0);

    // Above the minimum but not climbing as commanded: still latched.
    rig.probe = Some(36.0);
    let _ = rig.run(300, &cruise_state(), &height_command(400.0));
    assert!(rig.runtime.energy.underspeed);

    // Climbing as fast as the command asks releases the latch.
    let mut climbing = cruise_state();
    climbing.velocity_ned[2].0 = -3.0;
    let _ = rig.step(&climbing, &height_command(400.0));
    assert!(!rig.runtime.energy.underspeed);

    // So does the recovery airspeed, half way to cruise.
    rig.probe = Some(30.0);
    rig.runtime = Default::default();
    let _ = rig.step(&cruise_state(), &height_command(400.0));
    assert!(rig.runtime.energy.underspeed);
    rig.probe = Some(50.0);
    let _ = rig.run(300, &cruise_state(), &height_command(400.0));
    assert!(!rig.runtime.energy.underspeed);
}

#[test]
fn underspeed_needs_a_measurement_and_a_height_target() {
    // No probe: the ground speed is not the airflow.
    let mut rig = Rig::new();
    rig.probe = None;
    let mut slow = cruise_state();
    slow.velocity_ned[0].0 = 28.0;
    let _ = rig.step(&slow, &height_command(400.0));
    assert!(!rig.runtime.energy.underspeed);

    // Airspeed already on pitch: there is no height target to give up.
    let mut rig = Rig::new();
    rig.probe = Some(28.0);
    rig.fallback_from = Some(ControlMode::VelocityControl);
    let output = rig.step(&cruise_state(), &attitude_command(0.0, 0.0, 0.0));
    assert!(!rig.runtime.energy.underspeed);
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);
    assert!(rig.runtime.targets.pitch < reference_params().trim_pitch.0);
    assert_eq!(rig.runtime.energy.throttle_integrator, 0.0);
}

#[test]
fn fallback_without_any_airspeed_holds_the_trim_pitch() {
    let mut rig = Rig::new();
    rig.probe = None;
    rig.fallback_from = Some(ControlMode::VelocityControl);
    let mut blind = cruise_state();
    blind.valid_flags.remove(StateValidFlags::VELOCITY);
    let output = rig.step(&blind, &attitude_command(0.4, 0.2, 0.0));
    assert_eq!(rig.runtime.targets.bank, 0.0);
    assert_eq!(rig.runtime.targets.pitch, reference_params().trim_pitch.0);
    assert_eq!(output.collective.0, reference_params().cruise_throttle.0);
}

#[test]
fn direct_commands_clear_the_energy_memory() {
    let mut rig = Rig::new();
    let _ = rig.run(50, &cruise_state(), &climb_command(1.0));
    assert!(rig.runtime.energy.pitch_integrator != 0.0);
    let output = rig.step(&cruise_state(), &attitude_command(0.0, 0.1, 1.4));
    assert_eq!(rig.runtime.energy, Default::default());
    // A direct pitch command keeps the pitch and bounds the throttle.
    assert!((rig.runtime.targets.pitch - 0.1).abs() < 1.0e-4);
    assert_eq!(output.collective.0, 1.0);
    let _ = rig.step(&cruise_state(), &attitude_command(0.0, 0.9, 0.5));
    assert_eq!(rig.runtime.targets.pitch, 0.35);

    let _ = rig.run(50, &cruise_state(), &climb_command(1.0));
    let output = rig.step(&cruise_state(), &rate_command(0.0, 0.0, 0.0));
    assert_eq!(rig.runtime.energy, Default::default());
    assert_eq!(output.collective.0, 0.5);
}

#[test]
fn attitude_errors_move_the_surfaces_in_the_pilot_sense() {
    let mut rig = Rig::new();
    let right = rig.step(&cruise_state(), &attitude_command(0.3, 0.0, 0.5));
    assert!(right.roll.0 > 0.1, "bank right needs right aileron");
    // 0.3 rad over the 0.5 s time constant.
    assert!((rig.runtime.targets.rates[0] - 0.6).abs() < 1.0e-3);

    let up = rig.step(&cruise_state(), &attitude_command(0.0, 0.1, 0.5));
    assert!(up.pitch.0 > 0.1, "nose up needs up elevator");

    // A rate with no demand is opposed.
    let mut rolling = cruise_state();
    rolling.angular_velocity[0].0 = 0.3;
    let damped = rig.step(&rolling, &attitude_command(0.0, 0.0, 0.5));
    assert!(damped.roll.0 < -0.05);
}

#[test]
fn an_established_turn_gets_rudder_and_elevator_for_coordination() {
    let mut rig = Rig::new();
    let output = rig.step(&banked(0.5), &attitude_command(0.5, 0.0, 0.5));
    // 0.5 rad of bank at 55 m/s turns at g*tan(0.5)/55 = 0.0974 rad/s:
    // 0.0855 about the body yaw axis and 0.0467 about the pitch axis.
    let [roll, pitch, yaw] = rig.runtime.targets.rates;
    assert!(roll.abs() < 1.0e-3);
    assert!((pitch - 0.0467).abs() < 1.0e-3);
    assert!((yaw - 0.0855).abs() < 1.0e-3);
    assert!(output.yaw.0 > 0.0 && output.pitch.0 > 0.0);

    // A left turn mirrors the rudder and keeps the elevator.
    let output = rig.step(&banked(-0.5), &attitude_command(-0.5, 0.0, 0.5));
    assert!(output.yaw.0 < 0.0 && output.pitch.0 > 0.0);
}

#[test]
fn rate_demands_stop_at_the_rate_and_load_limits() {
    let mut rig = Rig::new();
    let _ = rig.step(&cruise_state(), &rate_command(5.0, -5.0, 5.0));
    assert_eq!(rig.runtime.targets.rates, [0.8, -0.3, 0.3]);

    // 1.05 g leaves 0.05 g of pitch rate at 55 m/s: 0.0089 rad/s.
    rig.limits.max_load_factor = 1.05;
    rig.limits.min_load_factor = 0.9;
    let _ = rig.step(&cruise_state(), &attitude_command(0.0, 0.3, 0.5));
    assert!((rig.runtime.targets.rates[1] - 0.0089).abs() < 2.0e-4);
    // 0.9 g allows 0.1 g of push: -0.0178 rad/s.
    let nose_high = StateEstimate {
        attitude: attitude(0.0, 0.3, 0.0),
        ..cruise_state()
    };
    let _ = rig.step(&nose_high, &attitude_command(0.0, -0.3, 0.5));
    assert!(rig.runtime.targets.rates[1] < 0.0);

    // A minimum above the maximum is not honoured: the range stays
    // zero g to the 1.05 g maximum.
    rig.limits.min_load_factor = 3.0;
    let _ = rig.step(&cruise_state(), &attitude_command(0.0, 0.3, 0.5));
    assert!((rig.runtime.targets.rates[1] - 0.0089).abs() < 2.0e-4);
}

#[test]
fn limits_that_are_not_usable_numbers_permit_nothing_and_do_not_panic() {
    let mut rig = Rig::new();
    rig.limits.max_roll = Radians(f32::NAN);
    rig.limits.max_pitch = Radians(-0.2);
    rig.limits.max_roll_rate = RadiansPerSecond(f32::INFINITY);
    rig.limits.max_pitch_rate = RadiansPerSecond(f32::NAN);
    rig.limits.max_yaw_rate = RadiansPerSecond(-1.0);
    rig.limits.max_climb_rate = MetersPerSecond(f32::NAN);
    rig.limits.max_descent_rate = MetersPerSecond(-3.0);
    rig.limits.max_load_factor = f32::NAN;
    rig.limits.min_load_factor = f32::NAN;
    rig.limits.min_airspeed = Some(MetersPerSecond(f32::NAN));
    rig.limits.max_airspeed = Some(MetersPerSecond(f32::NAN));
    for demand in [
        course_command(2.0),
        height_command(900.0),
        attitude_command(1.0, 1.0, 0.5),
        rate_command(3.0, 3.0, 3.0),
    ] {
        let output = rig.run(5, &banked(0.4), &demand);
        // No rate is permitted, so the demand is zero on each axis and
        // each surface only opposes the measured rate, which is zero.
        assert_eq!(rig.runtime.targets.rates, [0.0; 3]);
        assert_eq!(
            (output.roll.0, output.pitch.0, output.yaw.0),
            (0.0, 0.0, 0.0)
        );
        assert!((0.0..=1.0).contains(&output.collective.0));
    }
    assert_eq!(rig.runtime.targets.bank, 0.0);
    assert_eq!(rig.runtime.targets.pitch, 0.0);
}

#[test]
fn inputs_that_are_not_numbers_do_not_poison_the_loop_memory() {
    let mut rig = Rig::new();
    let mut broken = cruise_state();
    broken.position_ned[2] = Meters(f32::NAN);
    broken.angular_velocity[0] = RadiansPerSecond(f32::NAN);
    let _ = rig.run(5, &broken, &height_command(400.0));
    assert_eq!(rig.runtime.energy.pitch_integrator, 0.0);
    assert_eq!(rig.runtime.energy.throttle_integrator, 0.0);
    assert_eq!(rig.runtime.rates.integrators[0], 0.0);

    // With good inputs again the law commands numbers again.
    let output = rig.step(&cruise_state(), &height_command(400.0));
    assert!(output.roll.0.is_finite() && output.pitch.0.is_finite());
    assert!(output.collective.0.is_finite());
}

#[test]
fn surface_gains_follow_the_airspeed() {
    let demand = rate_command(0.1, 0.0, 0.0);
    let aileron_at = |indicated: f32| {
        let mut rig = Rig::new();
        rig.probe = Some(indicated);
        rig.step(&cruise_state(), &demand).roll.0
    };
    // Feedforward 0.8 and feedback 0.4 on a 0.1 rad/s demand.
    assert!((aileron_at(55.0) - 0.12).abs() < 1.0e-3);
    // Half the airspeed: feedforward twice, feedback four times.
    assert!((aileron_at(27.5) - 0.32).abs() < 1.0e-3);
    // Twice the airspeed: feedforward half, feedback a quarter.
    assert!((aileron_at(110.0) - 0.05).abs() < 1.0e-3);
    // The scale stops at its bounds.
    assert!((aileron_at(220.0) - 0.05).abs() < 1.0e-3);
    assert!((aileron_at(25.0) - 0.32).abs() < 1.0e-3);
}

#[test]
fn rate_integrators_run_in_flight_and_stop_at_their_limit() {
    let mut rig = Rig::new();
    let _ = rig.run(10, &cruise_state(), &rate_command(0.5, 0.0, 0.0));
    let expected = 10.0 * 0.2 * 0.5 * DT;
    assert!((rig.runtime.rates.integrators[0] - expected).abs() < 1.0e-5);
    let _ = rig.run(600, &cruise_state(), &rate_command(0.5, 0.0, 0.0));
    assert_eq!(rig.runtime.rates.integrators[0], 0.3);

    // Below the stall speed the vehicle is not flying: no integration.
    let mut rig = Rig::new();
    rig.probe = None;
    let mut taxi = cruise_state();
    taxi.velocity_ned[0].0 = 10.0;
    let _ = rig.run(10, &taxi, &rate_command(0.5, 0.0, 0.0));
    assert_eq!(rig.runtime.rates.integrators, [0.0; 3]);
}

#[test]
fn a_surface_at_its_stop_does_not_wind_up() {
    let mut params = reference_params();
    params.rate_p = [5.0, 5.0, 5.0];
    let mut rig = Rig::with(params);
    let output = rig.run(20, &cruise_state(), &rate_command(0.8, -0.3, 0.3));
    assert_eq!(
        (output.roll.0, output.pitch.0, output.yaw.0),
        (1.0, -1.0, 1.0)
    );
    assert_eq!(rig.runtime.rates.integrators, [0.0; 3]);

    // An error that leads out of the stop is integrated: the surface is
    // held at its stop by feedforward while the rate is past the demand.
    params.rate_ff = [2.0, 2.0, 2.0];
    let mut past_demand = cruise_state();
    past_demand.angular_velocity[0].0 = 0.81;
    let mut rig = Rig::with(params);
    rig.runtime.rates.integrators[0] = 0.3;
    let output = rig.step(&past_demand, &rate_command(0.8, 0.0, 0.0));
    assert_eq!(output.roll.0, 1.0);
    assert!(rig.runtime.rates.integrators[0] < 0.3);
}
