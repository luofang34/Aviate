//! Fallback and zero-thrust transfer guardrails.

use super::*;
use aviate_core::control::{IntegratorAction, ModeEntryDecision};

#[test]
fn automatic_outer_fallback_ignores_stale_inner_setpoints() {
    let controller = MultirotorController::default();
    let state = state();
    for requested in [
        ControlMode::AltitudeHold,
        ControlMode::PositionHold,
        ControlMode::VelocityControl,
        ControlMode::DeviationTracking,
    ] {
        let mut runtime = MultirotorRuntimeState {
            dt_sec: 0.01,
            ..Default::default()
        };
        let velocity = command(ControlMode::VelocityControl, velocity_setpoint(0.8));
        let prior = controller.step(
            &mut runtime,
            &state,
            &velocity,
            &VehicleControlMode::from_control_mode(velocity.mode),
            ConfigMode::Hover,
            &limits(),
        );
        let fallback = command(
            ControlMode::Attitude,
            Setpoint {
                attitude: Some(Quaternion::IDENTITY),
                collective_thrust: NormalizedThrust(0.93),
                ..Default::default()
            },
        );
        let decision = ModeEntryDecision::FallenBack {
            requested,
            effective: ControlMode::Attitude,
            missing: StateValidFlags::VELOCITY,
        };
        let result = controller.step_with_observation(
            &mut runtime,
            &state,
            &fallback,
            &VehicleControlMode::from_control_mode(fallback.mode).with_mode_entry(decision),
            ConfigMode::Hover,
            &limits(),
        );
        let observation = result.observation.multirotor.expect("multirotor witness");
        assert_eq!(
            result.axis_command.collective.0.to_bits(),
            prior.collective.0.to_bits()
        );
        assert_eq!(observation.setpoints.attitude, state.attitude);
        assert_eq!(
            observation.previous_topology,
            Some(EffectiveControlTopology::Velocity)
        );
        assert_eq!(
            observation.current_topology,
            EffectiveControlTopology::Attitude
        );
    }
}

#[test]
fn zero_thrust_observation_preserves_terms_and_reports_the_final_reset() {
    let controller = MultirotorController::default();
    let mut runtime = MultirotorRuntimeState {
        dt_sec: 0.01,
        ..Default::default()
    };
    runtime.velocity_loop.integrator_ned = Vector3::new(
        MetersPerSecond(0.2),
        MetersPerSecond(-0.1),
        MetersPerSecond(0.3),
    );
    let command = command(
        ControlMode::VelocityControl,
        Setpoint {
            velocity: Some([
                MetersPerSecond(1.0),
                MetersPerSecond(-1.0),
                MetersPerSecond(100.0),
            ]),
            ..Default::default()
        },
    );
    let result = controller.step_with_observation(
        &mut runtime,
        &state(),
        &command,
        &VehicleControlMode::from_control_mode(command.mode),
        ConfigMode::Hover,
        &limits(),
    );
    let observation = result.observation.multirotor.expect("zero-thrust witness");
    assert_eq!(
        observation.current_topology,
        EffectiveControlTopology::ZeroThrust
    );
    assert_ne!(observation.velocity_loop.p, [0.0; 3]);
    assert_eq!(
        observation.velocity_loop.integrator_before,
        [0.2, -0.1, 0.3]
    );
    assert_eq!(observation.velocity_loop.integrator_after, [0.0; 3]);
    assert_eq!(
        observation.velocity_loop.integrator_action,
        [IntegratorAction::Reset; 3]
    );
}

#[test]
fn explicit_attitude_collective_is_not_ramped() {
    let controller = MultirotorController::default();
    let mut runtime = MultirotorRuntimeState::default();
    runtime.last_axis_command.collective = NormalizedThrust(0.8);
    runtime.axis_command_primed = true;
    let command = command(
        ControlMode::Attitude,
        Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.23),
            ..Default::default()
        },
    );
    let result = controller.step(
        &mut runtime,
        &state(),
        &command,
        &VehicleControlMode::from_control_mode(command.mode),
        ConfigMode::Hover,
        &limits(),
    );
    assert_eq!(result.collective.0.to_bits(), 0.23_f32.to_bits());
}
