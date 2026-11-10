use super::*;
use crate::control::{CommandSource, Setpoint};
use crate::state::{EstimateQuality, StateValidFlags};
use crate::types::Meters;

fn default_state(altitude_m: Scalar) -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-altitude_m)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    }
}

fn default_limits() -> Limits {
    ResolvedKernelConfig::default().limits
}

fn armed_flags() -> VehicleControlMode {
    VehicleControlMode {
        flag_control_rates_enabled: true,
        flag_control_attitude_enabled: true,
        ..VehicleControlMode::default()
    }
}

#[test]
fn disarmed_or_terminated_silences_all_axes() {
    let ctl = SpacecraftController;
    let mut rt = NoControllerState;
    let state = default_state(200.0);
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(0.8),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Autopilot,
    };
    let out_disarmed = ctl.step(
        &mut rt,
        &state,
        &cmd,
        &VehicleControlMode::default(),
        ConfigMode::Hover,
        &default_limits(),
    );
    assert_eq!(out_disarmed, AxisCommand::default());

    let terminated_flags = VehicleControlMode {
        flag_control_termination_enabled: true,
        flag_control_rates_enabled: true,
        ..VehicleControlMode::default()
    };
    let out_terminated = ctl.step(
        &mut rt,
        &state,
        &cmd,
        &terminated_flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert_eq!(out_terminated, AxisCommand::default());
}

#[test]
fn attitude_setpoint_tracks_both_quaternion_hemispheres() {
    let ctl = SpacecraftController;
    let mut rt = NoControllerState;
    let state = default_state(500.0);
    let q_pos = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 0.2);
    let q_neg = Quaternion::new(-q_pos.w, -q_pos.x, -q_pos.y, -q_pos.z);

    let cmd_pos = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(q_pos),
            collective_thrust: NormalizedThrust(0.6),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Autopilot,
    };
    let cmd_neg = Command {
        setpoint: Setpoint {
            attitude: Some(q_neg),
            collective_thrust: NormalizedThrust(0.6),
            ..Default::default()
        },
        ..cmd_pos.clone()
    };

    let flags = armed_flags();
    let out_pos = ctl.step(
        &mut rt,
        &state,
        &cmd_pos,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    let out_neg = ctl.step(
        &mut rt,
        &state,
        &cmd_neg,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert!(out_pos.roll.0 > 0.0);
    assert!((out_pos.roll.0 - out_neg.roll.0).abs() < 1e-6);
    assert!((out_pos.collective.0 - 0.6).abs() < 1e-6);
}

#[test]
fn rate_setpoint_and_damping_fallback_work() {
    let ctl = SpacecraftController;
    let mut rt = NoControllerState;
    let mut state = default_state(500.0);
    state.angular_velocity = [
        RadiansPerSecond(0.0),
        RadiansPerSecond(0.5),
        RadiansPerSecond(0.0),
    ];
    let flags = armed_flags();

    let rate_cmd = Command {
        mode: ControlMode::Rate,
        setpoint: Setpoint {
            angular_rate: Some([
                RadiansPerSecond(1.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(-0.5),
            ]),
            collective_thrust: NormalizedThrust(0.4),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 2,
        source: CommandSource::Autopilot,
    };
    let out_rate = ctl.step(
        &mut rt,
        &state,
        &rate_cmd,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert!(out_rate.roll.0 > 0.0);
    assert!(out_rate.pitch.0 < 0.0);
    assert!(out_rate.yaw.0 < 0.0);

    let empty_cmd = Command {
        mode: ControlMode::Rate,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 3,
        source: CommandSource::Autopilot,
    };
    let out_damp = ctl.step(
        &mut rt,
        &state,
        &empty_cmd,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert!(out_damp.pitch.0 < 0.0);

    let dir_lock_cmd = Command {
        mode: ControlMode::VelocityControl,
        setpoint: Setpoint {
            velocity: Some([
                MetersPerSecond(0.0),
                MetersPerSecond(10.0),
                MetersPerSecond(0.0),
            ]),
            collective_thrust: NormalizedThrust(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 4,
        source: CommandSource::Autopilot,
    };
    let out_dir = ctl.step(
        &mut rt,
        &state,
        &dir_lock_cmd,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert!(out_dir.yaw.0 > 0.0);

    let entry_bank_cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            heading: Some(Radians(0.4)),
            vertical_deviation: Some(Meters(20.0)),
            collective_thrust: NormalizedThrust(1.0),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 5,
        source: CommandSource::Autopilot,
    };
    let out_bank = ctl.step(
        &mut rt,
        &state,
        &entry_bank_cmd,
        &flags,
        ConfigMode::Hover,
        &default_limits(),
    );
    assert!(out_bank.roll.0 > 0.0);
    assert!(out_bank.collective.0 < 0.6);
}

#[test]
fn cruise_mode_locks_fins_below_50m_and_allows_above_50m() {
    let ctl = SpacecraftController;
    let mut rt = NoControllerState;
    let flags = armed_flags();
    let cmd = Command {
        mode: ControlMode::Rate,
        setpoint: Setpoint {
            angular_rate: Some([
                RadiansPerSecond(1.0),
                RadiansPerSecond(1.0),
                RadiansPerSecond(1.0),
            ]),
            collective_thrust: NormalizedThrust(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 4,
        source: CommandSource::Autopilot,
    };

    let low_state = default_state(30.0);
    let locked = ctl.step(
        &mut rt,
        &low_state,
        &cmd,
        &flags,
        ConfigMode::Cruise,
        &default_limits(),
    );
    assert_eq!(locked.roll.0, 0.0);
    assert_eq!(locked.pitch.0, 0.0);
    assert_eq!(locked.yaw.0, 0.0);
    assert!((locked.collective.0 - 0.5).abs() < 1e-6);

    let high_state = default_state(120.0);
    let unlocked = ctl.step(
        &mut rt,
        &high_state,
        &cmd,
        &flags,
        ConfigMode::Cruise,
        &default_limits(),
    );
    assert!(unlocked.roll.0 > 0.0);
}

#[test]
fn proper_accel_limiter_caps_throttle_at_4g() {
    let zero_max = limit_thrust_for_proper_accel(
        NormalizedThrust(1.0),
        [MetersPerSecondSquared(0.0); 3],
        MetersPerSecondSquared(0.0),
        [0.0, 0.0, 1.0],
    );
    assert_eq!(zero_max.0, 0.0);

    let saturated_aero = limit_thrust_for_proper_accel(
        NormalizedThrust(1.0),
        [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(40.0),
        ],
        MetersPerSecondSquared(50.0),
        [0.0, 0.0, 1.0],
    );
    assert_eq!(saturated_aero.0, 0.0);

    let capped = limit_thrust_for_proper_accel(
        NormalizedThrust(1.0),
        [
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(9.81),
        ],
        MetersPerSecondSquared(58.86),
        [0.0, 0.0, 1.0],
    );
    assert!((capped.0 - 0.5).abs() < 1e-4);
}

#[test]
fn sas_modes_and_rcs_rw_allocators_cover_all_branches() {
    let mut state = default_state(400.0);
    state.angular_velocity = [
        RadiansPerSecond(0.2),
        RadiansPerSecond(-0.2),
        RadiansPerSecond(0.1),
    ];
    let limits = default_limits();

    let kill = evaluate_sas_mode(SasMode::KillRot, &state, &limits, NormalizedThrust(0.0));
    assert!(kill.roll.0 < 0.0);
    assert!(kill.pitch.0 > 0.0);

    let hold_q = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 0.3);
    let hold = evaluate_sas_mode(
        SasMode::HoldAttitude(hold_q),
        &state,
        &limits,
        NormalizedThrust(0.5),
    );
    assert!(hold.pitch.0 > 0.0);

    let lock_degenerate = sas_direction_lock_attitude(state.attitude, [0.0, 0.0, 0.0]);
    assert_eq!(lock_degenerate, state.attitude);

    let lock_aligned = sas_direction_lock_attitude(state.attitude, [1.0, 0.0, 0.0]);
    assert_eq!(lock_aligned, state.attitude);

    let lock_opposite = sas_direction_lock_attitude(state.attitude, [-1.0, 0.0, 0.0]);
    let rotated_x = lock_opposite.rotate_vector(Vector3::new(1.0, 0.0, 0.0));
    assert!(rotated_x.x < -0.99);

    let dir_cmd = evaluate_sas_mode(
        SasMode::DirectionLock([0.0, 1.0, 0.0]),
        &state,
        &limits,
        NormalizedThrust(0.7),
    );
    assert!(dir_cmd.yaw.0 > 0.0);

    let bank_cmd = evaluate_sas_mode(
        SasMode::EntryBank(Radians(0.4)),
        &state,
        &limits,
        NormalizedThrust(0.0),
    );
    assert!(bank_cmd.roll.0 > 0.0);

    let alloc = allocate_rw_and_rotational_rcs(
        [
            NormalizedSigned(0.4),
            NormalizedSigned(-0.6),
            NormalizedSigned(0.2),
        ],
        [Normalized(0.2), Normalized(0.9), Normalized(0.85)],
    );
    assert!((alloc.reaction_wheel_torque[0].0 - 0.4).abs() < 1e-6);
    assert_eq!(alloc.rcs_attitude_torque[0].0, 0.0);
    assert_eq!(alloc.reaction_wheel_torque[1].0, 0.0);
    assert!((alloc.rcs_attitude_torque[1].0 + 0.6).abs() < 1e-6);
    assert_eq!(alloc.reaction_wheel_torque[2].0, 0.0);
    assert!((alloc.rcs_attitude_torque[2].0 - 0.2).abs() < 1e-6);

    let trans = allocate_translational_rcs([
        MetersPerSecond(0.01),
        MetersPerSecond(0.22),
        MetersPerSecond(-0.62),
    ]);
    assert_eq!(trans[0].0, 0.0);
    assert!((trans[1].0 - 0.5).abs() < 1e-5);
    assert!((trans[2].0 + 1.0).abs() < 1e-5);
}

#[test]
fn config_binding_and_supported_modes() {
    let ctl = SpacecraftController;
    assert!(ctl
        .verify_config_binding(&ResolvedKernelConfig::default())
        .is_ok());
    assert!(ctl.supports_mode(ControlMode::Attitude));
    assert!(ctl.supports_mode(ControlMode::VelocityControl));
    assert!(!ctl.supports_mode(ControlMode::PositionHold));
}
