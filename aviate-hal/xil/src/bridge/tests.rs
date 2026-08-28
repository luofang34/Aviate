//! MAVLink flight-command mapping tests.

#![allow(clippy::expect_used, clippy::panic)]

use super::*;

fn attitude_target(q: [f32; 4], thrust: f32) -> SetAttitudeTarget {
    SetAttitudeTarget {
        type_mask: ATTITUDE_MASK,
        q,
        thrust,
        ..Default::default()
    }
}

fn position_type_mask(position: bool, velocity: bool, yaw: bool) -> u16 {
    let mut type_mask = ACCELERATION_MASK | position_target_typemask::YAW_RATE_IGNORE;
    if !position {
        type_mask |= POSITION_MASK;
    }
    if !velocity {
        type_mask |= VELOCITY_MASK;
    }
    if !yaw {
        type_mask |= position_target_typemask::YAW_IGNORE;
    }
    type_mask
}

fn position_target(type_mask: u16) -> SetPositionTargetLocalNed {
    SetPositionTargetLocalNed {
        coordinate_frame: LOCAL_NED_FRAME,
        type_mask,
        x: 1.0,
        y: -2.0,
        z: -3.0,
        vx: 4.0,
        vy: -5.0,
        vz: 6.0,
        yaw: 0.75,
        ..Default::default()
    }
}

#[test]
fn attitude_mapping_preserves_the_exact_attitude_and_collective() {
    let quaternion = [0.5, -0.5, 0.5, -0.5];
    let command =
        mavlink_to_command(&attitude_target(quaternion, 0.37)).expect("valid attitude target");

    assert_eq!(command.mode, ControlMode::Attitude);
    assert_eq!(
        command.setpoint.attitude,
        Some(Quaternion::new(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3]
        ))
    );
    assert_eq!(command.setpoint.collective_thrust, NormalizedThrust(0.37));
    assert!(command.setpoint.angular_rate.is_none());
}

#[test]
fn complete_position_and_velocity_targets_map_to_supported_modes() {
    let position =
        mavlink_position_to_command(&position_target(position_type_mask(true, false, true)))
            .expect("valid position target");
    assert_eq!(position.mode, ControlMode::PositionHold);
    assert_eq!(
        position.setpoint.position,
        Some([Meters(1.0), Meters(-2.0), Meters(-3.0)])
    );
    assert!(position.setpoint.velocity.is_none());
    assert_eq!(position.setpoint.heading, Some(Radians(0.75)));

    let velocity =
        mavlink_position_to_command(&position_target(position_type_mask(false, true, false)))
            .expect("valid velocity target");
    assert_eq!(velocity.mode, ControlMode::VelocityControl);
    assert!(velocity.setpoint.position.is_none());
    assert_eq!(
        velocity.setpoint.velocity,
        Some([
            MetersPerSecond(4.0),
            MetersPerSecond(-5.0),
            MetersPerSecond(6.0)
        ])
    );
    assert!(velocity.setpoint.heading.is_none());
}

#[test]
fn complete_position_takes_precedence_when_velocity_is_also_present() {
    let command =
        mavlink_position_to_command(&position_target(position_type_mask(true, true, true)))
            .expect("valid combined target");

    assert_eq!(command.mode, ControlMode::PositionHold);
    assert!(command.setpoint.position.is_some());
    assert!(command.setpoint.velocity.is_some());
}

#[test]
fn partial_position_and_velocity_vectors_are_rejected() {
    let partial_position = position_target(
        position_target_typemask::X_IGNORE
            | VELOCITY_MASK
            | ACCELERATION_MASK
            | position_target_typemask::YAW_IGNORE
            | position_target_typemask::YAW_RATE_IGNORE,
    );
    assert!(matches!(
        mavlink_position_to_command(&partial_position),
        Err(MavlinkCommandMappingError::PartialPositionVector { .. })
    ));

    let partial_velocity = position_target(
        POSITION_MASK
            | position_target_typemask::VX_IGNORE
            | ACCELERATION_MASK
            | position_target_typemask::YAW_IGNORE
            | position_target_typemask::YAW_RATE_IGNORE,
    );
    assert!(matches!(
        mavlink_position_to_command(&partial_velocity),
        Err(MavlinkCommandMappingError::PartialVelocityVector { .. })
    ));
}

#[test]
fn unsupported_attitude_and_position_masks_are_rejected() {
    let mut attitude = attitude_target([1.0, 0.0, 0.0, 0.0], 0.5);
    attitude.type_mask = attitude_target_typemask::ATTITUDE_IGNORE;
    assert!(matches!(
        mavlink_to_command(&attitude),
        Err(MavlinkCommandMappingError::UnsupportedAttitudeTypeMask { .. })
    ));

    let mut position = position_target(position_type_mask(true, false, false));
    position.type_mask &= !position_target_typemask::AX_IGNORE;
    assert!(matches!(
        mavlink_position_to_command(&position),
        Err(MavlinkCommandMappingError::UnsupportedPositionTypeMask { .. })
    ));
}

#[test]
fn non_finite_and_malformed_attitude_targets_are_rejected() {
    let non_finite = attitude_target([f32::NAN, 0.0, 0.0, 0.0], 0.5);
    assert!(matches!(
        mavlink_to_command(&non_finite),
        Err(MavlinkCommandMappingError::NonFiniteField { .. })
    ));

    let malformed = attitude_target([2.0, 0.0, 0.0, 0.0], 0.5);
    assert_eq!(
        mavlink_to_command(&malformed).expect_err("malformed quaternion"),
        MavlinkCommandMappingError::InvalidAttitudeQuaternion
    );

    let invalid_thrust = attitude_target([1.0, 0.0, 0.0, 0.0], f32::INFINITY);
    assert!(matches!(
        mavlink_to_command(&invalid_thrust),
        Err(MavlinkCommandMappingError::NonFiniteField { .. })
    ));
}
