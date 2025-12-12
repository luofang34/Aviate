use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::state::StateEstimate;
use aviate_core::types::{Meters, MetersPerSecond, Normalized, Radians, RadiansPerSecond};
use aviate_link::mavlink::protocol::{
    position_target_typemask, AttitudeQuaternion, SetAttitudeTarget, SetPositionTargetLocalNed,
};

// MAVLink → Aviate Command
pub fn mavlink_to_command(
    set_att: &SetAttitudeTarget,
    // set_pos: Option<&SetPositionTargetLocalNed>, // Not used yet
) -> Command {
    Command {
        mode: ControlMode::Attitude, // Basic mapping, ignoring type_mask for now
        setpoint: Setpoint {
            attitude: Some(Quaternion {
                w: set_att.q[0],
                x: set_att.q[1],
                y: set_att.q[2],
                z: set_att.q[3],
            }),
            angular_rate: Some([
                RadiansPerSecond(set_att.body_roll_rate),
                RadiansPerSecond(set_att.body_pitch_rate),
                RadiansPerSecond(set_att.body_yaw_rate),
            ]),
            collective_thrust: Normalized(set_att.thrust),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0, // Needs sequence tracking from MAVLink layer
        source: CommandSource::Gcs,
    }
}

/// Convert SET_POSITION_TARGET_LOCAL_NED to Aviate Command
///
/// Handles position/velocity setpoints in NED frame.
/// Note: Aviate uses NED internally, so no frame conversion needed.
pub fn mavlink_position_to_command(set_pos: &SetPositionTargetLocalNed) -> Command {
    let type_mask = set_pos.type_mask;

    // Determine control mode based on what's NOT ignored
    let position_valid = (type_mask & position_target_typemask::X_IGNORE) == 0
        && (type_mask & position_target_typemask::Y_IGNORE) == 0
        && (type_mask & position_target_typemask::Z_IGNORE) == 0;

    let velocity_valid = (type_mask & position_target_typemask::VX_IGNORE) == 0
        && (type_mask & position_target_typemask::VY_IGNORE) == 0
        && (type_mask & position_target_typemask::VZ_IGNORE) == 0;

    let yaw_valid = (type_mask & position_target_typemask::YAW_IGNORE) == 0;

    // Extract position if valid
    let position = if position_valid {
        Some([Meters(set_pos.x), Meters(set_pos.y), Meters(set_pos.z)])
    } else {
        None
    };

    // Extract velocity if valid
    let velocity = if velocity_valid {
        Some([
            MetersPerSecond(set_pos.vx),
            MetersPerSecond(set_pos.vy),
            MetersPerSecond(set_pos.vz),
        ])
    } else {
        None
    };

    // Extract yaw/heading if valid
    let heading = if yaw_valid {
        Some(Radians(set_pos.yaw))
    } else {
        None
    };

    // Select control mode based on setpoint type
    let mode = if position_valid {
        ControlMode::PositionHold
    } else if velocity_valid {
        ControlMode::VelocityControl
    } else {
        ControlMode::Attitude // Fallback
    };

    Command {
        mode,
        setpoint: Setpoint {
            position,
            velocity,
            heading,
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0, // Needs sequence tracking from MAVLink layer
        source: CommandSource::Gcs,
    }
}

// Aviate StateEstimate → MAVLink
pub fn state_to_attitude_quaternion(
    state: &StateEstimate,
    time_boot_ms: u32,
) -> AttitudeQuaternion {
    AttitudeQuaternion {
        time_boot_ms,
        q1: state.attitude.w,
        q2: state.attitude.x,
        q3: state.attitude.y,
        q4: state.attitude.z,
        rollspeed: state.angular_velocity[0].0,
        pitchspeed: state.angular_velocity[1].0,
        yawspeed: state.angular_velocity[2].0,
        repr_offset_q: [0.0; 4], // Not using offset
    }
}
