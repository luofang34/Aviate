use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::state::StateEstimate;
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust, Radians};
use aviate_link::mavlink::protocol::{
    attitude_target_typemask, position_target_typemask, AttitudeQuaternion, SetAttitudeTarget,
    SetPositionTargetLocalNed,
};
use thiserror::Error;

const LOCAL_NED_FRAME: u8 = 1;
const ATTITUDE_MASK: u8 = attitude_target_typemask::BODY_ROLL_RATE_IGNORE
    | attitude_target_typemask::BODY_PITCH_RATE_IGNORE
    | attitude_target_typemask::BODY_YAW_RATE_IGNORE;
const POSITION_MASK: u16 = position_target_typemask::X_IGNORE
    | position_target_typemask::Y_IGNORE
    | position_target_typemask::Z_IGNORE;
const VELOCITY_MASK: u16 = position_target_typemask::VX_IGNORE
    | position_target_typemask::VY_IGNORE
    | position_target_typemask::VZ_IGNORE;
const ACCELERATION_MASK: u16 = position_target_typemask::AX_IGNORE
    | position_target_typemask::AY_IGNORE
    | position_target_typemask::AZ_IGNORE;
const KNOWN_POSITION_MASK: u16 = POSITION_MASK
    | VELOCITY_MASK
    | ACCELERATION_MASK
    | position_target_typemask::FORCE_SET
    | position_target_typemask::YAW_IGNORE
    | position_target_typemask::YAW_RATE_IGNORE;

/// An error in a MAVLink flight-command mapping.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum MavlinkCommandMappingError {
    /// The attitude mask requests a field that the controller does not use.
    #[error("unsupported SET_ATTITUDE_TARGET type mask {type_mask:#04x}")]
    UnsupportedAttitudeTypeMask {
        /// The received MAVLink type mask.
        type_mask: u8,
    },
    /// An active MAVLink field is not finite.
    #[error("MAVLink field {field} is not finite")]
    NonFiniteField {
        /// The name of the invalid field.
        field: &'static str,
    },
    /// The attitude quaternion is not a unit quaternion.
    #[error("SET_ATTITUDE_TARGET quaternion is not normalized")]
    InvalidAttitudeQuaternion,
    /// The scalar thrust is outside its normalized range.
    #[error("SET_ATTITUDE_TARGET thrust is outside [0, 1]")]
    InvalidCollectiveThrust,
    /// The position target uses a frame that this mapping does not support.
    #[error("unsupported SET_POSITION_TARGET_LOCAL_NED frame {coordinate_frame}")]
    UnsupportedCoordinateFrame {
        /// The received MAVLink coordinate frame.
        coordinate_frame: u8,
    },
    /// The position mask enables only part of the position vector.
    #[error("partial position vector in MAVLink type mask {type_mask:#06x}")]
    PartialPositionVector {
        /// The received MAVLink type mask.
        type_mask: u16,
    },
    /// The position mask enables only part of the velocity vector.
    #[error("partial velocity vector in MAVLink type mask {type_mask:#06x}")]
    PartialVelocityVector {
        /// The received MAVLink type mask.
        type_mask: u16,
    },
    /// The position mask requests a field that the controller does not use.
    #[error("unsupported SET_POSITION_TARGET_LOCAL_NED type mask {type_mask:#06x}")]
    UnsupportedPositionTypeMask {
        /// The received MAVLink type mask.
        type_mask: u16,
    },
    /// The position target has no complete position or velocity vector.
    #[error("SET_POSITION_TARGET_LOCAL_NED has no position or velocity vector")]
    MissingPositionOrVelocity,
}

/// Convert a supported MAVLink attitude target to an Aviate command.
pub fn mavlink_to_command(
    set_att: &SetAttitudeTarget,
) -> Result<Command, MavlinkCommandMappingError> {
    if set_att.type_mask != ATTITUDE_MASK {
        return Err(MavlinkCommandMappingError::UnsupportedAttitudeTypeMask {
            type_mask: set_att.type_mask,
        });
    }
    validate_finite_fields(&[
        ("SET_ATTITUDE_TARGET.q[0]", set_att.q[0]),
        ("SET_ATTITUDE_TARGET.q[1]", set_att.q[1]),
        ("SET_ATTITUDE_TARGET.q[2]", set_att.q[2]),
        ("SET_ATTITUDE_TARGET.q[3]", set_att.q[3]),
        ("SET_ATTITUDE_TARGET.thrust", set_att.thrust),
    ])?;
    let attitude = Quaternion::new(set_att.q[0], set_att.q[1], set_att.q[2], set_att.q[3]);
    if !attitude.is_normalized_default() {
        return Err(MavlinkCommandMappingError::InvalidAttitudeQuaternion);
    }
    if !(0.0..=1.0).contains(&set_att.thrust) {
        return Err(MavlinkCommandMappingError::InvalidCollectiveThrust);
    }

    Ok(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(set_att.thrust),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Gcs,
    })
}

/// Convert SET_POSITION_TARGET_LOCAL_NED to Aviate Command
///
/// Aviate uses the NED frame, so this function does not convert the frame.
pub fn mavlink_position_to_command(
    set_pos: &SetPositionTargetLocalNed,
) -> Result<Command, MavlinkCommandMappingError> {
    let (position_active, velocity_active, yaw_active) = validate_position_mask(set_pos)?;
    validate_position_values(set_pos, position_active, velocity_active, yaw_active)?;

    let position = if position_active {
        Some([Meters(set_pos.x), Meters(set_pos.y), Meters(set_pos.z)])
    } else {
        None
    };
    let velocity = if velocity_active {
        Some([
            MetersPerSecond(set_pos.vx),
            MetersPerSecond(set_pos.vy),
            MetersPerSecond(set_pos.vz),
        ])
    } else {
        None
    };
    let heading = if yaw_active {
        Some(Radians(set_pos.yaw))
    } else {
        None
    };
    let mode = if position_active {
        ControlMode::PositionHold
    } else {
        ControlMode::VelocityControl
    };

    Ok(Command {
        mode,
        setpoint: Setpoint {
            position,
            velocity,
            heading,
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Gcs,
    })
}

fn validate_position_mask(
    target: &SetPositionTargetLocalNed,
) -> Result<(bool, bool, bool), MavlinkCommandMappingError> {
    if target.coordinate_frame != LOCAL_NED_FRAME {
        return Err(MavlinkCommandMappingError::UnsupportedCoordinateFrame {
            coordinate_frame: target.coordinate_frame,
        });
    }
    let type_mask = target.type_mask;
    let position_active = vector_is_active(type_mask, POSITION_MASK)
        .ok_or(MavlinkCommandMappingError::PartialPositionVector { type_mask })?;
    let velocity_active = vector_is_active(type_mask, VELOCITY_MASK)
        .ok_or(MavlinkCommandMappingError::PartialVelocityVector { type_mask })?;
    let unsupported = type_mask & !KNOWN_POSITION_MASK != 0
        || type_mask & ACCELERATION_MASK != ACCELERATION_MASK
        || type_mask & position_target_typemask::FORCE_SET != 0
        || type_mask & position_target_typemask::YAW_RATE_IGNORE == 0;
    if unsupported {
        return Err(MavlinkCommandMappingError::UnsupportedPositionTypeMask { type_mask });
    }
    if !position_active && !velocity_active {
        return Err(MavlinkCommandMappingError::MissingPositionOrVelocity);
    }
    let yaw_active = type_mask & position_target_typemask::YAW_IGNORE == 0;
    Ok((position_active, velocity_active, yaw_active))
}

fn vector_is_active(type_mask: u16, vector_mask: u16) -> Option<bool> {
    match type_mask & vector_mask {
        0 => Some(true),
        ignored if ignored == vector_mask => Some(false),
        _ => None,
    }
}

fn validate_position_values(
    target: &SetPositionTargetLocalNed,
    position_active: bool,
    velocity_active: bool,
    yaw_active: bool,
) -> Result<(), MavlinkCommandMappingError> {
    if position_active {
        validate_finite_fields(&[
            ("SET_POSITION_TARGET_LOCAL_NED.x", target.x),
            ("SET_POSITION_TARGET_LOCAL_NED.y", target.y),
            ("SET_POSITION_TARGET_LOCAL_NED.z", target.z),
        ])?;
    }
    if velocity_active {
        validate_finite_fields(&[
            ("SET_POSITION_TARGET_LOCAL_NED.vx", target.vx),
            ("SET_POSITION_TARGET_LOCAL_NED.vy", target.vy),
            ("SET_POSITION_TARGET_LOCAL_NED.vz", target.vz),
        ])?;
    }
    if yaw_active {
        validate_finite_fields(&[("SET_POSITION_TARGET_LOCAL_NED.yaw", target.yaw)])?;
    }
    Ok(())
}

fn validate_finite_fields(
    fields: &[(&'static str, f32)],
) -> Result<(), MavlinkCommandMappingError> {
    for &(field, value) in fields {
        if !value.is_finite() {
            return Err(MavlinkCommandMappingError::NonFiniteField { field });
        }
    }
    Ok(())
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

#[cfg(test)]
mod tests;
