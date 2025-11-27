use aviate_core::control::{Command, Setpoint, CommandSource, ControlMode};
use aviate_core::state::StateEstimate;
use aviate_core::math::Quaternion;
use aviate_core::types::{RadiansPerSecond, Normalized};
use aviate_mavlink::{SetAttitudeTarget, AttitudeQuaternion, LocalPositionNed};

// MAVLink → Aviate Command
pub fn mavlink_to_command(
    set_att: &SetAttitudeTarget,
    // set_pos: Option<&SetPositionTargetLocalNed>, // Not used yet
) -> Command {
    Command {
        mode: ControlMode::Attitude,  // Basic mapping, ignoring type_mask for now
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

// Aviate StateEstimate → MAVLink
pub fn state_to_attitude_quaternion(state: &StateEstimate, time_boot_ms: u32) -> AttitudeQuaternion {
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

pub fn state_to_local_position_ned(state: &StateEstimate, time_boot_ms: u32) -> LocalPositionNed {
    LocalPositionNed {
        time_boot_ms,
        x: state.position_ned[0].0,
        y: state.position_ned[1].0,
        z: state.position_ned[2].0,
        vx: state.velocity_ned[0].0,
        vy: state.velocity_ned[1].0,
        vz: state.velocity_ned[2].0,
    }
}
