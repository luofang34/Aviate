//! Streaming `no_std` MAVLink v2 frame extraction and command mapping.

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust, Radians};
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::protocol::{
    attitude_target_typemask, position_target_typemask, CommandLong, MavHeader, SetAttitudeTarget,
    SetPositionTargetLocalNed, MAVLINK_IFLAG_SIGNED, MAVLINK_SIGNATURE_LEN, MAVLINK_STX_V2,
};
use aviate_link::mavlink::{
    mav_cmd, mav_result, parse_mavlink_frame, LocalAddress, MavMessage, FORCE_ARM_DISARM_MAGIC,
};

use super::ring::DirectByteRing;

const MAX_MAVLINK_V2_FRAME_LEN: usize = 280;
const MIN_MAVLINK_V2_FRAME_LEN: usize = 12;
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

pub(super) enum InboundAction {
    Enqueue(SystemCommand),
    ImmediateAck { command: u16, result: u8 },
    Ignore,
}

/// Extract the next complete MAVLink v2 frame from `rx_ring` if available.
///
/// Returns `None` when the ring is empty or holds an incomplete trailing frame.
pub(super) fn next_inbound_action<const N: usize>(
    rx_ring: &mut DirectByteRing<N>,
    local: LocalAddress,
) -> Option<InboundAction> {
    let mut scratch = [0_u8; MAX_MAVLINK_V2_FRAME_LEN];
    while !rx_ring.is_empty() {
        let peeked = rx_ring.peek(&mut scratch);
        let &first = scratch.first()?;
        if first != MAVLINK_STX_V2 {
            let skip = scratch
                .iter()
                .take(peeked)
                .position(|&b| b == MAVLINK_STX_V2)
                .unwrap_or(peeked);
            let _ = rx_ring.consume(skip.max(1));
            continue;
        }
        if peeked < MIN_MAVLINK_V2_FRAME_LEN {
            if rx_ring.remaining_capacity() == 0 {
                let _ = rx_ring.consume(1);
                continue;
            }
            return None;
        }
        let payload_len = usize::from(scratch.get(1).copied().unwrap_or(0));
        let incompat = scratch.get(2).copied().unwrap_or(0);
        if incompat & !MAVLINK_IFLAG_SIGNED != 0 {
            let _ = rx_ring.consume(1);
            continue;
        }
        let sig_len = if incompat & MAVLINK_IFLAG_SIGNED != 0 {
            MAVLINK_SIGNATURE_LEN
        } else {
            0
        };
        let frame_len = MavHeader::SIZE + payload_len + 2 + sig_len;
        if peeked < frame_len {
            if rx_ring.remaining_capacity() == 0 && N < frame_len {
                let _ = rx_ring.consume(1);
                continue;
            }
            return None;
        }
        let Some(frame_slice) = scratch.get(..frame_len) else {
            let _ = rx_ring.consume(1);
            continue;
        };
        match parse_mavlink_frame(frame_slice) {
            Ok(parsed) => {
                let _ = rx_ring.consume(parsed.consumed);
                return Some(map_message(parsed.message, parsed.header, local));
            }
            Err(_) => {
                let _ = rx_ring.consume(1);
            }
        }
    }
    None
}

fn map_message(message: MavMessage, header: MavHeader, local: LocalAddress) -> InboundAction {
    match message {
        MavMessage::CommandLong(cmd) => map_command_long(&cmd, local),
        MavMessage::SetAttitudeTarget(target) => {
            if !local.accepts(target.target_system, target.target_component) {
                return InboundAction::Ignore;
            }
            match map_attitude_target(&target, header.seq) {
                Some(cmd) => InboundAction::Enqueue(SystemCommand::FlightControl(cmd)),
                None => InboundAction::Ignore,
            }
        }
        MavMessage::SetPositionTargetLocalNed(target) => {
            if !local.accepts(target.target_system, target.target_component) {
                return InboundAction::Ignore;
            }
            match map_position_target(&target, header.seq) {
                Some(cmd) => InboundAction::Enqueue(SystemCommand::FlightControl(cmd)),
                None => InboundAction::Ignore,
            }
        }
        _ => InboundAction::Ignore,
    }
}

fn map_command_long(cmd: &CommandLong, local: LocalAddress) -> InboundAction {
    if !local.accepts(cmd.target_system, cmd.target_component) {
        return InboundAction::Ignore;
    }
    if cmd.command != mav_cmd::COMPONENT_ARM_DISARM {
        return InboundAction::ImmediateAck {
            command: cmd.command,
            result: mav_result::UNSUPPORTED,
        };
    }
    let forced = cmd.param2 == FORCE_ARM_DISARM_MAGIC;
    if cmd.param1 == 1.0 {
        InboundAction::Enqueue(SystemCommand::Arm)
    } else if cmd.param1 == 0.0 && forced {
        InboundAction::Enqueue(SystemCommand::EmergencyTerminate)
    } else if cmd.param1 == 0.0 {
        InboundAction::Enqueue(SystemCommand::Disarm)
    } else {
        InboundAction::ImmediateAck {
            command: cmd.command,
            result: mav_result::DENIED,
        }
    }
}

fn map_attitude_target(set_att: &SetAttitudeTarget, seq: u8) -> Option<Command> {
    if set_att.type_mask != ATTITUDE_MASK {
        return None;
    }
    if !all_finite(&[
        set_att.q[0],
        set_att.q[1],
        set_att.q[2],
        set_att.q[3],
        set_att.thrust,
    ]) {
        return None;
    }
    let attitude = Quaternion::new(set_att.q[0], set_att.q[1], set_att.q[2], set_att.q[3]);
    if !attitude.is_normalized_default() || !(0.0..=1.0).contains(&set_att.thrust) {
        return None;
    }
    Some(Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(attitude),
            collective_thrust: NormalizedThrust(set_att.thrust),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: u32::from(seq),
        source: CommandSource::Gcs,
    })
}

fn map_position_target(set_pos: &SetPositionTargetLocalNed, seq: u8) -> Option<Command> {
    if set_pos.coordinate_frame != LOCAL_NED_FRAME {
        return None;
    }
    let mask = set_pos.type_mask;
    let position_active = vector_active(mask, POSITION_MASK)?;
    let velocity_active = vector_active(mask, VELOCITY_MASK)?;
    let unsupported = mask & !KNOWN_POSITION_MASK != 0
        || mask & ACCELERATION_MASK != ACCELERATION_MASK
        || mask & position_target_typemask::FORCE_SET != 0
        || mask & position_target_typemask::YAW_RATE_IGNORE == 0;
    if unsupported || (!position_active && !velocity_active) {
        return None;
    }
    let yaw_active = mask & position_target_typemask::YAW_IGNORE == 0;
    if position_active && !all_finite(&[set_pos.x, set_pos.y, set_pos.z]) {
        return None;
    }
    if velocity_active && !all_finite(&[set_pos.vx, set_pos.vy, set_pos.vz]) {
        return None;
    }
    if yaw_active && !set_pos.yaw.is_finite() {
        return None;
    }
    let position =
        position_active.then_some([Meters(set_pos.x), Meters(set_pos.y), Meters(set_pos.z)]);
    let velocity = velocity_active.then_some([
        MetersPerSecond(set_pos.vx),
        MetersPerSecond(set_pos.vy),
        MetersPerSecond(set_pos.vz),
    ]);
    let heading = yaw_active.then_some(Radians(set_pos.yaw));
    let mode = if position_active {
        ControlMode::PositionHold
    } else {
        ControlMode::VelocityControl
    };
    Some(Command {
        mode,
        setpoint: Setpoint {
            position,
            velocity,
            heading,
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: u32::from(seq),
        source: CommandSource::Gcs,
    })
}

fn vector_active(type_mask: u16, vector_mask: u16) -> Option<bool> {
    match type_mask & vector_mask {
        0 => Some(true),
        ignored if ignored == vector_mask => Some(false),
        _ => None,
    }
}

fn all_finite(values: &[f32]) -> bool {
    values.iter().all(|v| v.is_finite())
}
