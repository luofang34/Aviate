//! Streaming `no_std` MAVLink v2 frame extraction and command mapping.

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::math::Quaternion;
use aviate_core::types::{Meters, MetersPerSecond, NormalizedThrust, Radians};
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::protocol::{
    attitude_target_typemask, position_target_typemask, CommandLong, MavHeader, SetAttitudeTarget,
    SetPositionTargetGlobalInt, SetPositionTargetLocalNed, MAVLINK_IFLAG_SIGNED,
    MAVLINK_SIGNATURE_LEN, MAVLINK_STX_V2,
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
/// A height and a heading: `z` and `yaw` active, everything else
/// ignored (type mask 2555). A fixed wing flies it as a heading and a
/// height hold, with no horizontal position.
const HEIGHT_HEADING_MASK: u16 = position_target_typemask::X_IGNORE
    | position_target_typemask::Y_IGNORE
    | VELOCITY_MASK
    | ACCELERATION_MASK
    | position_target_typemask::YAW_RATE_IGNORE;
const KNOWN_POSITION_MASK: u16 = POSITION_MASK
    | VELOCITY_MASK
    | ACCELERATION_MASK
    | position_target_typemask::FORCE_SET
    | position_target_typemask::YAW_IGNORE
    | position_target_typemask::YAW_RATE_IGNORE;

/// System and component that sent an inbound frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Sender {
    pub(super) system: u8,
    pub(super) component: u8,
}

/// What a `MAV_CMD_DO_CHANGE_SPEED` asks of the airspeed target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum AirspeedRequest {
    /// Fly this indicated airspeed \[m/s\] until told otherwise.
    Set(f32),
    /// Return to the airframe's cruise airspeed.
    Default,
    /// Leave the target as it is.
    NoChange,
}

pub(super) enum InboundAction {
    Enqueue {
        command: SystemCommand,
        sender: Sender,
    },
    Airspeed {
        request: AirspeedRequest,
        sender: Sender,
    },
    /// A position target whose local position the board fills in from
    /// `target` in its present frame.
    EnqueueGeodetic {
        command: SystemCommand,
        target: super::GeodeticTarget,
        sender: Sender,
    },
    ImmediateAck {
        command: u16,
        result: u8,
        sender: Sender,
    },
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
    let sender = Sender {
        system: header.sysid,
        component: header.compid,
    };
    match message {
        MavMessage::CommandLong(cmd) => map_command_long(&cmd, local, sender),
        MavMessage::SetAttitudeTarget(target) => {
            if !local.accepts(target.target_system, target.target_component) {
                return InboundAction::Ignore;
            }
            match map_attitude_target(&target, header.seq) {
                Some(cmd) => InboundAction::Enqueue {
                    command: SystemCommand::FlightControl(cmd),
                    sender,
                },
                None => InboundAction::Ignore,
            }
        }
        MavMessage::SetPositionTargetGlobalInt(target) => {
            if !local.accepts(target.target_system, target.target_component) {
                return InboundAction::Ignore;
            }
            match map_global_target(&target, header.seq) {
                Some((command, geodetic)) => InboundAction::EnqueueGeodetic {
                    command: SystemCommand::FlightControl(command),
                    target: geodetic,
                    sender,
                },
                None => InboundAction::Ignore,
            }
        }
        MavMessage::SetPositionTargetLocalNed(target) => {
            if !local.accepts(target.target_system, target.target_component) {
                return InboundAction::Ignore;
            }
            match map_position_target(&target, header.seq) {
                Some(cmd) => InboundAction::Enqueue {
                    command: SystemCommand::FlightControl(cmd),
                    sender,
                },
                None => InboundAction::Ignore,
            }
        }
        _ => InboundAction::Ignore,
    }
}

fn map_command_long(cmd: &CommandLong, local: LocalAddress, sender: Sender) -> InboundAction {
    if !local.accepts(cmd.target_system, cmd.target_component) {
        return InboundAction::Ignore;
    }
    let refuse = |result: u8| InboundAction::ImmediateAck {
        command: cmd.command,
        result,
        sender,
    };
    if cmd.command == mav_cmd::DO_CHANGE_SPEED {
        return match airspeed_request(cmd) {
            Some(request) => InboundAction::Airspeed { request, sender },
            None => refuse(mav_result::DENIED),
        };
    }
    if cmd.command != mav_cmd::COMPONENT_ARM_DISARM {
        return refuse(mav_result::UNSUPPORTED);
    }
    let forced = cmd.param2 == FORCE_ARM_DISARM_MAGIC;
    let command = if cmd.param1 == 1.0 {
        SystemCommand::Arm
    } else if cmd.param1 == 0.0 && forced {
        SystemCommand::EmergencyTerminate
    } else if cmd.param1 == 0.0 {
        SystemCommand::Disarm
    } else {
        return refuse(mav_result::DENIED);
    };
    InboundAction::Enqueue { command, sender }
}

/// Speed type of `MAV_CMD_DO_CHANGE_SPEED` for an airspeed.
const SPEED_TYPE_AIRSPEED: f32 = 0.0;
/// `param2` of `MAV_CMD_DO_CHANGE_SPEED` that leaves the speed unchanged.
const SPEED_NO_CHANGE: f32 = -1.0;
/// `param2` of `MAV_CMD_DO_CHANGE_SPEED` that returns to the default speed.
const SPEED_DEFAULT: f32 = -2.0;

/// Read a `MAV_CMD_DO_CHANGE_SPEED`. Only the airspeed type is flown;
/// a ground speed, climb or descent speed is refused, as is a speed
/// that is not a finite positive number or one of the two markers.
/// The throttle parameter is not used: the law sets the throttle.
fn airspeed_request(cmd: &CommandLong) -> Option<AirspeedRequest> {
    if cmd.param1 != SPEED_TYPE_AIRSPEED {
        return None;
    }
    let speed = cmd.param2;
    if speed == SPEED_NO_CHANGE {
        Some(AirspeedRequest::NoChange)
    } else if speed == SPEED_DEFAULT {
        Some(AirspeedRequest::Default)
    } else if speed.is_finite() && speed > 0.0 {
        Some(AirspeedRequest::Set(speed))
    } else {
        None
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
    if mask == HEIGHT_HEADING_MASK {
        return height_and_heading(set_pos, seq);
    }
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

/// A geodetic position hold, with an optional velocity and yaw. Only an
/// altitude above mean sea level (`MAV_FRAME_GLOBAL_INT`) is read; the
/// position is left for the board to express in its present frame.
fn map_global_target(
    set_pos: &SetPositionTargetGlobalInt,
    seq: u8,
) -> Option<(Command, super::GeodeticTarget)> {
    if set_pos.coordinate_frame != SetPositionTargetGlobalInt::FRAME_GLOBAL_INT {
        return None;
    }
    let mask = set_pos.type_mask;
    let velocity_active = vector_active(mask, VELOCITY_MASK)?;
    let unsupported = mask & !KNOWN_POSITION_MASK != 0
        || mask & POSITION_MASK != 0
        || mask & ACCELERATION_MASK != ACCELERATION_MASK
        || mask & position_target_typemask::FORCE_SET != 0
        || mask & position_target_typemask::YAW_RATE_IGNORE == 0;
    if unsupported || !set_pos.alt.is_finite() {
        return None;
    }
    let yaw_active = mask & position_target_typemask::YAW_IGNORE == 0;
    if velocity_active && !all_finite(&[set_pos.vx, set_pos.vy, set_pos.vz]) {
        return None;
    }
    if yaw_active && !set_pos.yaw.is_finite() {
        return None;
    }
    let target = super::GeodeticTarget {
        latitude_deg: f64::from(set_pos.lat_int) * 1.0e-7,
        longitude_deg: f64::from(set_pos.lon_int) * 1.0e-7,
        altitude_msl_m: f64::from(set_pos.alt),
    };
    let command = Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            velocity: velocity_active.then_some([
                MetersPerSecond(set_pos.vx),
                MetersPerSecond(set_pos.vy),
                MetersPerSecond(set_pos.vz),
            ]),
            heading: yaw_active.then_some(Radians(set_pos.yaw)),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: u32::from(seq),
        source: CommandSource::Gcs,
    };
    Some((command, target))
}

/// Height hold on a heading: `z` is the local NED down coordinate, so
/// the height above the origin datum is `-z`; `yaw` is the true heading.
fn height_and_heading(set_pos: &SetPositionTargetLocalNed, seq: u8) -> Option<Command> {
    if !all_finite(&[set_pos.z, set_pos.yaw]) {
        return None;
    }
    Some(Command {
        mode: ControlMode::AltitudeHold,
        setpoint: Setpoint {
            altitude: Some(Meters(-set_pos.z)),
            heading: Some(Radians(set_pos.yaw)),
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
