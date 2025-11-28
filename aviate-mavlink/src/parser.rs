//! MAVLink message parser
//!
//! Parses MAVLink 2.0 frames from byte buffers.

use crate::messages::*;
use crate::{MAVLINK_STX_V2, MAX_PAYLOAD_LEN};

/// Parse error types
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ParseError {
    /// Buffer too short
    BufferTooShort,
    /// Invalid start byte
    InvalidStartByte,
    /// Payload length exceeds maximum
    PayloadTooLong,
    /// CRC mismatch
    CrcMismatch,
    /// Unsupported message ID
    UnsupportedMessage(u32),
    /// Invalid payload for message type
    InvalidPayload,
}

/// MAVLink 2.0 frame header
#[derive(Copy, Clone, Debug)]
pub struct MavHeader {
    pub payload_len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub seq: u8,
    pub sysid: u8,
    pub compid: u8,
    pub msgid: u32, // 24-bit in wire format
}

impl MavHeader {
    pub const SIZE: usize = 10; // STX + 9 header bytes
}

/// Parse a MAVLink message from a byte buffer
///
/// Returns the parsed message and the number of bytes consumed.
pub fn parse_mavlink(buf: &[u8]) -> Result<(MavMessage, usize), ParseError> {
    // Minimum frame: STX(1) + header(9) + payload(0) + checksum(2) = 12
    if buf.len() < 12 {
        return Err(ParseError::BufferTooShort);
    }

    // Check start byte
    if buf[0] != MAVLINK_STX_V2 {
        return Err(ParseError::InvalidStartByte);
    }

    // Parse header
    let payload_len = buf[1] as usize;
    if payload_len > MAX_PAYLOAD_LEN {
        return Err(ParseError::PayloadTooLong);
    }

    let header = MavHeader {
        payload_len: buf[1],
        incompat_flags: buf[2],
        compat_flags: buf[3],
        seq: buf[4],
        sysid: buf[5],
        compid: buf[6],
        msgid: (buf[7] as u32) | ((buf[8] as u32) << 8) | ((buf[9] as u32) << 16),
    };

    // Calculate total frame size
    let frame_size = MavHeader::SIZE + payload_len + 2; // +2 for CRC
    if buf.len() < frame_size {
        return Err(ParseError::BufferTooShort);
    }

    // Extract payload slice
    let payload = &buf[MavHeader::SIZE..MavHeader::SIZE + payload_len];

    // Verify CRC (X.25)
    let crc_offset = MavHeader::SIZE + payload_len;
    let received_crc = (buf[crc_offset] as u16) | ((buf[crc_offset + 1] as u16) << 8);
    let computed_crc = compute_crc(&buf[1..crc_offset], get_crc_extra(header.msgid));

    if received_crc != computed_crc {
        return Err(ParseError::CrcMismatch);
    }

    // Parse message based on ID
    let msg = parse_message_payload(header.msgid, payload)?;

    Ok((msg, frame_size))
}

/// Parse message payload based on message ID
fn parse_message_payload(msg_id: u32, payload: &[u8]) -> Result<MavMessage, ParseError> {
    match msg_id {
        Heartbeat::MSG_ID => parse_heartbeat(payload),
        SystemTime::MSG_ID => parse_system_time(payload),
        HilSensor::MSG_ID => parse_hil_sensor(payload),
        HilGps::MSG_ID => parse_hil_gps(payload),
        HilActuatorControls::MSG_ID => parse_hil_actuator_controls(payload),
        HilStateQuaternion::MSG_ID => parse_hil_state_quaternion(payload),
        AttitudeQuaternion::MSG_ID => parse_attitude_quaternion(payload),
        LocalPositionNed::MSG_ID => parse_local_position_ned(payload),
        SetAttitudeTarget::MSG_ID => parse_set_attitude_target(payload),
        SetPositionTargetLocalNed::MSG_ID => parse_set_position_target_local_ned(payload),
        CommandLong::MSG_ID => parse_command_long(payload),
        CommandAck::MSG_ID => parse_command_ack(payload),
        RcChannelsOverride::MSG_ID => parse_rc_channels_override(payload),
        ManualControl::MSG_ID => parse_manual_control(payload),
        SysStatus::MSG_ID => parse_sys_status(payload),
        Statustext::MSG_ID => parse_statustext(payload),
        _ => Ok(MavMessage::Unknown { msg_id }),
    }
}

fn parse_heartbeat(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < Heartbeat::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::Heartbeat(Heartbeat {
        custom_mode: read_u32_le(payload, 0),
        mav_type: payload[4],
        autopilot: payload[5],
        base_mode: payload[6],
        system_status: payload[7],
        mavlink_version: payload[8],
    }))
}

fn parse_system_time(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < SystemTime::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::SystemTime(SystemTime {
        time_unix_usec: read_u64_le(payload, 0),
        time_boot_ms: read_u32_le(payload, 8),
    }))
}

fn parse_hil_sensor(payload: &[u8]) -> Result<MavMessage, ParseError> {
    // MAVLink 2.0 trims trailing zeros - minimum is 62 bytes (fields_updated ends at 64, but zeros trimmed)
    // Core fields end at offset 60 (fields_updated u32)
    if payload.len() < 62 {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::HilSensor(HilSensor {
        time_usec: read_u64_le(payload, 0),
        xacc: read_f32_le(payload, 8),
        yacc: read_f32_le(payload, 12),
        zacc: read_f32_le(payload, 16),
        xgyro: read_f32_le(payload, 20),
        ygyro: read_f32_le(payload, 24),
        zgyro: read_f32_le(payload, 28),
        xmag: read_f32_le(payload, 32),
        ymag: read_f32_le(payload, 36),
        zmag: read_f32_le(payload, 40),
        abs_pressure: read_f32_le(payload, 44),
        diff_pressure: read_f32_le(payload, 48),
        pressure_alt: read_f32_le(payload, 52),
        temperature: read_f32_le(payload, 56),
        fields_updated: if payload.len() >= 64 { read_u32_le(payload, 60) } else { 0xFFFF },
        id: if payload.len() > 64 { payload[64] } else { 0 },
    }))
}

fn parse_hil_gps(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 36 {
        // Minimum without optional fields
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::HilGps(HilGps {
        time_usec: read_u64_le(payload, 0),
        lat: read_i32_le(payload, 8),
        lon: read_i32_le(payload, 12),
        alt: read_i32_le(payload, 16),
        eph: read_u16_le(payload, 20),
        epv: read_u16_le(payload, 22),
        vel: read_u16_le(payload, 24),
        vn: read_i16_le(payload, 26),
        ve: read_i16_le(payload, 28),
        vd: read_i16_le(payload, 30),
        cog: read_u16_le(payload, 32),
        fix_type: payload[34],
        satellites_visible: payload[35],
        id: if payload.len() > 36 { payload[36] } else { 0 },
        yaw: if payload.len() > 38 {
            read_u16_le(payload, 37)
        } else {
            0
        },
    }))
}

fn parse_hil_actuator_controls(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < HilActuatorControls::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    let mut controls = [0.0f32; 16];
    for (i, ctrl) in controls.iter_mut().enumerate() {
        *ctrl = read_f32_le(payload, 8 + i * 4);
    }

    Ok(MavMessage::HilActuatorControls(HilActuatorControls {
        time_usec: read_u64_le(payload, 0),
        controls,
        mode: payload[72],
        flags: read_u64_le(payload, 73),
    }))
}

fn parse_hil_state_quaternion(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < HilStateQuaternion::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::HilStateQuaternion(HilStateQuaternion {
        time_usec: read_u64_le(payload, 0),
        attitude_quaternion: [
            read_f32_le(payload, 8),
            read_f32_le(payload, 12),
            read_f32_le(payload, 16),
            read_f32_le(payload, 20),
        ],
        rollspeed: read_f32_le(payload, 24),
        pitchspeed: read_f32_le(payload, 28),
        yawspeed: read_f32_le(payload, 32),
        lat: read_i32_le(payload, 36),
        lon: read_i32_le(payload, 40),
        alt: read_i32_le(payload, 44),
        vx: read_i16_le(payload, 48),
        vy: read_i16_le(payload, 50),
        vz: read_i16_le(payload, 52),
        ind_airspeed: read_u16_le(payload, 54),
        true_airspeed: read_u16_le(payload, 56),
        xacc: read_i16_le(payload, 58),
        yacc: read_i16_le(payload, 60),
        zacc: read_i16_le(payload, 62),
    }))
}

fn parse_attitude_quaternion(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 32 { // Basic payload
        return Err(ParseError::InvalidPayload);
    }
    
    let mut offset_q = [0.0; 4];
    if payload.len() >= 48 {
        offset_q[0] = read_f32_le(payload, 32);
        offset_q[1] = read_f32_le(payload, 36);
        offset_q[2] = read_f32_le(payload, 40);
        offset_q[3] = read_f32_le(payload, 44);
    }

    Ok(MavMessage::AttitudeQuaternion(AttitudeQuaternion {
        time_boot_ms: read_u32_le(payload, 0),
        q1: read_f32_le(payload, 4),
        q2: read_f32_le(payload, 8),
        q3: read_f32_le(payload, 12),
        q4: read_f32_le(payload, 16),
        rollspeed: read_f32_le(payload, 20),
        pitchspeed: read_f32_le(payload, 24),
        yawspeed: read_f32_le(payload, 28),
        repr_offset_q: offset_q,
    }))
}

fn parse_local_position_ned(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < LocalPositionNed::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    
    Ok(MavMessage::LocalPositionNed(LocalPositionNed {
        time_boot_ms: read_u32_le(payload, 0),
        x: read_f32_le(payload, 4),
        y: read_f32_le(payload, 8),
        z: read_f32_le(payload, 12),
        vx: read_f32_le(payload, 16),
        vy: read_f32_le(payload, 20),
        vz: read_f32_le(payload, 24),
    }))
}

fn parse_set_attitude_target(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 39 { // Basic payload
        return Err(ParseError::InvalidPayload);
    }
    
    let mut thrust_body = [0.0; 3];
    if payload.len() >= 51 {
        thrust_body[0] = read_f32_le(payload, 39);
        thrust_body[1] = read_f32_le(payload, 43);
        thrust_body[2] = read_f32_le(payload, 47);
    }

    Ok(MavMessage::SetAttitudeTarget(SetAttitudeTarget {
        time_boot_ms: read_u32_le(payload, 0),
        q: [
            read_f32_le(payload, 4),
            read_f32_le(payload, 8),
            read_f32_le(payload, 12),
            read_f32_le(payload, 16),
        ],
        body_roll_rate: read_f32_le(payload, 20),
        body_pitch_rate: read_f32_le(payload, 24),
        body_yaw_rate: read_f32_le(payload, 28),
        thrust: read_f32_le(payload, 32),
        target_system: payload[36],
        target_component: payload[37],
        type_mask: payload[38],
        thrust_body,
    }))
}

fn parse_set_position_target_local_ned(payload: &[u8]) -> Result<MavMessage, ParseError> {
    // MAVLink wire format: 14 floats (56 bytes) + u32 (4) + u16 (2) + 3 u8s (3) = 51 bytes minimum
    // Actual layout: time_boot_ms(4) + x/y/z/vx/vy/vz/afx/afy/afz/yaw/yaw_rate(44) + type_mask(2) + target_system(1) + target_component(1) + coordinate_frame(1) = 53
    if payload.len() < 51 {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
        time_boot_ms: read_u32_le(payload, 0),
        x: read_f32_le(payload, 4),
        y: read_f32_le(payload, 8),
        z: read_f32_le(payload, 12),
        vx: read_f32_le(payload, 16),
        vy: read_f32_le(payload, 20),
        vz: read_f32_le(payload, 24),
        afx: read_f32_le(payload, 28),
        afy: read_f32_le(payload, 32),
        afz: read_f32_le(payload, 36),
        yaw: read_f32_le(payload, 40),
        yaw_rate: read_f32_le(payload, 44),
        type_mask: read_u16_le(payload, 48),
        target_system: payload[50],
        target_component: if payload.len() > 51 { payload[51] } else { 0 },
        coordinate_frame: if payload.len() > 52 { payload[52] } else { 1 }, // Default to LOCAL_NED
    }))
}

fn parse_command_long(payload: &[u8]) -> Result<MavMessage, ParseError> {
    // MAVLink 2.0 trims trailing zeros - minimum is 32 bytes (without confirmation)
    if payload.len() < 32 {
        return Err(ParseError::InvalidPayload);
    }

    Ok(MavMessage::CommandLong(CommandLong {
        param1: read_f32_le(payload, 0),
        param2: read_f32_le(payload, 4),
        param3: read_f32_le(payload, 8),
        param4: read_f32_le(payload, 12),
        param5: read_f32_le(payload, 16),
        param6: read_f32_le(payload, 20),
        param7: read_f32_le(payload, 24),
        command: read_u16_le(payload, 28),
        target_system: payload[30],
        target_component: payload[31],
        confirmation: if payload.len() > 32 { payload[32] } else { 0 },
    }))
}

fn parse_command_ack(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 3 { // Basic: command(2) + result(1)
        return Err(ParseError::InvalidPayload);
    }
    
    let progress = if payload.len() > 3 { payload[3] } else { 0 };
    let result_param2 = if payload.len() > 7 { read_i32_le(payload, 4) } else { 0 };
    let target_system = if payload.len() > 8 { payload[8] } else { 0 };
    let target_component = if payload.len() > 9 { payload[9] } else { 0 };

    Ok(MavMessage::CommandAck(CommandAck {
        command: read_u16_le(payload, 0),
        result: payload[2],
        progress,
        result_param2,
        target_system,
        target_component,
    }))
}

fn parse_rc_channels_override(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 18 { // Basic
        return Err(ParseError::InvalidPayload);
    }

    let chan9_18_start = 18;
    let mut msg = RcChannelsOverride {
        chan1_raw: read_u16_le(payload, 0),
        chan2_raw: read_u16_le(payload, 2),
        chan3_raw: read_u16_le(payload, 4),
        chan4_raw: read_u16_le(payload, 6),
        chan5_raw: read_u16_le(payload, 8),
        chan6_raw: read_u16_le(payload, 10),
        chan7_raw: read_u16_le(payload, 12),
        chan8_raw: read_u16_le(payload, 14),
        target_system: payload[16],
        target_component: payload[17],
        ..Default::default()
    };

    if payload.len() >= 38 {
        msg.chan9_raw = read_u16_le(payload, chan9_18_start);
        msg.chan10_raw = read_u16_le(payload, chan9_18_start + 2);
        msg.chan11_raw = read_u16_le(payload, chan9_18_start + 4);
        msg.chan12_raw = read_u16_le(payload, chan9_18_start + 6);
        msg.chan13_raw = read_u16_le(payload, chan9_18_start + 8);
        msg.chan14_raw = read_u16_le(payload, chan9_18_start + 10);
        msg.chan15_raw = read_u16_le(payload, chan9_18_start + 12);
        msg.chan16_raw = read_u16_le(payload, chan9_18_start + 14);
        msg.chan17_raw = read_u16_le(payload, chan9_18_start + 16);
        msg.chan18_raw = read_u16_le(payload, chan9_18_start + 18);
    }

    Ok(MavMessage::RcChannelsOverride(msg))
}

fn parse_manual_control(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 11 { // Basic
        return Err(ParseError::InvalidPayload);
    }

    let mut msg = ManualControl {
        x: read_i16_le(payload, 0),
        y: read_i16_le(payload, 2),
        z: read_i16_le(payload, 4),
        r: read_i16_le(payload, 6),
        buttons: read_u16_le(payload, 8),
        target: payload[10],
        ..Default::default()
    };

    let ext_start = 11;
    if payload.len() >= ext_start + 2 { msg.s = read_i16_le(payload, ext_start); }
    if payload.len() >= ext_start + 4 { msg.t = read_i16_le(payload, ext_start + 2); }
    if payload.len() >= ext_start + 6 { msg.aux1 = read_i16_le(payload, ext_start + 4); }
    if payload.len() >= ext_start + 8 { msg.aux2 = read_i16_le(payload, ext_start + 6); }
    if payload.len() >= ext_start + 10 { msg.aux3 = read_i16_le(payload, ext_start + 8); }
    if payload.len() >= ext_start + 12 { msg.aux4 = read_i16_le(payload, ext_start + 10); }
    if payload.len() >= ext_start + 14 { msg.aux5 = read_i16_le(payload, ext_start + 12); }
    if payload.len() >= ext_start + 16 { msg.aux6 = read_i16_le(payload, ext_start + 14); }

    Ok(MavMessage::ManualControl(msg))
}

fn parse_sys_status(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 31 { // Basic (43 in messages.rs but check wire size)
        return Err(ParseError::InvalidPayload);
    }

    let mut msg = SysStatus {
        onboard_control_sensors_present: read_u32_le(payload, 0),
        onboard_control_sensors_enabled: read_u32_le(payload, 4),
        onboard_control_sensors_health: read_u32_le(payload, 8),
        load: read_u16_le(payload, 12),
        voltage_battery: read_u16_le(payload, 14),
        current_battery: read_i16_le(payload, 16),
        drop_rate_comm: read_u16_le(payload, 18),
        errors_comm: read_u16_le(payload, 20),
        errors_count1: read_u16_le(payload, 22),
        errors_count2: read_u16_le(payload, 24),
        errors_count3: read_u16_le(payload, 26),
        errors_count4: read_u16_le(payload, 28),
        battery_remaining: payload[30] as i8,
        ..Default::default()
    };

    if payload.len() >= 43 {
        msg.onboard_control_sensors_present_extended = read_u32_le(payload, 31);
        msg.onboard_control_sensors_enabled_extended = read_u32_le(payload, 35);
        msg.onboard_control_sensors_health_extended = read_u32_le(payload, 39);
    }

    Ok(MavMessage::SysStatus(msg))
}

fn parse_statustext(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < 51 { // Basic
        return Err(ParseError::InvalidPayload);
    }

    let mut text = [0u8; 50];
    text.copy_from_slice(&payload[1..51]);

    let mut msg = Statustext {
        severity: payload[0],
        text,
        id: 0,
        chunk_seq: 0,
    };

    if payload.len() >= 54 {
        msg.id = read_u16_le(payload, 51);
        msg.chunk_seq = payload[53];
    }

    Ok(MavMessage::Statustext(msg))
}

// Byte reading helpers (little-endian)
fn read_u16_le(buf: &[u8], offset: usize) -> u16 {
    (buf[offset] as u16) | ((buf[offset + 1] as u16) << 8)
}

fn read_i16_le(buf: &[u8], offset: usize) -> i16 {
    read_u16_le(buf, offset) as i16
}

fn read_u32_le(buf: &[u8], offset: usize) -> u32 {
    (buf[offset] as u32)
        | ((buf[offset + 1] as u32) << 8)
        | ((buf[offset + 2] as u32) << 16)
        | ((buf[offset + 3] as u32) << 24)
}

fn read_i32_le(buf: &[u8], offset: usize) -> i32 {
    read_u32_le(buf, offset) as i32
}

fn read_u64_le(buf: &[u8], offset: usize) -> u64 {
    (buf[offset] as u64)
        | ((buf[offset + 1] as u64) << 8)
        | ((buf[offset + 2] as u64) << 16)
        | ((buf[offset + 3] as u64) << 24)
        | ((buf[offset + 4] as u64) << 32)
        | ((buf[offset + 5] as u64) << 40)
        | ((buf[offset + 6] as u64) << 48)
        | ((buf[offset + 7] as u64) << 56)
}

fn read_f32_le(buf: &[u8], offset: usize) -> f32 {
    f32::from_bits(read_u32_le(buf, offset))
}

/// X.25 CRC calculation for MAVLink
fn compute_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;

    for &byte in data {
        crc = crc_accumulate(byte, crc);
    }

    // Include CRC extra byte
    crc = crc_accumulate(crc_extra, crc);

    crc
}

fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let tmp = (byte ^ (crc as u8)) as u16;
    let tmp = tmp ^ ((tmp << 4) & 0xFF);  // Mask to 8 bits per X.25 CRC spec
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

/// Get CRC extra byte for message ID (from MAVLink XML definitions)
fn get_crc_extra(msg_id: u32) -> u8 {
    match msg_id {
        0 => 50,   // HEARTBEAT
        1 => 124,  // SYS_STATUS
        2 => 137,  // SYSTEM_TIME
        31 => 246, // ATTITUDE_QUATERNION
        32 => 185, // LOCAL_POSITION_NED
        69 => 243, // MANUAL_CONTROL
        70 => 124, // RC_CHANNELS_OVERRIDE
        76 => 152, // COMMAND_LONG
        77 => 143, // COMMAND_ACK
        82 => 49,  // SET_ATTITUDE_TARGET
        84 => 143, // SET_POSITION_TARGET_LOCAL_NED
        93 => 47,  // HIL_ACTUATOR_CONTROLS
        107 => 108, // HIL_SENSOR
        113 => 124, // HIL_GPS
        115 => 4,  // HIL_STATE_QUATERNION
        253 => 83, // STATUSTEXT
        _ => 0,    // Unknown message
    }
}
