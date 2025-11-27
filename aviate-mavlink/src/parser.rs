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
    if payload.len() < HilSensor::PAYLOAD_LEN {
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
        fields_updated: read_u32_le(payload, 60),
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
    let tmp = tmp ^ (tmp << 4);
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

/// Get CRC extra byte for message ID (from MAVLink XML definitions)
fn get_crc_extra(msg_id: u32) -> u8 {
    match msg_id {
        0 => 50,   // HEARTBEAT
        2 => 137,  // SYSTEM_TIME
        93 => 47,  // HIL_ACTUATOR_CONTROLS
        107 => 108, // HIL_SENSOR
        113 => 124, // HIL_GPS
        115 => 4,  // HIL_STATE_QUATERNION
        _ => 0,    // Unknown message
    }
}
