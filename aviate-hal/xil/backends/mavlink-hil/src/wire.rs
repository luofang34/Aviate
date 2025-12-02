//! MAVLink v2 Wire Protocol
//!
//! Implements the MAVLink v2 framing protocol for packet serialization and parsing.
//! Reference: https://mavlink.io/en/guide/serialization.html
//!
//! Frame structure:
//! ```text
//! | STX | LEN | INC | CMP | SEQ | SYS | CMP | MSG_ID (3) | PAYLOAD | CRC (2) |
//! | 0xFD|  1  |  1  |  1  |  1  |  1  |  1  |     3      |  0-255  |    2    |
//! ```

use crate::messages::{
    HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
    HIL_ACTUATOR_CONTROLS_ID, HIL_GPS_ID, HIL_SENSOR_ID, HIL_STATE_QUATERNION_ID,
};

/// MAVLink v2 start byte
pub const MAVLINK_STX_V2: u8 = 0xFD;

/// MAVLink v1 start byte (for compatibility)
pub const MAVLINK_STX_V1: u8 = 0xFE;

/// Minimum header size (STX + LEN + INC + CMP + SEQ + SYS + CMP + MSG_ID[3])
pub const HEADER_SIZE: usize = 10;

/// CRC size
pub const CRC_SIZE: usize = 2;

/// Maximum payload size
pub const MAX_PAYLOAD_SIZE: usize = 255;

/// Maximum frame size
pub const MAX_FRAME_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;

/// CRC-16/MCRF4XX seed
const CRC_INIT: u16 = 0xFFFF;

/// CRC extra bytes for each message type (from MAVLink message definitions)
/// These are used in the CRC calculation to ensure message format compatibility
const CRC_EXTRA_HIL_SENSOR: u8 = 108;
const CRC_EXTRA_HIL_GPS: u8 = 124;
const CRC_EXTRA_HIL_STATE_QUATERNION: u8 = 4;
const CRC_EXTRA_HIL_ACTUATOR_CONTROLS: u8 = 47;

/// Get CRC extra byte for a message ID
fn crc_extra(msg_id: u8) -> Option<u8> {
    match msg_id {
        HIL_SENSOR_ID => Some(CRC_EXTRA_HIL_SENSOR),
        HIL_GPS_ID => Some(CRC_EXTRA_HIL_GPS),
        HIL_STATE_QUATERNION_ID => Some(CRC_EXTRA_HIL_STATE_QUATERNION),
        HIL_ACTUATOR_CONTROLS_ID => Some(CRC_EXTRA_HIL_ACTUATOR_CONTROLS),
        _ => None,
    }
}

/// X.25 CRC (CRC-16/MCRF4XX)
pub fn crc_calculate(data: &[u8], extra: u8) -> u16 {
    let mut crc = CRC_INIT;
    for &byte in data {
        crc = crc_accumulate(byte, crc);
    }
    crc = crc_accumulate(extra, crc);
    crc
}

/// Accumulate one byte into CRC
#[inline]
fn crc_accumulate(byte: u8, mut crc: u16) -> u16 {
    let tmp = (byte ^ (crc as u8)) as u16;
    let tmp = tmp ^ (tmp << 4);
    crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
    crc
}

/// Parse error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// Not enough data
    Incomplete,
    /// Invalid start byte
    InvalidStartByte,
    /// CRC mismatch
    CrcMismatch,
    /// Unknown message ID
    UnknownMessageId(u8),
    /// Invalid payload
    InvalidPayload,
}

/// Parsed MAVLink frame
#[derive(Debug, Clone)]
pub struct MavFrame {
    /// Sequence number
    pub seq: u8,
    /// System ID
    pub sys_id: u8,
    /// Component ID
    pub comp_id: u8,
    /// Message
    pub message: HilMessage,
}

/// Parse a MAVLink v2 frame from a buffer
///
/// Returns the parsed frame and the number of bytes consumed.
pub fn parse_frame(data: &[u8]) -> Result<(MavFrame, usize), ParseError> {
    // Find start byte
    let start_pos = data.iter().position(|&b| b == MAVLINK_STX_V2);
    let start_pos = match start_pos {
        Some(pos) => pos,
        None => return Err(ParseError::Incomplete),
    };

    let data = &data[start_pos..];
    if data.len() < HEADER_SIZE {
        return Err(ParseError::Incomplete);
    }

    // Parse header
    let len = data[1] as usize;
    let _incompat_flags = data[2];
    let _compat_flags = data[3];
    let seq = data[4];
    let sys_id = data[5];
    let comp_id = data[6];
    let msg_id = data[7]; // Only use first byte for common messages (<256)

    let frame_len = HEADER_SIZE + len + CRC_SIZE;
    if data.len() < frame_len {
        return Err(ParseError::Incomplete);
    }

    // Get CRC extra for this message
    let extra = crc_extra(msg_id).ok_or(ParseError::UnknownMessageId(msg_id))?;

    // Verify CRC (over header[1..10] + payload)
    let crc_data = &data[1..HEADER_SIZE + len];
    let expected_crc = crc_calculate(crc_data, extra);
    let received_crc = u16::from_le_bytes([data[HEADER_SIZE + len], data[HEADER_SIZE + len + 1]]);

    if expected_crc != received_crc {
        return Err(ParseError::CrcMismatch);
    }

    // Parse payload
    let payload = &data[HEADER_SIZE..HEADER_SIZE + len];
    let message = parse_message(msg_id, payload)?;

    Ok((
        MavFrame {
            seq,
            sys_id,
            comp_id,
            message,
        },
        start_pos + frame_len,
    ))
}

/// Parse message payload
fn parse_message(msg_id: u8, payload: &[u8]) -> Result<HilMessage, ParseError> {
    match msg_id {
        HIL_SENSOR_ID => {
            let msg = HilSensor::from_bytes(payload).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::Sensor(msg))
        }
        HIL_GPS_ID => {
            let msg = HilGps::from_bytes(payload).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::Gps(msg))
        }
        HIL_STATE_QUATERNION_ID => {
            let msg = HilStateQuaternion::from_bytes(payload).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::StateQuaternion(msg))
        }
        HIL_ACTUATOR_CONTROLS_ID => {
            let msg = HilActuatorControls::from_bytes(payload).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::ActuatorControls(msg))
        }
        _ => Err(ParseError::UnknownMessageId(msg_id)),
    }
}

/// Serialize a HIL message to a MAVLink v2 frame
///
/// Returns the number of bytes written.
pub fn serialize_frame(
    msg: &HilMessage,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let (msg_id, payload_bytes) = match msg {
        HilMessage::Sensor(m) => (HIL_SENSOR_ID, m.to_bytes().to_vec()),
        HilMessage::Gps(m) => (HIL_GPS_ID, m.to_bytes().to_vec()),
        HilMessage::StateQuaternion(m) => (HIL_STATE_QUATERNION_ID, m.to_bytes().to_vec()),
        HilMessage::ActuatorControls(m) => (HIL_ACTUATOR_CONTROLS_ID, m.to_bytes().to_vec()),
    };

    let payload_len = payload_bytes.len();
    let frame_len = HEADER_SIZE + payload_len + CRC_SIZE;

    if buf.len() < frame_len {
        return None;
    }

    // Header
    buf[0] = MAVLINK_STX_V2;
    buf[1] = payload_len as u8;
    buf[2] = 0; // incompat_flags
    buf[3] = 0; // compat_flags
    buf[4] = seq;
    buf[5] = sys_id;
    buf[6] = comp_id;
    buf[7] = msg_id;
    buf[8] = 0; // msg_id high bytes (not used for common messages)
    buf[9] = 0;

    // Payload
    buf[HEADER_SIZE..HEADER_SIZE + payload_len].copy_from_slice(&payload_bytes);

    // CRC
    let extra = crc_extra(msg_id)?;
    let crc = crc_calculate(&buf[1..HEADER_SIZE + payload_len], extra);
    buf[HEADER_SIZE + payload_len..HEADER_SIZE + payload_len + 2]
        .copy_from_slice(&crc.to_le_bytes());

    Some(frame_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc_accumulate() {
        // Test known CRC calculation
        let mut crc = CRC_INIT;
        for &b in b"hello" {
            crc = crc_accumulate(b, crc);
        }
        // Just verify it produces a non-zero value
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_serialize_parse_roundtrip() {
        let sensor = HilSensor {
            time_usec: 1234567890,
            xacc: 0.1,
            yacc: 0.2,
            zacc: -9.81,
            xgyro: 0.01,
            ygyro: 0.02,
            zgyro: 0.03,
            xmag: 0.2,
            ymag: 0.0,
            zmag: 0.4,
            abs_pressure: 1013.25,
            diff_pressure: 0.0,
            pressure_alt: 100.0,
            temperature: 25.0,
            fields_updated: 0xFFFF_FFFF,
            id: 0,
        };

        let msg = HilMessage::Sensor(sensor);
        let mut buf = [0u8; 256];
        let len = serialize_frame(&msg, 42, 1, 1, &mut buf).expect("serialize failed");

        let (frame, consumed) = parse_frame(&buf[..len]).expect("parse failed");
        assert_eq!(consumed, len);
        assert_eq!(frame.seq, 42);
        assert_eq!(frame.sys_id, 1);
        assert_eq!(frame.comp_id, 1);

        if let HilMessage::Sensor(parsed) = frame.message {
            assert_eq!(sensor.time_usec, parsed.time_usec);
            assert!((sensor.zacc - parsed.zacc).abs() < 1e-6);
        } else {
            panic!("Wrong message type");
        }
    }

    #[test]
    fn test_parse_incomplete() {
        let buf = [MAVLINK_STX_V2, 0x10, 0x00]; // Incomplete header
        assert!(matches!(parse_frame(&buf), Err(ParseError::Incomplete)));
    }

    #[test]
    fn test_parse_no_start_byte() {
        let buf = [0x00, 0x01, 0x02, 0x03];
        assert!(matches!(parse_frame(&buf), Err(ParseError::Incomplete)));
    }

    #[test]
    fn test_gps_roundtrip() {
        let gps = HilGps {
            time_usec: 1234567890,
            lat: 473977420,
            lon: 85455940,
            alt: 488000,
            eph: 100,
            epv: 150,
            vel: 500,
            vn: 100,
            ve: 200,
            vd: -50,
            cog: 9000,
            fix_type: 3,
            satellites_visible: 12,
            id: 0,
            yaw: 0,
        };

        let msg = HilMessage::Gps(gps);
        let mut buf = [0u8; 256];
        let len = serialize_frame(&msg, 1, 1, 1, &mut buf).expect("serialize failed");

        let (frame, _) = parse_frame(&buf[..len]).expect("parse failed");
        if let HilMessage::Gps(parsed) = frame.message {
            assert_eq!(gps.lat, parsed.lat);
            assert_eq!(gps.lon, parsed.lon);
        } else {
            panic!("Wrong message type");
        }
    }

    #[test]
    fn test_actuator_controls_roundtrip() {
        let mut controls = HilActuatorControls::default();
        controls.time_usec = 1234567890;
        controls.controls[0] = 0.5;
        controls.controls[1] = 0.6;
        controls.mode = HilActuatorControls::MODE_FLAG_ARMED;

        let msg = HilMessage::ActuatorControls(controls);
        let mut buf = [0u8; 256];
        let len = serialize_frame(&msg, 1, 1, 1, &mut buf).expect("serialize failed");

        let (frame, _) = parse_frame(&buf[..len]).expect("parse failed");
        if let HilMessage::ActuatorControls(parsed) = frame.message {
            assert!(parsed.is_armed());
            assert!((controls.controls[0] - parsed.controls[0]).abs() < 1e-6);
        } else {
            panic!("Wrong message type");
        }
    }

    #[test]
    fn test_state_quaternion_roundtrip() {
        let state = HilStateQuaternion {
            time_usec: 1234567890,
            attitude_quaternion: [0.707, 0.0, 0.707, 0.0], // 90 deg pitch
            rollspeed: 0.01,
            pitchspeed: 0.02,
            yawspeed: 0.03,
            lat: 473977420,
            lon: 85455940,
            alt: 488000,
            vx: 100,
            vy: 200,
            vz: -50,
            ind_airspeed: 1500,
            true_airspeed: 1550,
            xacc: 0,
            yacc: 0,
            zacc: -1000,
        };

        let msg = HilMessage::StateQuaternion(state);
        let mut buf = [0u8; 256];
        let len = serialize_frame(&msg, 1, 1, 1, &mut buf).expect("serialize failed");

        let (frame, _) = parse_frame(&buf[..len]).expect("parse failed");
        if let HilMessage::StateQuaternion(parsed) = frame.message {
            assert_eq!(state.time_usec, parsed.time_usec);
            assert!((state.attitude_quaternion[0] - parsed.attitude_quaternion[0]).abs() < 1e-6);
            assert!((state.attitude_quaternion[2] - parsed.attitude_quaternion[2]).abs() < 1e-6);
            assert_eq!(state.lat, parsed.lat);
            assert_eq!(state.lon, parsed.lon);
            assert_eq!(state.zacc, parsed.zacc);
        } else {
            panic!("Wrong message type");
        }
    }
}
