//! MAVLink v2 Wire Protocol
//!
//! Implements the MAVLink v2 framing protocol for packet serialization and parsing.
//! Reference: <https://mavlink.io/en/guide/serialization.html>
//!
//! Frame structure:
//! ```text
//! | STX | LEN | INC | CMP | SEQ | SYS | CMP | MSG_ID (3) | PAYLOAD | CRC (2) |
//! | 0xFD|  1  |  1  |  1  |  1  |  1  |  1  |     3      |  0-255  |    2    |
//! ```

use crate::messages::{
    Heartbeat, HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
    HEARTBEAT_ID, HIL_ACTUATOR_CONTROLS_ID, HIL_GPS_ID, HIL_SENSOR_ID, HIL_STATE_QUATERNION_ID,
};

/// MAVLink v2 start byte
pub const MAVLINK_STX_V2: u8 = 0xFD;

/// MAVLink v1 start byte (for compatibility)
pub const MAVLINK_STX_V1: u8 = 0xFE;

/// Minimum header size for v2 (STX + LEN + INC + CMP + SEQ + SYS + CMP + MSG_ID\[3\])
pub const HEADER_SIZE: usize = 10;

/// Header size for v1 (STX + LEN + SEQ + SYS + CMP + MSG_ID)
pub const HEADER_SIZE_V1: usize = 6;

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
const CRC_EXTRA_HEARTBEAT: u8 = 50;
const CRC_EXTRA_HIL_SENSOR: u8 = 108;
const CRC_EXTRA_HIL_GPS: u8 = 124;
const CRC_EXTRA_HIL_STATE_QUATERNION: u8 = 4;
const CRC_EXTRA_HIL_ACTUATOR_CONTROLS: u8 = 47;

/// Get CRC extra byte for a message ID
fn crc_extra(msg_id: u8) -> Option<u8> {
    match msg_id {
        HEARTBEAT_ID => Some(CRC_EXTRA_HEARTBEAT),
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
/// Uses the exact jMAVSim/MAVLink algorithm with proper truncation
#[inline]
fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let tmp = ((byte as u16) ^ crc) & 0xff;
    let tmp = tmp ^ ((tmp << 4) & 0xff);
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
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
    /// A well-formed frame carrying a message this dialect subset does
    /// not decode. The frame's total length travels with the error so a
    /// caller skips the WHOLE frame: advancing a single byte instead
    /// would rescan the frame's own payload as if it were a new frame,
    /// turning every unmodelled message on the link into a burst of
    /// spurious CRC failures.
    UnknownMessage {
        /// The undecoded message id.
        msg_id: u8,
        /// Bytes to advance to reach the next frame.
        consumed: usize,
    },
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

/// Parse a MAVLink frame from a buffer (supports both v1 and v2)
///
/// Returns the parsed frame and the number of bytes consumed.
pub fn parse_frame(data: &[u8]) -> Result<(MavFrame, usize), ParseError> {
    // Find start byte (either v1 or v2)
    let start_pos = data
        .iter()
        .position(|&b| b == MAVLINK_STX_V2 || b == MAVLINK_STX_V1);
    let start_pos = match start_pos {
        Some(pos) => pos,
        None => return Err(ParseError::Incomplete),
    };

    let data = &data[start_pos..];
    let is_v2 = data[0] == MAVLINK_STX_V2;

    if is_v2 {
        parse_frame_v2(data, start_pos)
    } else {
        parse_frame_v1(data, start_pos)
    }
}

/// Parse a MAVLink v2 frame
fn parse_frame_v2(data: &[u8], start_pos: usize) -> Result<(MavFrame, usize), ParseError> {
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

    // Get CRC extra for this message. An unmodelled message is still a
    // COMPLETE frame whose length the header already gave us.
    let extra = crc_extra(msg_id).ok_or(ParseError::UnknownMessage {
        msg_id,
        consumed: start_pos + frame_len,
    })?;

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

/// Parse a MAVLink v1 frame
fn parse_frame_v1(data: &[u8], start_pos: usize) -> Result<(MavFrame, usize), ParseError> {
    if data.len() < HEADER_SIZE_V1 {
        return Err(ParseError::Incomplete);
    }

    // Parse v1 header: STX(1) + LEN(1) + SEQ(1) + SYS(1) + CMP(1) + MSG_ID(1)
    let len = data[1] as usize;
    let seq = data[2];
    let sys_id = data[3];
    let comp_id = data[4];
    let msg_id = data[5];

    let frame_len = HEADER_SIZE_V1 + len + CRC_SIZE;
    if data.len() < frame_len {
        return Err(ParseError::Incomplete);
    }

    // Get CRC extra for this message. An unmodelled message is still a
    // COMPLETE frame whose length the header already gave us.
    let extra = crc_extra(msg_id).ok_or(ParseError::UnknownMessage {
        msg_id,
        consumed: start_pos + frame_len,
    })?;

    // Verify CRC (over header[1..6] + payload for v1)
    let crc_data = &data[1..HEADER_SIZE_V1 + len];
    let expected_crc = crc_calculate(crc_data, extra);
    let received_crc =
        u16::from_le_bytes([data[HEADER_SIZE_V1 + len], data[HEADER_SIZE_V1 + len + 1]]);

    if expected_crc != received_crc {
        return Err(ParseError::CrcMismatch);
    }

    // Parse payload
    let payload = &data[HEADER_SIZE_V1..HEADER_SIZE_V1 + len];
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
/// Restores a MAVLink 2 payload to its full message length.
///
/// A v2 sender TRUNCATES trailing zero bytes, so a payload arrives
/// shorter than the message whenever its last fields are zero — a
/// stationary vehicle's sensor id, an unset extension. The truncated
/// bytes ARE zeros by definition, so the decoder is handed a
/// zero-extended copy. Refusing the short payload instead would reject
/// most of a real link's traffic and, worse, resync byte by byte
/// through frames that were never corrupt.
fn zero_extended<const N: usize>(payload: &[u8]) -> [u8; N] {
    let mut full = [0u8; N];
    let len = payload.len().min(N);
    full[..len].copy_from_slice(&payload[..len]);
    full
}

fn parse_message(msg_id: u8, payload: &[u8]) -> Result<HilMessage, ParseError> {
    match msg_id {
        HEARTBEAT_ID => {
            let full = zero_extended::<{ Heartbeat::SIZE }>(payload);
            let msg = Heartbeat::from_bytes(&full).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::Heartbeat(msg))
        }
        HIL_SENSOR_ID => {
            let full = zero_extended::<{ HilSensor::SIZE }>(payload);
            let msg = HilSensor::from_bytes(&full).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::Sensor(msg))
        }
        HIL_GPS_ID => {
            // The GNSS message's own extension fields are already
            // length-gated inside its decoder, so it takes the payload
            // as received.
            let msg = HilGps::from_bytes(payload).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::Gps(msg))
        }
        HIL_STATE_QUATERNION_ID => {
            let full = zero_extended::<{ HilStateQuaternion::SIZE }>(payload);
            let msg = HilStateQuaternion::from_bytes(&full).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::StateQuaternion(msg))
        }
        HIL_ACTUATOR_CONTROLS_ID => {
            let full = zero_extended::<{ HilActuatorControls::SIZE }>(payload);
            let msg = HilActuatorControls::from_bytes(&full).ok_or(ParseError::InvalidPayload)?;
            Ok(HilMessage::ActuatorControls(msg))
        }
        // Reached only when a crc_extra entry exists without a decoder;
        // the caller cannot know the frame length here, so it resyncs.
        _ => Err(ParseError::InvalidPayload),
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
        HilMessage::Heartbeat(m) => (HEARTBEAT_ID, m.to_bytes().to_vec()),
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

    fn serialize_parse(msg: &HilMessage) -> Option<(MavFrame, usize)> {
        let mut buf = [0u8; 256];
        let len = serialize_frame(msg, 1, 1, 1, &mut buf);
        assert!(len.is_some());
        let len = len?;

        let frame = parse_frame(&buf[..len]);
        assert!(frame.is_ok());
        let Ok((frame, consumed)) = frame else {
            return None;
        };
        Some((frame, consumed))
    }

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
        let len = serialize_frame(&msg, 42, 1, 1, &mut buf);
        assert!(len.is_some());
        let Some(len) = len else {
            return;
        };

        let frame = parse_frame(&buf[..len]);
        assert!(frame.is_ok());
        let Ok((frame, consumed)) = frame else {
            return;
        };
        assert_eq!(consumed, len);
        assert_eq!(frame.seq, 42);
        assert_eq!(frame.sys_id, 1);
        assert_eq!(frame.comp_id, 1);

        assert!(matches!(&frame.message, HilMessage::Sensor(_)));
        let HilMessage::Sensor(parsed) = frame.message else {
            return;
        };
        assert_eq!(sensor.time_usec, parsed.time_usec);
        assert!((sensor.zacc - parsed.zacc).abs() < 1e-6);
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
        let Some((frame, _)) = serialize_parse(&msg) else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::Gps(_)));
        let HilMessage::Gps(parsed) = frame.message else {
            return;
        };
        assert_eq!(gps.lat, parsed.lat);
        assert_eq!(gps.lon, parsed.lon);
    }

    #[test]
    fn test_actuator_controls_roundtrip() {
        let mut controls = HilActuatorControls {
            time_usec: 1234567890,
            ..Default::default()
        };
        controls.controls[0] = 0.5;
        controls.controls[1] = 0.6;
        controls.mode = HilActuatorControls::MODE_FLAG_ARMED;

        let msg = HilMessage::ActuatorControls(controls);
        let Some((frame, _)) = serialize_parse(&msg) else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::ActuatorControls(_)));
        let HilMessage::ActuatorControls(parsed) = frame.message else {
            return;
        };
        assert!(parsed.is_armed());
        assert!((controls.controls[0] - parsed.controls[0]).abs() < 1e-6);
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
        let Some((frame, _)) = serialize_parse(&msg) else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::StateQuaternion(_)));
        let HilMessage::StateQuaternion(parsed) = frame.message else {
            return;
        };
        assert_eq!(state.time_usec, parsed.time_usec);
        assert!((state.attitude_quaternion[0] - parsed.attitude_quaternion[0]).abs() < 1e-6);
        assert!((state.attitude_quaternion[2] - parsed.attitude_quaternion[2]).abs() < 1e-6);
        assert_eq!(state.lat, parsed.lat);
        assert_eq!(state.lon, parsed.lon);
        assert_eq!(state.zacc, parsed.zacc);
    }

    #[test]
    fn test_heartbeat_roundtrip() {
        let heartbeat = Heartbeat::new_quadrotor_hil(true);

        let msg = HilMessage::Heartbeat(heartbeat);
        let Some((frame, _)) = serialize_parse(&msg) else {
            return;
        };
        assert!(matches!(&frame.message, HilMessage::Heartbeat(_)));
        let HilMessage::Heartbeat(parsed) = frame.message else {
            return;
        };
        assert_eq!(heartbeat.mav_type, parsed.mav_type);
        assert_eq!(heartbeat.autopilot, parsed.autopilot);
        assert_eq!(heartbeat.base_mode, parsed.base_mode);
    }

    #[test]
    fn test_parse_v1_heartbeat() {
        // Build a MAVLink v1 HEARTBEAT frame manually
        // v1 format: STX(0xFE) + LEN + SEQ + SYS + CMP + MSG_ID + PAYLOAD + CRC
        let heartbeat = Heartbeat::new_quadrotor_hil(false);
        let payload = heartbeat.to_bytes();

        let mut buf = [0u8; 64];
        buf[0] = MAVLINK_STX_V1; // v1 start
        buf[1] = Heartbeat::SIZE as u8;
        buf[2] = 42; // seq
        buf[3] = 1; // sys_id
        buf[4] = 1; // comp_id
        buf[5] = HEARTBEAT_ID;
        buf[6..6 + Heartbeat::SIZE].copy_from_slice(&payload);

        // Calculate CRC over header[1..6] + payload
        let crc_data = &buf[1..6 + Heartbeat::SIZE];
        let crc = crc_calculate(crc_data, 50); // HEARTBEAT CRC extra = 50
        let crc_offset = 6 + Heartbeat::SIZE;
        buf[crc_offset..crc_offset + 2].copy_from_slice(&crc.to_le_bytes());

        let frame_len = 6 + Heartbeat::SIZE + 2;
        let frame = parse_frame(&buf[..frame_len]);
        assert!(frame.is_ok());
        let Ok((frame, consumed)) = frame else {
            return;
        };
        assert_eq!(consumed, frame_len);
        assert_eq!(frame.seq, 42);
        assert_eq!(frame.sys_id, 1);

        assert!(matches!(&frame.message, HilMessage::Heartbeat(_)));
        let HilMessage::Heartbeat(parsed) = frame.message else {
            return;
        };
        assert_eq!(parsed.mav_type, heartbeat.mav_type);
    }
}

#[cfg(test)]
mod unknown_message_tests {
    use super::{parse_frame, serialize_frame, HilMessage, HilSensor, ParseError};

    /// Builds a well-formed v2 frame for a message id this subset does
    /// not decode, using a CRC extra the parser will never resolve.
    fn undecodable_frame() -> Vec<u8> {
        // HIL_RC_INPUTS_RAW (92) is on the wire of real HIL bridges and
        // is deliberately not in this subset.
        let payload = [0u8; 33];
        let mut frame = vec![0xFD, payload.len() as u8, 0, 0, 0, 1, 1, 92, 0, 0];
        frame.extend_from_slice(&payload);
        frame.extend_from_slice(&[0, 0]); // CRC; never checked for an unknown id
        frame
    }

    #[test]
    fn an_undecodable_frame_reports_its_whole_length() {
        let frame = undecodable_frame();
        let parsed = parse_frame(&frame);
        assert!(matches!(
            parsed,
            Err(ParseError::UnknownMessage { msg_id: 92, .. })
        ));
        let Err(ParseError::UnknownMessage { consumed, .. }) = parsed else {
            return;
        };
        assert_eq!(
            consumed,
            frame.len(),
            "the caller must be able to skip the frame, not rescan its payload"
        );
    }

    #[test]
    fn a_known_frame_after_an_unknown_one_still_decodes() {
        // This is the failure the length matters for: a link carrying an
        // unmodelled message between sensor samples must not shred the
        // samples that follow it.
        let sensor = HilSensor {
            time_usec: 4_242,
            fields_updated: 0xFFFF_FFFF,
            ..HilSensor::default()
        };
        let mut buf = [0u8; 512];
        let len = serialize_frame(&HilMessage::Sensor(sensor), 0, 1, 1, &mut buf);
        assert!(len.is_some());
        let Some(len) = len else {
            return;
        };

        let mut stream = undecodable_frame();
        let skip = stream.len();
        stream.extend_from_slice(&buf[..len]);

        let parsed = parse_frame(&stream[skip..]);
        assert!(parsed.is_ok());
        let Ok((frame, _)) = parsed else {
            return;
        };
        assert!(matches!(frame.message, HilMessage::Sensor(_)));
    }
}

#[cfg(test)]
mod truncation_tests {
    use super::{parse_frame, serialize_frame, HilMessage, HilSensor};

    #[test]
    fn a_v2_truncated_payload_still_decodes() {
        // A v2 sender drops trailing zero bytes. A stationary vehicle's
        // sensor id is zero, so most real sensor frames arrive short —
        // and refusing them would reject most of the link's traffic.
        let sensor = HilSensor {
            time_usec: 1_234_567,
            zacc: -9.81,
            fields_updated: 0x0000_003F,
            id: 0,
            ..HilSensor::default()
        };
        let mut buf = [0u8; 512];
        let len = serialize_frame(&HilMessage::Sensor(sensor), 0, 1, 1, &mut buf);
        assert!(len.is_some());
        let Some(len) = len else {
            return;
        };

        // Rebuild the frame with the payload's trailing zero byte (the
        // sensor id) removed, exactly as a v2 sender would emit it.
        let mut truncated = buf[..len].to_vec();
        let payload_len = usize::from(truncated[1]);
        let crc_start = 10 + payload_len;
        let crc = crc_start + 2;
        assert_eq!(truncated.len(), crc);
        truncated.remove(crc_start - 1);
        truncated[1] = u8::try_from(payload_len - 1).unwrap_or(0);
        // Recompute the CRC over the shortened payload.
        let recomputed = super::crc_calculate(&truncated[1..10 + payload_len - 1], 108);
        let tail = truncated.len() - 2;
        truncated[tail..].copy_from_slice(&recomputed.to_le_bytes());

        let parsed = parse_frame(&truncated);
        assert!(parsed.is_ok(), "a truncated frame must decode: {parsed:?}");
        let Ok((frame, _)) = parsed else {
            return;
        };
        let HilMessage::Sensor(decoded) = frame.message else {
            return;
        };
        assert_eq!(decoded.time_usec, 1_234_567);
        assert!((decoded.zacc - (-9.81)).abs() < 1e-6);
        assert_eq!(decoded.id, 0, "the truncated byte reads back as zero");
    }
}
