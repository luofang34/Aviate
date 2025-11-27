//! MAVLink message serialization
//!
//! Serializes MAVLink 2.0 frames to byte buffers.

use crate::messages::*;
use crate::{MAVLINK_STX_V2, AVIATE_SYSTEM_ID, AVIATE_COMPONENT_ID};
use crate::parser::MavHeader;

/// Serialize a MAVLink message to a byte buffer
///
/// Returns the number of bytes written, or None if buffer is too small.
pub fn serialize_mavlink(msg: &MavMessage, seq: u8, buf: &mut [u8]) -> Option<usize> {
    match msg {
        MavMessage::Heartbeat(m) => serialize_heartbeat(m, seq, buf),
        MavMessage::SystemTime(m) => serialize_system_time(m, seq, buf),
        MavMessage::HilSensor(m) => serialize_hil_sensor(m, seq, buf),
        MavMessage::HilGps(m) => serialize_hil_gps(m, seq, buf),
        MavMessage::HilActuatorControls(m) => serialize_hil_actuator_controls(m, seq, buf),
        MavMessage::HilStateQuaternion(m) => serialize_hil_state_quaternion(m, seq, buf),
        MavMessage::Unknown { .. } => None,
    }
}

fn write_header(buf: &mut [u8], payload_len: u8, seq: u8, msgid: u32) -> usize {
    buf[0] = MAVLINK_STX_V2;
    buf[1] = payload_len;
    buf[2] = 0; // incompat_flags
    buf[3] = 0; // compat_flags
    buf[4] = seq;
    buf[5] = AVIATE_SYSTEM_ID;
    buf[6] = AVIATE_COMPONENT_ID;
    buf[7] = (msgid & 0xFF) as u8;
    buf[8] = ((msgid >> 8) & 0xFF) as u8;
    buf[9] = ((msgid >> 16) & 0xFF) as u8;
    MavHeader::SIZE
}

fn write_crc(buf: &mut [u8], offset: usize, crc_extra: u8) -> usize {
    let crc = compute_crc(&buf[1..offset], crc_extra);
    buf[offset] = (crc & 0xFF) as u8;
    buf[offset + 1] = ((crc >> 8) & 0xFF) as u8;
    offset + 2
}

fn serialize_heartbeat(msg: &Heartbeat, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + Heartbeat::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, Heartbeat::PAYLOAD_LEN as u8, seq, Heartbeat::MSG_ID);

    // Payload
    write_u32_le(buf, offset, msg.custom_mode);
    buf[offset + 4] = msg.mav_type;
    buf[offset + 5] = msg.autopilot;
    buf[offset + 6] = msg.base_mode;
    buf[offset + 7] = msg.system_status;
    buf[offset + 8] = msg.mavlink_version;

    Some(write_crc(buf, offset + Heartbeat::PAYLOAD_LEN, 50))
}

fn serialize_system_time(msg: &SystemTime, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SystemTime::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, SystemTime::PAYLOAD_LEN as u8, seq, SystemTime::MSG_ID);

    write_u64_le(buf, offset, msg.time_unix_usec);
    write_u32_le(buf, offset + 8, msg.time_boot_ms);

    Some(write_crc(buf, offset + SystemTime::PAYLOAD_LEN, 137))
}

fn serialize_hil_sensor(msg: &HilSensor, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + HilSensor::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, HilSensor::PAYLOAD_LEN as u8, seq, HilSensor::MSG_ID);

    write_u64_le(buf, offset, msg.time_usec);
    write_f32_le(buf, offset + 8, msg.xacc);
    write_f32_le(buf, offset + 12, msg.yacc);
    write_f32_le(buf, offset + 16, msg.zacc);
    write_f32_le(buf, offset + 20, msg.xgyro);
    write_f32_le(buf, offset + 24, msg.ygyro);
    write_f32_le(buf, offset + 28, msg.zgyro);
    write_f32_le(buf, offset + 32, msg.xmag);
    write_f32_le(buf, offset + 36, msg.ymag);
    write_f32_le(buf, offset + 40, msg.zmag);
    write_f32_le(buf, offset + 44, msg.abs_pressure);
    write_f32_le(buf, offset + 48, msg.diff_pressure);
    write_f32_le(buf, offset + 52, msg.pressure_alt);
    write_f32_le(buf, offset + 56, msg.temperature);
    write_u32_le(buf, offset + 60, msg.fields_updated);
    buf[offset + 64] = msg.id;

    Some(write_crc(buf, offset + HilSensor::PAYLOAD_LEN, 108))
}

fn serialize_hil_gps(msg: &HilGps, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + HilGps::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, HilGps::PAYLOAD_LEN as u8, seq, HilGps::MSG_ID);

    write_u64_le(buf, offset, msg.time_usec);
    write_i32_le(buf, offset + 8, msg.lat);
    write_i32_le(buf, offset + 12, msg.lon);
    write_i32_le(buf, offset + 16, msg.alt);
    write_u16_le(buf, offset + 20, msg.eph);
    write_u16_le(buf, offset + 22, msg.epv);
    write_u16_le(buf, offset + 24, msg.vel);
    write_i16_le(buf, offset + 26, msg.vn);
    write_i16_le(buf, offset + 28, msg.ve);
    write_i16_le(buf, offset + 30, msg.vd);
    write_u16_le(buf, offset + 32, msg.cog);
    buf[offset + 34] = msg.fix_type;
    buf[offset + 35] = msg.satellites_visible;
    buf[offset + 36] = msg.id;
    write_u16_le(buf, offset + 37, msg.yaw);

    Some(write_crc(buf, offset + HilGps::PAYLOAD_LEN, 124))
}

fn serialize_hil_actuator_controls(msg: &HilActuatorControls, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + HilActuatorControls::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, HilActuatorControls::PAYLOAD_LEN as u8, seq, HilActuatorControls::MSG_ID);

    write_u64_le(buf, offset, msg.time_usec);
    for i in 0..16 {
        write_f32_le(buf, offset + 8 + i * 4, msg.controls[i]);
    }
    buf[offset + 72] = msg.mode;
    write_u64_le(buf, offset + 73, msg.flags);

    Some(write_crc(buf, offset + HilActuatorControls::PAYLOAD_LEN, 47))
}

fn serialize_hil_state_quaternion(msg: &HilStateQuaternion, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + HilStateQuaternion::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, HilStateQuaternion::PAYLOAD_LEN as u8, seq, HilStateQuaternion::MSG_ID);

    write_u64_le(buf, offset, msg.time_usec);
    for i in 0..4 {
        write_f32_le(buf, offset + 8 + i * 4, msg.attitude_quaternion[i]);
    }
    write_f32_le(buf, offset + 24, msg.rollspeed);
    write_f32_le(buf, offset + 28, msg.pitchspeed);
    write_f32_le(buf, offset + 32, msg.yawspeed);
    write_i32_le(buf, offset + 36, msg.lat);
    write_i32_le(buf, offset + 40, msg.lon);
    write_i32_le(buf, offset + 44, msg.alt);
    write_i16_le(buf, offset + 48, msg.vx);
    write_i16_le(buf, offset + 50, msg.vy);
    write_i16_le(buf, offset + 52, msg.vz);
    write_u16_le(buf, offset + 54, msg.ind_airspeed);
    write_u16_le(buf, offset + 56, msg.true_airspeed);
    write_i16_le(buf, offset + 58, msg.xacc);
    write_i16_le(buf, offset + 60, msg.yacc);
    write_i16_le(buf, offset + 62, msg.zacc);

    Some(write_crc(buf, offset + HilStateQuaternion::PAYLOAD_LEN, 4))
}

// Byte writing helpers (little-endian)
fn write_u16_le(buf: &mut [u8], offset: usize, val: u16) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
}

fn write_i16_le(buf: &mut [u8], offset: usize, val: i16) {
    write_u16_le(buf, offset, val as u16);
}

fn write_u32_le(buf: &mut [u8], offset: usize, val: u32) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
    buf[offset + 2] = ((val >> 16) & 0xFF) as u8;
    buf[offset + 3] = ((val >> 24) & 0xFF) as u8;
}

fn write_i32_le(buf: &mut [u8], offset: usize, val: i32) {
    write_u32_le(buf, offset, val as u32);
}

fn write_u64_le(buf: &mut [u8], offset: usize, val: u64) {
    buf[offset] = (val & 0xFF) as u8;
    buf[offset + 1] = ((val >> 8) & 0xFF) as u8;
    buf[offset + 2] = ((val >> 16) & 0xFF) as u8;
    buf[offset + 3] = ((val >> 24) & 0xFF) as u8;
    buf[offset + 4] = ((val >> 32) & 0xFF) as u8;
    buf[offset + 5] = ((val >> 40) & 0xFF) as u8;
    buf[offset + 6] = ((val >> 48) & 0xFF) as u8;
    buf[offset + 7] = ((val >> 56) & 0xFF) as u8;
}

fn write_f32_le(buf: &mut [u8], offset: usize, val: f32) {
    write_u32_le(buf, offset, val.to_bits());
}

/// X.25 CRC calculation for MAVLink (same as in parser)
fn compute_crc(data: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;

    for &byte in data {
        crc = crc_accumulate(byte, crc);
    }

    crc = crc_accumulate(crc_extra, crc);

    crc
}

fn crc_accumulate(byte: u8, crc: u16) -> u16 {
    let tmp = (byte ^ (crc as u8)) as u16;
    let tmp = tmp ^ (tmp << 4);
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_roundtrip() {
        let msg = Heartbeat {
            mav_type: 2, // Quadrotor
            autopilot: 18, // Aviate
            base_mode: 128, // Armed
            custom_mode: 0,
            system_status: 4, // Active
            mavlink_version: 3,
        };

        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&MavMessage::Heartbeat(msg), 0, &mut buf).unwrap();

        let (parsed, consumed) = crate::parse_mavlink(&buf[..len]).unwrap();
        assert_eq!(consumed, len);

        if let MavMessage::Heartbeat(h) = parsed {
            assert_eq!(h.mav_type, msg.mav_type);
            assert_eq!(h.autopilot, msg.autopilot);
            assert_eq!(h.base_mode, msg.base_mode);
            assert_eq!(h.system_status, msg.system_status);
        } else {
            panic!("Expected Heartbeat message");
        }
    }

    #[test]
    fn test_hil_actuator_controls_roundtrip() {
        let mut msg = HilActuatorControls::default();
        msg.time_usec = 1234567890;
        msg.controls[0] = 0.5;
        msg.controls[1] = 0.6;
        msg.controls[2] = 0.7;
        msg.controls[3] = 0.8;
        msg.mode = 128;

        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&MavMessage::HilActuatorControls(msg), 42, &mut buf).unwrap();

        let (parsed, consumed) = crate::parse_mavlink(&buf[..len]).unwrap();
        assert_eq!(consumed, len);

        if let MavMessage::HilActuatorControls(h) = parsed {
            assert_eq!(h.time_usec, msg.time_usec);
            assert!((h.controls[0] - 0.5).abs() < 0.0001);
            assert!((h.controls[1] - 0.6).abs() < 0.0001);
            assert!((h.controls[2] - 0.7).abs() < 0.0001);
            assert!((h.controls[3] - 0.8).abs() < 0.0001);
            assert_eq!(h.mode, msg.mode);
        } else {
            panic!("Expected HilActuatorControls message");
        }
    }
}
