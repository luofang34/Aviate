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
        MavMessage::AttitudeQuaternion(m) => serialize_attitude_quaternion(m, seq, buf),
        MavMessage::LocalPositionNed(m) => serialize_local_position_ned(m, seq, buf),
        MavMessage::SetAttitudeTarget(m) => serialize_set_attitude_target(m, seq, buf),
        MavMessage::CommandLong(m) => serialize_command_long(m, seq, buf),
        MavMessage::CommandAck(m) => serialize_command_ack(m, seq, buf),
        MavMessage::RcChannelsOverride(m) => serialize_rc_channels_override(m, seq, buf),
        MavMessage::ManualControl(m) => serialize_manual_control(m, seq, buf),
        MavMessage::SysStatus(m) => serialize_sys_status(m, seq, buf),
        MavMessage::Statustext(m) => serialize_statustext(m, seq, buf),
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

fn serialize_attitude_quaternion(msg: &AttitudeQuaternion, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + AttitudeQuaternion::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, AttitudeQuaternion::PAYLOAD_LEN as u8, seq, AttitudeQuaternion::MSG_ID);

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.q1);
    write_f32_le(buf, offset + 8, msg.q2);
    write_f32_le(buf, offset + 12, msg.q3);
    write_f32_le(buf, offset + 16, msg.q4);
    write_f32_le(buf, offset + 20, msg.rollspeed);
    write_f32_le(buf, offset + 24, msg.pitchspeed);
    write_f32_le(buf, offset + 28, msg.yawspeed);
    write_f32_le(buf, offset + 32, msg.repr_offset_q[0]);
    write_f32_le(buf, offset + 36, msg.repr_offset_q[1]);
    write_f32_le(buf, offset + 40, msg.repr_offset_q[2]);
    write_f32_le(buf, offset + 44, msg.repr_offset_q[3]);

    Some(write_crc(buf, offset + AttitudeQuaternion::PAYLOAD_LEN, 246))
}

fn serialize_local_position_ned(msg: &LocalPositionNed, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + LocalPositionNed::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, LocalPositionNed::PAYLOAD_LEN as u8, seq, LocalPositionNed::MSG_ID);

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.x);
    write_f32_le(buf, offset + 8, msg.y);
    write_f32_le(buf, offset + 12, msg.z);
    write_f32_le(buf, offset + 16, msg.vx);
    write_f32_le(buf, offset + 20, msg.vy);
    write_f32_le(buf, offset + 24, msg.vz);

    Some(write_crc(buf, offset + LocalPositionNed::PAYLOAD_LEN, 185))
}

fn serialize_set_attitude_target(msg: &SetAttitudeTarget, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SetAttitudeTarget::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, SetAttitudeTarget::PAYLOAD_LEN as u8, seq, SetAttitudeTarget::MSG_ID);

    write_u32_le(buf, offset, msg.time_boot_ms);
    write_f32_le(buf, offset + 4, msg.q[0]);
    write_f32_le(buf, offset + 8, msg.q[1]);
    write_f32_le(buf, offset + 12, msg.q[2]);
    write_f32_le(buf, offset + 16, msg.q[3]);
    write_f32_le(buf, offset + 20, msg.body_roll_rate);
    write_f32_le(buf, offset + 24, msg.body_pitch_rate);
    write_f32_le(buf, offset + 28, msg.body_yaw_rate);
    write_f32_le(buf, offset + 32, msg.thrust);
    buf[offset + 36] = msg.target_system;
    buf[offset + 37] = msg.target_component;
    buf[offset + 38] = msg.type_mask;
    write_f32_le(buf, offset + 39, msg.thrust_body[0]);
    write_f32_le(buf, offset + 43, msg.thrust_body[1]);
    write_f32_le(buf, offset + 47, msg.thrust_body[2]);

    Some(write_crc(buf, offset + SetAttitudeTarget::PAYLOAD_LEN, 49))
}

fn serialize_command_long(msg: &CommandLong, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + CommandLong::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, CommandLong::PAYLOAD_LEN as u8, seq, CommandLong::MSG_ID);

    write_f32_le(buf, offset, msg.param1);
    write_f32_le(buf, offset + 4, msg.param2);
    write_f32_le(buf, offset + 8, msg.param3);
    write_f32_le(buf, offset + 12, msg.param4);
    write_f32_le(buf, offset + 16, msg.param5);
    write_f32_le(buf, offset + 20, msg.param6);
    write_f32_le(buf, offset + 24, msg.param7);
    write_u16_le(buf, offset + 28, msg.command);
    buf[offset + 30] = msg.target_system;
    buf[offset + 31] = msg.target_component;
    buf[offset + 32] = msg.confirmation;

    Some(write_crc(buf, offset + CommandLong::PAYLOAD_LEN, 152))
}

fn serialize_command_ack(msg: &CommandAck, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + CommandAck::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, CommandAck::PAYLOAD_LEN as u8, seq, CommandAck::MSG_ID);

    write_u16_le(buf, offset, msg.command);
    buf[offset + 2] = msg.result;
    buf[offset + 3] = msg.progress;
    write_i32_le(buf, offset + 4, msg.result_param2);
    buf[offset + 8] = msg.target_system;
    buf[offset + 9] = msg.target_component;

    Some(write_crc(buf, offset + CommandAck::PAYLOAD_LEN, 143))
}

fn serialize_rc_channels_override(msg: &RcChannelsOverride, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + RcChannelsOverride::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, RcChannelsOverride::PAYLOAD_LEN as u8, seq, RcChannelsOverride::MSG_ID);

    write_u16_le(buf, offset, msg.chan1_raw);
    write_u16_le(buf, offset + 2, msg.chan2_raw);
    write_u16_le(buf, offset + 4, msg.chan3_raw);
    write_u16_le(buf, offset + 6, msg.chan4_raw);
    write_u16_le(buf, offset + 8, msg.chan5_raw);
    write_u16_le(buf, offset + 10, msg.chan6_raw);
    write_u16_le(buf, offset + 12, msg.chan7_raw);
    write_u16_le(buf, offset + 14, msg.chan8_raw);
    buf[offset + 16] = msg.target_system;
    buf[offset + 17] = msg.target_component;
    
    // Extensions (Chan 9-18)
    let ext_offset = offset + 18;
    write_u16_le(buf, ext_offset, msg.chan9_raw);
    write_u16_le(buf, ext_offset + 2, msg.chan10_raw);
    write_u16_le(buf, ext_offset + 4, msg.chan11_raw);
    write_u16_le(buf, ext_offset + 6, msg.chan12_raw);
    write_u16_le(buf, ext_offset + 8, msg.chan13_raw);
    write_u16_le(buf, ext_offset + 10, msg.chan14_raw);
    write_u16_le(buf, ext_offset + 12, msg.chan15_raw);
    write_u16_le(buf, ext_offset + 14, msg.chan16_raw);
    write_u16_le(buf, ext_offset + 16, msg.chan17_raw);
    write_u16_le(buf, ext_offset + 18, msg.chan18_raw);

    Some(write_crc(buf, offset + RcChannelsOverride::PAYLOAD_LEN, 124))
}

fn serialize_manual_control(msg: &ManualControl, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + ManualControl::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, ManualControl::PAYLOAD_LEN as u8, seq, ManualControl::MSG_ID);

    write_i16_le(buf, offset, msg.x);
    write_i16_le(buf, offset + 2, msg.y);
    write_i16_le(buf, offset + 4, msg.z);
    write_i16_le(buf, offset + 6, msg.r);
    write_u16_le(buf, offset + 8, msg.buttons);
    buf[offset + 10] = msg.target;

    // Extensions
    let ext_offset = offset + 11;
    write_i16_le(buf, ext_offset, msg.s);
    write_i16_le(buf, ext_offset + 2, msg.t);
    write_i16_le(buf, ext_offset + 4, msg.aux1);
    write_i16_le(buf, ext_offset + 6, msg.aux2);
    write_i16_le(buf, ext_offset + 8, msg.aux3);
    write_i16_le(buf, ext_offset + 10, msg.aux4);
    write_i16_le(buf, ext_offset + 12, msg.aux5);
    write_i16_le(buf, ext_offset + 14, msg.aux6);

    Some(write_crc(buf, offset + ManualControl::PAYLOAD_LEN, 243))
}

fn serialize_sys_status(msg: &SysStatus, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + SysStatus::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, SysStatus::PAYLOAD_LEN as u8, seq, SysStatus::MSG_ID);

    write_u32_le(buf, offset, msg.onboard_control_sensors_present);
    write_u32_le(buf, offset + 4, msg.onboard_control_sensors_enabled);
    write_u32_le(buf, offset + 8, msg.onboard_control_sensors_health);
    write_u16_le(buf, offset + 12, msg.load);
    write_u16_le(buf, offset + 14, msg.voltage_battery);
    write_i16_le(buf, offset + 16, msg.current_battery);
    write_u16_le(buf, offset + 18, msg.drop_rate_comm);
    write_u16_le(buf, offset + 20, msg.errors_comm);
    write_u16_le(buf, offset + 22, msg.errors_count1);
    write_u16_le(buf, offset + 24, msg.errors_count2);
    write_u16_le(buf, offset + 26, msg.errors_count3);
    write_u16_le(buf, offset + 28, msg.errors_count4);
    buf[offset + 30] = msg.battery_remaining as u8;

    // Extensions
    let ext_offset = offset + 31;
    write_u32_le(buf, ext_offset, msg.onboard_control_sensors_present_extended);
    write_u32_le(buf, ext_offset + 4, msg.onboard_control_sensors_enabled_extended);
    write_u32_le(buf, ext_offset + 8, msg.onboard_control_sensors_health_extended);

    Some(write_crc(buf, offset + SysStatus::PAYLOAD_LEN, 124))
}

fn serialize_statustext(msg: &Statustext, seq: u8, buf: &mut [u8]) -> Option<usize> {
    let frame_size = MavHeader::SIZE + Statustext::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }

    let offset = write_header(buf, Statustext::PAYLOAD_LEN as u8, seq, Statustext::MSG_ID);

    buf[offset] = msg.severity;
    for i in 0..50 {
        buf[offset + 1 + i] = msg.text[i];
    }

    // Extensions
    let ext_offset = offset + 51;
    write_u16_le(buf, ext_offset, msg.id);
    buf[ext_offset + 2] = msg.chunk_seq;

    Some(write_crc(buf, offset + Statustext::PAYLOAD_LEN, 83))
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
    let tmp = tmp ^ ((tmp << 4) & 0xFF);  // Mask to 8 bits per X.25 CRC spec
    (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip<F, G>(create_msg: F, verify_msg: G) 
    where F: Fn() -> MavMessage, G: Fn(MavMessage)
    {
        let msg = create_msg();
        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&msg, 0, &mut buf).expect("Serialize failed");
        let (parsed, consumed) = crate::parse_mavlink(&buf[..len]).expect("Parse failed");
        assert_eq!(consumed, len, "Consumed length mismatch");
        verify_msg(parsed);
    }

    #[test]
    fn test_heartbeat() {
        roundtrip(
            || MavMessage::Heartbeat(Heartbeat {
                mav_type: 2, autopilot: 18, base_mode: 128, custom_mode: 0, system_status: 4, mavlink_version: 3
            }),
            |m| if let MavMessage::Heartbeat(h) = m {
                assert_eq!(h.mav_type, 2);
                assert_eq!(h.autopilot, 18);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_attitude_quaternion() {
        roundtrip(
            || MavMessage::AttitudeQuaternion(AttitudeQuaternion {
                time_boot_ms: 1000, q1: 1.0, q2: 0.0, q3: 0.0, q4: 0.0, 
                rollspeed: 0.1, pitchspeed: 0.2, yawspeed: 0.3, repr_offset_q: [0.0, 0.0, 0.0, 0.0]
            }),
            |m| if let MavMessage::AttitudeQuaternion(h) = m {
                assert_eq!(h.time_boot_ms, 1000);
                assert!((h.q1 - 1.0).abs() < 1e-5);
                assert!((h.rollspeed - 0.1).abs() < 1e-5);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_local_position_ned() {
        roundtrip(
            || MavMessage::LocalPositionNed(LocalPositionNed {
                time_boot_ms: 2000, x: 10.0, y: 20.0, z: -5.0, vx: 1.0, vy: 0.0, vz: 0.0
            }),
            |m| if let MavMessage::LocalPositionNed(h) = m {
                assert_eq!(h.time_boot_ms, 2000);
                assert!((h.x - 10.0).abs() < 1e-5);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_set_attitude_target() {
        roundtrip(
            || MavMessage::SetAttitudeTarget(SetAttitudeTarget {
                time_boot_ms: 3000, target_system: 1, target_component: 1, type_mask: 0,
                q: [1.0, 0.0, 0.0, 0.0], body_roll_rate: 0.1, body_pitch_rate: 0.0, body_yaw_rate: 0.0,
                thrust: 0.5, thrust_body: [0.0, 0.0, 0.0]
            }),
            |m| if let MavMessage::SetAttitudeTarget(h) = m {
                assert_eq!(h.time_boot_ms, 3000);
                assert!((h.thrust - 0.5).abs() < 1e-5);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_command_long() {
        roundtrip(
            || MavMessage::CommandLong(CommandLong {
                param1: 1.0, param2: 2.0, param3: 3.0, param4: 4.0, param5: 5.0, param6: 6.0, param7: 7.0,
                command: 400, target_system: 1, target_component: 1, confirmation: 0
            }),
            |m| if let MavMessage::CommandLong(h) = m {
                assert_eq!(h.command, 400);
                assert!((h.param1 - 1.0).abs() < 1e-5);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_command_ack() {
        roundtrip(
            || MavMessage::CommandAck(CommandAck {
                command: 400, result: 0, progress: 100, result_param2: 0, target_system: 1, target_component: 1
            }),
            |m| if let MavMessage::CommandAck(h) = m {
                assert_eq!(h.command, 400);
                assert_eq!(h.result, 0);
                assert_eq!(h.progress, 100);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_rc_channels_override() {
        roundtrip(
            || MavMessage::RcChannelsOverride(RcChannelsOverride {
                chan1_raw: 1500, chan2_raw: 1500, chan3_raw: 1000, chan4_raw: 1500,
                chan5_raw: 0, chan6_raw: 0, chan7_raw: 0, chan8_raw: 0,
                target_system: 1, target_component: 1,
                chan9_raw: 0, chan10_raw: 0, chan11_raw: 0, chan12_raw: 0,
                chan13_raw: 0, chan14_raw: 0, chan15_raw: 0, chan16_raw: 0,
                chan17_raw: 0, chan18_raw: 0
            }),
            |m| if let MavMessage::RcChannelsOverride(h) = m {
                assert_eq!(h.chan3_raw, 1000);
                assert_eq!(h.target_system, 1);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_manual_control() {
        roundtrip(
            || MavMessage::ManualControl(ManualControl {
                x: 100, y: -100, z: 500, r: 0, buttons: 1, target: 1,
                s: 0, t: 0, aux1: 0, aux2: 0, aux3: 0, aux4: 0, aux5: 0, aux6: 0
            }),
            |m| if let MavMessage::ManualControl(h) = m {
                assert_eq!(h.x, 100);
                assert_eq!(h.z, 500);
                assert_eq!(h.target, 1);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_sys_status() {
        roundtrip(
            || MavMessage::SysStatus(SysStatus {
                onboard_control_sensors_present: 1,
                onboard_control_sensors_enabled: 1,
                onboard_control_sensors_health: 1,
                load: 500, voltage_battery: 12000, current_battery: 100,
                drop_rate_comm: 0, errors_comm: 0,
                errors_count1: 0, errors_count2: 0, errors_count3: 0, errors_count4: 0,
                battery_remaining: 50,
                onboard_control_sensors_present_extended: 0,
                onboard_control_sensors_enabled_extended: 0,
                onboard_control_sensors_health_extended: 0
            }),
            |m| if let MavMessage::SysStatus(h) = m {
                assert_eq!(h.voltage_battery, 12000);
                assert_eq!(h.battery_remaining, 50);
            } else { panic!("Wrong message type") }
        );
    }

    #[test]
    fn test_statustext() {
        let mut text = [0u8; 50];
        text[0] = b'H'; text[1] = b'e'; text[2] = b'l'; text[3] = b'l'; text[4] = b'o';
        roundtrip(
            || MavMessage::Statustext(Statustext {
                severity: 6, text, id: 0, chunk_seq: 0
            }),
            |m| if let MavMessage::Statustext(h) = m {
                assert_eq!(h.severity, 6);
                assert_eq!(h.text[0], b'H');
            } else { panic!("Wrong message type") }
        );
    }

    // ==========================================================================
    // Pymavlink Interoperability Tests
    // Test vectors generated by pymavlink to verify cross-implementation compatibility
    // ==========================================================================

    // Generated by mavlink_interop_test_rust.py
    const PYMAVLINK_HEARTBEAT: &[u8] = &[253, 9, 0, 0, 0, 255, 190, 0, 0, 0, 0, 0, 0, 0, 2, 0, 128, 4, 3, 223, 47];
    const PYMAVLINK_COMMAND_ARM: &[u8] = &[253, 32, 0, 0, 0, 255, 190, 76, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 144, 1, 1, 1, 158, 78];
    const PYMAVLINK_SET_ATTITUDE: &[u8] = &[253, 39, 0, 0, 0, 255, 190, 82, 0, 0, 232, 3, 0, 0, 0, 0, 128, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63, 1, 1, 7, 237, 57];
    const PYMAVLINK_MANUAL_CONTROL: &[u8] = &[253, 11, 0, 0, 0, 255, 190, 69, 0, 0, 100, 0, 156, 255, 244, 1, 0, 0, 0, 0, 1, 35, 223];
    const PYMAVLINK_RC_OVERRIDE: &[u8] = &[253, 18, 0, 0, 0, 255, 190, 70, 0, 0, 220, 5, 220, 5, 232, 3, 220, 5, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 64, 169];
    const PYMAVLINK_HIL_SENSOR: &[u8] = &[253, 62, 0, 0, 0, 142, 1, 107, 0, 0, 64, 66, 15, 0, 0, 0, 0, 0, 205, 204, 204, 61, 205, 204, 76, 190, 195, 245, 28, 193, 10, 215, 35, 60, 10, 215, 163, 188, 0, 0, 0, 0, 205, 204, 76, 62, 0, 0, 0, 0, 205, 204, 204, 62, 0, 80, 125, 68, 0, 0, 0, 0, 0, 0, 200, 66, 0, 0, 200, 65, 255, 255, 103, 36];

    #[test]
    fn test_pymavlink_heartbeat() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_HEARTBEAT).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_HEARTBEAT.len());
        if let MavMessage::Heartbeat(h) = msg {
            assert_eq!(h.mav_type, 2); // MAV_TYPE_QUADROTOR
            assert_eq!(h.autopilot, 0); // MAV_AUTOPILOT_GENERIC
            assert_eq!(h.base_mode, 128); // SAFETY_ARMED
            assert_eq!(h.system_status, 4); // MAV_STATE_ACTIVE
        } else { panic!("Wrong message type"); }
    }

    #[test]
    fn test_pymavlink_command_arm() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_COMMAND_ARM).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_COMMAND_ARM.len());
        if let MavMessage::CommandLong(c) = msg {
            assert_eq!(c.command, 400); // MAV_CMD_COMPONENT_ARM_DISARM
            assert!((c.param1 - 1.0).abs() < 1e-5); // ARM
            assert_eq!(c.target_system, 1);
            assert_eq!(c.target_component, 1);
        } else { panic!("Wrong message type"); }
    }

    #[test]
    fn test_pymavlink_set_attitude_target() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_SET_ATTITUDE).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_SET_ATTITUDE.len());
        if let MavMessage::SetAttitudeTarget(a) = msg {
            assert_eq!(a.time_boot_ms, 1000);
            assert!((a.q[0] - 1.0).abs() < 1e-5); // w = 1
            assert!((a.q[1]).abs() < 1e-5); // x = 0
            assert!((a.thrust - 0.5).abs() < 1e-5);
            assert_eq!(a.type_mask, 7);
        } else { panic!("Wrong message type"); }
    }

    #[test]
    fn test_pymavlink_manual_control() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_MANUAL_CONTROL).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_MANUAL_CONTROL.len());
        if let MavMessage::ManualControl(m) = msg {
            assert_eq!(m.x, 100);
            assert_eq!(m.y, -100);
            assert_eq!(m.z, 500);
            assert_eq!(m.target, 1);
        } else { panic!("Wrong message type"); }
    }

    #[test]
    fn test_pymavlink_rc_channels_override() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_RC_OVERRIDE).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_RC_OVERRIDE.len());
        if let MavMessage::RcChannelsOverride(r) = msg {
            assert_eq!(r.chan1_raw, 1500);
            assert_eq!(r.chan2_raw, 1500);
            assert_eq!(r.chan3_raw, 1000);
            assert_eq!(r.chan4_raw, 1500);
            assert_eq!(r.target_system, 1);
        } else { panic!("Wrong message type"); }
    }

    #[test]
    fn test_pymavlink_hil_sensor() {
        let (msg, consumed) = crate::parse_mavlink(PYMAVLINK_HIL_SENSOR).expect("Parse failed");
        assert_eq!(consumed, PYMAVLINK_HIL_SENSOR.len());
        if let MavMessage::HilSensor(h) = msg {
            assert_eq!(h.time_usec, 1000000);
            assert!((h.xacc - 0.1).abs() < 0.01);
            assert!((h.yacc - (-0.2)).abs() < 0.01);
            assert!((h.zacc - (-9.81)).abs() < 0.01);
            assert!((h.abs_pressure - 1013.25).abs() < 0.1);
        } else { panic!("Wrong message type"); }
    }
}
