//! Geodetic-position MAVLink message layouts.

use super::{
    read_f32_le, read_i16_le, read_i32_le, read_u16_le, read_u32_le, read_u64_le, write_crc,
    write_f32_le, write_header, write_i16_le, write_i32_le, write_u16_le, write_u32_le,
    write_u64_le, MavHeader, MavMessage, ParseError,
};

/// Standard MAVLink `GPS_FIX_TYPE` values.
pub mod gps_fix_type {
    /// The receiver reports no fix.
    pub const NO_FIX: u8 = 1;
    /// Two-dimensional position.
    pub const FIX_2D: u8 = 2;
    /// Three-dimensional position.
    pub const FIX_3D: u8 = 3;
    /// RTK float solution.
    pub const RTK_FLOAT: u8 = 5;
    /// RTK fixed solution.
    pub const RTK_FIXED: u8 = 6;
}

/// Standard MAVLink `GLOBAL_POSITION_INT` (message 33): the fused global
/// position estimate.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct GlobalPositionInt {
    /// Time since system boot, in milliseconds.
    pub time_boot_ms: u32,
    /// Latitude, degrees × 1e7.
    pub lat: i32,
    /// Longitude, degrees × 1e7.
    pub lon: i32,
    /// Altitude above mean sea level, millimeters.
    pub alt: i32,
    /// Altitude above the local origin, millimeters.
    pub relative_alt: i32,
    /// North velocity, cm/s.
    pub vx: i16,
    /// East velocity, cm/s.
    pub vy: i16,
    /// Down velocity, cm/s.
    pub vz: i16,
    /// Heading in centidegrees, 0..35999, or `u16::MAX` when unknown.
    pub hdg: u16,
}

impl GlobalPositionInt {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 33;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 28;
    /// CRC extra from the MAVLink standard dialect.
    pub const CRC_EXTRA: u8 = 104;
}

/// Standard MAVLink `GPS_RAW_INT` (message 24): the receiver's own fix,
/// including the version-2 extension fields.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct GpsRawInt {
    /// Time since system boot, in microseconds.
    pub time_usec: u64,
    /// One of the `gps_fix_type` values.
    pub fix_type: u8,
    /// Latitude, degrees × 1e7.
    pub lat: i32,
    /// Longitude, degrees × 1e7.
    pub lon: i32,
    /// Altitude above mean sea level, millimeters.
    pub alt: i32,
    /// HDOP × 100, or `u16::MAX` when unknown.
    pub eph: u16,
    /// VDOP × 100, or `u16::MAX` when unknown.
    pub epv: u16,
    /// Ground speed, cm/s, or `u16::MAX` when unknown.
    pub vel: u16,
    /// Course over ground in centidegrees, or `u16::MAX` when unknown.
    pub cog: u16,
    /// Satellites visible, or `u8::MAX` when unknown.
    pub satellites_visible: u8,
    /// Altitude above the WGS-84 ellipsoid, millimeters.
    pub alt_ellipsoid: i32,
    /// 1-sigma horizontal position uncertainty, millimeters.
    pub h_acc: u32,
    /// 1-sigma vertical position uncertainty, millimeters.
    pub v_acc: u32,
    /// Speed uncertainty, mm/s.
    pub vel_acc: u32,
    /// Heading uncertainty, degrees × 1e5.
    pub hdg_acc: u32,
    /// Receiver yaw in centidegrees; 0 when the receiver gives no yaw.
    pub yaw: u16,
}

impl GpsRawInt {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 24;
    /// Wire payload length: 30 base bytes plus 22 extension bytes.
    pub const PAYLOAD_LEN: usize = 52;
    /// CRC extra from the MAVLink common dialect.
    pub const CRC_EXTRA: u8 = 24;
}

/// Standard MAVLink `GPS_GLOBAL_ORIGIN` (message 49): the geodetic
/// point at the origin of the local NED frame of `LOCAL_POSITION_NED`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct GpsGlobalOrigin {
    /// Latitude, degrees × 1e7.
    pub latitude: i32,
    /// Longitude, degrees × 1e7.
    pub longitude: i32,
    /// Altitude above mean sea level, millimeters.
    pub altitude: i32,
    /// Time since system boot, in microseconds (extension field).
    pub time_usec: u64,
}

impl GpsGlobalOrigin {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 49;
    /// Wire payload length: 12 base bytes plus the 8-byte extension.
    pub const PAYLOAD_LEN: usize = 20;
    /// CRC extra from the MAVLink common dialect.
    pub const CRC_EXTRA: u8 = 39;
}

/// Standard MAVLink `SET_POSITION_TARGET_GLOBAL_INT` (message 86): a
/// position, velocity and yaw target in geodetic coordinates. The
/// target is free of any local frame, so it stays right across a move
/// of the local origin.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct SetPositionTargetGlobalInt {
    /// Time since system boot, in milliseconds.
    pub time_boot_ms: u32,
    /// Latitude, degrees × 1e7.
    pub lat_int: i32,
    /// Longitude, degrees × 1e7.
    pub lon_int: i32,
    /// Altitude, meters, in the datum that `coordinate_frame` names.
    pub alt: f32,
    /// North velocity, m/s.
    pub vx: f32,
    /// East velocity, m/s.
    pub vy: f32,
    /// Down velocity, m/s.
    pub vz: f32,
    /// North acceleration or force.
    pub afx: f32,
    /// East acceleration or force.
    pub afy: f32,
    /// Down acceleration or force.
    pub afz: f32,
    /// Yaw, radians.
    pub yaw: f32,
    /// Yaw rate, radians per second.
    pub yaw_rate: f32,
    /// `POSITION_TARGET_TYPEMASK` bits of the ignored fields.
    pub type_mask: u16,
    /// Target system.
    pub target_system: u8,
    /// Target component.
    pub target_component: u8,
    /// `MAV_FRAME` of the altitude.
    pub coordinate_frame: u8,
}

impl SetPositionTargetGlobalInt {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 86;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 53;
    /// CRC extra from the MAVLink common dialect.
    pub const CRC_EXTRA: u8 = 5;
    /// `MAV_FRAME_GLOBAL_INT`: the altitude is above mean sea level.
    pub const FRAME_GLOBAL_INT: u8 = 5;
}

pub(super) fn parse_set_position_target_global_int(
    payload: &[u8],
) -> Result<MavMessage, ParseError> {
    if payload.len() < SetPositionTargetGlobalInt::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    let byte = |offset: usize| payload.get(offset).copied().unwrap_or(0);
    Ok(MavMessage::SetPositionTargetGlobalInt(
        SetPositionTargetGlobalInt {
            time_boot_ms: read_u32_le(payload, 0),
            lat_int: read_i32_le(payload, 4),
            lon_int: read_i32_le(payload, 8),
            alt: read_f32_le(payload, 12),
            vx: read_f32_le(payload, 16),
            vy: read_f32_le(payload, 20),
            vz: read_f32_le(payload, 24),
            afx: read_f32_le(payload, 28),
            afy: read_f32_le(payload, 32),
            afz: read_f32_le(payload, 36),
            yaw: read_f32_le(payload, 40),
            yaw_rate: read_f32_le(payload, 44),
            type_mask: read_u16_le(payload, 48),
            target_system: byte(50),
            target_component: byte(51),
            coordinate_frame: byte(52),
        },
    ))
}

pub(super) fn serialize_set_position_target_global_int(
    msg: &SetPositionTargetGlobalInt,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + SetPositionTargetGlobalInt::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        SetPositionTargetGlobalInt::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        SetPositionTargetGlobalInt::MSG_ID,
    );
    write_u32_le(buf, offset, msg.time_boot_ms);
    write_i32_le(buf, offset + 4, msg.lat_int);
    write_i32_le(buf, offset + 8, msg.lon_int);
    for (index, value) in [
        msg.alt,
        msg.vx,
        msg.vy,
        msg.vz,
        msg.afx,
        msg.afy,
        msg.afz,
        msg.yaw,
        msg.yaw_rate,
    ]
    .into_iter()
    .enumerate()
    {
        write_f32_le(buf, offset + 12 + 4 * index, value);
    }
    write_u16_le(buf, offset + 48, msg.type_mask);
    let tail = [
        msg.target_system,
        msg.target_component,
        msg.coordinate_frame,
    ];
    for (slot, value) in buf.iter_mut().skip(offset + 50).zip(tail) {
        *slot = value;
    }
    Some(write_crc(
        buf,
        offset + SetPositionTargetGlobalInt::PAYLOAD_LEN,
        SetPositionTargetGlobalInt::CRC_EXTRA,
    ))
}

pub(super) fn parse_gps_global_origin(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < GpsGlobalOrigin::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    Ok(MavMessage::GpsGlobalOrigin(GpsGlobalOrigin {
        latitude: read_i32_le(payload, 0),
        longitude: read_i32_le(payload, 4),
        altitude: read_i32_le(payload, 8),
        time_usec: read_u64_le(payload, 12),
    }))
}

pub(super) fn serialize_gps_global_origin(
    msg: &GpsGlobalOrigin,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + GpsGlobalOrigin::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        GpsGlobalOrigin::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        GpsGlobalOrigin::MSG_ID,
    );
    write_i32_le(buf, offset, msg.latitude);
    write_i32_le(buf, offset + 4, msg.longitude);
    write_i32_le(buf, offset + 8, msg.altitude);
    write_u64_le(buf, offset + 12, msg.time_usec);
    Some(write_crc(
        buf,
        offset + GpsGlobalOrigin::PAYLOAD_LEN,
        GpsGlobalOrigin::CRC_EXTRA,
    ))
}

pub(super) fn parse_global_position_int(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < GlobalPositionInt::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    Ok(MavMessage::GlobalPositionInt(GlobalPositionInt {
        time_boot_ms: read_u32_le(payload, 0),
        lat: read_i32_le(payload, 4),
        lon: read_i32_le(payload, 8),
        alt: read_i32_le(payload, 12),
        relative_alt: read_i32_le(payload, 16),
        vx: read_i16_le(payload, 20),
        vy: read_i16_le(payload, 22),
        vz: read_i16_le(payload, 24),
        hdg: read_u16_le(payload, 26),
    }))
}

pub(super) fn parse_gps_raw_int(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < GpsRawInt::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    Ok(MavMessage::GpsRawInt(GpsRawInt {
        time_usec: read_u64_le(payload, 0),
        lat: read_i32_le(payload, 8),
        lon: read_i32_le(payload, 12),
        alt: read_i32_le(payload, 16),
        eph: read_u16_le(payload, 20),
        epv: read_u16_le(payload, 22),
        vel: read_u16_le(payload, 24),
        cog: read_u16_le(payload, 26),
        fix_type: payload[28],
        satellites_visible: payload[29],
        alt_ellipsoid: read_i32_le(payload, 30),
        h_acc: read_u32_le(payload, 34),
        v_acc: read_u32_le(payload, 38),
        vel_acc: read_u32_le(payload, 42),
        hdg_acc: read_u32_le(payload, 46),
        yaw: read_u16_le(payload, 50),
    }))
}

pub(super) fn serialize_global_position_int(
    msg: &GlobalPositionInt,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + GlobalPositionInt::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        GlobalPositionInt::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        GlobalPositionInt::MSG_ID,
    );
    write_u32_le(buf, offset, msg.time_boot_ms);
    write_i32_le(buf, offset + 4, msg.lat);
    write_i32_le(buf, offset + 8, msg.lon);
    write_i32_le(buf, offset + 12, msg.alt);
    write_i32_le(buf, offset + 16, msg.relative_alt);
    write_i16_le(buf, offset + 20, msg.vx);
    write_i16_le(buf, offset + 22, msg.vy);
    write_i16_le(buf, offset + 24, msg.vz);
    write_u16_le(buf, offset + 26, msg.hdg);
    Some(write_crc(
        buf,
        offset + GlobalPositionInt::PAYLOAD_LEN,
        GlobalPositionInt::CRC_EXTRA,
    ))
}

pub(super) fn serialize_gps_raw_int(
    msg: &GpsRawInt,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + GpsRawInt::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        GpsRawInt::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        GpsRawInt::MSG_ID,
    );
    write_u64_le(buf, offset, msg.time_usec);
    write_i32_le(buf, offset + 8, msg.lat);
    write_i32_le(buf, offset + 12, msg.lon);
    write_i32_le(buf, offset + 16, msg.alt);
    write_u16_le(buf, offset + 20, msg.eph);
    write_u16_le(buf, offset + 22, msg.epv);
    write_u16_le(buf, offset + 24, msg.vel);
    write_u16_le(buf, offset + 26, msg.cog);
    buf[offset + 28] = msg.fix_type;
    buf[offset + 29] = msg.satellites_visible;
    write_i32_le(buf, offset + 30, msg.alt_ellipsoid);
    write_u32_le(buf, offset + 34, msg.h_acc);
    write_u32_le(buf, offset + 38, msg.v_acc);
    write_u32_le(buf, offset + 42, msg.vel_acc);
    write_u32_le(buf, offset + 46, msg.hdg_acc);
    write_u16_le(buf, offset + 50, msg.yaw);
    Some(write_crc(
        buf,
        offset + GpsRawInt::PAYLOAD_LEN,
        GpsRawInt::CRC_EXTRA,
    ))
}

#[cfg(test)]
mod tests;
