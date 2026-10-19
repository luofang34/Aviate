//! Geodetic-position MAVLink message layouts.

use super::{
    read_i16_le, read_i32_le, read_u16_le, read_u32_le, read_u64_le, write_crc, write_header,
    write_i16_le, write_i32_le, write_u16_le, write_u32_le, write_u64_le, MavHeader, MavMessage,
    ParseError,
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
