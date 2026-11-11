//! Standard MAVLink `VFR_HUD` (message 74): the instrument values of a
//! head-up display.

use super::{
    read_f32_le, read_i16_le, read_u16_le, write_crc, write_f32_le, write_header, write_i16_le,
    write_u16_le, MavHeader, MavMessage, ParseError,
};

/// Airspeed, groundspeed, heading, throttle, altitude and climb.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct VfrHud {
    /// Airspeed, m/s; indicated airspeed for a vehicle with a probe.
    pub airspeed: f32,
    /// Speed over the ground, m/s.
    pub groundspeed: f32,
    /// Altitude above mean sea level, m.
    pub alt: f32,
    /// Climb rate, m/s, positive up.
    pub climb: f32,
    /// Heading, degrees from north, 0 to 359.
    pub heading: i16,
    /// Throttle, percent.
    pub throttle: u16,
}

impl VfrHud {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 74;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 20;
    /// CRC extra from the MAVLink common dialect.
    pub const CRC_EXTRA: u8 = 20;
}

pub(super) fn parse_vfr_hud(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < VfrHud::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    Ok(MavMessage::VfrHud(VfrHud {
        airspeed: read_f32_le(payload, 0),
        groundspeed: read_f32_le(payload, 4),
        alt: read_f32_le(payload, 8),
        climb: read_f32_le(payload, 12),
        heading: read_i16_le(payload, 16),
        throttle: read_u16_le(payload, 18),
    }))
}

pub(super) fn serialize_vfr_hud(
    msg: &VfrHud,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + VfrHud::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        VfrHud::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        VfrHud::MSG_ID,
    );
    write_f32_le(buf, offset, msg.airspeed);
    write_f32_le(buf, offset + 4, msg.groundspeed);
    write_f32_le(buf, offset + 8, msg.alt);
    write_f32_le(buf, offset + 12, msg.climb);
    write_i16_le(buf, offset + 16, msg.heading);
    write_u16_le(buf, offset + 18, msg.throttle);
    Some(write_crc(
        buf,
        offset + VfrHud::PAYLOAD_LEN,
        VfrHud::CRC_EXTRA,
    ))
}

#[cfg(test)]
mod tests;
