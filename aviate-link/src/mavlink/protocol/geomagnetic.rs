//! Aviate dialect geomagnetic-reference message layout.

use super::{
    read_f32_le, read_u64_le, write_crc, write_f32_le, write_header, write_u64_le, MavHeader,
    MavMessage, ParseError,
};

/// Wire values for `AVIATE_GEOMAGNETIC_SOURCE`.
pub mod aviate_geomagnetic_source {
    /// No declination is applied; the estimated heading is magnetic.
    pub const NONE: u8 = 0;
    /// A geomagnetic model at the vehicle's position and date.
    pub const MODEL: u8 = 1;
    /// A configured value for the operating area.
    pub const CONFIGURED: u8 = 2;
}

/// Aviate dialect `AVIATE_GEOMAGNETIC_REFERENCE` (message 20001): the
/// declination the estimator applies, and its source.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AviateGeomagneticReference {
    /// Time since system boot, in microseconds.
    pub time_usec: u64,
    /// Declination in radians, positive east; NaN with source NONE.
    pub declination: f32,
    /// One of the `AVIATE_GEOMAGNETIC_SOURCE` wire values.
    pub source: u8,
}

impl AviateGeomagneticReference {
    /// Private Aviate dialect message ID.
    pub const MSG_ID: u32 = 20_001;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 13;
    /// CRC extra derived from the `aviate.xml` definition.
    pub const CRC_EXTRA: u8 = 235;
}

pub(super) fn parse_aviate_geomagnetic_reference(payload: &[u8]) -> Result<MavMessage, ParseError> {
    if payload.len() < AviateGeomagneticReference::PAYLOAD_LEN {
        return Err(ParseError::InvalidPayload);
    }
    Ok(MavMessage::AviateGeomagneticReference(
        AviateGeomagneticReference {
            time_usec: read_u64_le(payload, 0),
            declination: read_f32_le(payload, 8),
            source: payload[12],
        },
    ))
}

pub(super) fn serialize_aviate_geomagnetic_reference(
    msg: &AviateGeomagneticReference,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    if buf.len() < MavHeader::SIZE + AviateGeomagneticReference::PAYLOAD_LEN + 2 {
        return None;
    }
    let offset = write_header(
        buf,
        AviateGeomagneticReference::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        AviateGeomagneticReference::MSG_ID,
    );
    write_u64_le(buf, offset, msg.time_usec);
    write_f32_le(buf, offset + 8, msg.declination);
    buf[offset + 12] = msg.source;
    Some(write_crc(
        buf,
        offset + AviateGeomagneticReference::PAYLOAD_LEN,
        AviateGeomagneticReference::CRC_EXTRA,
    ))
}

#[cfg(test)]
mod tests;
