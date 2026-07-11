//! Estimator-status MAVLink message layouts.

use super::{
    read_f32_le, read_u16_le, read_u64_le, write_crc, write_f32_le, write_header, write_u16_le,
    write_u64_le, MavHeader, MavMessage, ParseError,
};

/// Standard MAVLink `ESTIMATOR_STATUS_FLAGS` values used by Aviate.
pub mod estimator_status_flags {
    /// Attitude output is valid.
    pub const ATTITUDE: u16 = 1;
    /// Horizontal velocity output is valid.
    pub const VELOCITY_HORIZ: u16 = 2;
    /// Vertical velocity output is valid.
    pub const VELOCITY_VERT: u16 = 4;
    /// Horizontal position relative to the local origin is valid.
    pub const POS_HORIZ_REL: u16 = 8;
}

/// Wire values for `AVIATE_ESTIMATE_QUALITY`.
pub mod aviate_estimate_quality {
    /// The estimate must not be used.
    pub const UNUSABLE: u8 = 0;
    /// Valid dimensions remain usable with degraded quality.
    pub const DEGRADED: u8 = 1;
    /// Valid dimensions meet nominal quality criteria.
    pub const GOOD: u8 = 2;
}

/// Standard MAVLink `ESTIMATOR_STATUS` (message 230).
#[derive(Copy, Clone, Debug, Default)]
pub struct EstimatorStatus {
    /// Time since boot or Unix epoch, in microseconds.
    pub time_usec: u64,
    /// Velocity innovation test ratio, or NaN when unavailable.
    pub vel_ratio: f32,
    /// Horizontal-position innovation test ratio, or NaN when unavailable.
    pub pos_horiz_ratio: f32,
    /// Vertical-position innovation test ratio, or NaN when unavailable.
    pub pos_vert_ratio: f32,
    /// Magnetometer innovation test ratio, or NaN when unavailable.
    pub mag_ratio: f32,
    /// Height-above-ground innovation test ratio, or NaN when unavailable.
    pub hagl_ratio: f32,
    /// True-airspeed innovation test ratio, or NaN when unavailable.
    pub tas_ratio: f32,
    /// Horizontal position accuracy, or NaN when unavailable.
    pub pos_horiz_accuracy: f32,
    /// Vertical position accuracy, or NaN when unavailable.
    pub pos_vert_accuracy: f32,
    /// Conservative standard estimator-validity flags.
    pub flags: u16,
}

impl EstimatorStatus {
    /// MAVLink common message ID.
    pub const MSG_ID: u32 = 230;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 42;
}

/// Aviate dialect estimator status (message 20000).
#[derive(Copy, Clone, Debug, Default)]
pub struct AviateEstimatorStatus {
    /// Time since system boot, in microseconds.
    pub time_usec: u64,
    /// Conservative projection into standard estimator flags.
    pub standard_flags: u16,
    /// Raw `StateValidFlags` bits.
    pub valid_flags: u8,
    /// One of the `AVIATE_ESTIMATE_QUALITY` wire values.
    pub quality: u8,
}

impl AviateEstimatorStatus {
    /// Private Aviate dialect message ID.
    pub const MSG_ID: u32 = 20_000;
    /// Wire payload length.
    pub const PAYLOAD_LEN: usize = 12;
}

pub(super) fn parse_estimator_status(payload: &[u8]) -> Result<MavMessage, ParseError> {
    let payload = zero_extend_payload::<{ EstimatorStatus::PAYLOAD_LEN }>(payload)?;

    Ok(MavMessage::EstimatorStatus(EstimatorStatus {
        time_usec: read_u64_le(&payload, 0),
        vel_ratio: read_f32_le(&payload, 8),
        pos_horiz_ratio: read_f32_le(&payload, 12),
        pos_vert_ratio: read_f32_le(&payload, 16),
        mag_ratio: read_f32_le(&payload, 20),
        hagl_ratio: read_f32_le(&payload, 24),
        tas_ratio: read_f32_le(&payload, 28),
        pos_horiz_accuracy: read_f32_le(&payload, 32),
        pos_vert_accuracy: read_f32_le(&payload, 36),
        flags: read_u16_le(&payload, 40),
    }))
}

pub(super) fn parse_aviate_estimator_status(payload: &[u8]) -> Result<MavMessage, ParseError> {
    let payload = zero_extend_payload::<{ AviateEstimatorStatus::PAYLOAD_LEN }>(payload)?;

    Ok(MavMessage::AviateEstimatorStatus(AviateEstimatorStatus {
        time_usec: read_u64_le(&payload, 0),
        standard_flags: read_u16_le(&payload, 8),
        valid_flags: payload[10],
        quality: payload[11],
    }))
}

fn zero_extend_payload<const N: usize>(payload: &[u8]) -> Result<[u8; N], ParseError> {
    if payload.is_empty() || payload.len() > N {
        return Err(ParseError::InvalidPayload);
    }

    // MAVLink 2 removes zero bytes from the payload tail on the wire.
    let mut full_payload = [0u8; N];
    full_payload[..payload.len()].copy_from_slice(payload);
    Ok(full_payload)
}

pub(super) fn serialize_estimator_status(
    msg: &EstimatorStatus,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + EstimatorStatus::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }
    let offset = write_header(
        buf,
        EstimatorStatus::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        EstimatorStatus::MSG_ID,
    );
    write_u64_le(buf, offset, msg.time_usec);
    write_f32_le(buf, offset + 8, msg.vel_ratio);
    write_f32_le(buf, offset + 12, msg.pos_horiz_ratio);
    write_f32_le(buf, offset + 16, msg.pos_vert_ratio);
    write_f32_le(buf, offset + 20, msg.mag_ratio);
    write_f32_le(buf, offset + 24, msg.hagl_ratio);
    write_f32_le(buf, offset + 28, msg.tas_ratio);
    write_f32_le(buf, offset + 32, msg.pos_horiz_accuracy);
    write_f32_le(buf, offset + 36, msg.pos_vert_accuracy);
    write_u16_le(buf, offset + 40, msg.flags);
    Some(write_crc(buf, offset + EstimatorStatus::PAYLOAD_LEN, 163))
}

pub(super) fn serialize_aviate_estimator_status(
    msg: &AviateEstimatorStatus,
    seq: u8,
    sys_id: u8,
    comp_id: u8,
    buf: &mut [u8],
) -> Option<usize> {
    let frame_size = MavHeader::SIZE + AviateEstimatorStatus::PAYLOAD_LEN + 2;
    if buf.len() < frame_size {
        return None;
    }
    let offset = write_header(
        buf,
        AviateEstimatorStatus::PAYLOAD_LEN as u8,
        seq,
        sys_id,
        comp_id,
        AviateEstimatorStatus::MSG_ID,
    );
    write_u64_le(buf, offset, msg.time_usec);
    write_u16_le(buf, offset + 8, msg.standard_flags);
    buf[offset + 10] = msg.valid_flags;
    buf[offset + 11] = msg.quality;
    Some(write_crc(
        buf,
        offset + AviateEstimatorStatus::PAYLOAD_LEN,
        50,
    ))
}

#[cfg(test)]
mod tests;
