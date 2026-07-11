//! Mapping from Aviate estimator semantics to MAVLink status frames.

use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};

use super::super::protocol::{
    aviate_estimate_quality, estimator_status_flags, serialize_mavlink, AviateEstimatorStatus,
    EstimatorStatus, MavMessage,
};
use crate::errors::{TelemetryError, TelemetryResult};
use crate::queue::DefaultTelemetryQueue;
use crate::telemetry::TelemetrySnapshot;

/// Project Aviate validity into flags whose standard MAVLink semantics match.
///
/// Angular-rate validity and local vertical-position validity remain available
/// only in `AVIATE_ESTIMATOR_STATUS`; standard MAVLink defines no exact flags
/// for those dimensions. Only nominal-quality estimates advertise standard
/// flags because the standard entries describe their outputs as good.
pub fn standard_estimator_flags(state: &StateEstimate) -> u16 {
    if state.quality != EstimateQuality::Good {
        return 0;
    }

    let mut flags = 0;
    if state.valid_flags.contains(StateValidFlags::ATTITUDE) {
        flags |= estimator_status_flags::ATTITUDE;
    }
    if state.valid_flags.contains(StateValidFlags::VELOCITY) {
        flags |= estimator_status_flags::VELOCITY_HORIZ | estimator_status_flags::VELOCITY_VERT;
    }
    if state.valid_flags.contains(StateValidFlags::POSITION) {
        flags |= estimator_status_flags::POS_HORIZ_REL;
    }
    flags
}

/// Format standard MAVLink `ESTIMATOR_STATUS`.
///
/// Innovation ratios and accuracies remain NaN until the estimator exports
/// those values. Validity is carried only in `flags`.
pub fn format_estimator_status(
    state: &StateEstimate,
    time_ms: u32,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let unavailable = f32::NAN;
    let status = EstimatorStatus {
        time_usec: u64::from(time_ms) * 1_000,
        vel_ratio: unavailable,
        pos_horiz_ratio: unavailable,
        pos_vert_ratio: unavailable,
        mag_ratio: unavailable,
        hagl_ratio: unavailable,
        tas_ratio: unavailable,
        pos_horiz_accuracy: unavailable,
        pos_vert_accuracy: unavailable,
        flags: standard_estimator_flags(state),
    };
    let len = serialize_mavlink(
        &MavMessage::EstimatorStatus(status),
        *seq,
        sys_id,
        comp_id,
        buf,
    )
    .ok_or(TelemetryError::Protocol)?;
    *seq = seq.wrapping_add(1);
    Ok(len)
}

/// Format lossless Aviate estimator quality and validity.
pub fn format_aviate_estimator_status(
    state: &StateEstimate,
    time_ms: u32,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let quality = match state.quality {
        EstimateQuality::Good => aviate_estimate_quality::GOOD,
        EstimateQuality::Degraded => aviate_estimate_quality::DEGRADED,
        EstimateQuality::Unusable => aviate_estimate_quality::UNUSABLE,
    };
    let status = AviateEstimatorStatus {
        time_usec: u64::from(time_ms) * 1_000,
        standard_flags: standard_estimator_flags(state),
        valid_flags: state.valid_flags.bits(),
        quality,
    };
    let len = serialize_mavlink(
        &MavMessage::AviateEstimatorStatus(status),
        *seq,
        sys_id,
        comp_id,
        buf,
    )
    .ok_or(TelemetryError::Protocol)?;
    *seq = seq.wrapping_add(1);
    Ok(len)
}

pub(super) fn enqueue_estimator_status(
    snapshot: &TelemetrySnapshot,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    queue: &mut DefaultTelemetryQueue,
    buf: &mut [u8],
) {
    if let Ok(len) =
        format_estimator_status(&snapshot.state, snapshot.time_ms, sys_id, comp_id, seq, buf)
    {
        queue.push(&buf[..len]).ok();
    }
    if let Ok(len) =
        format_aviate_estimator_status(&snapshot.state, snapshot.time_ms, sys_id, comp_id, seq, buf)
    {
        queue.push(&buf[..len]).ok();
    }
}
