//! The heading reference the estimator applies, as telemetry.

use aviate_core::sensor::{GeomagneticReference, GeomagneticSource};

use super::super::protocol::{
    aviate_geomagnetic_source, serialize_mavlink, AviateGeomagneticReference, MavMessage,
};
use crate::errors::{TelemetryError, TelemetryResult};

/// Format `AVIATE_GEOMAGNETIC_REFERENCE` for `reference`, or the
/// no-reference frame (NaN declination, source NONE) for `None`.
pub fn format_geomagnetic_reference(
    reference: Option<&GeomagneticReference>,
    time_ms: u32,
    ids: (u8, u8),
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let (declination, source) = match reference {
        Some(r) => (
            r.declination.0,
            match r.source {
                GeomagneticSource::Model => aviate_geomagnetic_source::MODEL,
                GeomagneticSource::Configured => aviate_geomagnetic_source::CONFIGURED,
            },
        ),
        None => (f32::NAN, aviate_geomagnetic_source::NONE),
    };
    let msg = MavMessage::AviateGeomagneticReference(AviateGeomagneticReference {
        time_usec: u64::from(time_ms) * 1_000,
        declination,
        source,
    });
    let len = serialize_mavlink(&msg, *seq, ids.0, ids.1, buf).ok_or(TelemetryError::Protocol)?;
    *seq = seq.wrapping_add(1);
    Ok(len)
}

#[cfg(test)]
mod tests;
