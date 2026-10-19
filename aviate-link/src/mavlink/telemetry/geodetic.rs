//! Global position frames: the fused estimate and the receiver's own fix.
//!
//! The estimate is local NED; its global position exists only once the
//! vehicle knows the geodetic origin that frame is measured from.

use aviate_core::math::Quaternion;
use aviate_core::state::StateEstimate;
use aviate_core::types::FloatExt;
use aviate_hal_io::{GnssFix, RawGnssReading};

use super::super::protocol::{
    gps_fix_type, serialize_mavlink, GlobalPositionInt, GpsRawInt, MavMessage,
};
use crate::errors::{TelemetryError, TelemetryResult};
use crate::telemetry::GeodeticOrigin;

/// Meters per degree of latitude in the flat-earth projection. Over the
/// few hundred meters a local frame spans, the flat-earth error is far
/// below a receiver's accuracy.
const METERS_PER_DEGREE: f64 = 111_111.0;

/// Latitude and longitude, in degrees, of a local NED offset from `origin`.
pub fn ned_to_lat_lon(origin: &GeodeticOrigin, north_m: f32, east_m: f32) -> (f64, f64) {
    let lat_cos = f64::from(FloatExt::cos((origin.latitude_deg as f32).to_radians()));
    (
        origin.latitude_deg + f64::from(north_m) / METERS_PER_DEGREE,
        origin.longitude_deg + f64::from(east_m) / (METERS_PER_DEGREE * lat_cos),
    )
}

fn degrees_e7(deg: f64) -> i32 {
    (deg * 1e7) as i32
}

fn meters_to_mm(m: f32) -> i32 {
    (m * 1_000.0) as i32
}

/// Velocity in cm/s, saturated to the wire type.
fn cm_per_s(mps: f32) -> i16 {
    (mps * 100.0).clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
}

/// An angle in radians as centidegrees in `0..36000`.
fn centidegrees(rad: f32) -> u16 {
    let mut cdeg = (rad.to_degrees() * 100.0) % 36_000.0;
    if cdeg < 0.0 {
        cdeg += 36_000.0;
    }
    (cdeg as u16).min(35_999)
}

/// Yaw of a body FRD → world NED attitude, radians from north.
fn yaw_rad(q: &Quaternion) -> f32 {
    FloatExt::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
}

/// Format the fused global position as `GLOBAL_POSITION_INT`.
///
/// Relative altitude is measured from the local origin. Like
/// `LOCAL_POSITION_NED`, the frame carries the latest estimate whatever its
/// quality; the paired estimator status decides whether it may be used.
pub fn format_global_position(
    state: &StateEstimate,
    origin: &GeodeticOrigin,
    time_ms: u32,
    ids: (u8, u8),
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let [n, e, d] = [
        state.position_ned[0].0,
        state.position_ned[1].0,
        state.position_ned[2].0,
    ];
    let (lat, lon) = ned_to_lat_lon(origin, n, e);
    let msg = MavMessage::GlobalPositionInt(GlobalPositionInt {
        time_boot_ms: time_ms,
        lat: degrees_e7(lat),
        lon: degrees_e7(lon),
        alt: meters_to_mm(origin.altitude_msl_m - d),
        relative_alt: meters_to_mm(-d),
        vx: cm_per_s(state.velocity_ned[0].0),
        vy: cm_per_s(state.velocity_ned[1].0),
        vz: cm_per_s(state.velocity_ned[2].0),
        hdg: centidegrees(yaw_rad(&state.attitude)),
    });
    let len = serialize_mavlink(&msg, *seq, ids.0, ids.1, buf).ok_or(TelemetryError::Protocol)?;
    *seq = seq.wrapping_add(1);
    Ok(len)
}

fn wire_fix_type(fix: GnssFix) -> u8 {
    match fix {
        GnssFix::None => gps_fix_type::NO_FIX,
        GnssFix::TwoD => gps_fix_type::FIX_2D,
        GnssFix::ThreeD => gps_fix_type::FIX_3D,
        GnssFix::RtkFloat => gps_fix_type::RTK_FLOAT,
        GnssFix::RtkFixed => gps_fix_type::RTK_FIXED,
    }
}

/// Below this ground speed the direction of travel is noise, so the
/// course is reported unknown.
const MIN_COURSE_SPEED_MPS: f32 = 0.2;

/// Format the receiver's own fix as `GPS_RAW_INT`.
///
/// The receiver reading carries one height and no geoid separation, so
/// the same height is stated as both the sea-level and the ellipsoidal
/// altitude. DOP is not part of the reading and is reported unknown; the
/// stated 1-sigma accuracies travel in the extension fields.
pub fn format_gps_raw(
    fix: &RawGnssReading,
    time_ms: u32,
    ids: (u8, u8),
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let [vn, ve, _] = fix.vel_ned;
    let ground_speed = FloatExt::sqrt(vn * vn + ve * ve);
    let alt = meters_to_mm(fix.alt_m);
    let msg = MavMessage::GpsRawInt(GpsRawInt {
        time_usec: u64::from(time_ms) * 1_000,
        fix_type: wire_fix_type(fix.fix),
        lat: degrees_e7(fix.lat_deg),
        lon: degrees_e7(fix.lon_deg),
        alt,
        eph: u16::MAX,
        epv: u16::MAX,
        vel: (ground_speed * 100.0).min(f32::from(u16::MAX - 1)) as u16,
        cog: if ground_speed >= MIN_COURSE_SPEED_MPS {
            centidegrees(FloatExt::atan2(ve, vn))
        } else {
            u16::MAX
        },
        satellites_visible: fix.satellites,
        alt_ellipsoid: alt,
        h_acc: (fix.h_acc * 1_000.0) as u32,
        v_acc: (fix.v_acc * 1_000.0) as u32,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
    });
    let len = serialize_mavlink(&msg, *seq, ids.0, ids.1, buf).ok_or(TelemetryError::Protocol)?;
    *seq = seq.wrapping_add(1);
    Ok(len)
}

#[cfg(test)]
mod tests;
