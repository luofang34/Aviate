//! Global position telemetry tests.

use aviate_core::math::Quaternion;
use aviate_core::state::StateEstimate;
use aviate_core::types::{Meters, MetersPerSecond};
use aviate_hal_io::{GnssFix, RawGnssReading};

use super::*;
use crate::mavlink::protocol::{parse_mavlink, MavMessage};
use crate::mavlink::telemetry::{MavlinkCycleFormatter, TelemetryConfig};
use crate::queue::DefaultTelemetryQueue;
use crate::telemetry::{TelemetryCycleFormatter, TelemetrySnapshot};

const ORIGIN: GeodeticOrigin = GeodeticOrigin {
    latitude_deg: 47.397_742,
    longitude_deg: 8.545_992_1,
    altitude_msl_m: 488.0,
};

/// Frames one cycle can emit; more than the formatter ever queues.
const MAX_FRAMES: usize = 16;

/// The frames of one cycle, in emission order.
struct Cycle {
    frames: [Option<MavMessage>; MAX_FRAMES],
}

impl Cycle {
    fn iter(&self) -> impl Iterator<Item = &MavMessage> {
        self.frames.iter().flatten()
    }

    fn position(&self, wanted: impl Fn(&MavMessage) -> bool) -> Option<usize> {
        self.iter().position(wanted)
    }

    fn global(&self) -> Option<GlobalPositionInt> {
        self.iter().find_map(|m| match m {
            MavMessage::GlobalPositionInt(g) => Some(*g),
            _ => None,
        })
    }

    fn raw(&self) -> Option<GpsRawInt> {
        self.iter().find_map(|m| match m {
            MavMessage::GpsRawInt(r) => Some(*r),
            _ => None,
        })
    }
}

/// Every frame one position-rate cycle emits.
fn one_position_cycle(snapshot: &TelemetrySnapshot) -> Cycle {
    let mut cycle = Cycle {
        frames: [None; MAX_FRAMES],
    };
    let Ok(mut formatter) = MavlinkCycleFormatter::new(&TelemetryConfig::default(), 4) else {
        return cycle;
    };
    let mut queue = DefaultTelemetryQueue::new();
    formatter.format_cycle(snapshot, &mut queue);
    let mut count = 0;
    while queue.pop_with(|frame| {
        if let (Ok((message, _, _)), Some(slot)) =
            (parse_mavlink(frame), cycle.frames.get_mut(count))
        {
            *slot = Some(message);
            count += 1;
        }
    }) {}
    cycle
}

/// 90° yaw about NED down: the body faces east.
fn facing_east() -> Quaternion {
    let half = core::f32::consts::FRAC_1_SQRT_2;
    Quaternion {
        w: half,
        x: 0.0,
        y: 0.0,
        z: half,
    }
}

fn hovering_state() -> StateEstimate {
    StateEstimate {
        attitude: facing_east(),
        position_ned: [Meters(100.0), Meters(-50.0), Meters(-12.5)],
        velocity_ned: [
            MetersPerSecond(1.5),
            MetersPerSecond(-0.75),
            MetersPerSecond(-0.2),
        ],
        ..StateEstimate::default()
    }
}

fn receiver_fix() -> RawGnssReading {
    RawGnssReading {
        lat_deg: 47.397_8,
        lon_deg: 8.546,
        alt_m: 493.0,
        vel_ned: [1.0, 1.0, 0.0],
        fix: GnssFix::ThreeD,
        h_acc: 0.5,
        v_acc: 0.8,
        satellites: 14,
        ..RawGnssReading::default()
    }
}

#[test]
fn a_local_offset_maps_to_the_origin_plus_flat_earth_degrees() {
    assert_eq!(
        ned_to_lat_lon(&ORIGIN, 0.0, 0.0),
        (ORIGIN.latitude_deg, ORIGIN.longitude_deg)
    );
    let (lat, lon) = ned_to_lat_lon(&ORIGIN, 111.111, 0.0);
    assert!((lat - (ORIGIN.latitude_deg + 0.001)).abs() < 1e-9);
    assert_eq!(lon, ORIGIN.longitude_deg);
    let (_, lon) = ned_to_lat_lon(&ORIGIN, 0.0, 100.0);
    // 100 m east at 47.4° N is 100 / (111 111 · cos 47.4°) degrees.
    assert!((lon - ORIGIN.longitude_deg - 0.001_329_6).abs() < 1e-6);
}

#[test]
fn the_estimate_is_stated_globally_around_the_origin() {
    let snapshot = TelemetrySnapshot {
        time_ms: 4_000,
        state: hovering_state(),
        geodetic_origin: Some(ORIGIN),
        ..TelemetrySnapshot::default()
    };
    let cycle = one_position_cycle(&snapshot);
    assert!(cycle.global().is_some(), "no GLOBAL_POSITION_INT");
    let Some(global) = cycle.global() else {
        return;
    };
    let (lat, lon) = ned_to_lat_lon(&ORIGIN, 100.0, -50.0);
    assert_eq!(global.time_boot_ms, 4_000);
    assert_eq!(global.lat, (lat * 1e7) as i32);
    assert_eq!(global.lon, (lon * 1e7) as i32);
    assert_eq!(global.alt, 500_500);
    assert_eq!(global.relative_alt, 12_500);
    assert_eq!((global.vx, global.vy, global.vz), (150, -75, -20));
    assert_eq!(global.hdg, 9_000);

    // It rides the estimate group, so its status precedes it.
    let status_at = cycle.position(|m| matches!(m, MavMessage::AviateEstimatorStatus(_)));
    let global_at = cycle.position(|m| matches!(m, MavMessage::GlobalPositionInt(_)));
    assert!(status_at.is_some() && status_at < global_at);
}

#[test]
fn no_global_position_is_stated_without_an_origin() {
    let snapshot = TelemetrySnapshot {
        state: hovering_state(),
        ..TelemetrySnapshot::default()
    };
    let cycle = one_position_cycle(&snapshot);
    assert!(cycle
        .position(|m| matches!(m, MavMessage::LocalPositionNed(_)))
        .is_some());
    assert!(cycle.global().is_none());
}

#[test]
fn the_receiver_fix_is_reported_as_gps_raw_int() {
    let snapshot = TelemetrySnapshot {
        time_ms: 4_000,
        gnss_fix: Some(receiver_fix()),
        ..TelemetrySnapshot::default()
    };
    let raw = one_position_cycle(&snapshot).raw();
    assert!(raw.is_some(), "no GPS_RAW_INT");
    let Some(raw) = raw else {
        return;
    };
    assert_eq!(raw.time_usec, 4_000_000);
    assert_eq!(raw.fix_type, gps_fix_type::FIX_3D);
    assert_eq!(raw.lat, (47.397_8f64 * 1e7) as i32);
    assert_eq!(raw.lon, (8.546f64 * 1e7) as i32);
    assert_eq!(raw.alt, 493_000);
    assert_eq!(raw.alt_ellipsoid, 493_000);
    assert_eq!((raw.h_acc, raw.v_acc), (500, 800));
    assert_eq!(raw.satellites_visible, 14);
    assert_eq!((raw.eph, raw.epv), (u16::MAX, u16::MAX));
    assert_eq!(raw.vel, 141);
    assert_eq!(raw.cog, 4_500);
}

#[test]
fn a_stationary_receiver_reports_no_course() {
    let fix = RawGnssReading {
        vel_ned: [0.05, 0.0, 0.0],
        ..receiver_fix()
    };
    let snapshot = TelemetrySnapshot {
        gnss_fix: Some(fix),
        ..TelemetrySnapshot::default()
    };
    assert_eq!(
        one_position_cycle(&snapshot).raw().map(|r| r.cog),
        Some(u16::MAX)
    );
}

#[test]
fn no_receiver_report_means_no_gps_raw_int() {
    assert!(one_position_cycle(&TelemetrySnapshot::default())
        .raw()
        .is_none());
}
