//! Geodetic message layouts, pinned with pymavlink golden frames.

use super::*;
use crate::mavlink::protocol::{parse_mavlink, serialize_mavlink};

// Golden frames below were produced by pymavlink 2.4.41
// (pymavlink.dialects.v20.common) with srcSystem=1, srcComponent=1.

// GLOBAL_POSITION_INT, seq=4.
const PYMAVLINK_GLOBAL_POSITION_INT: &[u8] = &[
    0xFD, 0x1C, 0x00, 0x00, 0x04, 0x01, 0x01, 0x21, 0x00, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x4C, 0x52,
    0x40, 0x1C, 0xD1, 0x03, 0x18, 0x05, 0xC8, 0x85, 0x07, 0x00, 0x88, 0x13, 0x00, 0x00, 0x96, 0x00,
    0xB5, 0xFF, 0xEC, 0xFF, 0x28, 0x23, 0x8D, 0xE1,
];

// GPS_RAW_INT with extensions, seq=5. The zero vel_acc, hdg_acc and yaw
// extensions truncate the payload from 52 to 40 bytes.
const PYMAVLINK_GPS_RAW_INT: &[u8] = &[
    0xFD, 0x28, 0x00, 0x00, 0x05, 0x01, 0x01, 0x18, 0x00, 0x00, 0x00, 0xCA, 0x5B, 0x07, 0x00, 0x00,
    0x00, 0x00, 0x4C, 0x52, 0x40, 0x1C, 0xD1, 0x03, 0x18, 0x05, 0xC8, 0x85, 0x07, 0x00, 0xFF, 0xFF,
    0xFF, 0xFF, 0xA8, 0x00, 0xC7, 0x18, 0x03, 0x0E, 0xC8, 0x85, 0x07, 0x00, 0xF4, 0x01, 0x00, 0x00,
    0x20, 0x03, 0x56, 0x66,
];

fn global_position() -> GlobalPositionInt {
    GlobalPositionInt {
        time_boot_ms: 123_456,
        lat: 473_977_420,
        lon: 85_459_921,
        alt: 493_000,
        relative_alt: 5_000,
        vx: 150,
        vy: -75,
        vz: -20,
        hdg: 9_000,
    }
}

fn gps_raw() -> GpsRawInt {
    GpsRawInt {
        time_usec: 123_456_000,
        fix_type: gps_fix_type::FIX_3D,
        lat: 473_977_420,
        lon: 85_459_921,
        alt: 493_000,
        eph: u16::MAX,
        epv: u16::MAX,
        vel: 168,
        cog: 6_343,
        satellites_visible: 14,
        alt_ellipsoid: 493_000,
        h_acc: 500,
        v_acc: 800,
        vel_acc: 0,
        hdg_acc: 0,
        yaw: 0,
    }
}

fn assert_serializes_to(msg: MavMessage, seq: u8, expected: &[u8]) {
    let mut buf = [0u8; 128];
    let len = serialize_mavlink(&msg, seq, 1, 1, &mut buf).unwrap_or_default();
    assert_eq!(&buf[..len], expected);
}

#[test]
fn global_position_int_matches_pymavlink() {
    assert_serializes_to(
        MavMessage::GlobalPositionInt(global_position()),
        4,
        PYMAVLINK_GLOBAL_POSITION_INT,
    );
}

#[test]
fn gps_raw_int_with_extensions_matches_pymavlink() {
    assert_serializes_to(MavMessage::GpsRawInt(gps_raw()), 5, PYMAVLINK_GPS_RAW_INT);
}

#[test]
fn pymavlink_frames_parse_to_the_same_fields() {
    let global = match parse_mavlink(PYMAVLINK_GLOBAL_POSITION_INT) {
        Ok((MavMessage::GlobalPositionInt(m), _, _)) => Some(m),
        _ => None,
    };
    assert_eq!(global, Some(global_position()));
    let raw = match parse_mavlink(PYMAVLINK_GPS_RAW_INT) {
        Ok((MavMessage::GpsRawInt(m), _, _)) => Some(m),
        _ => None,
    };
    assert_eq!(raw, Some(gps_raw()));
}
