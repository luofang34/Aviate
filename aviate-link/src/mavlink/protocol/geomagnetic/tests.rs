//! Geomagnetic-reference layout, pinned with pymavlink golden frames
//! generated from `aviate.xml` (pymavlink 2.4.41,
//! `scripts/generate_mavlink_dialect.sh`), srcSystem=1, srcComponent=1.

use super::*;
use crate::mavlink::protocol::{parse_mavlink, serialize_mavlink};

// Model source, declination -0.2113 rad, seq=3.
const PYMAVLINK_MODEL: &[u8] = &[
    0xFD, 0x0D, 0x00, 0x00, 0x03, 0x01, 0x01, 0x21, 0x4E, 0x00, 0x00, 0x09, 0x3D, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x07, 0x5F, 0x58, 0xBE, 0x01, 0x02, 0x36,
];

// No reference: NaN declination, source NONE (truncated to 12 bytes), seq=4.
const PYMAVLINK_NONE: &[u8] = &[
    0xFD, 0x0C, 0x00, 0x00, 0x04, 0x01, 0x01, 0x21, 0x4E, 0x00, 0x00, 0x09, 0x3D, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xC0, 0x7F, 0x93, 0x73,
];

fn assert_serializes_to(msg: AviateGeomagneticReference, seq: u8, expected: &[u8]) {
    let mut buf = [0u8; 64];
    let len = serialize_mavlink(
        &MavMessage::AviateGeomagneticReference(msg),
        seq,
        1,
        1,
        &mut buf,
    )
    .unwrap_or_default();
    assert_eq!(&buf[..len], expected);
}

#[test]
fn model_reference_matches_pymavlink() {
    let msg = AviateGeomagneticReference {
        time_usec: 4_000_000,
        declination: -0.2113,
        source: aviate_geomagnetic_source::MODEL,
    };
    assert_serializes_to(msg, 3, PYMAVLINK_MODEL);
    let parsed = match parse_mavlink(PYMAVLINK_MODEL) {
        Ok((MavMessage::AviateGeomagneticReference(m), _, _)) => Some(m),
        _ => None,
    };
    assert_eq!(parsed, Some(msg));
}

#[test]
fn no_reference_matches_pymavlink() {
    let msg = AviateGeomagneticReference {
        time_usec: 4_000_000,
        declination: f32::NAN,
        source: aviate_geomagnetic_source::NONE,
    };
    assert_serializes_to(msg, 4, PYMAVLINK_NONE);
    let parsed = match parse_mavlink(PYMAVLINK_NONE) {
        Ok((MavMessage::AviateGeomagneticReference(m), _, _)) => Some(m),
        _ => None,
    };
    assert!(parsed
        .is_some_and(|m| m.declination.is_nan() && m.source == aviate_geomagnetic_source::NONE));
}
