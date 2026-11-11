//! `VFR_HUD`, pinned with a pymavlink 2.4.41 golden frame.

use super::*;
use crate::mavlink::protocol::{parse_mavlink, serialize_mavlink};

// VFR_HUD, seq=9, from system 1, component 1. The zero high byte of the
// throttle truncates the payload from 20 to 19 bytes.
const PYMAVLINK_VFR_HUD: &[u8] = &[
    0xFD, 0x13, 0x00, 0x00, 0x09, 0x01, 0x01, 0x4A, 0x00, 0x00, 0x00, 0x00, 0x52, 0x42, 0x00, 0x00,
    0x41, 0x42, 0x00, 0xC0, 0xA0, 0x43, 0x00, 0x00, 0xE0, 0xBF, 0x13, 0x01, 0x54, 0xB4, 0x49,
];

fn hud() -> VfrHud {
    VfrHud {
        airspeed: 52.5,
        groundspeed: 48.25,
        alt: 321.5,
        climb: -1.75,
        heading: 275,
        throttle: 84,
    }
}

#[test]
fn vfr_hud_matches_pymavlink_both_ways() {
    let mut buf = [0u8; 64];
    let len = serialize_mavlink(&MavMessage::VfrHud(hud()), 9, 1, 1, &mut buf).unwrap_or_default();
    assert_eq!(&buf[..len], PYMAVLINK_VFR_HUD);
    let parsed = match parse_mavlink(PYMAVLINK_VFR_HUD) {
        Ok((MavMessage::VfrHud(m), _, _)) => Some(m),
        _ => None,
    };
    assert_eq!(parsed, Some(hud()));
}
