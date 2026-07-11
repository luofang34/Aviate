use super::*;
use crate::mavlink::protocol::{crc_accumulate, parse_mavlink, serialize_mavlink};

#[test]
fn standard_status_round_trips() {
    let source = EstimatorStatus {
        time_usec: 42_000,
        vel_ratio: 0.1,
        pos_horiz_ratio: 0.2,
        pos_vert_ratio: 0.3,
        mag_ratio: 0.4,
        hagl_ratio: 0.5,
        tas_ratio: 0.6,
        pos_horiz_accuracy: 1.2,
        pos_vert_accuracy: 2.3,
        flags: estimator_status_flags::ATTITUDE | estimator_status_flags::VELOCITY_HORIZ,
    };
    let mut buf = [0u8; 64];
    let len = serialize_mavlink(&MavMessage::EstimatorStatus(source), 7, 1, 1, &mut buf)
        .unwrap_or_default();
    let parsed = match parse_mavlink(&buf[..len]) {
        Ok((MavMessage::EstimatorStatus(status), _, _)) => Some(status),
        _ => None,
    };
    assert!(parsed.is_some());
    let parsed = parsed.unwrap_or_default();

    assert_eq!(parsed.time_usec, source.time_usec);
    assert_eq!(parsed.flags, source.flags);
    assert_eq!(parsed.pos_vert_accuracy, source.pos_vert_accuracy);
}

#[test]
fn aviate_quality_values_round_trip_losslessly() {
    for quality in [
        aviate_estimate_quality::GOOD,
        aviate_estimate_quality::DEGRADED,
        aviate_estimate_quality::UNUSABLE,
    ] {
        let source = AviateEstimatorStatus {
            time_usec: 99_000,
            valid_flags: aviate_state_valid_flags::ATTITUDE
                | aviate_state_valid_flags::ANGULAR_RATE
                | aviate_state_valid_flags::POSITION
                | aviate_state_valid_flags::VELOCITY,
            quality,
        };
        let mut buf = [0u8; 32];
        let len = serialize_mavlink(
            &MavMessage::AviateEstimatorStatus(source),
            quality,
            1,
            1,
            &mut buf,
        )
        .unwrap_or_default();
        let parsed = match parse_mavlink(&buf[..len]) {
            Ok((MavMessage::AviateEstimatorStatus(status), _, _)) => Some(status),
            _ => None,
        };
        assert!(parsed.is_some());
        let parsed = parsed.unwrap_or_default();

        assert_eq!(parsed.time_usec, source.time_usec);
        assert_eq!(parsed.valid_flags, source.valid_flags);
        assert_eq!(parsed.quality, quality);
    }
}

// Golden frames produced by pymavlink 2.4.41 generated from aviate.xml
// (srcSystem=1, srcComponent=1).

#[test]
fn aviate_status_matches_pymavlink_dialect_vector() {
    // GOOD with every validity bit set: the non-zero tail keeps the full
    // 10-byte payload on the wire.
    const PYMAVLINK_FRAME: [u8; 22] = [
        253, 10, 0, 0, 7, 1, 1, 32, 78, 0, 184, 130, 1, 0, 0, 0, 0, 0, 15, 2, 238, 73,
    ];
    let status = AviateEstimatorStatus {
        time_usec: 99_000,
        valid_flags: 0x0F,
        quality: aviate_estimate_quality::GOOD,
    };
    let mut buf = [0u8; PYMAVLINK_FRAME.len()];
    let len = serialize_mavlink(
        &MavMessage::AviateEstimatorStatus(status),
        7,
        1,
        1,
        &mut buf,
    )
    .unwrap_or_default();

    assert_eq!(len, PYMAVLINK_FRAME.len());
    assert_eq!(buf, PYMAVLINK_FRAME);
}

#[test]
fn aviate_degraded_status_matches_pymavlink_dialect_vector() {
    // DEGRADED with attitude and angular-rate valid; quality=1 in the last
    // byte keeps the full payload.
    const PYMAVLINK_FRAME: [u8; 22] = [
        253, 10, 0, 0, 4, 1, 1, 32, 78, 0, 144, 208, 3, 0, 0, 0, 0, 0, 3, 1, 137, 232,
    ];
    let status = AviateEstimatorStatus {
        time_usec: 250_000,
        valid_flags: aviate_state_valid_flags::ATTITUDE | aviate_state_valid_flags::ANGULAR_RATE,
        quality: aviate_estimate_quality::DEGRADED,
    };
    let mut buf = [0u8; PYMAVLINK_FRAME.len()];
    let len = serialize_mavlink(
        &MavMessage::AviateEstimatorStatus(status),
        4,
        1,
        1,
        &mut buf,
    )
    .unwrap_or_default();

    assert_eq!(len, PYMAVLINK_FRAME.len());
    assert_eq!(buf, PYMAVLINK_FRAME);

    let parsed = match parse_mavlink(&PYMAVLINK_FRAME) {
        Ok((MavMessage::AviateEstimatorStatus(status), _, _)) => Some(status),
        _ => None,
    }
    .unwrap_or_default();
    assert_eq!(parsed.valid_flags, 0x03);
    assert_eq!(parsed.quality, aviate_estimate_quality::DEGRADED);
}

#[test]
fn mavlink_two_zero_tail_truncation_is_restored() {
    // Unusable with empty flags truncates to a 2-byte payload; the parser
    // must restore the zero tail. Both frames are pymavlink output.
    const AVIATE_UNUSABLE: [u8; 14] = [253, 2, 0, 0, 7, 1, 1, 32, 78, 0, 104, 66, 104, 226];
    const STANDARD_ZERO: [u8; 14] = [253, 2, 0, 0, 7, 1, 1, 230, 0, 0, 104, 66, 51, 209];

    let aviate = match parse_mavlink(&AVIATE_UNUSABLE) {
        Ok((MavMessage::AviateEstimatorStatus(status), _, _)) => Some(status),
        _ => None,
    }
    .unwrap_or_default();
    assert_eq!(aviate.time_usec, 17_000);
    assert_eq!(aviate.valid_flags, 0);
    assert_eq!(aviate.quality, aviate_estimate_quality::UNUSABLE);

    let standard = match parse_mavlink(&STANDARD_ZERO) {
        Ok((MavMessage::EstimatorStatus(status), _, _)) => Some(status),
        _ => None,
    }
    .unwrap_or_default();
    assert_eq!(standard.time_usec, 17_000);
    assert_eq!(standard.flags, 0);
    assert_eq!(standard.pos_vert_accuracy, 0.0);
}

#[test]
fn aviate_unusable_serializer_truncates_to_pymavlink_frame() {
    // The serializer must emit the same truncated bytes pymavlink does for
    // the fail-safe state, not a zero-padded full payload.
    const AVIATE_UNUSABLE: [u8; 14] = [253, 2, 0, 0, 7, 1, 1, 32, 78, 0, 104, 66, 104, 226];
    let status = AviateEstimatorStatus {
        time_usec: 17_000,
        valid_flags: 0,
        quality: aviate_estimate_quality::UNUSABLE,
    };
    let mut buf = [0u8; 32];
    let len = serialize_mavlink(
        &MavMessage::AviateEstimatorStatus(status),
        7,
        1,
        1,
        &mut buf,
    )
    .unwrap_or_default();

    assert_eq!(len, AVIATE_UNUSABLE.len());
    assert_eq!(&buf[..len], &AVIATE_UNUSABLE);
}

#[test]
fn aviate_xml_wire_facts_match_rust_constants() {
    // aviate.xml is the canonical wire contract. Folding its field list
    // into the CRC seed here means an XML edit that changes the wire
    // (field order, type, or name) fails cargo test instead of surfacing
    // as silent CRC mismatches on a peer generated from the XML.
    let xml = include_str!("../../../../message_definitions/aviate.xml");

    let start = xml.find("name=\"AVIATE_ESTIMATOR_STATUS\"");
    assert!(start.is_some(), "message block missing from aviate.xml");
    let Some(start) = start else {
        return;
    };
    let block_len = xml[start..].find("</message>");
    assert!(block_len.is_some(), "unterminated message block");
    let Some(block_len) = block_len else {
        return;
    };
    let block = &xml[start..start + block_len];

    // Collect (type, name, size) in declaration order.
    let mut fields = [("", "", 0usize); 8];
    let mut count = 0;
    let mut rest = block;
    while let Some(pos) = rest.find("<field type=\"") {
        let after = &rest[pos + 13..];
        let Some(type_end) = after.find('"') else {
            break;
        };
        let field_type = &after[..type_end];
        let Some(name_pos) = after[type_end..].find("name=\"") else {
            break;
        };
        let name_rest = &after[type_end + name_pos + 6..];
        let Some(name_end) = name_rest.find('"') else {
            break;
        };
        let field_name = &name_rest[..name_end];
        let size = match field_type {
            "uint64_t" | "int64_t" | "double" => 8,
            "uint32_t" | "int32_t" | "float" => 4,
            "uint16_t" | "int16_t" => 2,
            "uint8_t" | "int8_t" | "char" => 1,
            _ => 0,
        };
        assert!(size != 0, "unhandled field type in aviate.xml");
        fields[count] = (field_type, field_name, size);
        count += 1;
        rest = &name_rest[name_end..];
    }
    assert_eq!(count, 3);

    let payload_len: usize = fields[..count].iter().map(|f| f.2).sum();
    assert_eq!(payload_len, AviateEstimatorStatus::PAYLOAD_LEN);

    // MAVLink reorders fields by size (descending, stable) on the wire.
    let wire = &mut fields[..count];
    let mut i = 1;
    while i < wire.len() {
        let mut j = i;
        while j > 0 && wire[j - 1].2 < wire[j].2 {
            wire.swap(j - 1, j);
            j -= 1;
        }
        i += 1;
    }

    fn feed(crc: u16, text: &str) -> u16 {
        let mut crc = crc;
        for &byte in text.as_bytes() {
            crc = crc_accumulate(byte, crc);
        }
        crc
    }
    let mut crc = feed(0xFFFF, "AVIATE_ESTIMATOR_STATUS ");
    for (field_type, field_name, _) in wire.iter() {
        crc = feed(crc, field_type);
        crc = feed(crc, " ");
        crc = feed(crc, field_name);
        crc = feed(crc, " ");
    }
    let crc_extra = ((crc & 0xFF) ^ (crc >> 8)) as u8;
    assert_eq!(crc_extra, AviateEstimatorStatus::CRC_EXTRA);

    // Enum wire values are part of the contract.
    assert!(xml.contains("value=\"0\" name=\"AVIATE_ESTIMATE_QUALITY_UNUSABLE\""));
    assert!(xml.contains("value=\"1\" name=\"AVIATE_ESTIMATE_QUALITY_DEGRADED\""));
    assert!(xml.contains("value=\"2\" name=\"AVIATE_ESTIMATE_QUALITY_GOOD\""));
    assert_eq!(aviate_estimate_quality::UNUSABLE, 0);
    assert_eq!(aviate_estimate_quality::DEGRADED, 1);
    assert_eq!(aviate_estimate_quality::GOOD, 2);

    assert!(xml.contains("value=\"1\" name=\"AVIATE_STATE_VALID_ATTITUDE\""));
    assert!(xml.contains("value=\"2\" name=\"AVIATE_STATE_VALID_ANGULAR_RATE\""));
    assert!(xml.contains("value=\"4\" name=\"AVIATE_STATE_VALID_POSITION\""));
    assert!(xml.contains("value=\"8\" name=\"AVIATE_STATE_VALID_VELOCITY\""));
    assert_eq!(aviate_state_valid_flags::ATTITUDE, 1);
    assert_eq!(aviate_state_valid_flags::ANGULAR_RATE, 2);
    assert_eq!(aviate_state_valid_flags::POSITION, 4);
    assert_eq!(aviate_state_valid_flags::VELOCITY, 8);
}
