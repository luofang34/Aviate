use super::*;
use crate::mavlink::protocol::{parse_mavlink, serialize_mavlink};

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
            standard_flags: estimator_status_flags::POS_HORIZ_REL,
            valid_flags: 0x0f,
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
        assert_eq!(parsed.standard_flags, source.standard_flags);
        assert_eq!(parsed.valid_flags, source.valid_flags);
        assert_eq!(parsed.quality, quality);
    }
}

#[test]
fn aviate_status_matches_pymavlink_dialect_vector() {
    const PYMAVLINK_FRAME: [u8; 24] = [
        253, 12, 0, 0, 7, 1, 1, 32, 78, 0, 184, 130, 1, 0, 0, 0, 0, 0, 8, 0, 15, 2, 140, 7,
    ];
    let status = AviateEstimatorStatus {
        time_usec: 99_000,
        standard_flags: estimator_status_flags::POS_HORIZ_REL,
        valid_flags: 0x0f,
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
fn mavlink_two_zero_tail_truncation_is_restored() {
    const AVIATE_UNUSABLE: [u8; 14] = [253, 2, 0, 0, 7, 1, 1, 32, 78, 0, 104, 66, 32, 235];
    const STANDARD_ZERO: [u8; 14] = [253, 2, 0, 0, 7, 1, 1, 230, 0, 0, 104, 66, 51, 209];

    let aviate = match parse_mavlink(&AVIATE_UNUSABLE) {
        Ok((MavMessage::AviateEstimatorStatus(status), _, _)) => Some(status),
        _ => None,
    }
    .unwrap_or_default();
    assert_eq!(aviate.time_usec, 17_000);
    assert_eq!(aviate.standard_flags, 0);
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
