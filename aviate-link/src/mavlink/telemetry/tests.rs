use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};

use super::*;
use crate::mavlink::protocol::{
    aviate_estimate_quality, estimator_status_flags, parse_mavlink, MavMessage,
};
use crate::queue::DefaultTelemetryQueue;
use crate::telemetry::{TelemetryCycleFormatter, TelemetrySnapshot};

fn parsed_message(buf: &[u8]) -> Option<MavMessage> {
    parse_mavlink(buf).ok().map(|(message, _, _)| message)
}

#[test]
fn quality_and_validity_are_lossless() {
    for (quality, wire_quality) in [
        (EstimateQuality::Good, aviate_estimate_quality::GOOD),
        (EstimateQuality::Degraded, aviate_estimate_quality::DEGRADED),
        (EstimateQuality::Unusable, aviate_estimate_quality::UNUSABLE),
    ] {
        let state = StateEstimate {
            quality,
            valid_flags: StateValidFlags::all(),
            ..StateEstimate::default()
        };
        let mut seq = 0;
        let mut buf = [0u8; 64];
        let len = format_aviate_estimator_status(&state, 17, 1, 1, &mut seq, &mut buf)
            .unwrap_or_default();
        let parsed = parsed_message(&buf[..len]);
        assert!(matches!(parsed, Some(MavMessage::AviateEstimatorStatus(_))));
        let status = match parsed {
            Some(MavMessage::AviateEstimatorStatus(status)) => status,
            _ => Default::default(),
        };

        assert_eq!(status.time_usec, 17_000);
        assert_eq!(status.valid_flags, StateValidFlags::all().bits());
        assert_eq!(status.quality, wire_quality);
        let expected_standard = if quality == EstimateQuality::Good {
            estimator_status_flags::ATTITUDE
                | estimator_status_flags::VELOCITY_HORIZ
                | estimator_status_flags::VELOCITY_VERT
                | estimator_status_flags::POS_HORIZ_REL
        } else {
            0
        };
        assert_eq!(status.standard_flags, expected_standard);
    }
}

#[test]
fn standard_flags_map_only_matching_valid_dimensions() {
    let cases = [
        (StateValidFlags::ATTITUDE, estimator_status_flags::ATTITUDE),
        (
            StateValidFlags::VELOCITY,
            estimator_status_flags::VELOCITY_HORIZ | estimator_status_flags::VELOCITY_VERT,
        ),
        (
            StateValidFlags::POSITION,
            estimator_status_flags::POS_HORIZ_REL,
        ),
        (StateValidFlags::ANGULAR_RATE, 0),
    ];

    for (valid_flags, expected) in cases {
        let state = StateEstimate {
            quality: EstimateQuality::Good,
            valid_flags,
            ..StateEstimate::default()
        };
        assert_eq!(estimator::standard_estimator_flags(&state), expected);
    }
}

#[test]
fn fresh_unusable_estimate_clears_standard_flags() {
    let state = StateEstimate {
        quality: EstimateQuality::Unusable,
        valid_flags: StateValidFlags::all(),
        ..StateEstimate::default()
    };
    let mut seq = 0;
    let mut buf = [0u8; 64];
    let len = format_estimator_status(&state, 9_999, 1, 1, &mut seq, &mut buf).unwrap_or_default();
    let parsed = parsed_message(&buf[..len]);
    assert!(matches!(parsed, Some(MavMessage::EstimatorStatus(_))));
    let status = match parsed {
        Some(MavMessage::EstimatorStatus(status)) => status,
        _ => Default::default(),
    };

    assert_eq!(status.time_usec, 9_999_000);
    assert_eq!(status.flags, 0);
}

#[test]
fn degraded_estimate_does_not_claim_good_standard_outputs() {
    let state = StateEstimate {
        quality: EstimateQuality::Degraded,
        valid_flags: StateValidFlags::all(),
        ..StateEstimate::default()
    };

    assert_eq!(estimator::standard_estimator_flags(&state), 0);
}

#[test]
fn attitude_and_position_bytes_ignore_validity_metadata() {
    let good = StateEstimate {
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
        ..StateEstimate::default()
    };
    let unusable = StateEstimate {
        quality: EstimateQuality::Unusable,
        valid_flags: StateValidFlags::empty(),
        ..good.clone()
    };
    let mut good_buf = [0u8; 64];
    let mut unusable_buf = [0u8; 64];
    let good_attitude = format_attitude(&good, 42, 1, 1, &mut 0, &mut good_buf).unwrap_or_default();
    let unusable_attitude =
        format_attitude(&unusable, 42, 1, 1, &mut 0, &mut unusable_buf).unwrap_or_default();
    assert_eq!(good_attitude, unusable_attitude);
    assert_eq!(
        &good_buf[..good_attitude],
        &unusable_buf[..unusable_attitude]
    );

    let good_position =
        format_local_position(&good, 42, 1, 1, &mut 0, &mut good_buf).unwrap_or_default();
    let unusable_position =
        format_local_position(&unusable, 42, 1, 1, &mut 0, &mut unusable_buf).unwrap_or_default();
    assert_eq!(good_position, unusable_position);
    assert_eq!(
        &good_buf[..good_position],
        &unusable_buf[..unusable_position]
    );
}

#[test]
fn cycle_formatter_emits_status_at_configured_rate() {
    let cfg = TelemetryConfig {
        estimator_status_hz: 4,
        ..TelemetryConfig::default()
    };
    let mut formatter = MavlinkCycleFormatter::new(&cfg, 100);
    let mut queue = DefaultTelemetryQueue::new();
    let mut snapshot = TelemetrySnapshot {
        iteration: 1,
        ..TelemetrySnapshot::default()
    };

    formatter.format_cycle(&snapshot, &mut queue);
    assert_eq!(queue.len(), 0);
    snapshot.iteration = 25;
    formatter.format_cycle(&snapshot, &mut queue);

    let mut ids = [0u32; 4];
    let mut count = 0;
    while queue.pop_with(|frame| {
        if let Some(message) = parsed_message(frame) {
            ids[count] = message.msg_id();
            count = count.wrapping_add(1);
        }
    }) {}
    assert_eq!(count, 3);
    assert!(ids[..count].contains(&32));
    assert!(ids[..count].contains(&230));
    assert!(ids[..count].contains(&20_000));
}

#[test]
fn each_numeric_snapshot_is_preceded_by_same_time_status() {
    let cfg = TelemetryConfig {
        attitude_hz: 10,
        position_hz: 1,
        estimator_status_hz: 4,
        ..TelemetryConfig::default()
    };
    let mut formatter = MavlinkCycleFormatter::new(&cfg, 100);
    let mut queue = DefaultTelemetryQueue::new();
    let snapshot = TelemetrySnapshot {
        time_ms: 200,
        iteration: 20,
        state: StateEstimate {
            quality: EstimateQuality::Unusable,
            valid_flags: StateValidFlags::empty(),
            ..StateEstimate::default()
        },
        ..TelemetrySnapshot::default()
    };

    formatter.format_cycle(&snapshot, &mut queue);
    let mut ids = [0u32; 3];
    let mut times = [0u64; 3];
    let mut count = 0;
    while queue.pop_with(|frame| {
        if let Some(message) = parsed_message(frame) {
            ids[count] = message.msg_id();
            times[count] = match message {
                MavMessage::EstimatorStatus(status) => status.time_usec,
                MavMessage::AviateEstimatorStatus(status) => status.time_usec,
                MavMessage::AttitudeQuaternion(attitude) => {
                    u64::from(attitude.time_boot_ms) * 1_000
                }
                _ => 0,
            };
            count = count.wrapping_add(1);
        }
    }) {}

    assert_eq!(&ids[..count], &[230, 20_000, 31]);
    assert_eq!(&times[..count], &[200_000; 3]);
}

#[test]
fn estimate_group_is_dropped_whole_when_queue_has_no_room() {
    let cfg = TelemetryConfig::default();
    let mut formatter = MavlinkCycleFormatter::new(&cfg, 100);
    let mut queue = DefaultTelemetryQueue::new();
    for _ in 0..30 {
        assert!(queue.push(&[0]).is_ok());
    }
    let snapshot = TelemetrySnapshot {
        iteration: 10,
        ..TelemetrySnapshot::default()
    };

    formatter.format_cycle(&snapshot, &mut queue);

    assert_eq!(queue.len(), 30);
}
