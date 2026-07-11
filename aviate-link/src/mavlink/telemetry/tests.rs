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
        // 0x0F pins the AVIATE_STATE_VALID_FLAGS wire encoding of "all
        // four dimensions valid" independently of internal bit layout.
        assert_eq!(status.valid_flags, 0x0F);
        assert_eq!(status.quality, wire_quality);
    }
}

#[test]
fn every_internal_validity_flag_has_a_distinct_wire_bit() {
    // Forces a deliberate wire decision whenever StateValidFlags gains a
    // flag: an unmapped flag would encode to 0 and fail here, instead of
    // silently vanishing from telemetry.
    let mut seen: u8 = 0;
    let mut count = 0;
    for flag in StateValidFlags::all().iter() {
        let wire = estimator::wire_valid_flags(flag);
        assert_ne!(
            wire, 0,
            "{flag:?} has no AVIATE_STATE_VALID_FLAGS wire mapping"
        );
        assert_eq!(wire.count_ones(), 1, "{flag:?} must map to a single bit");
        assert_eq!(seen & wire, 0, "{flag:?} collides with another wire bit");
        seen |= wire;
        count += 1;
    }
    // One wire bit per aviate.xml AVIATE_STATE_VALID_FLAGS entry.
    assert_eq!(count, 4);
    assert_eq!(seen, 0x0F);
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
    let formatter = MavlinkCycleFormatter::new(&cfg, 100);
    assert!(formatter.is_ok());
    let Ok(mut formatter) = formatter else {
        return;
    };
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
    let formatter = MavlinkCycleFormatter::new(&cfg, 100);
    assert!(formatter.is_ok());
    let Ok(mut formatter) = formatter else {
        return;
    };
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
    let formatter = MavlinkCycleFormatter::new(&cfg, 100);
    assert!(formatter.is_ok());
    let Ok(mut formatter) = formatter else {
        return;
    };
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

#[test]
fn transmitted_estimator_status_matches_pymavlink() {
    // pymavlink 2.4.41 golden frames for the exact standard frames Aviate
    // transmits: NaN ratios and accuracies, validity only in flags.
    // Round-trip tests cannot see a NaN bit-pattern or CRC divergence
    // that only a cross-implementation comparison exposes.
    const GOOD_ALL_VALID: &[u8] = &[
        253, 41, 0, 0, 2, 1, 1, 230, 0, 0, 104, 66, 0, 0, 0, 0, 0, 0, 0, 0, 192, 127, 0, 0, 192,
        127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0,
        192, 127, 15, 200, 247,
    ];
    const UNUSABLE_NO_FLAGS: &[u8] = &[
        253, 40, 0, 0, 3, 1, 1, 230, 0, 0, 104, 66, 0, 0, 0, 0, 0, 0, 0, 0, 192, 127, 0, 0, 192,
        127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0, 192, 127, 0, 0,
        192, 127, 223, 186,
    ];

    let good = StateEstimate {
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
        ..StateEstimate::default()
    };
    let mut seq = 2;
    let mut buf = [0u8; 64];
    let len = format_estimator_status(&good, 17, 1, 1, &mut seq, &mut buf).unwrap_or_default();
    assert_eq!(&buf[..len], GOOD_ALL_VALID);

    let unusable = StateEstimate {
        quality: EstimateQuality::Unusable,
        valid_flags: StateValidFlags::all(),
        ..StateEstimate::default()
    };
    let mut seq = 3;
    let len = format_estimator_status(&unusable, 17, 1, 1, &mut seq, &mut buf).unwrap_or_default();
    assert_eq!(&buf[..len], UNUSABLE_NO_FLAGS);
}

#[test]
fn zero_rates_are_rejected_at_construction() {
    type ZeroOut = fn(&mut TelemetryConfig);
    let fields: [(&str, ZeroOut); 4] = [
        ("heartbeat_hz", |c| c.heartbeat_hz = 0),
        ("attitude_hz", |c| c.attitude_hz = 0),
        ("position_hz", |c| c.position_hz = 0),
        ("estimator_status_hz", |c| c.estimator_status_hz = 0),
    ];
    for (name, zero_out) in fields {
        let mut cfg = TelemetryConfig::default();
        zero_out(&mut cfg);
        let result = MavlinkCycleFormatter::new(&cfg, 100);
        assert!(
            matches!(result, Err(TelemetryError::ZeroRate(field)) if field == name),
            "{name} = 0 must be rejected"
        );
    }
    assert!(matches!(
        MavlinkCycleFormatter::new(&TelemetryConfig::default(), 0),
        Err(TelemetryError::ZeroRate("loop_hz"))
    ));
}

#[test]
fn achieved_rate_never_exceeds_requested_rate() {
    // 3 Hz does not divide the 400 Hz loop; flooring the divider would
    // emit 4 heartbeats in the first second instead of 3.
    let cfg = TelemetryConfig {
        heartbeat_hz: 3,
        ..TelemetryConfig::default()
    };
    let formatter = MavlinkCycleFormatter::with_ids(&cfg, 400, 1, 1);
    assert!(formatter.is_ok());
    let Ok(mut formatter) = formatter else {
        return;
    };

    let seconds = 10u64;
    let mut heartbeats = 0u64;
    let mut queue = DefaultTelemetryQueue::new();
    for iteration in 0..(400 * seconds) {
        let snapshot = TelemetrySnapshot {
            iteration: iteration as u32,
            ..TelemetrySnapshot::default()
        };
        formatter.format_cycle(&snapshot, &mut queue);
        while queue.pop_with(|frame| {
            if let Some(message) = parsed_message(frame) {
                if message.msg_id() == 0 {
                    heartbeats = heartbeats.wrapping_add(1);
                }
            }
        }) {}
    }

    assert!(
        heartbeats <= 3 * seconds,
        "requested 3 Hz, got {heartbeats} heartbeats in {seconds} s"
    );
    // The stream must still run close to the requested rate.
    assert!(heartbeats >= 3 * seconds - 1);
}
