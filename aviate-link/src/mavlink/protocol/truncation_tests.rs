//! MAVLink 2 trailing-zero payload truncation semantics, pinned with
//! pymavlink golden frames in both directions.

use super::*;

fn parse_frame(input: &[u8]) -> Option<(MavMessage, usize)> {
    let parsed = parse_mavlink(input);
    assert!(parsed.is_ok(), "Parse failed: {:?}", parsed.err());
    let Ok((msg, _sig, consumed)) = parsed else {
        return None;
    };
    Some((msg, consumed))
}

// --- MAVLink 2 trailing-zero payload truncation ---
//
// Golden frames below were produced by pymavlink 2.4.41
// (pymavlink.dialects.v20.common) with srcSystem=1, srcComponent=1.
// Compliant senders strip trailing zero payload bytes, so these frames
// are shorter than the declared layouts.

// COMMAND_LONG, seq=7: command=400, param1=1.0, broadcast targets and
// confirmation zero. Payload truncates from 33 to 30 bytes.
const PYMAVLINK_COMMAND_LONG_BROADCAST: &[u8] = &[
    0xFD, 0x1E, 0x00, 0x00, 0x07, 0x01, 0x01, 0x4C, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x01, 0xCA, 0xEA,
];

// COMMAND_ACK, seq=9: command=400, result=0 (MAV_RESULT_ACCEPTED).
// Payload truncates from 10 to 2 bytes.
const PYMAVLINK_COMMAND_ACK_ACCEPTED: &[u8] = &[
    0xFD, 0x02, 0x00, 0x00, 0x09, 0x01, 0x01, 0x4D, 0x00, 0x00, 0x90, 0x01, 0x97, 0x6A,
];

// SYSTEM_TIME, seq=3: both fields zero. The all-zero payload truncates
// to the protocol-required minimum of one byte.
const PYMAVLINK_SYSTEM_TIME_ZERO: &[u8] = &[
    0xFD, 0x01, 0x00, 0x00, 0x03, 0x01, 0x01, 0x02, 0x00, 0x00, 0x00, 0xD5, 0xBB,
];

// ATTITUDE_QUATERNION, seq=5: identity quaternion, zero rates and time.
// Payload truncates from 48 to 8 bytes (through q1 = 1.0).
const PYMAVLINK_ATTITUDE_Q_IDENTITY: &[u8] = &[
    0xFD, 0x08, 0x00, 0x00, 0x05, 0x01, 0x01, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x3F, 0xAA, 0x07,
];

// HEARTBEAT, seq=1: quadrotor, active. mavlink_version=3 keeps the
// final byte non-zero, so the payload stays at the full 9 bytes.
const PYMAVLINK_HEARTBEAT_ACTIVE: &[u8] = &[
    0xFD, 0x09, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
    0x00, 0x04, 0x03, 0x96, 0x44,
];

// LOCAL_POSITION_NED, seq=2: position set, zero velocity. Payload
// truncates from 28 to 16 bytes.
const PYMAVLINK_LOCAL_POSITION_ZERO_VEL: &[u8] = &[
    0xFD, 0x10, 0x00, 0x00, 0x02, 0x01, 0x01, 0x20, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0x00, 0x00, 0x20, 0xC1, 0xD8, 0x0E,
];

#[test]
fn test_parse_truncated_command_long_broadcast() {
    let Some((msg, consumed)) = parse_frame(PYMAVLINK_COMMAND_LONG_BROADCAST) else {
        return;
    };
    assert_eq!(consumed, PYMAVLINK_COMMAND_LONG_BROADCAST.len());
    assert!(
        matches!(&msg, MavMessage::CommandLong(_)),
        "Expected CommandLong, got {msg:?}"
    );
    let MavMessage::CommandLong(c) = msg else {
        return;
    };
    assert_eq!(c.command, 400);
    assert!((c.param1 - 1.0).abs() < 1e-5);
    assert_eq!(c.target_system, 0);
    assert_eq!(c.target_component, 0);
    assert_eq!(c.confirmation, 0);
}

#[test]
fn test_parse_truncated_command_ack_accepted() {
    let Some((msg, _)) = parse_frame(PYMAVLINK_COMMAND_ACK_ACCEPTED) else {
        return;
    };
    assert!(
        matches!(&msg, MavMessage::CommandAck(_)),
        "Expected CommandAck, got {msg:?}"
    );
    let MavMessage::CommandAck(a) = msg else {
        return;
    };
    assert_eq!(a.command, 400);
    assert_eq!(a.result, 0); // MAV_RESULT_ACCEPTED
    assert_eq!(a.progress, 0);
    assert_eq!(a.result_param2, 0);
}

#[test]
fn test_parse_minimum_single_byte_payload() {
    let Some((msg, _)) = parse_frame(PYMAVLINK_SYSTEM_TIME_ZERO) else {
        return;
    };
    assert!(
        matches!(&msg, MavMessage::SystemTime(_)),
        "Expected SystemTime, got {msg:?}"
    );
    let MavMessage::SystemTime(t) = msg else {
        return;
    };
    assert_eq!(t.time_unix_usec, 0);
    assert_eq!(t.time_boot_ms, 0);
}

#[test]
fn test_parse_truncated_attitude_quaternion() {
    let Some((msg, _)) = parse_frame(PYMAVLINK_ATTITUDE_Q_IDENTITY) else {
        return;
    };
    assert!(
        matches!(&msg, MavMessage::AttitudeQuaternion(_)),
        "Expected AttitudeQuaternion, got {msg:?}"
    );
    let MavMessage::AttitudeQuaternion(q) = msg else {
        return;
    };
    assert!((q.q1 - 1.0).abs() < 1e-6);
    assert!(q.q2.abs() < 1e-6 && q.q3.abs() < 1e-6 && q.q4.abs() < 1e-6);
    assert!(q.rollspeed.abs() < 1e-6);
}

#[test]
fn test_serializer_truncation_matches_pymavlink() {
    struct Case {
        msg: MavMessage,
        seq: u8,
        expected: &'static [u8],
    }
    let cases = [
        Case {
            msg: MavMessage::CommandLong(CommandLong {
                param1: 1.0,
                command: 400,
                ..Default::default()
            }),
            seq: 7,
            expected: PYMAVLINK_COMMAND_LONG_BROADCAST,
        },
        Case {
            msg: MavMessage::CommandAck(CommandAck {
                command: 400,
                ..Default::default()
            }),
            seq: 9,
            expected: PYMAVLINK_COMMAND_ACK_ACCEPTED,
        },
        Case {
            msg: MavMessage::SystemTime(SystemTime::default()),
            seq: 3,
            expected: PYMAVLINK_SYSTEM_TIME_ZERO,
        },
        Case {
            msg: MavMessage::AttitudeQuaternion(AttitudeQuaternion {
                q1: 1.0,
                ..Default::default()
            }),
            seq: 5,
            expected: PYMAVLINK_ATTITUDE_Q_IDENTITY,
        },
        Case {
            msg: MavMessage::Heartbeat(Heartbeat {
                mav_type: 2, // MAV_TYPE_QUADROTOR
                autopilot: 0,
                base_mode: 0,
                custom_mode: 0,
                system_status: 4, // MAV_STATE_ACTIVE
                mavlink_version: 3,
            }),
            seq: 1,
            expected: PYMAVLINK_HEARTBEAT_ACTIVE,
        },
        Case {
            msg: MavMessage::LocalPositionNed(LocalPositionNed {
                time_boot_ms: 1000,
                x: 1.5,
                y: -2.5,
                z: -10.0,
                ..Default::default()
            }),
            seq: 2,
            expected: PYMAVLINK_LOCAL_POSITION_ZERO_VEL,
        },
    ];

    for case in &cases {
        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&case.msg, case.seq, 1, 1, &mut buf);
        assert_eq!(
            len,
            Some(case.expected.len()),
            "Frame length mismatch for {:?}",
            case.msg
        );
        let Some(len) = len else {
            return;
        };
        assert_eq!(&buf[..len], case.expected, "Frame bytes for {:?}", case.msg);
    }
}

#[test]
fn test_parse_ignores_bytes_beyond_declared_layout() {
    // A newer dialect may append extension fields this parser does not
    // know. Build a HEARTBEAT with one extra payload byte and a valid
    // CRC over the on-wire payload.
    let mut buf = [0u8; 64];
    let offset = write_header(&mut buf, 10, 0, 1, 1, Heartbeat::MSG_ID);
    let payload: [u8; 10] = [0x2A, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x04, 0x03, 0xAB];
    buf[offset..offset + 10].copy_from_slice(&payload);
    let crc = compute_crc(&buf[1..offset + 10], get_crc_extra(Heartbeat::MSG_ID));
    buf[offset + 10] = (crc & 0xFF) as u8;
    buf[offset + 11] = ((crc >> 8) & 0xFF) as u8;

    let Some((msg, consumed)) = parse_frame(&buf[..offset + 12]) else {
        return;
    };
    assert_eq!(consumed, offset + 12);
    assert!(
        matches!(&msg, MavMessage::Heartbeat(_)),
        "Expected Heartbeat, got {msg:?}"
    );
    let MavMessage::Heartbeat(h) = msg else {
        return;
    };
    assert_eq!(h.custom_mode, 0x2A);
    assert_eq!(h.mav_type, 2);
    assert_eq!(h.system_status, 4);
    assert_eq!(h.mavlink_version, 3);
}

#[test]
fn test_parse_rejects_empty_payload_for_known_message() {
    // The truncation rule always retains at least one payload byte, so
    // a zero-length payload for a known message is not a legal frame.
    let mut buf = [0u8; 16];
    let offset = write_header(&mut buf, 0, 0, 1, 1, Heartbeat::MSG_ID);
    let crc = compute_crc(&buf[1..offset], get_crc_extra(Heartbeat::MSG_ID));
    buf[offset] = (crc & 0xFF) as u8;
    buf[offset + 1] = ((crc >> 8) & 0xFF) as u8;

    let result = parse_mavlink(&buf[..offset + 2]);
    assert!(matches!(result, Err(ParseError::InvalidPayload)));
}

#[test]
fn test_round_trip_survives_zero_tail_truncation() {
    // STATUSTEXT pads its 50-byte text field with zeros; the serialized
    // frame must truncate the padding and still decode identically.
    let mut text = [0u8; 50];
    text[..5].copy_from_slice(b"READY");
    let msg = MavMessage::Statustext(Statustext {
        severity: 6,
        text,
        ..Default::default()
    });

    let mut buf = [0u8; 256];
    let len = serialize_mavlink(&msg, 0, 1, 1, &mut buf);
    assert!(len.is_some(), "Serialization failed");
    let Some(len) = len else {
        return;
    };
    assert!(
        usize::from(buf[1]) < Statustext::PAYLOAD_LEN,
        "Zero padding must be truncated on the wire"
    );

    let Some((parsed, _)) = parse_frame(&buf[..len]) else {
        return;
    };
    assert!(
        matches!(&parsed, MavMessage::Statustext(_)),
        "Expected Statustext, got {parsed:?}"
    );
    let MavMessage::Statustext(s) = parsed else {
        return;
    };
    assert_eq!(s.severity, 6);
    assert_eq!(&s.text[..5], b"READY");
    assert!(s.text[5..].iter().all(|&b| b == 0));
}

// MANUAL_CONTROL, seq=8, base-only values with a zero extension tail:
// truncates from 30 to 11 bytes.
const PYMAVLINK_MANUAL_CONTROL_BASE: &[u8] = &[
    0xFD, 0x0B, 0x00, 0x00, 0x08, 0x01, 0x01, 0x45, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x94, 0x37,
];

// RC_CHANNELS_OVERRIDE, seq=10: chan9..18 zero, truncates 38 -> 18.
const PYMAVLINK_RC_OVERRIDE_TRUNCATED: &[u8] = &[
    0xFD, 0x12, 0x00, 0x00, 0x0A, 0x01, 0x01, 0x46, 0x00, 0x00, 0xDC, 0x05, 0xDD, 0x05, 0xDE, 0x05,
    0xDF, 0x05, 0xE0, 0x05, 0xE1, 0x05, 0xE2, 0x05, 0xE3, 0x05, 0x01, 0x01, 0x2C, 0xD0,
];

// SYS_STATUS, seq=11: extended sensor fields zero, truncates 43 -> 31.
const PYMAVLINK_SYS_STATUS_TRUNCATED: &[u8] = &[
    0xFD, 0x1F, 0x00, 0x00, 0x0B, 0x01, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xF4, 0x01, 0x5C, 0x2B, 0xDC, 0x05, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x65, 0xBA,
];

// SET_ATTITUDE_TARGET, seq=12: thrust_body extension zero, 51 -> 39.
const PYMAVLINK_SET_ATT_TRUNCATED: &[u8] = &[
    0xFD, 0x27, 0x00, 0x00, 0x0C, 0x01, 0x01, 0x52, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0x01, 0x01,
    0x07, 0xB3, 0xAD,
];

// SET_POSITION_TARGET_LOCAL_NED, seq=13: coordinate_frame=1 keeps the
// full 53-byte payload — the no-truncation anchor for setpoints.
const PYMAVLINK_SET_POS_FULL: &[u8] = &[
    0xFD, 0x35, 0x00, 0x00, 0x0D, 0x01, 0x01, 0x54, 0x00, 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00,
    0x80, 0x3F, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x0F, 0x01, 0x01, 0x01, 0x23,
    0x01,
];

#[test]
fn test_serializer_truncation_matches_pymavlink_remaining_messages() {
    struct Case {
        msg: MavMessage,
        seq: u8,
        expected: &'static [u8],
    }
    let cases = [
        Case {
            msg: MavMessage::ManualControl(ManualControl {
                target: 1,
                ..Default::default()
            }),
            seq: 8,
            expected: PYMAVLINK_MANUAL_CONTROL_BASE,
        },
        Case {
            msg: MavMessage::RcChannelsOverride(RcChannelsOverride {
                chan1_raw: 1500,
                chan2_raw: 1501,
                chan3_raw: 1502,
                chan4_raw: 1503,
                chan5_raw: 1504,
                chan6_raw: 1505,
                chan7_raw: 1506,
                chan8_raw: 1507,
                target_system: 1,
                target_component: 1,
                ..Default::default()
            }),
            seq: 10,
            expected: PYMAVLINK_RC_OVERRIDE_TRUNCATED,
        },
        Case {
            msg: MavMessage::SysStatus(SysStatus {
                onboard_control_sensors_present: 1,
                onboard_control_sensors_enabled: 1,
                onboard_control_sensors_health: 1,
                load: 500,
                voltage_battery: 11100,
                current_battery: 1500,
                battery_remaining: 88,
                ..Default::default()
            }),
            seq: 11,
            expected: PYMAVLINK_SYS_STATUS_TRUNCATED,
        },
        Case {
            msg: MavMessage::SetAttitudeTarget(SetAttitudeTarget {
                time_boot_ms: 1000,
                target_system: 1,
                target_component: 1,
                type_mask: 7,
                q: [1.0, 0.0, 0.0, 0.0],
                thrust: 0.5,
                ..Default::default()
            }),
            seq: 12,
            expected: PYMAVLINK_SET_ATT_TRUNCATED,
        },
        Case {
            msg: MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed {
                time_boot_ms: 2000,
                target_system: 1,
                target_component: 1,
                coordinate_frame: 1,
                type_mask: 0x0FF8,
                x: 1.0,
                y: 2.0,
                z: -3.0,
                yaw: 0.25,
                ..Default::default()
            }),
            seq: 13,
            expected: PYMAVLINK_SET_POS_FULL,
        },
    ];

    for case in &cases {
        let mut buf = [0u8; 256];
        let len = serialize_mavlink(&case.msg, case.seq, 1, 1, &mut buf);
        assert_eq!(
            len,
            Some(case.expected.len()),
            "Frame length mismatch for {:?}",
            case.msg
        );
        let Some(len) = len else {
            return;
        };
        assert_eq!(&buf[..len], case.expected, "Frame bytes for {:?}", case.msg);
    }
}

// STATUSTEXT, seq=14: severity=6, text "READY", zero-padded tail and
// extension fields truncate 54 -> 6.
const PYMAVLINK_STATUSTEXT_TRUNCATED: &[u8] = &[
    0xFD, 0x06, 0x00, 0x00, 0x0E, 0x01, 0x01, 0xFD, 0x00, 0x00, 0x06, 0x52, 0x45, 0x41, 0x44, 0x59,
    0xD9, 0x08,
];

#[test]
fn test_statustext_serializer_matches_pymavlink() {
    let mut text = [0u8; 50];
    text[..5].copy_from_slice(b"READY");
    let msg = MavMessage::Statustext(Statustext {
        severity: 6,
        text,
        ..Default::default()
    });
    let mut buf = [0u8; 256];
    let len = serialize_mavlink(&msg, 14, 1, 1, &mut buf);
    assert_eq!(len, Some(PYMAVLINK_STATUSTEXT_TRUNCATED.len()));
    let Some(len) = len else {
        return;
    };
    assert_eq!(&buf[..len], PYMAVLINK_STATUSTEXT_TRUNCATED);

    let Some((parsed, _)) = parse_frame(PYMAVLINK_STATUSTEXT_TRUNCATED) else {
        return;
    };
    assert!(matches!(&parsed, MavMessage::Statustext(_)));
    let MavMessage::Statustext(st) = parsed else {
        return;
    };
    assert_eq!(st.severity, 6);
    assert_eq!(&st.text[..5], b"READY");
    assert!(st.text[5..].iter().all(|&b| b == 0));
    assert_eq!(st.id, 0);
    assert_eq!(st.chunk_seq, 0);
}
