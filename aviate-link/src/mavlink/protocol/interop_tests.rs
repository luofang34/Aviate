//! Cross-implementation interop tests: byte-exact comparison against
//! pymavlink-generated golden frames, and serializer determinism.

use super::*;

fn parse_frame(input: &[u8]) -> Option<(MavMessage, usize)> {
    let parsed = parse_mavlink(input);
    assert!(parsed.is_ok(), "Parse failed: {:?}", parsed.err());
    let Ok((msg, _sig, consumed)) = parsed else {
        return None;
    };
    Some((msg, consumed))
}

// MANUAL_CONTROL, seq=6, every extension field distinct — pins the
// common.xml extension order (buttons2 @11, enabled_extensions @13,
// s @14, t @16, aux1..aux6 @18..29). pymavlink 2.4.41 output; the
// non-zero tail keeps the full 30-byte payload on the wire.
const PYMAVLINK_MANUAL_CONTROL_EXT: &[u8] = &[
    0xFD, 0x1E, 0x00, 0x00, 0x06, 0x01, 0x01, 0x45, 0x00, 0x00, 0x64, 0x00, 0x38, 0xFF, 0x2C, 0x01,
    0x70, 0xFE, 0x05, 0x00, 0x01, 0x07, 0x00, 0x01, 0x0B, 0x00, 0xF4, 0xFF, 0x15, 0x00, 0xEA, 0xFF,
    0x17, 0x00, 0xE8, 0xFF, 0x19, 0x00, 0xE6, 0xFF, 0xC8, 0xA2,
];

fn manual_control_ext_message() -> ManualControl {
    ManualControl {
        x: 100,
        y: -200,
        z: 300,
        r: -400,
        buttons: 5,
        target: 1,
        buttons2: 7,
        enabled_extensions: 1,
        s: 11,
        t: -12,
        aux1: 21,
        aux2: -22,
        aux3: 23,
        aux4: -24,
        aux5: 25,
        aux6: -26,
    }
}

#[test]
fn test_manual_control_extension_offsets_match_pymavlink() {
    let Some((msg, consumed)) = parse_frame(PYMAVLINK_MANUAL_CONTROL_EXT) else {
        return;
    };
    assert_eq!(consumed, PYMAVLINK_MANUAL_CONTROL_EXT.len());
    assert!(matches!(&msg, MavMessage::ManualControl(_)));
    let MavMessage::ManualControl(m) = msg else {
        return;
    };
    let expected = manual_control_ext_message();
    assert_eq!(m.x, expected.x);
    assert_eq!(m.y, expected.y);
    assert_eq!(m.z, expected.z);
    assert_eq!(m.r, expected.r);
    assert_eq!(m.buttons, expected.buttons);
    assert_eq!(m.target, expected.target);
    assert_eq!(m.buttons2, expected.buttons2);
    assert_eq!(m.enabled_extensions, expected.enabled_extensions);
    assert_eq!(m.s, expected.s);
    assert_eq!(m.t, expected.t);
    assert_eq!(m.aux1, expected.aux1);
    assert_eq!(m.aux2, expected.aux2);
    assert_eq!(m.aux3, expected.aux3);
    assert_eq!(m.aux4, expected.aux4);
    assert_eq!(m.aux5, expected.aux5);
    assert_eq!(m.aux6, expected.aux6);
}

#[test]
fn test_manual_control_serializer_matches_pymavlink() {
    let msg = MavMessage::ManualControl(manual_control_ext_message());
    let mut buf = [0u8; 64];
    let len = serialize_mavlink(&msg, 6, 1, 1, &mut buf);
    assert_eq!(len, Some(PYMAVLINK_MANUAL_CONTROL_EXT.len()));
    let Some(len) = len else {
        return;
    };
    assert_eq!(&buf[..len], PYMAVLINK_MANUAL_CONTROL_EXT);
}

#[test]
fn test_serialization_is_independent_of_buffer_contents() {
    // A serializer that leaves any declared payload byte unwritten lets
    // stale caller memory reach the wire. One entry per MavMessage
    // variant the serializer supports.
    let messages = [
        MavMessage::Heartbeat(Heartbeat::default()),
        MavMessage::SysStatus(SysStatus::default()),
        MavMessage::SystemTime(SystemTime::default()),
        MavMessage::AttitudeQuaternion(AttitudeQuaternion::default()),
        MavMessage::LocalPositionNed(LocalPositionNed::default()),
        MavMessage::ManualControl(ManualControl::default()),
        MavMessage::RcChannelsOverride(RcChannelsOverride::default()),
        MavMessage::CommandLong(CommandLong::default()),
        MavMessage::CommandAck(CommandAck::default()),
        MavMessage::SetAttitudeTarget(SetAttitudeTarget::default()),
        MavMessage::SetPositionTargetLocalNed(SetPositionTargetLocalNed::default()),
        MavMessage::Statustext(Statustext::default()),
    ];

    for msg in &messages {
        let mut zeroed = [0u8; 256];
        let mut dirty = [0xFFu8; 256];
        let len_zeroed = serialize_mavlink(msg, 0, 1, 1, &mut zeroed);
        let len_dirty = serialize_mavlink(msg, 0, 1, 1, &mut dirty);
        assert_eq!(len_zeroed, len_dirty, "Length differs for {msg:?}");
        let (Some(a), Some(b)) = (len_zeroed, len_dirty) else {
            return;
        };
        assert_eq!(
            &zeroed[..a],
            &dirty[..b],
            "Serialized bytes depend on prior buffer contents for {msg:?}"
        );
    }
}
