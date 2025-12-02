//! Enum validation tests for SEU-resilient center-code decoding (Spec §15.3)
//!
//! These tests verify the 16-bit center-code implementation for all control-plane enums.
//! Center-codes provide Hamming distance ≥8 between variants for SEU protection.

use aviate_core::control::{CommandSource, ConfigMode, ControlLawV1, ControlMode, SafetyLevelV1};
use aviate_core::{ChannelHealthV1, EnumValidationError, InitState};

// ============================================================================
// ControlLawV1 Tests
// ============================================================================

#[test]
fn control_law_center_codes_are_valid() {
    // Test all valid center codes decode correctly
    assert_eq!(ControlLawV1::try_from(0x0000u16), Ok(ControlLawV1::Primary));
    assert_eq!(
        ControlLawV1::try_from(0x5555u16),
        Ok(ControlLawV1::Alternate)
    );
    assert_eq!(ControlLawV1::try_from(0xAAAAu16), Ok(ControlLawV1::Direct));
    assert_eq!(ControlLawV1::try_from(0xFFFFu16), Ok(ControlLawV1::Backup));
}

#[test]
fn control_law_invalid_codes_rejected() {
    // Non-center codes should be rejected
    assert_eq!(ControlLawV1::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(ControlLawV1::try_from(0x1234u16), Err(EnumValidationError));
    assert_eq!(ControlLawV1::try_from(0x8000u16), Err(EnumValidationError));
}

#[test]
fn control_law_decode_center_returns_nearest() {
    // decode_center should return nearest variant with distance
    let (law, dist, is_center) = ControlLawV1::decode_center(0x0000);
    assert_eq!(law, ControlLawV1::Primary);
    assert_eq!(dist, 0);
    assert!(is_center);

    // 1-bit flip from Primary (0x0000)
    let (law, dist, is_center) = ControlLawV1::decode_center(0x0001);
    assert_eq!(law, ControlLawV1::Primary);
    assert_eq!(dist, 1);
    assert!(!is_center);

    // 1-bit flip from Alternate (0x5555)
    let (law, dist, is_center) = ControlLawV1::decode_center(0x5554);
    assert_eq!(law, ControlLawV1::Alternate);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn control_law_to_code_roundtrip() {
    assert_eq!(ControlLawV1::Primary.to_code(), 0x0000);
    assert_eq!(ControlLawV1::Alternate.to_code(), 0x5555);
    assert_eq!(ControlLawV1::Direct.to_code(), 0xAAAA);
    assert_eq!(ControlLawV1::Backup.to_code(), 0xFFFF);

    // Roundtrip
    for law in [
        ControlLawV1::Primary,
        ControlLawV1::Alternate,
        ControlLawV1::Direct,
        ControlLawV1::Backup,
    ] {
        assert_eq!(ControlLawV1::try_from(law.to_code()), Ok(law));
    }
}

#[test]
fn control_law_try_from_u8() {
    assert_eq!(ControlLawV1::try_from(0u8), Ok(ControlLawV1::Primary));
    assert_eq!(ControlLawV1::try_from(1u8), Ok(ControlLawV1::Alternate));
    assert_eq!(ControlLawV1::try_from(2u8), Ok(ControlLawV1::Direct));
    assert_eq!(ControlLawV1::try_from(3u8), Ok(ControlLawV1::Backup));
    assert_eq!(ControlLawV1::try_from(4u8), Err(EnumValidationError));
    assert_eq!(ControlLawV1::try_from(255u8), Err(EnumValidationError));
}

#[test]
fn control_law_severity_ordering() {
    assert!(ControlLawV1::Primary.severity() < ControlLawV1::Alternate.severity());
    assert!(ControlLawV1::Alternate.severity() < ControlLawV1::Direct.severity());
    assert!(ControlLawV1::Direct.severity() < ControlLawV1::Backup.severity());
}

// ============================================================================
// SafetyLevelV1 Tests
// ============================================================================

#[test]
fn safety_level_center_codes_are_valid() {
    assert_eq!(
        SafetyLevelV1::try_from(0x0000u16),
        Ok(SafetyLevelV1::FlightNormal)
    );
    assert_eq!(
        SafetyLevelV1::try_from(0x5555u16),
        Ok(SafetyLevelV1::FlightMarginal)
    );
    assert_eq!(
        SafetyLevelV1::try_from(0xAAAAu16),
        Ok(SafetyLevelV1::FlightUrgent)
    );
    assert_eq!(
        SafetyLevelV1::try_from(0xFFFFu16),
        Ok(SafetyLevelV1::FlightEmergency)
    );
}

#[test]
fn safety_level_invalid_codes_rejected() {
    assert_eq!(SafetyLevelV1::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(SafetyLevelV1::try_from(0x1234u16), Err(EnumValidationError));
}

#[test]
fn safety_level_decode_center_returns_nearest() {
    let (level, dist, is_center) = SafetyLevelV1::decode_center(0x0000);
    assert_eq!(level, SafetyLevelV1::FlightNormal);
    assert_eq!(dist, 0);
    assert!(is_center);

    let (level, dist, is_center) = SafetyLevelV1::decode_center(0x0002);
    assert_eq!(level, SafetyLevelV1::FlightNormal);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn safety_level_to_code_roundtrip() {
    for level in [
        SafetyLevelV1::FlightNormal,
        SafetyLevelV1::FlightMarginal,
        SafetyLevelV1::FlightUrgent,
        SafetyLevelV1::FlightEmergency,
    ] {
        assert_eq!(SafetyLevelV1::try_from(level.to_code()), Ok(level));
    }
}

#[test]
fn safety_level_try_from_u8() {
    assert_eq!(
        SafetyLevelV1::try_from(0u8),
        Ok(SafetyLevelV1::FlightNormal)
    );
    assert_eq!(
        SafetyLevelV1::try_from(1u8),
        Ok(SafetyLevelV1::FlightMarginal)
    );
    assert_eq!(
        SafetyLevelV1::try_from(2u8),
        Ok(SafetyLevelV1::FlightUrgent)
    );
    assert_eq!(
        SafetyLevelV1::try_from(3u8),
        Ok(SafetyLevelV1::FlightEmergency)
    );
    assert_eq!(SafetyLevelV1::try_from(4u8), Err(EnumValidationError));
}

// ============================================================================
// ControlMode Tests (6 variants)
// ============================================================================

#[test]
fn control_mode_center_codes_are_valid() {
    assert_eq!(ControlMode::try_from(0x0000u16), Ok(ControlMode::Rate));
    assert_eq!(ControlMode::try_from(0x2222u16), Ok(ControlMode::Attitude));
    assert_eq!(
        ControlMode::try_from(0x4444u16),
        Ok(ControlMode::AltitudeHold)
    );
    assert_eq!(
        ControlMode::try_from(0x6666u16),
        Ok(ControlMode::PositionHold)
    );
    assert_eq!(
        ControlMode::try_from(0x8888u16),
        Ok(ControlMode::VelocityControl)
    );
    assert_eq!(
        ControlMode::try_from(0xAAAAu16),
        Ok(ControlMode::DeviationTracking)
    );
}

#[test]
fn control_mode_invalid_codes_rejected() {
    assert_eq!(ControlMode::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(ControlMode::try_from(0x1111u16), Err(EnumValidationError));
    assert_eq!(ControlMode::try_from(0xFFFFu16), Err(EnumValidationError));
}

#[test]
fn control_mode_decode_center_returns_nearest() {
    let (mode, dist, is_center) = ControlMode::decode_center(0x0000);
    assert_eq!(mode, ControlMode::Rate);
    assert_eq!(dist, 0);
    assert!(is_center);

    // Near Attitude (0x2222)
    let (mode, dist, is_center) = ControlMode::decode_center(0x2223);
    assert_eq!(mode, ControlMode::Attitude);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn control_mode_to_code_roundtrip() {
    for mode in [
        ControlMode::Rate,
        ControlMode::Attitude,
        ControlMode::AltitudeHold,
        ControlMode::PositionHold,
        ControlMode::VelocityControl,
        ControlMode::DeviationTracking,
    ] {
        assert_eq!(ControlMode::try_from(mode.to_code()), Ok(mode));
    }
}

#[test]
fn control_mode_try_from_u8() {
    assert_eq!(ControlMode::try_from(0u8), Ok(ControlMode::Rate));
    assert_eq!(ControlMode::try_from(1u8), Ok(ControlMode::Attitude));
    assert_eq!(ControlMode::try_from(2u8), Ok(ControlMode::AltitudeHold));
    assert_eq!(ControlMode::try_from(3u8), Ok(ControlMode::PositionHold));
    assert_eq!(ControlMode::try_from(4u8), Ok(ControlMode::VelocityControl));
    assert_eq!(
        ControlMode::try_from(5u8),
        Ok(ControlMode::DeviationTracking)
    );
    assert_eq!(ControlMode::try_from(6u8), Err(EnumValidationError));
}

// ============================================================================
// CommandSource Tests
// ============================================================================

#[test]
fn command_source_center_codes_are_valid() {
    assert_eq!(CommandSource::try_from(0x0000u16), Ok(CommandSource::Pilot));
    assert_eq!(
        CommandSource::try_from(0x5555u16),
        Ok(CommandSource::Autopilot)
    );
    assert_eq!(CommandSource::try_from(0xAAAAu16), Ok(CommandSource::Gcs));
    assert_eq!(
        CommandSource::try_from(0xFFFFu16),
        Ok(CommandSource::Failsafe)
    );
}

#[test]
fn command_source_invalid_codes_rejected() {
    assert_eq!(CommandSource::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(CommandSource::try_from(0x1234u16), Err(EnumValidationError));
}

#[test]
fn command_source_to_code_roundtrip() {
    for src in [
        CommandSource::Pilot,
        CommandSource::Autopilot,
        CommandSource::Gcs,
        CommandSource::Failsafe,
    ] {
        assert_eq!(CommandSource::try_from(src.to_code()), Ok(src));
    }
}

#[test]
fn command_source_try_from_u8() {
    assert_eq!(CommandSource::try_from(0u8), Ok(CommandSource::Pilot));
    assert_eq!(CommandSource::try_from(1u8), Ok(CommandSource::Autopilot));
    assert_eq!(CommandSource::try_from(2u8), Ok(CommandSource::Gcs));
    assert_eq!(CommandSource::try_from(3u8), Ok(CommandSource::Failsafe));
    assert_eq!(CommandSource::try_from(4u8), Err(EnumValidationError));
}

// ============================================================================
// ConfigMode Tests
// ============================================================================

#[test]
fn config_mode_center_codes_are_valid() {
    assert_eq!(ConfigMode::try_from(0x0000u16), Ok(ConfigMode::Hover));
    assert_eq!(ConfigMode::try_from(0x5555u16), Ok(ConfigMode::Cruise));
    assert_eq!(ConfigMode::try_from(0xAAAAu16), Ok(ConfigMode::Transition));
    assert_eq!(ConfigMode::try_from(0xFFFFu16), Ok(ConfigMode::Degraded));
}

#[test]
fn config_mode_invalid_codes_rejected() {
    assert_eq!(ConfigMode::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(ConfigMode::try_from(0x7777u16), Err(EnumValidationError));
}

#[test]
fn config_mode_to_code_roundtrip() {
    for mode in [
        ConfigMode::Hover,
        ConfigMode::Cruise,
        ConfigMode::Transition,
        ConfigMode::Degraded,
    ] {
        assert_eq!(ConfigMode::try_from(mode.to_code()), Ok(mode));
    }
}

#[test]
fn config_mode_try_from_u8() {
    assert_eq!(ConfigMode::try_from(0u8), Ok(ConfigMode::Hover));
    assert_eq!(ConfigMode::try_from(1u8), Ok(ConfigMode::Cruise));
    assert_eq!(ConfigMode::try_from(2u8), Ok(ConfigMode::Transition));
    assert_eq!(ConfigMode::try_from(3u8), Ok(ConfigMode::Degraded));
    assert_eq!(ConfigMode::try_from(4u8), Err(EnumValidationError));
}

// ============================================================================
// InitState Tests (9 variants)
// ============================================================================

#[test]
fn init_state_center_codes_are_valid() {
    assert_eq!(InitState::try_from(0x0000u16), Ok(InitState::PowerOn));
    assert_eq!(InitState::try_from(0x1C71u16), Ok(InitState::ConfigLoading));
    assert_eq!(InitState::try_from(0x38E2u16), Ok(InitState::SensorInit));
    assert_eq!(
        InitState::try_from(0x5553u16),
        Ok(InitState::EstimatorConverging)
    );
    assert_eq!(InitState::try_from(0x71C4u16), Ok(InitState::PreArm));
    assert_eq!(InitState::try_from(0x8E35u16), Ok(InitState::Ready));
    assert_eq!(InitState::try_from(0xAAA6u16), Ok(InitState::Armed));
    assert_eq!(InitState::try_from(0xC717u16), Ok(InitState::Disarmed));
    assert_eq!(InitState::try_from(0xE388u16), Ok(InitState::Fault));
}

#[test]
fn init_state_invalid_codes_rejected() {
    assert_eq!(InitState::try_from(0x0001u16), Err(EnumValidationError));
    assert_eq!(InitState::try_from(0x1234u16), Err(EnumValidationError));
    assert_eq!(InitState::try_from(0xFFFFu16), Err(EnumValidationError));
}

#[test]
fn init_state_decode_center_returns_nearest() {
    let (state, dist, is_center) = InitState::decode_center(0x0000);
    assert_eq!(state, InitState::PowerOn);
    assert_eq!(dist, 0);
    assert!(is_center);

    // 1-bit flip from PowerOn
    let (state, dist, is_center) = InitState::decode_center(0x0001);
    assert_eq!(state, InitState::PowerOn);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn init_state_to_code_roundtrip() {
    for state in [
        InitState::PowerOn,
        InitState::ConfigLoading,
        InitState::SensorInit,
        InitState::EstimatorConverging,
        InitState::PreArm,
        InitState::Ready,
        InitState::Armed,
        InitState::Disarmed,
        InitState::Fault,
    ] {
        assert_eq!(InitState::try_from(state.to_code()), Ok(state));
    }
}

#[test]
fn init_state_try_from_u8() {
    assert_eq!(InitState::try_from(0u8), Ok(InitState::PowerOn));
    assert_eq!(InitState::try_from(1u8), Ok(InitState::ConfigLoading));
    assert_eq!(InitState::try_from(2u8), Ok(InitState::SensorInit));
    assert_eq!(InitState::try_from(3u8), Ok(InitState::EstimatorConverging));
    assert_eq!(InitState::try_from(4u8), Ok(InitState::PreArm));
    assert_eq!(InitState::try_from(5u8), Ok(InitState::Ready));
    assert_eq!(InitState::try_from(6u8), Ok(InitState::Armed));
    assert_eq!(InitState::try_from(7u8), Ok(InitState::Disarmed));
    assert_eq!(InitState::try_from(8u8), Ok(InitState::Fault));
    assert_eq!(InitState::try_from(9u8), Err(EnumValidationError));
}

// ============================================================================
// ChannelHealthV1 Tests
// ============================================================================

#[test]
fn channel_health_center_codes_are_valid() {
    assert_eq!(
        ChannelHealthV1::try_from(0x0000u16),
        Ok(ChannelHealthV1::Operative)
    );
    assert_eq!(
        ChannelHealthV1::try_from(0x5555u16),
        Ok(ChannelHealthV1::Degraded)
    );
    assert_eq!(
        ChannelHealthV1::try_from(0xAAAAu16),
        Ok(ChannelHealthV1::Failed)
    );
    assert_eq!(
        ChannelHealthV1::try_from(0xFFFFu16),
        Ok(ChannelHealthV1::Offline)
    );
}

#[test]
fn channel_health_invalid_codes_rejected() {
    assert_eq!(
        ChannelHealthV1::try_from(0x0001u16),
        Err(EnumValidationError)
    );
    assert_eq!(
        ChannelHealthV1::try_from(0x1234u16),
        Err(EnumValidationError)
    );
}

#[test]
fn channel_health_decode_center_returns_nearest() {
    let (health, dist, is_center) = ChannelHealthV1::decode_center(0x0000);
    assert_eq!(health, ChannelHealthV1::Operative);
    assert_eq!(dist, 0);
    assert!(is_center);

    let (health, dist, is_center) = ChannelHealthV1::decode_center(0x0001);
    assert_eq!(health, ChannelHealthV1::Operative);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn channel_health_to_code_roundtrip() {
    for health in [
        ChannelHealthV1::Operative,
        ChannelHealthV1::Degraded,
        ChannelHealthV1::Failed,
        ChannelHealthV1::Offline,
    ] {
        assert_eq!(ChannelHealthV1::try_from(health.to_code()), Ok(health));
    }
}

#[test]
fn channel_health_try_from_u8() {
    assert_eq!(
        ChannelHealthV1::try_from(0u8),
        Ok(ChannelHealthV1::Operative)
    );
    assert_eq!(
        ChannelHealthV1::try_from(1u8),
        Ok(ChannelHealthV1::Degraded)
    );
    assert_eq!(ChannelHealthV1::try_from(2u8), Ok(ChannelHealthV1::Failed));
    assert_eq!(ChannelHealthV1::try_from(3u8), Ok(ChannelHealthV1::Offline));
    assert_eq!(ChannelHealthV1::try_from(4u8), Err(EnumValidationError));
}

// ============================================================================
// Hamming Distance Verification
// ============================================================================

#[test]
fn control_law_hamming_distance_minimum_8() {
    // 4-variant enums using 0x0000, 0x5555, 0xAAAA, 0xFFFF have Hamming distance 8
    let codes = [0x0000u16, 0x5555u16, 0xAAAAu16, 0xFFFFu16];
    for i in 0..codes.len() {
        for j in (i + 1)..codes.len() {
            let dist = (codes[i] ^ codes[j]).count_ones();
            assert!(
                dist >= 8,
                "Hamming distance between {:04X} and {:04X} is {}, expected >= 8",
                codes[i],
                codes[j],
                dist
            );
        }
    }
}

#[test]
fn control_mode_hamming_distance_reasonable() {
    // 6-variant enum with codes spaced evenly across 16-bit space
    let codes = [
        0x0000u16, 0x2222u16, 0x4444u16, 0x6666u16, 0x8888u16, 0xAAAAu16,
    ];
    for i in 0..codes.len() {
        for j in (i + 1)..codes.len() {
            let dist = (codes[i] ^ codes[j]).count_ones();
            // Adjacent codes differ by at least 4 bits
            assert!(
                dist >= 4,
                "Hamming distance between {:04X} and {:04X} is {}, expected >= 4",
                codes[i],
                codes[j],
                dist
            );
        }
    }
}

// ============================================================================
// SEU Simulation Tests
// ============================================================================

#[test]
fn single_bit_flip_detected() {
    // Simulating a single-event upset (1-bit flip) should be detected
    let original_code = ControlLawV1::Primary.to_code(); // 0x0000
    let corrupted = original_code ^ 0x0001; // Flip bit 0

    // TryFrom should reject
    assert_eq!(ControlLawV1::try_from(corrupted), Err(EnumValidationError));

    // But decode_center should identify nearest variant
    let (law, dist, is_center) = ControlLawV1::decode_center(corrupted);
    assert_eq!(law, ControlLawV1::Primary);
    assert_eq!(dist, 1);
    assert!(!is_center);
}

#[test]
fn multiple_bit_flip_detected() {
    // 3-bit flip should still be detected and rejected
    let original_code = ControlLawV1::Alternate.to_code(); // 0x5555
    let corrupted = original_code ^ 0x0007; // Flip bits 0, 1, 2

    assert_eq!(ControlLawV1::try_from(corrupted), Err(EnumValidationError));

    let (law, dist, _is_center) = ControlLawV1::decode_center(corrupted);
    assert_eq!(law, ControlLawV1::Alternate); // Still nearest
    assert_eq!(dist, 3);
}
