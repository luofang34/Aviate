//! Tests for §15 Fault Model
//!
//! Covers:
//! - FaultFlags bitfield operations
//! - FaultCategory enumeration
//! - FaultAction enumeration
//! - FaultHandlingTable lookups
//! - ControlLawV1 degradation ordering

use aviate_core::control::ControlLawV1;
use aviate_core::fault::{
    FaultAction, FaultCategory, FaultFlags, FaultHandlingTable, FaultResponse,
};

// =============================================================================
// FaultFlags - Empty/Single
// =============================================================================

#[test]
fn fault_flags_empty() {
    let flags = FaultFlags::empty();
    assert!(flags.is_empty());
    assert!(!flags.contains(FaultFlags::IMU0_FAILED));
    assert!(!flags.contains(FaultFlags::ALL_IMU_FAILED));
}

#[test]
fn fault_flags_single_imu() {
    let flags = FaultFlags::IMU0_FAILED;
    assert!(!flags.is_empty());
    assert!(flags.contains(FaultFlags::IMU0_FAILED));
    assert!(!flags.contains(FaultFlags::IMU1_FAILED));
    assert!(!flags.contains(FaultFlags::GNSS0_LOST));
}

#[test]
fn fault_flags_single_gnss() {
    let flags = FaultFlags::GNSS0_LOST;
    assert!(flags.contains(FaultFlags::GNSS0_LOST));
    assert!(!flags.contains(FaultFlags::GNSS1_LOST));
}

// =============================================================================
// FaultFlags - Multiple Flags
// =============================================================================

#[test]
fn fault_flags_combine_with_or() {
    let flags = FaultFlags::IMU0_FAILED | FaultFlags::GNSS0_LOST;
    assert!(flags.contains(FaultFlags::IMU0_FAILED));
    assert!(flags.contains(FaultFlags::GNSS0_LOST));
    assert!(!flags.contains(FaultFlags::BARO_FAILED));
}

#[test]
fn fault_flags_all_imu_failed() {
    let flags = FaultFlags::ALL_IMU_FAILED;
    assert!(flags.contains(FaultFlags::ALL_IMU_FAILED));
    // ALL_IMU_FAILED is a separate flag, not combination of IMU0/1/2
}

#[test]
fn fault_flags_combine_multiple() {
    let flags = FaultFlags::IMU0_FAILED
        | FaultFlags::IMU1_FAILED
        | FaultFlags::GNSS0_LOST
        | FaultFlags::BARO_FAILED;

    assert!(flags.contains(FaultFlags::IMU0_FAILED));
    assert!(flags.contains(FaultFlags::IMU1_FAILED));
    assert!(flags.contains(FaultFlags::GNSS0_LOST));
    assert!(flags.contains(FaultFlags::BARO_FAILED));
    assert!(!flags.contains(FaultFlags::IMU2_FAILED));
}

// =============================================================================
// FaultFlags - Intersection/Removal
// =============================================================================

#[test]
fn fault_flags_intersection() {
    let a = FaultFlags::IMU0_FAILED | FaultFlags::GNSS0_LOST;
    let b = FaultFlags::GNSS0_LOST | FaultFlags::BARO_FAILED;

    let intersection = a & b;
    assert!(intersection.contains(FaultFlags::GNSS0_LOST));
    assert!(!intersection.contains(FaultFlags::IMU0_FAILED));
    assert!(!intersection.contains(FaultFlags::BARO_FAILED));
}

#[test]
fn fault_flags_removal() {
    let mut flags = FaultFlags::IMU0_FAILED | FaultFlags::GNSS0_LOST;
    flags.remove(FaultFlags::IMU0_FAILED);

    assert!(!flags.contains(FaultFlags::IMU0_FAILED));
    assert!(flags.contains(FaultFlags::GNSS0_LOST));
}

#[test]
fn fault_flags_insert() {
    let mut flags = FaultFlags::empty();
    flags.insert(FaultFlags::NUMERIC_ERROR);

    assert!(flags.contains(FaultFlags::NUMERIC_ERROR));
}

// =============================================================================
// FaultFlags - Actuator/Estimator Categories
// =============================================================================

#[test]
fn fault_flags_actuator_faults() {
    let flags = FaultFlags::ACTUATOR_FAULT | FaultFlags::ACTUATOR_NUMERIC;
    assert!(flags.contains(FaultFlags::ACTUATOR_FAULT));
    assert!(flags.contains(FaultFlags::ACTUATOR_NUMERIC));
    assert!(!flags.contains(FaultFlags::ACTUATOR_FALLBACK));
}

#[test]
fn fault_flags_estimator_faults() {
    let flags = FaultFlags::ESTIMATOR_DIVERGED | FaultFlags::ATTITUDE_UNCERTAIN;
    assert!(flags.contains(FaultFlags::ESTIMATOR_DIVERGED));
    assert!(flags.contains(FaultFlags::ATTITUDE_UNCERTAIN));
}

#[test]
fn fault_flags_timing_faults() {
    let flags = FaultFlags::TIMING_VIOLATION | FaultFlags::TIMING_PERSISTENT;
    assert!(flags.contains(FaultFlags::TIMING_VIOLATION));
    assert!(flags.contains(FaultFlags::TIMING_PERSISTENT));
}

// =============================================================================
// FaultCategory - All Variants
// =============================================================================

#[test]
fn fault_category_sensor_variants() {
    let categories = [
        FaultCategory::ImuFailed,
        FaultCategory::ImuAllFailed,
        FaultCategory::GnssLost,
        FaultCategory::GnssAllLost,
        FaultCategory::BaroFailed,
        FaultCategory::MagFailed,
        FaultCategory::AirspeedFailed,
    ];
    assert_eq!(categories.len(), 7);
}

#[test]
fn fault_category_actuator_variants() {
    let categories = [
        FaultCategory::ActuatorFailed,
        FaultCategory::ActuatorSaturated,
        FaultCategory::ActuatorDisagreement,
        FaultCategory::ActuatorNumericError,
        FaultCategory::ActuatorFallbackPersistent,
    ];
    assert_eq!(categories.len(), 5);
}

#[test]
fn fault_category_estimation_variants() {
    let categories = [
        FaultCategory::EstimatorDiverged,
        FaultCategory::AttitudeUncertain,
        FaultCategory::PositionUncertain,
        FaultCategory::NumericError,
    ];
    assert_eq!(categories.len(), 4);
}

// =============================================================================
// FaultAction - All Variants
// =============================================================================

#[test]
fn fault_action_variants() {
    let actions = [
        FaultAction::Monitor,
        FaultAction::Isolate,
        FaultAction::Degrade,
        FaultAction::Emergency,
    ];
    assert_eq!(actions.len(), 4);
}

// =============================================================================
// ControlLawV1 - Degradation Ordering
// =============================================================================

#[test]
fn control_law_degradation_order() {
    // §14: Control law degradation is monotonic
    assert!(ControlLawV1::Primary < ControlLawV1::Alternate);
    assert!(ControlLawV1::Alternate < ControlLawV1::Direct);
    assert!(ControlLawV1::Direct < ControlLawV1::Backup);
}

#[test]
fn control_law_max_degradation() {
    let laws = [
        ControlLawV1::Primary,
        ControlLawV1::Alternate,
        ControlLawV1::Alternate,
        ControlLawV1::Direct,
        ControlLawV1::Backup,
    ];

    assert_eq!(laws.iter().max(), Some(&ControlLawV1::Backup));
}

#[test]
fn control_law_equality() {
    assert_eq!(ControlLawV1::Primary, ControlLawV1::Primary);
    assert_ne!(ControlLawV1::Primary, ControlLawV1::Alternate);
}

// =============================================================================
// FaultHandlingTable - Default Configuration
// =============================================================================

#[test]
fn fault_table_has_entries() {
    let table = FaultHandlingTable::DEFAULT;
    assert!(!table.entries.is_empty());
}

#[test]
fn fault_table_imu_all_failed_emergency() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::ImuAllFailed);

    assert!(entry.is_some(), "ImuAllFailed should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Emergency);
    assert_eq!(e.degrade_to, Some(ControlLawV1::Backup));
    assert_eq!(e.max_response_time_ms, 0);
}

#[test]
fn fault_table_gnss_all_lost_degrade() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::GnssAllLost);

    assert!(entry.is_some(), "GnssAllLost should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Degrade);
    assert_eq!(e.degrade_to, Some(ControlLawV1::Alternate));
}

#[test]
fn fault_table_numeric_error_emergency() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::NumericError);

    assert!(entry.is_some(), "NumericError should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Emergency);
    assert_eq!(e.degrade_to, Some(ControlLawV1::Backup));
}

#[test]
fn fault_table_imu_single_isolate() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::ImuFailed);

    assert!(entry.is_some(), "ImuFailed should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Isolate);
    assert!(e.degrade_to.is_none()); // Just isolate, don't degrade
}

#[test]
fn fault_table_command_timeout_degrade() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::CommandTimeout);

    assert!(entry.is_some(), "CommandTimeout should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Degrade);
}

#[test]
fn fault_table_actuator_numeric_monitor() {
    let table = FaultHandlingTable::DEFAULT;

    let entry = table
        .entries
        .iter()
        .find(|e| e.fault == FaultCategory::ActuatorNumericError);

    assert!(entry.is_some(), "ActuatorNumericError should have an entry");
    let Some(e) = entry else { return };
    assert_eq!(e.action, FaultAction::Monitor);
}

// =============================================================================
// FaultResponse - Structure
// =============================================================================

#[test]
fn fault_response_fields() {
    let response = FaultResponse {
        fault: FaultCategory::EstimatorDiverged,
        action: FaultAction::Degrade,
        degrade_to: Some(ControlLawV1::Alternate),
        max_response_time_ms: 50,
    };

    assert_eq!(response.fault, FaultCategory::EstimatorDiverged);
    assert_eq!(response.action, FaultAction::Degrade);
    assert_eq!(response.degrade_to, Some(ControlLawV1::Alternate));
    assert_eq!(response.max_response_time_ms, 50);
}

#[test]
fn fault_response_no_degradation() {
    let response = FaultResponse {
        fault: FaultCategory::ImuFailed,
        action: FaultAction::Isolate,
        degrade_to: None,
        max_response_time_ms: 10,
    };

    assert!(response.degrade_to.is_none());
}
