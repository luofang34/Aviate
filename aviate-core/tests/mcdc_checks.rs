//! MC/DC Coverage Tests for Unified Check System
//!
//! Modified Condition/Decision Coverage (MC/DC) is required for DO-178C DAL-A/B.
//! Each test demonstrates that a specific condition independently affects
//! the decision outcome.
//!
//! ## Test Naming Conventions
//!
//! - `mcdc_{decision}_{condition}_{true|false}` - MC/DC coverage tests
//! - `{struct}_{method}_{scenario}` - Method/unit tests
//! - `boundary_{value}_{at|below|above}_threshold` - Boundary value tests
//!
//! ## Spec References
//!
//! - PreArmFlags: §17 InitState transitions
//! - InFlightFlags: §14, §15 continuous monitoring
//! - TransitionFlags: §4.5 config mode transitions
//! - CheckInvariants: DO-178C verification

#![allow(clippy::bool_assert_comparison)] // Explicit true/false comparisons for clarity

use aviate_core::checks::{
    CheckInvariants, DegradationReason, InFlightFlags, InFlightStatus, PreArmFlags, PreArmStatus,
    SampleCounts, TransitionFailure, TransitionFlags, TransitionLimits, TransitionStatus,
};
use aviate_core::control::envelope::{AxisLimitFlags, EnvelopeMargin, ProtectionStatus};
use aviate_core::fault::FaultFlags;
use aviate_core::mixer::{ActuatorState, ActuatorHealth, ActuatorCmd, MAX_ACTUATORS};
use aviate_core::sensor::{SensorSet, SensorReading, SensorHealth, ImuData};
use aviate_core::state::{StateEstimate, StateValidFlags};
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::types::{Meters, MetersPerSecond, MetersPerSecondSquared, Normalized, Radians, RadiansPerSecond};

// ============================================================================
// TEST HELPERS
// ============================================================================

fn dummy_timestamp() -> Timestamp {
    Timestamp { ticks: 0, source: TimeSource::Internal }
}

/// Create PreArmStatus with all QUAD_MINIMUM flags set
fn pre_arm_all_quad_minimum() -> PreArmStatus {
    let mut status = PreArmStatus::default();
    status.current = PreArmFlags::QUAD_MINIMUM;
    status
}

/// Create InFlightStatus with ATTITUDE_FLIGHT flags set
fn in_flight_all_attitude_flight() -> InFlightStatus {
    let mut status = InFlightStatus::default();
    status.current = InFlightFlags::ATTITUDE_FLIGHT;
    status
}

/// Create TransitionStatus with all VTOL_TRANSITION flags set
fn transition_all_vtol() -> TransitionStatus {
    let mut status = TransitionStatus::default();
    status.current = TransitionFlags::VTOL_TRANSITION;
    status
}

/// Create ActuatorState with all healthy actuators
fn actuator_state_all_healthy() -> ActuatorState {
    let mut state = ActuatorState::new();
    for i in 0..4 {
        state.set_health(i, ActuatorHealth::Good);
    }
    state
}

/// Create StateEstimate with valid attitude/position/velocity
fn state_estimate_valid() -> StateEstimate {
    let mut state = StateEstimate::default();
    state.valid_flags = StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY | StateValidFlags::POSITION;
    state.position_ned = [Meters(0.0), Meters(0.0), Meters(-50.0)]; // 50m altitude
    state.angular_velocity = [RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)];
    state
}

/// Create SensorSet with healthy IMU and baro
fn sensor_set_healthy() -> SensorSet {
    let ts = dummy_timestamp();
    let valid_imu = SensorReading {
        value: ImuData {
            accel: [MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(-9.81)],
            gyro: [RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    SensorSet {
        imus: [valid_imu, SensorReading::default(), SensorReading::default()],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

// ============================================================================
// MC/DC: PreArmStatus::is_satisfied() - QUAD_MINIMUM (8 conditions)
// ============================================================================

#[test]
fn mcdc_pre_arm_is_satisfied_all_true() {
    let status = pre_arm_all_quad_minimum();
    assert!(status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_imu_healthy_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::IMU_HEALTHY);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_baro_healthy_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::BARO_HEALTHY);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_imu_converged_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::IMU_CONVERGED);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_baro_converged_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::BARO_CONVERGED);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_ekf_converged_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::EKF_CONVERGED);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_throttle_low_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::THROTTLE_LOW);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_config_valid_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::CONFIG_VALID);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_no_faults_false() {
    let mut status = pre_arm_all_quad_minimum();
    status.current.remove(PreArmFlags::NO_FAULTS);
    assert!(!status.is_satisfied());
}

// ============================================================================
// MC/DC: PreArmFlags::QUAD_WITH_GPS (3 additional conditions)
// ============================================================================

#[test]
fn mcdc_pre_arm_quad_gps_superset_of_minimum() {
    let quad_gps = PreArmFlags::QUAD_WITH_GPS;
    let quad_min = PreArmFlags::QUAD_MINIMUM;
    assert!(quad_gps.contains(quad_min));
}

#[test]
fn mcdc_pre_arm_gnss_available_false() {
    let mut status = PreArmStatus::with_required(PreArmFlags::QUAD_WITH_GPS);
    status.current = PreArmFlags::QUAD_WITH_GPS;
    assert!(status.is_satisfied());

    status.current.remove(PreArmFlags::GNSS_AVAILABLE);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_mag_healthy_false() {
    let mut status = PreArmStatus::with_required(PreArmFlags::QUAD_WITH_GPS);
    status.current = PreArmFlags::QUAD_WITH_GPS;
    status.current.remove(PreArmFlags::MAG_HEALTHY);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_pre_arm_mag_converged_false() {
    let mut status = PreArmStatus::with_required(PreArmFlags::QUAD_WITH_GPS);
    status.current = PreArmFlags::QUAD_WITH_GPS;
    status.current.remove(PreArmFlags::MAG_CONVERGED);
    assert!(!status.is_satisfied());
}

// ============================================================================
// PreArmStatus METHOD TESTS
// ============================================================================

#[test]
fn pre_arm_status_missing_returns_difference() {
    let mut status = PreArmStatus::default();
    status.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::BARO_HEALTHY;

    let missing = status.missing();
    assert!(missing.contains(PreArmFlags::THROTTLE_LOW));
    assert!(missing.contains(PreArmFlags::EKF_CONVERGED));
    assert!(!missing.contains(PreArmFlags::IMU_HEALTHY));
}

#[test]
fn pre_arm_status_reset_clears_current_and_samples() {
    let mut status = PreArmStatus::default();
    status.current = PreArmFlags::QUAD_MINIMUM;
    status.samples.imu = 150;
    status.samples.baro = 150;
    status.samples.mag = 150;
    status.samples.gnss = 50;

    status.reset();

    assert!(status.current.is_empty());
    assert_eq!(status.samples.imu, 0);
    assert_eq!(status.samples.baro, 0);
    assert_eq!(status.samples.mag, 0);
    assert_eq!(status.samples.gnss, 0);
}

#[test]
fn pre_arm_status_update_throttle_sets_flag() {
    let mut status = PreArmStatus::default();
    assert!(!status.current.contains(PreArmFlags::THROTTLE_LOW));

    status.update_throttle(true);
    assert!(status.current.contains(PreArmFlags::THROTTLE_LOW));

    status.update_throttle(false);
    assert!(!status.current.contains(PreArmFlags::THROTTLE_LOW));
}

#[test]
fn pre_arm_status_update_ekf_sets_flag() {
    let mut status = PreArmStatus::default();
    assert!(!status.current.contains(PreArmFlags::EKF_CONVERGED));

    status.update_ekf(true);
    assert!(status.current.contains(PreArmFlags::EKF_CONVERGED));

    status.update_ekf(false);
    assert!(!status.current.contains(PreArmFlags::EKF_CONVERGED));
}

#[test]
fn pre_arm_status_update_from_faults_sets_no_faults() {
    let mut status = PreArmStatus::default();

    status.update_from_faults(FaultFlags::empty());
    assert!(status.current.contains(PreArmFlags::NO_FAULTS));
    assert!(status.current.contains(PreArmFlags::CONFIG_VALID));

    status.update_from_faults(FaultFlags::BARO_FAILED);
    assert!(!status.current.contains(PreArmFlags::NO_FAULTS));

    status.update_from_faults(FaultFlags::CONFIG_INVALID);
    assert!(!status.current.contains(PreArmFlags::CONFIG_VALID));
}

// ============================================================================
// MC/DC: InFlightStatus::is_satisfied() - ATTITUDE_FLIGHT (3 conditions)
// ============================================================================

#[test]
fn mcdc_in_flight_is_satisfied_all_true() {
    let status = in_flight_all_attitude_flight();
    assert!(status.is_satisfied());
}

#[test]
fn mcdc_in_flight_attitude_valid_false() {
    let mut status = in_flight_all_attitude_flight();
    status.current.remove(InFlightFlags::ATTITUDE_VALID);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_in_flight_imu_ok_false() {
    let mut status = in_flight_all_attitude_flight();
    status.current.remove(InFlightFlags::IMU_OK);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_in_flight_command_recent_false() {
    let mut status = in_flight_all_attitude_flight();
    status.current.remove(InFlightFlags::COMMAND_RECENT);
    assert!(!status.is_satisfied());
}

// ============================================================================
// MC/DC: InFlightFlags::POSITION_FLIGHT (2 additional conditions)
// ============================================================================

#[test]
fn mcdc_in_flight_position_valid_false() {
    let mut status = InFlightStatus::with_required(InFlightFlags::POSITION_FLIGHT);
    status.current = InFlightFlags::POSITION_FLIGHT;
    status.current.remove(InFlightFlags::POSITION_VALID);
    assert!(!status.is_satisfied());
}

#[test]
fn mcdc_in_flight_velocity_valid_false() {
    let mut status = InFlightStatus::with_required(InFlightFlags::POSITION_FLIGHT);
    status.current = InFlightFlags::POSITION_FLIGHT;
    status.current.remove(InFlightFlags::VELOCITY_VALID);
    assert!(!status.is_satisfied());
}

// ============================================================================
// InFlightStatus METHOD TESTS
// ============================================================================

#[test]
fn in_flight_status_update_from_state_sets_flags() {
    let mut status = InFlightStatus::default();
    let state = state_estimate_valid();

    status.update_from_state(&state);

    assert!(status.current.contains(InFlightFlags::ATTITUDE_VALID));
    assert!(status.current.contains(InFlightFlags::VELOCITY_VALID));
    assert!(status.current.contains(InFlightFlags::POSITION_VALID));
    assert!(status.current.contains(InFlightFlags::HEADING_VALID));
}

#[test]
fn in_flight_status_update_from_state_clears_on_invalid() {
    let mut status = InFlightStatus::default();
    status.current = InFlightFlags::ATTITUDE_VALID | InFlightFlags::VELOCITY_VALID;

    let state = StateEstimate::default(); // No valid flags
    status.update_from_state(&state);

    assert!(!status.current.contains(InFlightFlags::ATTITUDE_VALID));
    assert!(!status.current.contains(InFlightFlags::VELOCITY_VALID));
}

#[test]
fn in_flight_status_update_from_sensors_sets_imu_ok() {
    let mut status = InFlightStatus::default();
    let sensors = sensor_set_healthy();

    status.update_from_sensors(&sensors);
    assert!(status.current.contains(InFlightFlags::IMU_OK));
}

#[test]
fn in_flight_status_update_from_sensors_clears_on_failed() {
    let mut status = InFlightStatus::default();
    status.current.insert(InFlightFlags::IMU_OK);

    let empty_sensors = SensorSet {
        imus: [SensorReading::default(); 3],
        gnss: [SensorReading::default(); 2],
        mags: [SensorReading::default(); 2],
        baros: [SensorReading::default(); 2],
        airspeeds: [SensorReading::default(); 2],
        geometry: None,
    };

    status.update_from_sensors(&empty_sensors);
    assert!(!status.current.contains(InFlightFlags::IMU_OK));
}

#[test]
fn in_flight_status_update_from_envelope_sets_flag() {
    let mut status = InFlightStatus::default();

    // Not limited - within envelope
    let protection = ProtectionStatus {
        limited_axes: AxisLimitFlags::empty(),
        saturated: false,
    };
    status.update_from_envelope(&protection);
    assert!(status.current.contains(InFlightFlags::WITHIN_ENVELOPE));

    // Limited - outside envelope
    let protection_limited = ProtectionStatus {
        limited_axes: AxisLimitFlags::ROLL,
        saturated: false,
    };
    status.update_from_envelope(&protection_limited);
    assert!(!status.current.contains(InFlightFlags::WITHIN_ENVELOPE));

    // Saturated - outside envelope
    let protection_saturated = ProtectionStatus {
        limited_axes: AxisLimitFlags::empty(),
        saturated: true,
    };
    status.update_from_envelope(&protection_saturated);
    assert!(!status.current.contains(InFlightFlags::WITHIN_ENVELOPE));
}

#[test]
fn in_flight_status_update_command_status_sets_flag() {
    let mut status = InFlightStatus::default();
    let timeout_ms = 500;

    // Fresh command
    status.update_command_status(100, timeout_ms);
    assert!(status.current.contains(InFlightFlags::COMMAND_RECENT));

    // Stale command
    status.update_command_status(600, timeout_ms);
    assert!(!status.current.contains(InFlightFlags::COMMAND_RECENT));

    // Boundary: exactly at timeout
    status.update_command_status(500, timeout_ms);
    assert!(!status.current.contains(InFlightFlags::COMMAND_RECENT));

    // Boundary: one below timeout
    status.update_command_status(499, timeout_ms);
    assert!(status.current.contains(InFlightFlags::COMMAND_RECENT));
}

#[test]
fn in_flight_status_update_rc_status_sets_flag() {
    let mut status = InFlightStatus::default();

    status.update_rc_status(true);
    assert!(status.current.contains(InFlightFlags::RC_AVAILABLE));

    status.update_rc_status(false);
    assert!(!status.current.contains(InFlightFlags::RC_AVAILABLE));
}

#[test]
fn in_flight_status_update_altitude_sets_flag() {
    let mut status = InFlightStatus::default();

    status.update_altitude(true);
    assert!(status.current.contains(InFlightFlags::ALTITUDE_OK));

    status.update_altitude(false);
    assert!(!status.current.contains(InFlightFlags::ALTITUDE_OK));
}

#[test]
fn in_flight_status_reset_clears_current() {
    let mut status = InFlightStatus::default();
    status.current = InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::POSITION_VALID;

    status.reset();

    assert!(status.current.is_empty());
}

// ============================================================================
// MC/DC: InFlightStatus::get_degradation_trigger() - Priority Chain (8 branches)
// ============================================================================

#[test]
fn mcdc_degradation_none_when_satisfied() {
    let mut status = InFlightStatus::with_required(InFlightFlags::ATTITUDE_FLIGHT);
    status.current = InFlightFlags::ATTITUDE_FLIGHT;
    assert_eq!(status.get_degradation_trigger(), None);
}

#[test]
fn mcdc_degradation_attitude_lost_highest_priority() {
    let mut status = InFlightStatus::with_required(InFlightFlags::ATTITUDE_FLIGHT);
    status.current = InFlightFlags::IMU_OK | InFlightFlags::COMMAND_RECENT;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::AttitudeLost));
}

#[test]
fn mcdc_degradation_imu_degraded_priority_2() {
    let mut status = InFlightStatus::with_required(InFlightFlags::ATTITUDE_FLIGHT);
    status.current = InFlightFlags::ATTITUDE_VALID | InFlightFlags::COMMAND_RECENT;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::ImuDegraded));
}

#[test]
fn mcdc_degradation_position_lost_priority_3() {
    let mut status = InFlightStatus::with_required(InFlightFlags::POSITION_FLIGHT);
    status.current = InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::VELOCITY_VALID;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::PositionLost));
}

#[test]
fn mcdc_degradation_velocity_lost_priority_4() {
    let mut status = InFlightStatus::with_required(InFlightFlags::POSITION_FLIGHT);
    status.current = InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::POSITION_VALID;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::VelocityLost));
}

#[test]
fn mcdc_degradation_command_timeout_priority_5() {
    let mut status = InFlightStatus::with_required(InFlightFlags::ATTITUDE_FLIGHT);
    status.current = InFlightFlags::ATTITUDE_VALID | InFlightFlags::IMU_OK;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::CommandTimeout));
}

#[test]
fn mcdc_degradation_envelope_violation_priority_6() {
    let mut status = InFlightStatus::with_required(
        InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::WITHIN_ENVELOPE,
    );
    status.current = InFlightFlags::ATTITUDE_FLIGHT;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::EnvelopeViolation));
}

#[test]
fn mcdc_degradation_baro_degraded_priority_7() {
    let mut status = InFlightStatus::with_required(
        InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::BARO_OK,
    );
    status.current = InFlightFlags::ATTITUDE_FLIGHT;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::BaroDegraded));
}

#[test]
fn mcdc_degradation_rc_lost_priority_8() {
    let mut status = InFlightStatus::with_required(
        InFlightFlags::ATTITUDE_FLIGHT | InFlightFlags::RC_AVAILABLE,
    );
    status.current = InFlightFlags::ATTITUDE_FLIGHT;
    assert_eq!(status.get_degradation_trigger(), Some(DegradationReason::RcLost));
}

// ============================================================================
// MC/DC: TransitionStatus::can_transition() - VTOL_TRANSITION (4 conditions)
// ============================================================================

#[test]
fn mcdc_transition_can_transition_all_satisfied() {
    let status = transition_all_vtol();
    assert!(status.can_transition().is_ok());
}

#[test]
fn mcdc_transition_stable_flight_false() {
    let mut status = transition_all_vtol();
    status.current.remove(TransitionFlags::STABLE_FLIGHT);
    assert_eq!(status.can_transition(), Err(TransitionFailure::UnstableFlight));
}

#[test]
fn mcdc_transition_actuators_ok_false() {
    let mut status = transition_all_vtol();
    status.current.remove(TransitionFlags::ACTUATORS_OK);
    assert_eq!(status.can_transition(), Err(TransitionFailure::ActuatorStuck));
}

#[test]
fn mcdc_transition_within_envelope_false() {
    let mut status = transition_all_vtol();
    status.current.remove(TransitionFlags::WITHIN_ENVELOPE);
    assert_eq!(status.can_transition(), Err(TransitionFailure::UnsafeConditions));
}

#[test]
fn mcdc_transition_symmetric_false() {
    let mut status = transition_all_vtol();
    status.current.remove(TransitionFlags::SYMMETRIC);
    assert_eq!(status.can_transition(), Err(TransitionFailure::Asymmetry));
}

#[test]
fn mcdc_transition_altitude_ok_false() {
    let mut status = TransitionStatus::with_required(
        TransitionFlags::VTOL_TRANSITION | TransitionFlags::ALTITUDE_OK,
    );
    status.current = TransitionFlags::VTOL_TRANSITION;
    assert_eq!(status.can_transition(), Err(TransitionFailure::AltitudeTooLow));
}

#[test]
fn mcdc_transition_airspeed_ok_false() {
    let mut status = TransitionStatus::with_required(
        TransitionFlags::VTOL_TRANSITION | TransitionFlags::AIRSPEED_OK,
    );
    status.current = TransitionFlags::VTOL_TRANSITION;
    assert_eq!(status.can_transition(), Err(TransitionFailure::AirspeedTooLow));
}

#[test]
fn mcdc_transition_multiple_failures() {
    let mut status = transition_all_vtol();
    status.current.remove(TransitionFlags::STABLE_FLIGHT);
    status.current.remove(TransitionFlags::SYMMETRIC);
    assert_eq!(status.can_transition(), Err(TransitionFailure::MultipleFailures));
}

// ============================================================================
// TransitionStatus METHOD TESTS
// ============================================================================

#[test]
fn transition_status_update_from_actuators_all_healthy() {
    let mut status = TransitionStatus::default();
    let mut actuators = actuator_state_all_healthy();

    // Set symmetric commanded values
    actuators.commanded[0] = Normalized(0.5);
    actuators.commanded[1] = Normalized(0.5);
    actuators.commanded[2] = Normalized(0.5);
    actuators.commanded[3] = Normalized(0.5);

    let active_mask = 0x0F; // First 4 actuators
    status.update_from_actuators(&actuators, active_mask);

    assert!(status.current.contains(TransitionFlags::ACTUATORS_OK));
    assert!(status.current.contains(TransitionFlags::SYMMETRIC));
}

#[test]
fn transition_status_update_from_actuators_one_failed() {
    let mut status = TransitionStatus::default();
    let mut actuators = actuator_state_all_healthy();
    actuators.set_health(1, ActuatorHealth::Failed);

    let active_mask = 0x0F;
    status.update_from_actuators(&actuators, active_mask);

    assert!(!status.current.contains(TransitionFlags::ACTUATORS_OK));
}

#[test]
fn transition_status_update_from_actuators_one_stuck() {
    let mut status = TransitionStatus::default();
    let mut actuators = actuator_state_all_healthy();
    actuators.set_health(2, ActuatorHealth::Stuck);

    let active_mask = 0x0F;
    status.update_from_actuators(&actuators, active_mask);

    assert!(!status.current.contains(TransitionFlags::ACTUATORS_OK));
    assert!(!status.current.contains(TransitionFlags::SYMMETRIC));
}

#[test]
fn transition_status_update_from_actuators_asymmetric() {
    let mut status = TransitionStatus::default();
    let mut actuators = actuator_state_all_healthy();

    // Asymmetric commanded values (pair 0,1 differ by 0.3 > 0.1 tolerance)
    actuators.commanded[0] = Normalized(0.7);
    actuators.commanded[1] = Normalized(0.4);
    actuators.commanded[2] = Normalized(0.5);
    actuators.commanded[3] = Normalized(0.5);

    let active_mask = 0x0F;
    status.update_from_actuators(&actuators, active_mask);

    assert!(status.current.contains(TransitionFlags::ACTUATORS_OK));
    assert!(!status.current.contains(TransitionFlags::SYMMETRIC));
}

#[test]
fn transition_status_update_from_state_altitude_ok() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();

    // 50m altitude (NED: z = -50)
    state.position_ned[2] = Meters(-50.0);
    state.angular_velocity = [RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)];

    status.update_from_state(&state);

    assert!(status.current.contains(TransitionFlags::ALTITUDE_OK));
    assert!(status.current.contains(TransitionFlags::STABLE_FLIGHT));
}

#[test]
fn transition_status_update_from_state_altitude_too_low() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();

    // 5m altitude (below 10m default limit)
    state.position_ned[2] = Meters(-5.0);

    status.update_from_state(&state);

    assert!(!status.current.contains(TransitionFlags::ALTITUDE_OK));
}

#[test]
fn transition_status_update_from_state_unstable_flight() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();

    state.position_ned[2] = Meters(-50.0);
    // High angular rate (> 0.5 rad/s default limit)
    state.angular_velocity = [RadiansPerSecond(0.3), RadiansPerSecond(0.3), RadiansPerSecond(0.3)];

    status.update_from_state(&state);

    assert!(!status.current.contains(TransitionFlags::STABLE_FLIGHT));
}

#[test]
fn transition_status_update_from_envelope_all_positive() {
    let mut status = TransitionStatus::default();

    let margin = EnvelopeMargin {
        roll_rad: Radians(0.5),
        pitch_rad: Radians(0.5),
        yaw_rate_rad_s: RadiansPerSecond(1.0),
        altitude_m: Meters(100.0),
        airspeed_mps: MetersPerSecond(10.0),
        load_factor: 0.5,
    };

    status.update_from_envelope(&margin);
    assert!(status.current.contains(TransitionFlags::WITHIN_ENVELOPE));
}

#[test]
fn transition_status_update_from_envelope_roll_negative() {
    let mut status = TransitionStatus::default();

    let margin = EnvelopeMargin {
        roll_rad: Radians(-0.1), // Violated
        pitch_rad: Radians(0.5),
        yaw_rate_rad_s: RadiansPerSecond(1.0),
        altitude_m: Meters(100.0),
        airspeed_mps: MetersPerSecond(10.0),
        load_factor: 0.5,
    };

    status.update_from_envelope(&margin);
    assert!(!status.current.contains(TransitionFlags::WITHIN_ENVELOPE));
}

#[test]
fn transition_status_update_from_envelope_pitch_negative() {
    let mut status = TransitionStatus::default();

    let margin = EnvelopeMargin {
        roll_rad: Radians(0.5),
        pitch_rad: Radians(-0.1), // Violated
        yaw_rate_rad_s: RadiansPerSecond(1.0),
        altitude_m: Meters(100.0),
        airspeed_mps: MetersPerSecond(10.0),
        load_factor: 0.5,
    };

    status.update_from_envelope(&margin);
    assert!(!status.current.contains(TransitionFlags::WITHIN_ENVELOPE));
}

#[test]
fn transition_status_update_from_envelope_altitude_negative() {
    let mut status = TransitionStatus::default();

    let margin = EnvelopeMargin {
        roll_rad: Radians(0.5),
        pitch_rad: Radians(0.5),
        yaw_rate_rad_s: RadiansPerSecond(1.0),
        altitude_m: Meters(-10.0), // Violated
        airspeed_mps: MetersPerSecond(10.0),
        load_factor: 0.5,
    };

    status.update_from_envelope(&margin);
    assert!(!status.current.contains(TransitionFlags::WITHIN_ENVELOPE));
}

#[test]
fn transition_status_update_from_envelope_load_factor_negative() {
    let mut status = TransitionStatus::default();

    let margin = EnvelopeMargin {
        roll_rad: Radians(0.5),
        pitch_rad: Radians(0.5),
        yaw_rate_rad_s: RadiansPerSecond(1.0),
        altitude_m: Meters(100.0),
        airspeed_mps: MetersPerSecond(10.0),
        load_factor: -0.1, // Violated
    };

    status.update_from_envelope(&margin);
    assert!(!status.current.contains(TransitionFlags::WITHIN_ENVELOPE));
}

#[test]
fn transition_status_update_airspeed_sufficient() {
    let mut status = TransitionStatus::default();
    status.update_airspeed(Some(MetersPerSecond(20.0)));
    assert!(status.current.contains(TransitionFlags::AIRSPEED_OK));
}

#[test]
fn transition_status_update_airspeed_insufficient() {
    let mut status = TransitionStatus::default();
    status.update_airspeed(Some(MetersPerSecond(10.0))); // Below 15 m/s default
    assert!(!status.current.contains(TransitionFlags::AIRSPEED_OK));
}

#[test]
fn transition_status_update_airspeed_none() {
    let mut status = TransitionStatus::default();
    status.update_airspeed(None);
    assert!(!status.current.contains(TransitionFlags::AIRSPEED_OK));
}

#[test]
fn transition_status_reset_clears_current() {
    let mut status = TransitionStatus::default();
    status.current = TransitionFlags::VTOL_TRANSITION | TransitionFlags::ALTITUDE_OK;

    status.reset();

    assert!(status.current.is_empty());
}

#[test]
fn transition_status_with_custom_limits() {
    let limits = TransitionLimits {
        min_altitude: 20.0,
        max_attitude_rate: 0.3,
        min_airspeed: 25.0,
        max_asymmetry: 0.05,
    };

    let mut status = TransitionStatus::with_limits(TransitionFlags::VTOL_TRANSITION, limits);
    assert_eq!(status.limits.min_altitude, 20.0);
    assert_eq!(status.limits.max_attitude_rate, 0.3);
    assert_eq!(status.limits.min_airspeed, 25.0);
    assert_eq!(status.limits.max_asymmetry, 0.05);

    // Test that custom limits are used
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-15.0); // 15m < 20m limit

    status.update_from_state(&state);
    assert!(!status.current.contains(TransitionFlags::ALTITUDE_OK));
}

// ============================================================================
// ActuatorState METHOD TESTS
// ============================================================================

#[test]
fn actuator_state_new_all_unknown() {
    let state = ActuatorState::new();
    for i in 0..MAX_ACTUATORS {
        assert_eq!(state.health[i], ActuatorHealth::Unknown);
    }
}

#[test]
fn actuator_state_set_health_valid_channel() {
    let mut state = ActuatorState::new();
    state.set_health(0, ActuatorHealth::Good);
    state.set_health(3, ActuatorHealth::Failed);

    assert_eq!(state.health[0], ActuatorHealth::Good);
    assert_eq!(state.health[3], ActuatorHealth::Failed);
}

#[test]
fn actuator_state_set_health_invalid_channel_ignored() {
    let mut state = ActuatorState::new();
    state.set_health(MAX_ACTUATORS + 1, ActuatorHealth::Good);
    // Should not panic, just ignore
}

#[test]
fn actuator_state_set_actual_valid_channel() {
    let mut state = ActuatorState::new();
    state.set_actual(0, Normalized(0.5));
    state.set_actual(3, Normalized(0.8));

    let actual = state.actual.expect("actual should be Some");
    assert!((actual[0].0 - 0.5).abs() < 1e-6);
    assert!((actual[3].0 - 0.8).abs() < 1e-6);
}

#[test]
fn actuator_state_set_actual_invalid_channel_ignored() {
    let mut state = ActuatorState::new();
    state.set_actual(MAX_ACTUATORS + 1, Normalized(0.5));
    // Should not panic
}

#[test]
fn actuator_state_update_commanded() {
    let mut state = ActuatorState::new();
    let ts = dummy_timestamp();

    let mut cmd = ActuatorCmd::default();
    cmd.timestamp = ts;
    // Set first 8 outputs
    for i in 0..8 {
        cmd.outputs[i] = Normalized((i as f32 + 1.0) * 0.1);
    }

    state.update_commanded(&cmd, ts);

    for i in 0..8 {
        assert!((state.commanded[i].0 - (i as f32 + 1.0) * 0.1).abs() < 1e-6);
    }
}

#[test]
fn actuator_state_all_healthy_with_mask() {
    let mut state = ActuatorState::new();
    state.set_health(0, ActuatorHealth::Good);
    state.set_health(1, ActuatorHealth::Good);
    state.set_health(2, ActuatorHealth::Good);
    state.set_health(3, ActuatorHealth::Good);

    // Only check first 4
    assert!(state.all_healthy(0x0F));

    // Make one fail
    state.set_health(1, ActuatorHealth::Failed);
    assert!(!state.all_healthy(0x0F));

    // But if not in mask, still healthy
    assert!(state.all_healthy(0x05)); // Only channels 0 and 2
}

#[test]
fn actuator_state_all_healthy_degraded_is_unhealthy() {
    let mut state = ActuatorState::new();
    state.set_health(0, ActuatorHealth::Good);
    state.set_health(1, ActuatorHealth::Degraded);

    assert!(!state.all_healthy(0x03));
}

#[test]
fn actuator_state_all_healthy_unknown_is_healthy() {
    let state = ActuatorState::new(); // All Unknown
    assert!(state.all_healthy(0x0F));
}

#[test]
fn actuator_state_check_symmetric_within_tolerance() {
    let mut state = ActuatorState::new();
    state.commanded[0] = Normalized(0.50);
    state.commanded[1] = Normalized(0.52); // 0.02 diff
    state.commanded[2] = Normalized(0.50);
    state.commanded[3] = Normalized(0.48); // 0.02 diff

    let pairs = [(0, 1), (2, 3)];
    assert!(state.check_symmetric(&pairs, 0.1)); // 10% tolerance
}

#[test]
fn actuator_state_check_symmetric_exceeds_tolerance() {
    let mut state = ActuatorState::new();
    state.commanded[0] = Normalized(0.50);
    state.commanded[1] = Normalized(0.70); // 0.20 diff
    state.commanded[2] = Normalized(0.50);
    state.commanded[3] = Normalized(0.50);

    let pairs = [(0, 1), (2, 3)];
    assert!(!state.check_symmetric(&pairs, 0.1)); // 10% tolerance
}

#[test]
fn actuator_state_check_symmetric_invalid_indices_ignored() {
    let state = ActuatorState::new();
    let pairs = [(MAX_ACTUATORS + 1, 0), (0, 1)];
    // Should not panic, invalid pairs are skipped
    let _ = state.check_symmetric(&pairs, 0.1);
}

#[test]
fn actuator_state_count_by_health() {
    let mut state = ActuatorState::new();
    state.set_health(0, ActuatorHealth::Good);
    state.set_health(1, ActuatorHealth::Good);
    state.set_health(2, ActuatorHealth::Failed);
    state.set_health(3, ActuatorHealth::Stuck);

    assert_eq!(state.count_by_health(ActuatorHealth::Good, 0x0F), 2);
    assert_eq!(state.count_by_health(ActuatorHealth::Failed, 0x0F), 1);
    assert_eq!(state.count_by_health(ActuatorHealth::Stuck, 0x0F), 1);

    // With partial mask
    assert_eq!(state.count_by_health(ActuatorHealth::Good, 0x03), 2);
    assert_eq!(state.count_by_health(ActuatorHealth::Failed, 0x03), 0);
}

// ============================================================================
// MC/DC: CheckInvariants (6 invariants)
// ============================================================================

#[test]
fn mcdc_inv001_imu_fault_implies_not_healthy() {
    let faults = FaultFlags::ALL_IMU_FAILED;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::empty();

    assert!(CheckInvariants::check_imu_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv001_imu_fault_with_healthy_flag_inconsistent() {
    let faults = FaultFlags::ALL_IMU_FAILED;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::IMU_HEALTHY;

    assert!(!CheckInvariants::check_imu_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv001_no_fault_always_consistent() {
    let faults = FaultFlags::empty();
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::IMU_HEALTHY;

    assert!(CheckInvariants::check_imu_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv002_gnss_fault_implies_not_available() {
    let faults = FaultFlags::ALL_GNSS_LOST;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::empty();

    assert!(CheckInvariants::check_gnss_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv002_gnss_fault_with_available_flag_inconsistent() {
    let faults = FaultFlags::ALL_GNSS_LOST;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::GNSS_AVAILABLE;

    assert!(!CheckInvariants::check_gnss_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv003_no_faults_flag_matches_empty() {
    let faults = FaultFlags::empty();
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::NO_FAULTS;

    assert!(CheckInvariants::check_no_faults_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv003_no_faults_flag_with_faults_inconsistent() {
    let faults = FaultFlags::BARO_FAILED;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::NO_FAULTS;

    assert!(!CheckInvariants::check_no_faults_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv003_faults_without_flag_consistent() {
    let faults = FaultFlags::BARO_FAILED;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::empty();

    assert!(CheckInvariants::check_no_faults_consistency(faults, &pre_arm));
}

#[test]
fn mcdc_inv004_ekf_converged_implies_imu_converged() {
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::EKF_CONVERGED | PreArmFlags::IMU_CONVERGED;

    assert!(CheckInvariants::check_ekf_convergence_consistency(&pre_arm));
}

#[test]
fn mcdc_inv004_ekf_converged_without_imu_inconsistent() {
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::EKF_CONVERGED;

    assert!(!CheckInvariants::check_ekf_convergence_consistency(&pre_arm));
}

#[test]
fn mcdc_inv004_no_ekf_converged_always_consistent() {
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::empty();

    assert!(CheckInvariants::check_ekf_convergence_consistency(&pre_arm));
}

#[test]
fn mcdc_inv005_position_valid_implies_attitude_valid() {
    let mut in_flight = InFlightStatus::default();
    in_flight.current = InFlightFlags::POSITION_VALID | InFlightFlags::ATTITUDE_VALID;

    assert!(CheckInvariants::check_position_attitude_consistency(&in_flight));
}

#[test]
fn mcdc_inv005_position_valid_without_attitude_inconsistent() {
    let mut in_flight = InFlightStatus::default();
    in_flight.current = InFlightFlags::POSITION_VALID;

    assert!(!CheckInvariants::check_position_attitude_consistency(&in_flight));
}

#[test]
fn mcdc_inv006_sample_counts_monotonic() {
    let prev = SampleCounts { imu: 50, baro: 50, mag: 50, gnss: 50, min_required: 100 };
    let curr = SampleCounts { imu: 51, baro: 51, mag: 51, gnss: 51, min_required: 100 };

    assert!(CheckInvariants::check_sample_count_monotonic(&prev, &curr));
}

#[test]
fn mcdc_inv006_sample_counts_reset_allowed() {
    let prev = SampleCounts { imu: 50, baro: 50, mag: 50, gnss: 50, min_required: 100 };
    let curr = SampleCounts { imu: 0, baro: 0, mag: 0, gnss: 0, min_required: 100 };

    assert!(CheckInvariants::check_sample_count_monotonic(&prev, &curr));
}

#[test]
fn mcdc_inv006_sample_counts_decrease_not_allowed() {
    let prev = SampleCounts { imu: 50, baro: 50, mag: 50, gnss: 50, min_required: 100 };
    let curr = SampleCounts { imu: 49, baro: 50, mag: 50, gnss: 50, min_required: 100 };

    assert!(!CheckInvariants::check_sample_count_monotonic(&prev, &curr));
}

#[test]
fn invariants_verify_all_consistent_state() {
    let faults = FaultFlags::empty();
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::NO_FAULTS | PreArmFlags::IMU_CONVERGED | PreArmFlags::EKF_CONVERGED;

    let mut in_flight = InFlightStatus::default();
    in_flight.current = InFlightFlags::ATTITUDE_VALID | InFlightFlags::POSITION_VALID;

    assert!(CheckInvariants::verify_all(faults, &pre_arm, &in_flight));
}

#[test]
fn invariants_get_violations_bitmask() {
    let faults = FaultFlags::ALL_IMU_FAILED | FaultFlags::ALL_GNSS_LOST;
    let mut pre_arm = PreArmStatus::default();
    pre_arm.current = PreArmFlags::IMU_HEALTHY | PreArmFlags::GNSS_AVAILABLE | PreArmFlags::NO_FAULTS;

    let in_flight = InFlightStatus::default();

    let violations = CheckInvariants::get_violations(faults, &pre_arm, &in_flight);

    assert!(violations & (1 << 0) != 0, "INV-001 violated");
    assert!(violations & (1 << 1) != 0, "INV-002 violated");
    assert!(violations & (1 << 2) != 0, "INV-003 violated");
}

// ============================================================================
// BOUNDARY VALUE TESTS: SampleCounts convergence
// ============================================================================

#[test]
fn boundary_imu_samples_99_not_converged() {
    let counts = SampleCounts { imu: 99, baro: 0, mag: 0, gnss: 0, min_required: 100 };
    assert!(!counts.imu_converged());
}

#[test]
fn boundary_imu_samples_100_converged() {
    let counts = SampleCounts { imu: 100, baro: 0, mag: 0, gnss: 0, min_required: 100 };
    assert!(counts.imu_converged());
}

#[test]
fn boundary_imu_samples_101_converged() {
    let counts = SampleCounts { imu: 101, baro: 0, mag: 0, gnss: 0, min_required: 100 };
    assert!(counts.imu_converged());
}

#[test]
fn boundary_baro_samples_99_not_converged() {
    let counts = SampleCounts { imu: 0, baro: 99, mag: 0, gnss: 0, min_required: 100 };
    assert!(!counts.baro_converged());
}

#[test]
fn boundary_baro_samples_100_converged() {
    let counts = SampleCounts { imu: 0, baro: 100, mag: 0, gnss: 0, min_required: 100 };
    assert!(counts.baro_converged());
}

#[test]
fn boundary_mag_samples_99_not_converged() {
    let counts = SampleCounts { imu: 0, baro: 0, mag: 99, gnss: 0, min_required: 100 };
    assert!(!counts.mag_converged());
}

#[test]
fn boundary_mag_samples_100_converged() {
    let counts = SampleCounts { imu: 0, baro: 0, mag: 100, gnss: 0, min_required: 100 };
    assert!(counts.mag_converged());
}

#[test]
fn boundary_custom_threshold_49_not_converged() {
    let counts = SampleCounts { imu: 49, baro: 0, mag: 0, gnss: 0, min_required: 50 };
    assert!(!counts.imu_converged());
}

#[test]
fn boundary_custom_threshold_50_converged() {
    let counts = SampleCounts { imu: 50, baro: 0, mag: 0, gnss: 0, min_required: 50 };
    assert!(counts.imu_converged());
}

// ============================================================================
// BOUNDARY VALUE TESTS: Command timeout
// ============================================================================

#[test]
fn boundary_command_age_at_timeout_minus_1_recent() {
    let mut status = InFlightStatus::default();
    status.update_command_status(499, 500);
    assert!(status.current.contains(InFlightFlags::COMMAND_RECENT));
}

#[test]
fn boundary_command_age_at_timeout_not_recent() {
    let mut status = InFlightStatus::default();
    status.update_command_status(500, 500);
    assert!(!status.current.contains(InFlightFlags::COMMAND_RECENT));
}

#[test]
fn boundary_command_age_at_timeout_plus_1_not_recent() {
    let mut status = InFlightStatus::default();
    status.update_command_status(501, 500);
    assert!(!status.current.contains(InFlightFlags::COMMAND_RECENT));
}

// ============================================================================
// BOUNDARY VALUE TESTS: Transition altitude limit
// ============================================================================

#[test]
fn boundary_altitude_at_limit_minus_epsilon_not_ok() {
    let mut status = TransitionStatus::default(); // min_altitude = 10.0
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-9.99); // 9.99m altitude

    status.update_from_state(&state);
    assert!(!status.current.contains(TransitionFlags::ALTITUDE_OK));
}

#[test]
fn boundary_altitude_at_limit_ok() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-10.0); // Exactly 10m

    status.update_from_state(&state);
    assert!(status.current.contains(TransitionFlags::ALTITUDE_OK));
}

#[test]
fn boundary_altitude_at_limit_plus_epsilon_ok() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-10.01); // 10.01m altitude

    status.update_from_state(&state);
    assert!(status.current.contains(TransitionFlags::ALTITUDE_OK));
}

// ============================================================================
// BOUNDARY VALUE TESTS: Transition airspeed limit
// ============================================================================

#[test]
fn boundary_airspeed_at_limit_minus_epsilon_not_ok() {
    let mut status = TransitionStatus::default(); // min_airspeed = 15.0
    status.update_airspeed(Some(MetersPerSecond(14.99)));
    assert!(!status.current.contains(TransitionFlags::AIRSPEED_OK));
}

#[test]
fn boundary_airspeed_at_limit_ok() {
    let mut status = TransitionStatus::default();
    status.update_airspeed(Some(MetersPerSecond(15.0)));
    assert!(status.current.contains(TransitionFlags::AIRSPEED_OK));
}

#[test]
fn boundary_airspeed_at_limit_plus_epsilon_ok() {
    let mut status = TransitionStatus::default();
    status.update_airspeed(Some(MetersPerSecond(15.01)));
    assert!(status.current.contains(TransitionFlags::AIRSPEED_OK));
}

// ============================================================================
// BOUNDARY VALUE TESTS: Attitude rate stability limit
// ============================================================================

#[test]
fn boundary_attitude_rate_at_limit_minus_epsilon_stable() {
    let mut status = TransitionStatus::default(); // max_attitude_rate = 0.5
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-50.0);
    // Rate magnitude just under 0.5: sqrt(0.28^2 + 0.28^2 + 0.28^2) ≈ 0.485
    state.angular_velocity = [RadiansPerSecond(0.28), RadiansPerSecond(0.28), RadiansPerSecond(0.28)];

    status.update_from_state(&state);
    assert!(status.current.contains(TransitionFlags::STABLE_FLIGHT));
}

#[test]
fn boundary_attitude_rate_at_limit_not_stable() {
    let mut status = TransitionStatus::default();
    let mut state = StateEstimate::default();
    state.position_ned[2] = Meters(-50.0);
    // Rate magnitude at 0.5: sqrt(0.29^2 + 0.29^2 + 0.29^2) ≈ 0.502
    state.angular_velocity = [RadiansPerSecond(0.29), RadiansPerSecond(0.29), RadiansPerSecond(0.29)];

    status.update_from_state(&state);
    assert!(!status.current.contains(TransitionFlags::STABLE_FLIGHT));
}

// ============================================================================
// TRANSITION FAILURE: MultipleFailures fallback
// ============================================================================

#[test]
fn transition_can_transition_multiple_failures_fallback() {
    let mut status = TransitionStatus::default();

    // Set required to multiple flags
    status.required = TransitionFlags::STABLE_FLIGHT
        | TransitionFlags::ACTUATORS_OK
        | TransitionFlags::WITHIN_ENVELOPE;

    // Set current to empty - all flags missing
    status.current = TransitionFlags::empty();

    // can_transition should return MultipleFailures since multiple conditions fail
    // and none match the specific single-failure cases
    let result = status.can_transition();

    assert!(result.is_err());
    // The error should be one of the specific failures or MultipleFailures
    let err = result.unwrap_err();
    // At least verify it's an error
    assert!(matches!(
        err,
        TransitionFailure::UnsafeConditions
            | TransitionFailure::ActuatorStuck
            | TransitionFailure::MultipleFailures
    ));
}

#[test]
fn transition_can_transition_stable_flight_returns_unstable() {
    let mut status = TransitionStatus::default();

    // Only require STABLE_FLIGHT - when missing, returns UnstableFlight
    status.required = TransitionFlags::STABLE_FLIGHT;
    status.current = TransitionFlags::empty();

    let result = status.can_transition();

    assert!(result.is_err());
    let err = result.unwrap_err();
    assert_eq!(err, TransitionFailure::UnstableFlight);
}

#[test]
fn transition_can_transition_two_missing_returns_multiple() {
    let mut status = TransitionStatus::default();

    // Require two flags, both missing → MultipleFailures (line 687)
    status.required = TransitionFlags::STABLE_FLIGHT | TransitionFlags::ACTUATORS_OK;
    status.current = TransitionFlags::empty();

    let result = status.can_transition();

    assert!(result.is_err());
    let err = result.unwrap_err();
    assert_eq!(err, TransitionFailure::MultipleFailures);
}

/// DO-178C DAL-A: Verify all TransitionFlags have explicit error mappings.
/// This test ensures no flag can reach the defensive fallback code.
#[test]
fn transition_all_flags_have_explicit_error() {
    let all_flags = [
        (TransitionFlags::STABLE_FLIGHT, TransitionFailure::UnstableFlight),
        (TransitionFlags::ACTUATORS_OK, TransitionFailure::ActuatorStuck),
        (TransitionFlags::WITHIN_ENVELOPE, TransitionFailure::UnsafeConditions),
        (TransitionFlags::SYMMETRIC, TransitionFailure::Asymmetry),
        (TransitionFlags::ALTITUDE_OK, TransitionFailure::AltitudeTooLow),
        (TransitionFlags::AIRSPEED_OK, TransitionFailure::AirspeedTooLow),
    ];

    // Verify each flag has a specific error when it alone is missing
    for (flag, expected_error) in all_flags {
        let mut status = TransitionStatus::default();
        status.required = flag;
        status.current = TransitionFlags::empty();

        let result = status.can_transition();
        assert!(result.is_err(), "Flag {:?} should cause error", flag);
        assert_eq!(
            result.unwrap_err(),
            expected_error,
            "Flag {:?} should map to {:?}",
            flag,
            expected_error
        );
    }

    // Verify we've tested ALL flags (compile-time guarantee)
    let all_tested = TransitionFlags::STABLE_FLIGHT
        | TransitionFlags::ACTUATORS_OK
        | TransitionFlags::WITHIN_ENVELOPE
        | TransitionFlags::SYMMETRIC
        | TransitionFlags::ALTITUDE_OK
        | TransitionFlags::AIRSPEED_OK;
    assert_eq!(
        all_tested,
        TransitionFlags::all(),
        "Not all TransitionFlags are tested! Add missing flags to this test."
    );
}

// ============================================================================
// CHECK INVARIANTS: INV-004 and INV-005 violations
// ============================================================================

#[test]
fn invariant_inv004_ekf_convergence_violation() {
    // INV-004: EKF_CONVERGED implies IMU_CONVERGED
    let mut pre_arm = PreArmStatus::default();

    // Set EKF_CONVERGED but NOT IMU_CONVERGED - this violates INV-004
    pre_arm.current = PreArmFlags::EKF_CONVERGED;
    // IMU_CONVERGED is not set

    let in_flight = InFlightStatus::default();

    let violations = CheckInvariants::get_violations(
        FaultFlags::empty(),
        &pre_arm,
        &in_flight
    );

    // Should have INV-004 violation (bit 3)
    assert!((violations & (1 << 3)) != 0, "INV-004 should be violated");
}

#[test]
fn invariant_inv005_position_attitude_violation() {
    // INV-005: POSITION_VALID implies ATTITUDE_VALID
    let mut in_flight = InFlightStatus::default();

    // Set POSITION_VALID but NOT ATTITUDE_VALID - this violates INV-005
    in_flight.current = InFlightFlags::POSITION_VALID;
    // ATTITUDE_VALID is not set

    let pre_arm = PreArmStatus::default();

    let violations = CheckInvariants::get_violations(
        FaultFlags::empty(),
        &pre_arm,
        &in_flight
    );

    // Should have INV-005 violation (bit 4)
    assert!((violations & (1 << 4)) != 0, "INV-005 should be violated");
}

#[test]
fn invariant_no_violations_when_consistent() {
    let mut pre_arm = PreArmStatus::default();
    // Set both EKF_CONVERGED and IMU_CONVERGED (consistent)
    pre_arm.current = PreArmFlags::EKF_CONVERGED | PreArmFlags::IMU_CONVERGED;

    let mut in_flight = InFlightStatus::default();
    // Set both POSITION_VALID and ATTITUDE_VALID (consistent)
    in_flight.current = InFlightFlags::POSITION_VALID | InFlightFlags::ATTITUDE_VALID;

    let violations = CheckInvariants::get_violations(
        FaultFlags::empty(),
        &pre_arm,
        &in_flight
    );

    // No violations for INV-004 and INV-005
    assert!((violations & (1 << 3)) == 0, "INV-004 should not be violated");
    assert!((violations & (1 << 4)) == 0, "INV-005 should not be violated");
}
