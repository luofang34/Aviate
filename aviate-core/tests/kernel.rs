//! Tests for §17 Initialization and §20 Core Interface
//!
//! Covers:
//! - InitState state machine transitions
//! - Arm/disarm behavior and error cases
//! - AviateKernel step() behavior
//! - Safe output when not armed
//! - Spec types (ChannelId, ChannelHealth, ChannelStatus, etc.)

use aviate_core::{
    InitState, ArmError, AviateKernel,
    ChannelId, ChannelHealth, ChannelStatus,
    CycleTiming, TimingStats, EnvelopeMargin,
    DegradationReason, ConfigTransitionState, TransitionFailure,
    HealthReport,
};
use aviate_core::control::{
    ConfigMode, ControlMode, ControlLaw, Setpoint, CommandSource, Command,
};
use aviate_core::control::mc::McController;
use aviate_core::mixer::{QuadXMixer, ModeConfig};
use aviate_core::sensor::SensorReading;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::types::{Normalized, Meters, MetersPerSecond};
use aviate_core::time::{Timestamp, TimeSource};
use aviate_core::fault::FaultFlags;
use aviate_core::sensor::SensorSet;
use aviate_core::state::EstimateQuality;

fn dummy_timestamp() -> Timestamp {
    Timestamp { ticks: 0, source: TimeSource::Internal }
}

fn make_kernel() -> AviateKernel<McController, QuadXMixer> {
    let mixer = QuadXMixer { timestamp_source: dummy_timestamp };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    AviateKernel::new(McController::default(), mixer, mode_config)
}

fn make_empty_sensors() -> SensorSet {
    SensorSet {
        imus: [SensorReading::default(), SensorReading::default(), SensorReading::default()],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

// =============================================================================
// InitState - allows_active_control()
// =============================================================================

#[test]
fn init_state_only_armed_allows_control() {
    assert!(!InitState::PowerOn.allows_active_control());
    assert!(!InitState::ConfigLoading.allows_active_control());
    assert!(!InitState::SensorInit.allows_active_control());
    assert!(!InitState::EstimatorConverging.allows_active_control());
    assert!(!InitState::PreArm.allows_active_control());
    assert!(!InitState::Ready.allows_active_control());
    assert!(InitState::Armed.allows_active_control(), "Only Armed allows control");
    assert!(!InitState::Disarmed.allows_active_control());
    assert!(!InitState::Fault.allows_active_control());
}

// =============================================================================
// InitState - forced_control_law()
// =============================================================================

#[test]
fn init_state_forces_frozen_when_not_armed() {
    assert_eq!(InitState::PowerOn.forced_control_law(), Some(ControlLaw::Frozen));
    assert_eq!(InitState::Ready.forced_control_law(), Some(ControlLaw::Frozen));
    assert_eq!(InitState::Disarmed.forced_control_law(), Some(ControlLaw::Frozen));
    assert_eq!(InitState::Fault.forced_control_law(), Some(ControlLaw::Frozen));
}

#[test]
fn init_state_no_forced_law_when_armed() {
    assert_eq!(InitState::Armed.forced_control_law(), None);
}

// =============================================================================
// ArmError - Variants
// =============================================================================

#[test]
fn arm_error_not_ready() {
    assert_eq!(ArmError::NotReady, ArmError::NotReady);
    assert_ne!(ArmError::NotReady, ArmError::Faulted);
}

#[test]
fn arm_error_all_variants_distinct() {
    let errors = [
        ArmError::NotReady,
        ArmError::Faulted,
        ArmError::AlreadyArmed,
        ArmError::ConfigInvalid,
    ];

    // All variants should be distinct
    for i in 0..errors.len() {
        for j in (i + 1)..errors.len() {
            assert_ne!(errors[i], errors[j]);
        }
    }
}

// =============================================================================
// Kernel - Init State Machine
// =============================================================================

#[test]
fn kernel_starts_in_power_on() {
    let kernel = make_kernel();
    assert_eq!(kernel.init_state, InitState::PowerOn);
}

#[test]
fn kernel_transitions_through_init_states() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    // Initialize EKF first
    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    // Step through init states
    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
        if kernel.init_state == InitState::Ready {
            break;
        }
    }

    assert_eq!(kernel.init_state, InitState::Ready);
}

#[test]
fn kernel_is_ready_returns_correct_value() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    assert!(!kernel.is_ready());

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    assert!(kernel.is_ready());
}

// =============================================================================
// Kernel - Arm Success
// =============================================================================

#[test]
fn kernel_arm_succeeds_when_ready() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    let result = kernel.arm();
    assert!(result.is_ok());
    assert_eq!(kernel.init_state, InitState::Armed);
}

// =============================================================================
// Kernel - Arm Failures
// =============================================================================

#[test]
fn kernel_arm_fails_when_not_ready() {
    let mut kernel = make_kernel();

    let result = kernel.arm();
    assert_eq!(result, Err(ArmError::NotReady));
}

#[test]
fn kernel_arm_fails_when_already_armed() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    kernel.arm().unwrap();

    let result = kernel.arm();
    assert_eq!(result, Err(ArmError::AlreadyArmed));
}

#[test]
fn kernel_arm_fails_when_faulted() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Inject a fault
    kernel.faults = FaultFlags::IMU0_FAILED;

    let result = kernel.arm();
    assert_eq!(result, Err(ArmError::Faulted));
}

// =============================================================================
// Kernel - Disarm
// =============================================================================

#[test]
fn kernel_disarm_transitions_to_disarmed() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    kernel.arm().unwrap();
    kernel.disarm();

    assert_eq!(kernel.init_state, InitState::Disarmed);
}

// =============================================================================
// Kernel - Step Output
// =============================================================================

#[test]
fn kernel_outputs_safe_when_not_armed() {
    let mut kernel = make_kernel();

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.8),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    let output = kernel.step(&cmd);

    // Should output safe values (0.0) when not armed
    for i in 0..4 {
        assert!((output.outputs[i].0).abs() < 1e-5,
                "Motor {} should be 0 when not armed", i);
    }
}

#[test]
fn kernel_outputs_control_when_armed() {
    let mut kernel = make_kernel();
    let sensors = make_empty_sensors();

    kernel.ekf.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
        Quaternion::IDENTITY,
    );

    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    let output = kernel.step(&cmd);

    // Should output ~0.5 on all motors with zero R/P/Y
    for i in 0..4 {
        assert!((output.outputs[i].0 - 0.5).abs() < 0.1,
                "Motor {} should be ~0.5 when armed with 0.5 thrust", i);
    }
}

// =============================================================================
// ChannelId - Constants
// =============================================================================

#[test]
fn channel_id_primary() {
    assert_eq!(ChannelId::PRIMARY.0, 0);
}

#[test]
fn channel_id_secondary() {
    assert_eq!(ChannelId::SECONDARY.0, 1);
}

#[test]
fn channel_id_tertiary() {
    assert_eq!(ChannelId::TERTIARY.0, 2);
}

#[test]
fn channel_id_max_channels() {
    assert_eq!(ChannelId::MAX_CHANNELS, 3);
}

// =============================================================================
// ChannelHealth - Default
// =============================================================================

#[test]
fn channel_health_default_operative() {
    let health = ChannelHealth::default();
    assert_eq!(health, ChannelHealth::Operative);
}

#[test]
fn channel_health_variants() {
    let _operative = ChannelHealth::Operative;
    let _degraded = ChannelHealth::Degraded;
    let _failed = ChannelHealth::Failed;
    let _testing = ChannelHealth::Testing;
}

// =============================================================================
// ChannelStatus - Default
// =============================================================================

#[test]
fn channel_status_default() {
    let status = ChannelStatus::default();

    assert_eq!(status.mode, ControlMode::Rate);
    assert_eq!(status.config_mode, ConfigMode::Hover);
    assert_eq!(status.law, ControlLaw::Normal);
    assert_eq!(status.health, ChannelHealth::Operative);
    assert!(status.faults.is_empty());
    assert_eq!(status.confidence, EstimateQuality::Good);
}

// =============================================================================
// CycleTiming - Default
// =============================================================================

#[test]
fn cycle_timing_default() {
    let timing = CycleTiming::default();

    assert_eq!(timing.cycle_start_us, 0);
    assert_eq!(timing.cycle_end_us, 0);
    assert_eq!(timing.duration_us, 0);
    assert!(timing.deadline_met);
}

// =============================================================================
// TimingStats - Default
// =============================================================================

#[test]
fn timing_stats_default() {
    let stats = TimingStats::default();

    assert_eq!(stats.last_cycle_us, 0);
    assert_eq!(stats.max_cycle_us, 0);
    assert_eq!(stats.min_cycle_us, 0);
    assert_eq!(stats.deadline_violations, 0);
    assert_eq!(stats.consecutive_violations, 0);
    assert_eq!(stats.total_cycles, 0);
}

// =============================================================================
// EnvelopeMargin - Default
// =============================================================================

#[test]
fn envelope_margin_default() {
    let margin = EnvelopeMargin::default();

    assert_eq!(margin.roll_rad.0, 0.0);
    assert_eq!(margin.pitch_rad.0, 0.0);
    assert_eq!(margin.yaw_rate_rad_s.0, 0.0);
    assert_eq!(margin.altitude_m.0, 0.0);
    assert_eq!(margin.airspeed_mps.0, 0.0);
    assert_eq!(margin.load_factor, 0.0);
}

// =============================================================================
// DegradationReason - Variants
// =============================================================================

#[test]
fn degradation_reason_all_variants() {
    let reasons = [
        DegradationReason::SensorLoss,
        DegradationReason::ActuatorFault,
        DegradationReason::ActuatorNumericError,
        DegradationReason::EstimatorDivergence,
        DegradationReason::EnvelopeExceedance,
        DegradationReason::CommandTimeout,
        DegradationReason::TimingViolation,
        DegradationReason::NumericError,
        DegradationReason::ExplicitRequest,
    ];

    assert_eq!(reasons.len(), 9);
}

// =============================================================================
// ConfigTransitionState - Default and Variants
// =============================================================================

#[test]
fn config_transition_default_stable_hover() {
    let state = ConfigTransitionState::default();
    match state {
        ConfigTransitionState::Stable(mode) => {
            assert_eq!(mode, ConfigMode::Hover);
        }
        _ => panic!("Default should be Stable(Hover)"),
    }
}

#[test]
fn config_transition_switching_state() {
    let state = ConfigTransitionState::Switching {
        from: ConfigMode::Hover,
        to: ConfigMode::Cruise,
        progress: 0.5,
    };

    match state {
        ConfigTransitionState::Switching { from, to, progress } => {
            assert_eq!(from, ConfigMode::Hover);
            assert_eq!(to, ConfigMode::Cruise);
            assert!((progress - 0.5).abs() < 1e-6);
        }
        _ => panic!("Should be Switching"),
    }
}

#[test]
fn config_transition_failed_state() {
    let state = ConfigTransitionState::Failed {
        intended: ConfigMode::Cruise,
        actual: ConfigMode::Hover,
        reason: TransitionFailure::ActuatorStuck,
    };

    match state {
        ConfigTransitionState::Failed { intended, actual, reason } => {
            assert_eq!(intended, ConfigMode::Cruise);
            assert_eq!(actual, ConfigMode::Hover);
            assert_eq!(reason, TransitionFailure::ActuatorStuck);
        }
        _ => panic!("Should be Failed"),
    }
}

// =============================================================================
// TransitionFailure - Variants
// =============================================================================

#[test]
fn transition_failure_variants() {
    let failures = [
        TransitionFailure::ActuatorStuck,
        TransitionFailure::Asymmetry,
        TransitionFailure::Timeout,
        TransitionFailure::UnsafeConditions,
    ];

    assert_eq!(failures.len(), 4);
}
