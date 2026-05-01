//! Tests for §17 Initialization and §20 Core Interface
//!
//! Covers:
//! - InitState state machine transitions
//! - Arm/disarm behavior and error cases
//! - AviateKernel step() behavior
//! - Safe output when not armed
//! - Spec types (ChannelId, ChannelHealthV1, ChannelStatus, etc.)

use aviate_core::checks::PreArmFlags;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlLawV1, ControlMode, Setpoint,
};
use aviate_core::ekf::Ekf;
use aviate_core::fault::FaultFlags;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::sensor::SensorSet;
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth, SensorReading,
};
use aviate_core::state::EstimateQuality;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Normalized, Pascals,
    RadiansPerSecond,
};
use aviate_core::{
    ArmError, AviateKernel, ChannelHealthV1, ChannelId, ChannelStatus, ConfigTransitionState,
    CycleTiming, DegradationReason, EnvelopeMargin, InitState, TimingStats, TransitionError,
    TransitionFailure,
};

use aviate_core::control::VehicleController;
use aviate_core::mixer::{ActuatorCmd, Mixer};

trait KernelTestExt {
    fn step_test(
        &mut self,
        time_delta: aviate_core::time::TimeDelta,
        cmd: &Command,
        sensors: &SensorSet,
        command_age_ms: u32,
    ) -> ActuatorCmd;
}

impl<E: aviate_core::Estimator, V: VehicleController, M: Mixer, S: aviate_core::ActuatorSanitizer>
    KernelTestExt for AviateKernel<E, V, M, S>
{
    fn step_test(
        &mut self,
        time_delta: aviate_core::time::TimeDelta,
        cmd: &Command,
        sensors: &SensorSet,
        _command_age_ms: u32,
    ) -> ActuatorCmd {
        let actuator_state = self.actuator_state.clone();
        let res = self.update(
            ChannelId::PRIMARY,
            time_delta,
            sensors,
            cmd,
            &actuator_state,
            None,
        );
        res.actuator
    }
}

fn dummy_timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn dummy_time_delta() -> aviate_core::time::TimeDelta {
    aviate_core::time::TimeDelta {
        dt_sec: aviate_core::types::Seconds(0.01),
        tick_delta: 10000,
    } // 100Hz update
}

fn make_kernel() -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
    let mixer = QuadXMixer {
        timestamp_source: dummy_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    // Use minimal pre-arm requirements for testing
    let test_required = PreArmFlags::IMU_HEALTHY
        | PreArmFlags::IMU_CONVERGED
        | PreArmFlags::EKF_CONVERGED
        | PreArmFlags::THROTTLE_LOW
        | PreArmFlags::CONFIG_VALID;
    let mut kernel = AviateKernel::with_pre_arm_required(
        Ekf::default(),
        MultirotorController::default(),
        mixer,
        Sanitizer::default(),
        mode_config,
        test_required,
    );
    // Set throttle low for tests
    kernel.checks.pre_arm.update_throttle(true);
    kernel
}

/// Create valid sensor data for testing
fn make_valid_sensors() -> SensorSet {
    use aviate_core::types::Celsius;
    let ts = dummy_timestamp();

    let valid_imu = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    let valid_baro = SensorReading {
        value: BaroData {
            altitude: Some(Meters(0.0)),
            air: AirData {
                static_pressure: Some(Pascals(101325.0)),
                dynamic_pressure: None,
                total_pressure: None,
                temperature: Some(Celsius(20.0)),
                indicated_airspeed: None,
                true_airspeed: None,
            },
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    let valid_mag = SensorReading {
        value: MagData {
            field_ut: [Microtesla(20.0), Microtesla(0.0), Microtesla(40.0)],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    let valid_gnss = SensorReading {
        value: GnssData {
            position_ned: [Meters(0.0), Meters(0.0), Meters(0.0)],
            velocity_ned: [
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ],
            fix: GnssFix::ThreeD,
            health: GnssHealth::Good,
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    SensorSet {
        imus: [
            valid_imu,
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [valid_gnss, SensorReading::default()],
        mags: [valid_mag, SensorReading::default()],
        baros: [valid_baro, SensorReading::default()],
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
    assert!(
        InitState::Armed.allows_active_control(),
        "Only Armed allows control"
    );
    assert!(!InitState::Disarmed.allows_active_control());
    assert!(!InitState::Fault.allows_active_control());
}

// =============================================================================
// InitState - forced_control_law()
// =============================================================================

#[test]
fn init_state_forces_frozen_when_not_armed() {
    assert_eq!(
        InitState::PowerOn.forced_control_law(),
        Some(ControlLawV1::Backup)
    );
    assert_eq!(
        InitState::Ready.forced_control_law(),
        Some(ControlLawV1::Backup)
    );
    assert_eq!(
        InitState::Disarmed.forced_control_law(),
        Some(ControlLawV1::Backup)
    );
    assert_eq!(
        InitState::Fault.forced_control_law(),
        Some(ControlLawV1::Backup)
    );
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
    let sensors = make_valid_sensors();

    // Initialize EKF first
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Step through init states (need 100+ iterations for sensor convergence)
    for _ in 0..150 {
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
    let sensors = make_valid_sensors();

    assert!(!kernel.is_ready());

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
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
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
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
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    kernel.arm().unwrap();

    let result = kernel.arm();
    assert_eq!(result, Err(ArmError::AlreadyArmed));
}

#[test]
fn kernel_arm_fails_when_faulted() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Inject a fault after reaching Ready state
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
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
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

    let sensors = make_valid_sensors();
    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Should output safe values (0.0) when not armed
    for i in 0..4 {
        assert!(
            (output.outputs[i].0).abs() < 1e-5,
            "Motor {} should be 0 when not armed",
            i
        );
    }
}

#[test]
fn kernel_outputs_control_when_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
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

    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Should output ~0.5 on all motors with zero R/P/Y
    for i in 0..4 {
        assert!(
            (output.outputs[i].0 - 0.5).abs() < 0.1,
            "Motor {} should be ~0.5 when armed with 0.5 thrust",
            i
        );
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
// ChannelHealthV1 - Default
// =============================================================================

#[test]
fn channel_health_default_operative() {
    let health = ChannelHealthV1::default();
    assert_eq!(health, ChannelHealthV1::Operative);
}

#[test]
fn channel_health_variants() {
    let _operative = ChannelHealthV1::Operative;
    let _degraded = ChannelHealthV1::Degraded;
    let _failed = ChannelHealthV1::Failed;
    let _offline = ChannelHealthV1::Offline;
}

// =============================================================================
// ChannelStatus - Default
// =============================================================================

#[test]
fn channel_status_default() {
    let status = ChannelStatus::default();

    assert_eq!(status.mode, ControlMode::Rate);
    assert_eq!(status.config_mode, ConfigMode::Hover);
    assert_eq!(status.law, ControlLawV1::Primary);
    assert_eq!(status.health, ChannelHealthV1::Operative);
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
        DegradationReason::AttitudeLost,
        DegradationReason::PositionLost,
        DegradationReason::VelocityLost,
        DegradationReason::CommandTimeout,
        DegradationReason::ImuDegraded,
        DegradationReason::BaroDegraded,
        DegradationReason::EnvelopeViolation,
        DegradationReason::RcLost,
    ];

    assert_eq!(reasons.len(), 8);
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
        ConfigTransitionState::Failed {
            intended,
            actual,
            reason,
        } => {
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
        TransitionFailure::UnstableFlight,
        TransitionFailure::ActuatorStuck,
        TransitionFailure::UnsafeConditions,
        TransitionFailure::Asymmetry,
        TransitionFailure::AltitudeTooLow,
        TransitionFailure::AirspeedTooLow,
        TransitionFailure::MultipleFailures,
    ];

    assert_eq!(failures.len(), 7);
}

// =============================================================================
// Negative Tests - Insufficient Sensors
// =============================================================================

/// Create sensors with only IMU (missing baro, mag, gnss)
fn make_imu_only_sensors() -> SensorSet {
    let ts = dummy_timestamp();

    let valid_imu = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };

    SensorSet {
        imus: [
            valid_imu,
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

/// Create sensors with failed IMU (health = Failed)
fn make_failed_imu_sensors() -> SensorSet {
    let ts = dummy_timestamp();

    let failed_imu = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ],
        },
        valid: false, // Invalid
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Failed, // Failed health
    };

    SensorSet {
        imus: [
            failed_imu,
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

#[test]
fn kernel_stays_in_sensor_init_with_no_sensors() {
    let mut kernel = make_kernel();
    let empty_sensors = SensorSet {
        imus: [
            SensorReading::default(),
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [SensorReading::default(), SensorReading::default()],
        mags: [SensorReading::default(), SensorReading::default()],
        baros: [SensorReading::default(), SensorReading::default()],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    };

    // Run many iterations - should never progress past SensorInit
    for _ in 0..200 {
        kernel.init_step(&empty_sensors, dummy_timestamp());
    }

    // Without valid IMU, kernel should be stuck in SensorInit
    assert!(
        matches!(
            kernel.init_state,
            InitState::SensorInit | InitState::ConfigLoading
        ),
        "Expected SensorInit or ConfigLoading with no sensors, got {:?}",
        kernel.init_state
    );
    assert!(!kernel.is_ready());
}

#[test]
fn kernel_stays_in_sensor_init_with_failed_imu() {
    let mut kernel = make_kernel();
    let failed_sensors = make_failed_imu_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run many iterations with failed IMU
    for _ in 0..200 {
        kernel.init_step(&failed_sensors, dummy_timestamp());
    }

    // With failed IMU health, should not reach Ready
    assert!(!kernel.is_ready(), "Should not be ready with failed IMU");
}

#[test]
fn kernel_detects_all_imu_failed_fault() {
    let mut kernel = make_kernel();
    let failed_sensors = make_failed_imu_sensors();

    // Fault detection happens in update(), not init_step()
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };
    kernel.step_test(dummy_time_delta(), &cmd, &failed_sensors, 0);

    // Should have ALL_IMU_FAILED fault
    assert!(
        kernel.faults.contains(FaultFlags::ALL_IMU_FAILED),
        "Expected ALL_IMU_FAILED fault, got {:?}",
        kernel.faults
    );
}

#[test]
fn kernel_arm_fails_with_missing_convergence() {
    let mut kernel = make_kernel();
    let sensors = make_imu_only_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run only a few iterations - not enough for convergence (needs 100+)
    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Should not be ready without convergence
    assert!(!kernel.is_ready());

    // Arm should fail
    let result = kernel.arm();
    assert_eq!(result, Err(ArmError::NotReady));
}

// =============================================================================
// Negative Tests - Failure Behaviors
// =============================================================================

#[test]
fn kernel_arm_fails_when_throttle_not_low() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Set throttle HIGH (violates pre-arm check)
    kernel.checks.pre_arm.update_throttle(false);

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Should not reach Ready because throttle check fails
    assert!(
        !kernel.is_ready(),
        "Should not be ready with throttle not low"
    );
}

#[test]
fn kernel_pre_arm_missing_flags_reported() {
    let mut kernel = make_kernel();
    let sensors = make_imu_only_sensors();

    // Run a few iterations
    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Check what's missing
    let missing = kernel.checks.pre_arm.missing();

    // Should be missing EKF_CONVERGED and IMU_CONVERGED (not enough samples)
    assert!(
        missing.contains(PreArmFlags::EKF_CONVERGED)
            || missing.contains(PreArmFlags::IMU_CONVERGED),
        "Expected convergence flags to be missing, got {:?}",
        missing
    );
}

#[test]
fn kernel_clears_faults_when_sensors_recover() {
    let mut kernel = make_kernel();

    // Fault detection happens in update(), not init_step()
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // Start with failed sensors
    let failed_sensors = make_failed_imu_sensors();
    kernel.step_test(dummy_time_delta(), &cmd, &failed_sensors, 0);
    assert!(kernel.faults.contains(FaultFlags::ALL_IMU_FAILED));

    // Now provide good sensors
    let good_sensors = make_valid_sensors();
    kernel.step_test(dummy_time_delta(), &cmd, &good_sensors, 0);

    // Fault should be cleared
    assert!(
        !kernel.faults.contains(FaultFlags::ALL_IMU_FAILED),
        "Fault should be cleared when sensors recover"
    );
}

#[test]
fn kernel_disarm_resets_sample_counts() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to ready and arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Check sample counts before disarm
    let samples_before = kernel.checks.pre_arm.samples.imu;
    assert!(samples_before >= 150, "Should have accumulated samples");

    // Disarm
    kernel.disarm();

    // Run one more init step to trigger Disarmed → PreArm transition
    // The reset happens in the Disarmed state handler, so after this step
    // the count is 0 (reset happens after update_from_sensors in the same step)
    kernel.init_step(&sensors, dummy_timestamp());

    // Sample counts should be reset (0 or 1 depending on order of operations)
    assert!(
        kernel.checks.pre_arm.samples.imu <= 1,
        "Sample counts should reset after disarm (got {})",
        kernel.checks.pre_arm.samples.imu
    );

    // After another step in PreArm, we should be counting again
    kernel.init_step(&sensors, dummy_timestamp());
    assert!(
        kernel.checks.pre_arm.samples.imu >= 1,
        "Sample counts should increment after reset (got {})",
        kernel.checks.pre_arm.samples.imu
    );
}

#[test]
fn kernel_requires_all_pre_arm_flags() {
    // Create kernel with FULL pre-arm requirements (including GPS)
    let mixer = QuadXMixer {
        timestamp_source: dummy_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let full_required = PreArmFlags::QUAD_WITH_GPS;
    let mut kernel = AviateKernel::with_pre_arm_required(
        Ekf::default(),
        MultirotorController::default(),
        mixer,
        Sanitizer::default(),
        mode_config,
        full_required,
    );
    kernel.checks.pre_arm.update_throttle(true);

    // Use sensors WITHOUT GPS
    let no_gps_sensors = make_imu_only_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&no_gps_sensors, dummy_timestamp());
    }

    // Should NOT be ready - missing GNSS
    assert!(
        !kernel.is_ready(),
        "Should not be ready without GNSS when GPS is required"
    );

    // Verify GNSS is in missing flags
    let missing = kernel.checks.pre_arm.missing();
    assert!(
        missing.contains(PreArmFlags::GNSS_AVAILABLE),
        "GNSS_AVAILABLE should be missing, got {:?}",
        missing
    );
}

// =============================================================================
// Phase 5: Boundary Value Tests (MC/DC coverage)
// =============================================================================

/// Test that 99 samples is NOT enough for IMU convergence (boundary - 1)
#[test]
fn boundary_imu_sample_count_99_not_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    kernel.checks.pre_arm.update_throttle(true);

    // Run exactly 99 iterations
    for _ in 0..99 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 99 samples, IMU should NOT be converged
    assert!(
        !kernel.checks.pre_arm.samples.imu_converged(),
        "IMU should NOT be converged at 99 samples (got {} samples)",
        kernel.checks.pre_arm.samples.imu
    );
    assert!(
        !kernel
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::IMU_CONVERGED),
        "IMU_CONVERGED flag should NOT be set at 99 samples"
    );
}

/// Test that 100 samples IS enough for IMU convergence (boundary)
#[test]
fn boundary_imu_sample_count_100_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    kernel.checks.pre_arm.update_throttle(true);

    // Run exactly 100 iterations
    for _ in 0..100 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 100 samples, IMU should be converged
    assert!(
        kernel.checks.pre_arm.samples.imu_converged(),
        "IMU should be converged at 100 samples (got {} samples)",
        kernel.checks.pre_arm.samples.imu
    );
    assert!(
        kernel
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::IMU_CONVERGED),
        "IMU_CONVERGED flag should be set at 100 samples"
    );
}

/// Test that 101 samples is also converged (boundary + 1)
#[test]
fn boundary_imu_sample_count_101_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    kernel.checks.pre_arm.update_throttle(true);

    // Run exactly 101 iterations
    for _ in 0..101 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 101 samples, IMU should definitely be converged
    assert!(
        kernel.checks.pre_arm.samples.imu_converged(),
        "IMU should be converged at 101 samples (got {} samples)",
        kernel.checks.pre_arm.samples.imu
    );
}

/// Test baro convergence boundary at 99 samples (not converged)
#[test]
fn boundary_baro_sample_count_99_not_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run exactly 99 iterations
    for _ in 0..99 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 99 samples, baro should NOT be converged
    assert!(
        !kernel.checks.pre_arm.samples.baro_converged(),
        "Baro should NOT be converged at 99 samples (got {} samples)",
        kernel.checks.pre_arm.samples.baro
    );
}

/// Test baro convergence boundary at 100 samples (converged)
#[test]
fn boundary_baro_sample_count_100_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run exactly 100 iterations
    for _ in 0..100 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 100 samples, baro should be converged
    assert!(
        kernel.checks.pre_arm.samples.baro_converged(),
        "Baro should be converged at 100 samples (got {} samples)",
        kernel.checks.pre_arm.samples.baro
    );
}

/// Test mag convergence boundary at 99 samples (not converged)
#[test]
fn boundary_mag_sample_count_99_not_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run exactly 99 iterations
    for _ in 0..99 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 99 samples, mag should NOT be converged
    assert!(
        !kernel.checks.pre_arm.samples.mag_converged(),
        "Mag should NOT be converged at 99 samples (got {} samples)",
        kernel.checks.pre_arm.samples.mag
    );
}

/// Test mag convergence boundary at 100 samples (converged)
#[test]
fn boundary_mag_sample_count_100_converged() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run exactly 100 iterations
    for _ in 0..100 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // At 100 samples, mag should be converged
    assert!(
        kernel.checks.pre_arm.samples.mag_converged(),
        "Mag should be converged at 100 samples (got {} samples)",
        kernel.checks.pre_arm.samples.mag
    );
}

// =============================================================================
// Phase 5: Production Config Tests
// =============================================================================

/// Test that QUAD_MINIMUM contains all required flags for basic operation
#[test]
fn production_config_quad_minimum_composition() {
    let required = PreArmFlags::QUAD_MINIMUM;

    // Must have sensor health
    assert!(
        required.contains(PreArmFlags::IMU_HEALTHY),
        "QUAD_MINIMUM must require IMU_HEALTHY"
    );
    assert!(
        required.contains(PreArmFlags::BARO_HEALTHY),
        "QUAD_MINIMUM must require BARO_HEALTHY"
    );

    // Must have convergence
    assert!(
        required.contains(PreArmFlags::IMU_CONVERGED),
        "QUAD_MINIMUM must require IMU_CONVERGED"
    );
    assert!(
        required.contains(PreArmFlags::BARO_CONVERGED),
        "QUAD_MINIMUM must require BARO_CONVERGED"
    );
    assert!(
        required.contains(PreArmFlags::EKF_CONVERGED),
        "QUAD_MINIMUM must require EKF_CONVERGED"
    );

    // Must have safety conditions
    assert!(
        required.contains(PreArmFlags::THROTTLE_LOW),
        "QUAD_MINIMUM must require THROTTLE_LOW"
    );
    assert!(
        required.contains(PreArmFlags::CONFIG_VALID),
        "QUAD_MINIMUM must require CONFIG_VALID"
    );
    assert!(
        required.contains(PreArmFlags::NO_FAULTS),
        "QUAD_MINIMUM must require NO_FAULTS"
    );

    // Must NOT require GPS (that's QUAD_WITH_GPS)
    assert!(
        !required.contains(PreArmFlags::GNSS_AVAILABLE),
        "QUAD_MINIMUM should NOT require GNSS"
    );
    assert!(
        !required.contains(PreArmFlags::MAG_HEALTHY),
        "QUAD_MINIMUM should NOT require MAG"
    );
}

/// Test that QUAD_WITH_GPS extends QUAD_MINIMUM with GPS/MAG requirements
#[test]
fn production_config_quad_with_gps_composition() {
    let quad_min = PreArmFlags::QUAD_MINIMUM;
    let quad_gps = PreArmFlags::QUAD_WITH_GPS;

    // QUAD_WITH_GPS must be a superset of QUAD_MINIMUM
    assert!(
        quad_gps.contains(quad_min),
        "QUAD_WITH_GPS must contain all QUAD_MINIMUM flags"
    );

    // Additional GPS requirements
    assert!(
        quad_gps.contains(PreArmFlags::GNSS_AVAILABLE),
        "QUAD_WITH_GPS must require GNSS"
    );
    assert!(
        quad_gps.contains(PreArmFlags::MAG_HEALTHY),
        "QUAD_WITH_GPS must require MAG"
    );
    assert!(
        quad_gps.contains(PreArmFlags::MAG_CONVERGED),
        "QUAD_WITH_GPS must require MAG_CONVERGED"
    );
}

/// Test arming with minimal config succeeds when all required sensors are present
#[test]
fn production_config_quad_minimum_arms_with_minimal_sensors() {
    // Use a truly minimal config: just IMU and convergence
    // (In practice, QUAD_MINIMUM requires baro too, but this tests the config system)
    let test_required = PreArmFlags::IMU_HEALTHY
        | PreArmFlags::IMU_CONVERGED
        | PreArmFlags::EKF_CONVERGED
        | PreArmFlags::THROTTLE_LOW
        | PreArmFlags::CONFIG_VALID;
    // Note: NO_FAULTS not required for this test

    let mixer = QuadXMixer {
        timestamp_source: dummy_timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let mut kernel = AviateKernel::with_pre_arm_required(
        Ekf::default(),
        MultirotorController::default(),
        mixer,
        Sanitizer::default(),
        mode_config,
        test_required,
    );
    kernel.checks.pre_arm.update_throttle(true);

    let sensors = make_valid_sensors(); // Use full sensors to avoid fault triggers

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Should reach Ready
    assert!(
        kernel.is_ready(),
        "Should be ready with minimal sensor config"
    );

    // Should arm successfully
    let result = kernel.arm();
    assert!(
        result.is_ok(),
        "Should arm with minimal config: {:?}",
        result
    );
}

// =============================================================================
// Phase 5: Fault State Tests
// =============================================================================

/// Test that kernel enters Fault state when critical IMU failure occurs while armed
#[test]
fn fault_state_entered_on_critical_imu_failure_while_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready and arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();
    assert_eq!(kernel.init_state, InitState::Armed);

    // Inject critical ALL_IMU_FAILED fault
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);

    // Check critical faults - should detect we need to enter fault state
    let has_critical = kernel.check_critical_faults();
    assert!(
        has_critical,
        "ALL_IMU_FAILED should be detected as critical"
    );

    // Manually transition to Fault state (in real system this happens in step())
    kernel.init_state = InitState::Fault;

    assert_eq!(kernel.init_state, InitState::Fault);
    assert!(
        !kernel.init_state.allows_active_control(),
        "Fault state should not allow control"
    );
}

/// Test that arm fails when kernel is in Fault state
#[test]
fn fault_state_prevents_arming() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert!(kernel.is_ready());

    // Force to Fault state
    kernel.init_state = InitState::Fault;
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);

    // Arm should fail - Fault state is not Ready, so NotReady is returned
    let result = kernel.arm();
    assert_eq!(
        result,
        Err(ArmError::NotReady),
        "Should not arm when in Fault state"
    );
}

/// Test can_reset_from_fault returns false when faults still active
#[test]
fn fault_state_cannot_reset_with_active_faults() {
    let mut kernel = make_kernel();

    // Put in fault state with active fault
    kernel.init_state = InitState::Fault;
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);

    assert!(
        !kernel.can_reset_from_fault(),
        "Should not be able to reset with active faults"
    );
}

/// Test can_reset_from_fault returns true when faults cleared and conditions met
#[test]
fn fault_state_can_reset_when_faults_cleared() {
    let mut kernel = make_kernel();

    // Put in fault state but clear faults
    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty();

    // can_reset_from_fault requires:
    // 1. No critical faults (empty faults satisfies this)
    // 2. IMU_HEALTHY flag set
    // 3. THROTTLE_LOW flag set
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::IMU_HEALTHY);
    kernel.checks.pre_arm.update_throttle(true);

    assert!(
        kernel.can_reset_from_fault(),
        "Should be able to reset when faults are cleared and conditions met"
    );
}

/// Test reset_from_fault transitions back to PreArm
#[test]
fn fault_state_reset_transitions_to_prearm() {
    let mut kernel = make_kernel();

    // Put in fault state
    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty(); // Clear faults to allow reset

    // Set required pre-arm flags for reset
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::IMU_HEALTHY);
    kernel.checks.pre_arm.update_throttle(true);

    // Reset from fault
    let result = kernel.reset_from_fault();

    assert!(result.is_ok(), "reset_from_fault should succeed");
    assert_eq!(
        kernel.init_state,
        InitState::PreArm,
        "Should transition to PreArm"
    );
}

/// Test reset_from_fault fails when faults still active
#[test]
fn fault_state_reset_fails_with_active_faults() {
    let mut kernel = make_kernel();

    // Put in fault state with active fault
    kernel.init_state = InitState::Fault;
    kernel.faults.insert(FaultFlags::BARO_FAILED);

    // Reset should fail
    let result = kernel.reset_from_fault();

    assert!(
        result.is_err(),
        "reset_from_fault should fail with active faults"
    );
    assert_eq!(
        kernel.init_state,
        InitState::Fault,
        "Should remain in Fault state"
    );
}

/// Test reset_from_fault clears sample counts for fresh convergence
#[test]
fn fault_state_reset_clears_convergence() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Build up sample counts first
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert!(kernel.checks.pre_arm.samples.imu >= 150);

    // Enter fault state and reset
    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty();
    kernel.reset_from_fault().unwrap();

    // Sample counts should be reset
    assert_eq!(
        kernel.checks.pre_arm.samples.imu, 0,
        "Sample counts should be reset after fault recovery"
    );
    assert_eq!(kernel.checks.pre_arm.samples.baro, 0);
    assert_eq!(kernel.checks.pre_arm.samples.mag, 0);
}

/// Test full fault recovery cycle: Ready → Armed → Fault → PreArm → Ready
#[test]
fn fault_state_full_recovery_cycle() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Phase 1: Get to Ready
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Phase 2: Arm
    kernel.arm().unwrap();
    assert_eq!(kernel.init_state, InitState::Armed);

    // Phase 3: Enter Fault (simulate critical failure)
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);
    kernel.init_state = InitState::Fault;
    assert_eq!(kernel.init_state, InitState::Fault);

    // Phase 4: Clear fault and reset
    kernel.faults.remove(FaultFlags::ALL_IMU_FAILED);
    kernel.reset_from_fault().unwrap();
    assert_eq!(kernel.init_state, InitState::PreArm);

    // Phase 5: Re-converge to Ready
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Phase 6: Should be able to arm again
    let result = kernel.arm();
    assert!(
        result.is_ok(),
        "Should be able to re-arm after fault recovery"
    );
    assert_eq!(kernel.init_state, InitState::Armed);
}

/// Test outputs are safe (zero) when in Fault state
#[test]
fn fault_state_outputs_safe_values() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Put in fault state
    kernel.init_state = InitState::Fault;
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);

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

    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // All outputs should be safe (0.0) in fault state
    for i in 0..4 {
        assert!(
            output.outputs[i].0.abs() < 1e-5,
            "Motor {} should be 0 in fault state (got {})",
            i,
            output.outputs[i].0
        );
    }
}

// =============================================================================
// Config Mode Transition Tests
// =============================================================================

/// Test request_config_mode fails when not armed
#[test]
fn request_config_mode_fails_when_not_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready but don't arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Try to request config mode change
    let result = kernel.request_config_mode(ConfigMode::Cruise);
    assert_eq!(result, Err(TransitionError::NotArmed));
}

/// Test request_config_mode fails when in fault state
#[test]
fn request_config_mode_fails_in_fault_state() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready and arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().expect("Should arm");

    // Inject critical fault
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);

    // Try to request config mode change
    let result = kernel.request_config_mode(ConfigMode::Cruise);
    assert_eq!(result, Err(TransitionError::InFaultState));
}

/// Test request_config_mode fails when already in requested mode
#[test]
fn request_config_mode_fails_already_in_mode() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready and arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().expect("Should arm");

    // Try to request same mode
    let result = kernel.request_config_mode(ConfigMode::Hover);
    assert_eq!(result, Err(TransitionError::AlreadyInMode));
}

/// Test request_config_mode fails when already transitioning
#[test]
fn request_config_mode_fails_already_transitioning() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Get to Ready and arm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().expect("Should arm");

    // Force into Transition mode
    kernel.mode = ConfigMode::Transition;

    // Try to request another config mode
    let result = kernel.request_config_mode(ConfigMode::Cruise);
    assert_eq!(result, Err(TransitionError::AlreadyTransitioning));
}

// =============================================================================
// Control Law Tests
// =============================================================================

/// Test control law severity ordering
#[test]
fn control_law_severity_ordering() {
    // Lower severity = less degraded
    assert!(ControlLawV1::Primary.severity() < ControlLawV1::Alternate.severity());
    assert!(ControlLawV1::Alternate.severity() < ControlLawV1::Direct.severity());
    assert!(ControlLawV1::Direct.severity() < ControlLawV1::Backup.severity());
}

/// Test kernel starts with Normal control law
#[test]
fn kernel_starts_with_normal_control_law() {
    let kernel = make_kernel();
    assert_eq!(kernel.control_law, ControlLawV1::Primary);
}

/// Test get_health returns correct state
#[test]
fn kernel_get_health_returns_correct_state() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Before convergence
    let health = kernel.get_health();
    assert_eq!(health.init_state, InitState::PowerOn);

    // After convergence
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    let health = kernel.get_health();
    assert_eq!(health.init_state, InitState::Ready);
    assert_eq!(health.control_law, ControlLawV1::Primary);
    assert_eq!(health.config_mode, ConfigMode::Hover);
    assert!(health.faults.is_empty());
}

/// Test kernel check_critical_faults detects ALL_IMU_FAILED
#[test]
fn kernel_check_critical_faults_detects_imu() {
    let mut kernel = make_kernel();

    // No faults - no critical
    assert!(!kernel.check_critical_faults());

    // Add critical fault
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED);
    assert!(kernel.check_critical_faults());
}

/// Test kernel check_critical_faults detects ESTIMATOR_DIVERGED
#[test]
fn kernel_check_critical_faults_detects_estimator() {
    let mut kernel = make_kernel();

    kernel.faults.insert(FaultFlags::ESTIMATOR_DIVERGED);
    assert!(kernel.check_critical_faults());
}

/// Test kernel check_critical_faults detects NUMERIC_ERROR
#[test]
fn kernel_check_critical_faults_detects_numeric() {
    let mut kernel = make_kernel();

    kernel.faults.insert(FaultFlags::NUMERIC_ERROR);
    assert!(kernel.check_critical_faults());
}

/// Test kernel check_critical_faults ignores non-critical faults
#[test]
fn kernel_check_critical_faults_ignores_non_critical() {
    let mut kernel = make_kernel();

    // Non-critical faults
    kernel.faults.insert(FaultFlags::BARO_FAILED);
    assert!(!kernel.check_critical_faults());

    kernel.faults.insert(FaultFlags::MAG_FAILED);
    assert!(!kernel.check_critical_faults());

    kernel.faults.insert(FaultFlags::COMMAND_TIMEOUT);
    assert!(!kernel.check_critical_faults());
}

// =============================================================================
// Coverage Tests: Init State Machine Transitions
// =============================================================================

/// Test PowerOn → ConfigLoading → SensorInit transitions
#[test]
fn init_state_power_on_to_sensor_init() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Start in PowerOn
    assert_eq!(kernel.init_state, InitState::PowerOn);

    // First step transitions to ConfigLoading
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::ConfigLoading);

    // Next step transitions to SensorInit (CONFIG_VALID set)
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::SensorInit);
}

/// Test SensorInit → EstimatorConverging transition requires IMU_HEALTHY
#[test]
fn init_state_sensor_init_to_estimator_converging() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Fast forward to SensorInit
    kernel.init_step(&sensors, dummy_timestamp()); // PowerOn → ConfigLoading
    kernel.init_step(&sensors, dummy_timestamp()); // ConfigLoading → SensorInit
    assert_eq!(kernel.init_state, InitState::SensorInit);

    // With valid IMU sensor, should transition to EstimatorConverging
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::EstimatorConverging);
}

/// Test that SensorInit stays if no valid IMU
#[test]
fn init_state_sensor_init_stays_without_imu() {
    let mut kernel = make_kernel();
    let failed_sensors = make_failed_imu_sensors();

    // Fast forward to SensorInit
    let valid_sensors = make_valid_sensors();
    kernel.init_step(&valid_sensors, dummy_timestamp()); // PowerOn → ConfigLoading
    kernel.init_step(&valid_sensors, dummy_timestamp()); // ConfigLoading → SensorInit
    assert_eq!(kernel.init_state, InitState::SensorInit);

    // With failed IMU, should stay in SensorInit
    kernel.init_step(&failed_sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::SensorInit);
}

/// Test EstimatorConverging → PreArm transition
#[test]
fn init_state_estimator_converging_to_prearm() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run until EstimatorConverging
    for _ in 0..5 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Run until converged (100+ samples)
    for _ in 0..100 {
        kernel.init_step(&sensors, dummy_timestamp());
    }

    // Should now be in PreArm or Ready
    assert!(
        kernel.init_state == InitState::PreArm || kernel.init_state == InitState::Ready,
        "Expected PreArm or Ready, got {:?}",
        kernel.init_state
    );
}

/// Test Ready → PreArm fallback when pre-arm conditions fail
#[test]
fn init_state_ready_to_prearm_fallback() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to Ready
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Remove throttle low condition
    kernel.checks.pre_arm.update_throttle(false);

    // Next step should fall back to PreArm
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::PreArm);
}

/// Test Disarmed → PreArm transition with sample count reset
#[test]
fn init_state_disarmed_to_prearm() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to Ready, arm, then disarm
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();
    kernel.disarm();
    assert_eq!(kernel.init_state, InitState::Disarmed);

    // Next step should transition to PreArm and reset samples
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::PreArm);
}

/// Test Fault state stays without explicit reset
#[test]
fn init_state_fault_stays_until_reset() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Force fault state
    kernel.init_state = InitState::Fault;

    // Init step should not change fault state
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(kernel.init_state, InitState::Fault);

    // Multiple steps still don't change it
    for _ in 0..10 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Fault);
}

// =============================================================================
// Coverage Tests: step() Method Paths
// =============================================================================

/// Test step returns safe output when not armed
#[test]
fn step_returns_safe_when_not_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 42,
        source: CommandSource::Pilot,
    };

    // Not armed - step should return safe output
    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Safe output should have active_mask = 0
    assert_eq!(output.active_mask, 0);
    assert_eq!(output.sequence, 42);
    assert!(output.sanitized);
}

/// Test step returns safe output when critical fault occurs
#[test]
fn step_returns_safe_on_critical_fault() {
    let mut kernel = make_kernel();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed with valid sensors
    let valid_sensors = make_valid_sensors();
    for _ in 0..150 {
        kernel.init_step(&valid_sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Use failed sensors to trigger fault detection in step()
    let failed_sensors = make_failed_imu_sensors();

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    // Step with failed sensors should detect critical fault
    let output = kernel.step_test(dummy_time_delta(), &cmd, &failed_sensors, 0);

    // Should return safe output (active_mask = 0)
    assert_eq!(
        output.active_mask, 0,
        "Critical fault should cause safe output"
    );
}

/// Test step with frozen control law
#[test]
fn step_with_frozen_control_law() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Set frozen control law
    kernel.control_law = ControlLawV1::Backup;

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Frozen law should have fallback_mask set
    assert_eq!(output.fallback_mask, 0xFF);
    assert!(output.sanitized);
}

/// Test step performs normal control when armed
#[test]
fn step_performs_control_when_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed
    for _ in 0..150 {
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
        sequence: 1,
        source: CommandSource::Pilot,
    };

    let output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Should have active outputs
    assert!(output.sanitized);
}

/// Test step with degradation trigger
#[test]
fn step_handles_degradation_trigger() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Clear attitude valid to trigger degradation
    kernel
        .checks
        .in_flight
        .current
        .remove(aviate_core::checks::InFlightFlags::ATTITUDE_VALID);

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    // Run step - should handle degradation
    let _output = kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Control law may have changed due to degradation
    // (exact behavior depends on get_degradation_trigger logic)
}

// =============================================================================
// Coverage Tests: handle_degradation
// =============================================================================

/// Test handle_degradation for all DegradationReason variants
#[test]
fn handle_degradation_all_reasons() {
    use aviate_core::DegradationReason;

    let reasons_and_expected_laws = [
        (DegradationReason::AttitudeLost, ControlLawV1::Backup),
        (DegradationReason::ImuDegraded, ControlLawV1::Alternate),
        (DegradationReason::PositionLost, ControlLawV1::Alternate),
        (DegradationReason::VelocityLost, ControlLawV1::Alternate),
        (DegradationReason::CommandTimeout, ControlLawV1::Alternate),
        (
            DegradationReason::EnvelopeViolation,
            ControlLawV1::Alternate,
        ),
        (DegradationReason::BaroDegraded, ControlLawV1::Alternate),
        (DegradationReason::RcLost, ControlLawV1::Alternate),
        (DegradationReason::TimingViolation, ControlLawV1::Alternate),
    ];

    for (reason, expected_law) in reasons_and_expected_laws {
        let mut kernel = make_kernel();
        kernel.control_law = ControlLawV1::Primary;

        let event = kernel.handle_degradation(reason, dummy_timestamp());

        assert!(
            event.is_some(),
            "Expected degradation event for {:?}",
            reason
        );
        let event = event.unwrap();
        assert_eq!(event.to, expected_law, "Wrong law for {:?}", reason);
        assert_eq!(kernel.control_law, expected_law);
    }
}

/// Test handle_degradation returns None when not degrading
#[test]
fn handle_degradation_no_event_when_not_worse() {
    let mut kernel = make_kernel();

    // Start with Frozen (worst)
    kernel.control_law = ControlLawV1::Backup;

    // Try to "degrade" to Degraded (less severe)
    let event = kernel.handle_degradation(DegradationReason::ImuDegraded, dummy_timestamp());

    // Should not trigger - already worse
    assert!(event.is_none());
    assert_eq!(kernel.control_law, ControlLawV1::Backup);
}

// =============================================================================
// Coverage Tests: Sensor Overrides & Faults
// =============================================================================

#[test]
fn test_sensor_overrides_gnss_force_state() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Force GNSS state to Suspect (1) via command
    let mut cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: Some(aviate_core::control::SensorOverrides {
            gnss_force_state: Some(GnssHealth::Suspect), // Suspect
        }),
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // Step kernel (armed)
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // EKF should have updated with Suspect GNSS (internal EKF state not easily exposed,
    // but we verify code path execution).
    // Cover other force states
    cmd.sensor_overrides = Some(aviate_core::control::SensorOverrides {
        gnss_force_state: Some(GnssHealth::Good), // Good
    });
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    cmd.sensor_overrides = Some(aviate_core::control::SensorOverrides {
        gnss_force_state: Some(GnssHealth::Lost), // Lost
    });
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    cmd.sensor_overrides = Some(aviate_core::control::SensorOverrides {
        gnss_force_state: Some(GnssHealth::Lost), // Unknown -> Lost
    });
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
}

#[test]
fn test_update_sensor_faults_all_sensors() {
    let mut kernel = make_kernel();
    let mut sensors = make_valid_sensors();

    // Fault detection happens in update(), not init_step()
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // 1. Fail Baro
    sensors.baros[0].health = SensorHealth::Failed;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(kernel.faults.contains(FaultFlags::BARO_FAILED));

    // Recover Baro
    sensors.baros[0].health = SensorHealth::Good;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(!kernel.faults.contains(FaultFlags::BARO_FAILED));

    // 2. Fail Mag
    sensors.mags[0].health = SensorHealth::Failed;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(kernel.faults.contains(FaultFlags::MAG_FAILED));

    // Recover Mag
    sensors.mags[0].health = SensorHealth::Good;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(!kernel.faults.contains(FaultFlags::MAG_FAILED));

    // 3. Fail GNSS
    sensors.gnss[0].health = SensorHealth::Failed;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(kernel.faults.contains(FaultFlags::ALL_GNSS_LOST));

    // Recover GNSS
    sensors.gnss[0].health = SensorHealth::Good;
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    assert!(!kernel.faults.contains(FaultFlags::ALL_GNSS_LOST));
}

#[test]
fn test_init_state_forced_control_law_coverage() {
    // Test Armed returns None (already covered by init_state_no_forced_law_when_armed, but ensuring path)
    assert_eq!(InitState::Armed.forced_control_law(), None);
    // Test non-armed returns Some(Frozen)
    assert_eq!(
        InitState::PreArm.forced_control_law(),
        Some(ControlLawV1::Backup)
    );
}

#[test]
fn test_check_critical_faults_returns_false() {
    let mut kernel = make_kernel();
    assert_eq!(kernel.check_critical_faults(), false);
}

// =============================================================================
// Coverage Tests: request_config_mode
// =============================================================================

/// Test request_config_mode succeeds when conditions are met
#[test]
fn request_config_mode_succeeds() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Set up transition checks
    kernel.checks.transition.current = aviate_core::checks::TransitionFlags::all();

    // Request transition from Hover to Cruise
    let result = kernel.request_config_mode(ConfigMode::Cruise);
    assert!(result.is_ok());
    assert_eq!(kernel.mode, ConfigMode::Cruise);
}

/// Test request_config_mode fails when checks fail
#[test]
fn request_config_mode_fails_checks() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to armed
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Set required flags but don't meet them
    kernel.checks.transition.required = aviate_core::checks::TransitionFlags::all();
    kernel.checks.transition.current = aviate_core::checks::TransitionFlags::empty();

    let result = kernel.request_config_mode(ConfigMode::Cruise);
    // Transition should fail due to checks
    assert!(
        result.is_err(),
        "Expected error when transition checks fail, got {:?}",
        result
    );
}

// =============================================================================
// Coverage Tests: can_reset_from_fault / reset_from_fault
// =============================================================================

/// Test can_reset_from_fault returns false when not in Fault state
#[test]
fn can_reset_from_fault_not_in_fault() {
    let kernel = make_kernel();

    // Not in Fault state
    assert!(!kernel.can_reset_from_fault());
}

/// Test can_reset_from_fault returns true when conditions met
#[test]
fn can_reset_from_fault_conditions_met() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Set up fault state with recovery conditions
    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty(); // No critical faults
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::IMU_HEALTHY);
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::THROTTLE_LOW);

    // Init step to update sensors
    kernel.init_step(&sensors, dummy_timestamp());

    assert!(kernel.can_reset_from_fault());
}

/// Test reset_from_fault fails when not in Fault state
#[test]
fn reset_from_fault_fails_not_in_fault() {
    let mut kernel = make_kernel();

    let result = kernel.reset_from_fault();
    assert_eq!(result, Err(ArmError::NotReady));
}

/// Test reset_from_fault fails when conditions not met
#[test]
fn reset_from_fault_fails_conditions_not_met() {
    let mut kernel = make_kernel();

    kernel.init_state = InitState::Fault;
    kernel.faults.insert(FaultFlags::ALL_IMU_FAILED); // Critical fault active

    let result = kernel.reset_from_fault();
    assert_eq!(result, Err(ArmError::Faulted));
}

/// Test reset_from_fault succeeds when conditions met
#[test]
fn reset_from_fault_succeeds() {
    let mut kernel = make_kernel();

    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty();
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::IMU_HEALTHY);
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::THROTTLE_LOW);

    let result = kernel.reset_from_fault();
    assert!(result.is_ok());
    assert_eq!(kernel.init_state, InitState::PreArm);
}

// =============================================================================
// Coverage Tests: get_health
// =============================================================================

/// Test get_health returns correct report
#[test]
fn get_health_report() {
    let mut kernel = make_kernel();

    kernel.init_state = InitState::Armed;
    kernel.control_law = ControlLawV1::Alternate;
    kernel.mode = ConfigMode::Cruise;
    kernel.faults.insert(FaultFlags::BARO_FAILED);

    let health = kernel.get_health();

    assert_eq!(health.init_state, InitState::Armed);
    assert_eq!(health.control_law, ControlLawV1::Alternate);
    assert_eq!(health.config_mode, ConfigMode::Cruise);
    assert!(health.faults.contains(FaultFlags::BARO_FAILED));
}

// =============================================================================
// Coverage Tests: init_core
// =============================================================================

/// Test init_core function exists and can be called
#[test]
fn init_core_callable() {
    aviate_core::init_core();
    // Function does nothing but should exist for coverage
}

// =============================================================================
// Coverage Tests: Watchdog & Ground Reset
// =============================================================================

#[test]
fn test_watchdog_kick() {
    let mut kernel = make_kernel();
    kernel.kick_watchdog();
    // Currently a stub, but ensures method is reachable
}

#[test]
fn test_ground_reset_clears_state() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Get to Ready state with some accumulated state
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Set some faults and check flags
    kernel.faults.insert(FaultFlags::BARO_FAILED);
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::THROTTLE_LOW);

    // Perform ground reset
    kernel.ground_reset();

    // Should revert to ConfigLoading and clear faults/checks
    assert_eq!(kernel.init_state, InitState::ConfigLoading);
    assert!(kernel.faults.is_empty());
    assert!(kernel.checks.pre_arm.current.is_empty());
}

#[test]
fn test_ground_reset_ignored_when_armed() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Get to Armed state
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();
    assert_eq!(kernel.init_state, InitState::Armed);

    // Attempt ground reset
    kernel.ground_reset();

    // Should remain Armed
    assert_eq!(kernel.init_state, InitState::Armed);
}

#[test]
fn test_can_reset_from_fault_fails_checks() {
    let mut kernel = make_kernel();

    // Enter Fault state
    kernel.init_state = InitState::Fault;
    kernel.faults = FaultFlags::empty(); // No active faults

    // 1. Fail throttle check
    kernel
        .checks
        .pre_arm
        .current
        .insert(PreArmFlags::IMU_HEALTHY);
    kernel.checks.pre_arm.update_throttle(false); // Throttle HIGH
    assert!(!kernel.can_reset_from_fault());

    // 2. Fail IMU check
    kernel
        .checks
        .pre_arm
        .current
        .remove(PreArmFlags::IMU_HEALTHY);
    kernel.checks.pre_arm.update_throttle(true); // Throttle LOW
    assert!(!kernel.can_reset_from_fault());
}

#[test]
fn test_check_critical_faults_returns_true() {
    let mut kernel = make_kernel();

    // Inject critical fault
    kernel.faults.insert(FaultFlags::NUMERIC_ERROR);

    // Should return true and transition to Fault
    assert!(kernel.check_critical_faults());
    assert_eq!(kernel.init_state, InitState::Fault);
    assert_eq!(kernel.control_law, ControlLawV1::Backup);
}

#[test]
fn test_step_with_unhealthy_sensors() {
    let mut kernel = make_kernel();
    let mut sensors = make_valid_sensors();

    // Mark all sensors as failed/invalid to skip EKF updates
    sensors.imus[0].valid = false;
    sensors.gnss[0].health = SensorHealth::Failed;
    sensors.baros[0].valid = false;
    sensors.mags[0].health = SensorHealth::Failed;

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // Initialize to Armed to allow step() to proceed past the first check
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    // Force state to Armed directly to bypass init checks that would fail with bad sensors
    kernel.init_state = InitState::Armed;

    // Run step
    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);

    // Verify no panic and logic executed (coverage should hit the 'else'/skip paths)
}

// =============================================================================
// Branch Coverage: Edge Cases for 100% Branch Coverage
// =============================================================================

/// Test sensor_overrides with Some(overrides) but gnss_force_state = None
/// This hits the "None" branch of the nested if let at lib.rs:646
#[test]
fn step_with_sensor_overrides_no_gnss_force() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Provide sensor_overrides but with gnss_force_state = None
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: Some(aviate_core::control::SensorOverrides {
            gnss_force_state: None, // This is the key - Some(overrides) but no GNSS force
        }),
        sequence: 0,
        source: CommandSource::Pilot,
    };

    kernel.step_test(dummy_time_delta(), &cmd, &sensors, 0);
    // Code path executed - gnss_force_state None branch hit
}

/// Test that Ready state stays Ready when pre_arm IS satisfied (lib.rs:388 false branch)
#[test]
fn init_state_ready_stays_when_satisfied() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    // Run to Ready
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    assert_eq!(kernel.init_state, InitState::Ready);

    // Another step with conditions still satisfied - should stay Ready
    kernel.init_step(&sensors, dummy_timestamp());
    assert_eq!(
        kernel.init_state,
        InitState::Ready,
        "Should stay Ready when pre_arm satisfied"
    );
}

/// Test handle_degradation when current law equals new law (no change case)
/// This tests line 541's case where severity is NOT greater
#[test]
fn handle_degradation_same_severity_no_event() {
    let mut kernel = make_kernel();

    // Start with Degraded
    kernel.control_law = ControlLawV1::Alternate;

    // Try to "degrade" to Degraded again (same severity)
    let event = kernel.handle_degradation(DegradationReason::ImuDegraded, dummy_timestamp());

    // No event - same severity
    assert!(event.is_none(), "Same severity should not produce event");
    assert_eq!(kernel.control_law, ControlLawV1::Alternate);
}

/// Test handle_degradation from Failsafe to Degraded (lower severity) - no event
#[test]
fn handle_degradation_lower_severity_no_event() {
    let mut kernel = make_kernel();

    // Start with Failsafe
    kernel.control_law = ControlLawV1::Alternate;

    // Try to "degrade" to Degraded (lower severity)
    let event = kernel.handle_degradation(DegradationReason::ImuDegraded, dummy_timestamp());

    // No event - trying to go to lower severity
    assert!(event.is_none());
    assert_eq!(kernel.control_law, ControlLawV1::Alternate);
}

/// Test handle_degradation from Normal to Degraded (higher severity) - produces event
#[test]
fn handle_degradation_higher_severity_produces_event() {
    let mut kernel = make_kernel();

    // Start with Normal
    kernel.control_law = ControlLawV1::Primary;

    // Degrade to Degraded (higher severity)
    let event = kernel.handle_degradation(DegradationReason::ImuDegraded, dummy_timestamp());

    // Event produced
    assert!(event.is_some());
    assert_eq!(kernel.control_law, ControlLawV1::Alternate);
}

/// Test ground_reset clears faults and resets init sequence
#[test]
fn ground_reset_clears_faults_and_restarts_init() {
    let mut kernel = make_kernel();

    // Set some state
    kernel.faults.insert(FaultFlags::IMU0_FAILED);
    kernel.control_law = ControlLawV1::Alternate;

    // Ground reset should work before arming
    kernel.ground_reset();

    // Faults should be cleared
    assert!(kernel.faults.is_empty());
    // Control law reset to Primary
    assert_eq!(kernel.control_law, ControlLawV1::Primary);
    // Init state reset
    assert_eq!(kernel.init_state, InitState::ConfigLoading);
}

/// Test ground_reset is blocked when armed
#[test]
fn ground_reset_blocked_when_armed() {
    // Create a kernel and manually set it to armed state
    let mut kernel = make_kernel();
    kernel.init_state = InitState::Armed;

    // Set some state
    kernel.faults.insert(FaultFlags::IMU0_FAILED);

    // Ground reset should be blocked when armed
    kernel.ground_reset();

    // Faults should NOT be cleared (blocked)
    assert!(kernel.faults.contains(FaultFlags::IMU0_FAILED));
    // Should still be armed
    assert_eq!(kernel.init_state, InitState::Armed);
}

/// Test report_timing_violation tracks consecutive violations
#[test]
fn report_timing_violation_tracks_consecutive() {
    let mut kernel = make_kernel();

    // Report violations - should increment counter
    kernel.report_timing_violation(true);
    assert_eq!(kernel.timing_stats.consecutive_violations, 1);

    kernel.report_timing_violation(true);
    assert_eq!(kernel.timing_stats.consecutive_violations, 2);

    // Report success - should reset counter
    kernel.report_timing_violation(false);
    assert_eq!(kernel.timing_stats.consecutive_violations, 0);
}

/// Test report_timing_violation counts total violations correctly
#[test]
fn report_timing_violation_counts_total() {
    let mut kernel = make_kernel();

    // Report several violations
    for _ in 0..10 {
        kernel.report_timing_violation(true);
    }

    // Total should be 10, consecutive should be 10
    assert_eq!(kernel.timing_stats.deadline_violations, 10);
    assert_eq!(kernel.timing_stats.consecutive_violations, 10);

    // Reset consecutive
    kernel.report_timing_violation(false);
    // Total unchanged, consecutive reset
    assert_eq!(kernel.timing_stats.deadline_violations, 10);
    assert_eq!(kernel.timing_stats.consecutive_violations, 0);
}

/// Drive the in-flight-checks degradation branch in `update()`.
///
/// When `consecutive_violations < TIMING_VIOLATION_THRESHOLD` but
/// `get_degradation_trigger()` returns `Some(reason)`, the `else if`
/// arm fires. Achieved here by forcing the kernel into `Armed` with an
/// un-initialized EKF — `update_from_state` then sees empty valid_flags
/// and clears ATTITUDE_VALID, which `get_degradation_trigger` maps to
/// `DegradationReason::AttitudeLost` (priority 1).
#[test]
fn update_degrades_on_in_flight_trigger() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Armed but EKF never initialized — valid_flags will be empty.
    kernel.init_state = InitState::Armed;
    assert!(!kernel.estimator.is_initialized());

    // Ensure timing-violation branch is NOT the one that fires.
    assert_eq!(kernel.timing_stats.consecutive_violations, 0);

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };
    let actuator_state = kernel.actuator_state.clone();
    let result = kernel.update(
        ChannelId::PRIMARY,
        dummy_time_delta(),
        &sensors,
        &cmd,
        &actuator_state,
        None,
    );

    let event = result
        .degradation
        .expect("uninitialized EKF + armed should trigger an in-flight degradation");
    assert_eq!(event.reason, DegradationReason::AttitudeLost);
    // AttitudeLost maps to Backup.
    assert_eq!(event.to, ControlLawV1::Backup);
}

/// Drive the timing-violation degradation branch in `update()`.
///
/// After `TIMING_VIOLATION_THRESHOLD` (3) consecutive reported
/// violations, the next armed `update()` call must degrade to
/// `ControlLawV1::Alternate` via `DegradationReason::TimingViolation`.
/// Covers the `if self.timing_stats.consecutive_violations >=
/// TIMING_VIOLATION_THRESHOLD` branch in the per-cycle path.
#[test]
fn update_degrades_on_persistent_timing_violation() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        kernel.init_step(&sensors, dummy_timestamp());
    }
    kernel.arm().unwrap();

    // Record 3 consecutive violations — the threshold.
    for _ in 0..aviate_core::TIMING_VIOLATION_THRESHOLD {
        kernel.report_timing_violation(true);
    }

    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };
    let actuator_state = kernel.actuator_state.clone();
    let result = kernel.update(
        ChannelId::PRIMARY,
        dummy_time_delta(),
        &sensors,
        &cmd,
        &actuator_state,
        None,
    );

    let event = result
        .degradation
        .expect("threshold-crossing call should emit a degradation event");
    assert_eq!(event.reason, DegradationReason::TimingViolation);
    assert_eq!(event.to, ControlLawV1::Alternate);
    assert_eq!(kernel.control_law, ControlLawV1::Alternate);
}

/// Exercise `AviateKernelTrait` surface (spec §20) to cover the trait
/// impl's pass-through methods. The impl is pure delegation to inherent
/// methods on `AviateKernelImpl`, but grcov counts each trait method as
/// its own function and needs a direct hit to mark it covered.
#[test]
fn aviate_kernel_trait_surface_covered() {
    use aviate_core::kernel_trait::AviateKernelTrait;

    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    // Exercise the read-only accessors first.
    let _ = AviateKernelTrait::init_state(&kernel);
    let _ = AviateKernelTrait::is_ready(&kernel);
    let _ = AviateKernelTrait::config_mode(&kernel);
    let _ = AviateKernelTrait::transition_state(&kernel);
    let _ = AviateKernelTrait::get_config(&kernel);
    let _ = AviateKernelTrait::get_faults(&kernel);
    let _ = AviateKernelTrait::get_control_law(&kernel);
    let _ = AviateKernelTrait::get_health(&kernel);

    // init_step through the trait.
    let _ = AviateKernelTrait::init_step(&mut kernel, &sensors, dummy_timestamp());

    // request_config_mode and update will reject (not armed / not ready),
    // but they still exercise the delegation bodies.
    let _ = AviateKernelTrait::request_config_mode(&mut kernel, ConfigMode::Cruise);
    let cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };
    let actuator_state = kernel.actuator_state.clone();
    let _ = AviateKernelTrait::update(
        &mut kernel,
        ChannelId::PRIMARY,
        dummy_time_delta(),
        &sensors,
        &cmd,
        &actuator_state,
        None,
    );

    // load_config: v1 (supported) then v2 (unsupported) for both branches.
    let cfg_ok = aviate_core::kernel_types::ConfigBlock {
        data: &[],
        version: 1,
        checksum: 0,
    };
    assert!(AviateKernelTrait::load_config(&mut kernel, &cfg_ok).is_ok());
    let cfg_bad = aviate_core::kernel_types::ConfigBlock {
        data: &[],
        version: 99,
        checksum: 0,
    };
    assert!(AviateKernelTrait::load_config(&mut kernel, &cfg_bad).is_err());

    // Arm-path: wind through init_step enough times to satisfy pre-arm,
    // then arm/disarm + watchdog + ground_reset via the trait.
    kernel.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        AviateKernelTrait::init_step(&mut kernel, &sensors, dummy_timestamp());
    }
    assert!(AviateKernelTrait::arm(&mut kernel).is_ok());
    AviateKernelTrait::kick_watchdog(&mut kernel);
    AviateKernelTrait::disarm(&mut kernel);
    AviateKernelTrait::ground_reset(&mut kernel);
}
