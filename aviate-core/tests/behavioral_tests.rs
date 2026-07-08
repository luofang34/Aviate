//! Behavioral verification suite for aviate-core.
//!
//! These tests witness the 200-series behavioral LLRs in
//! `cert/trace/llr.toml`. They complement the existing structural
//! tests (kernel.rs, ekf_tests.rs, mixer_tests.rs) by pinning
//! cycle-level kernel behavior — disarmed safety, latched-fault
//! inhibition, atomic mode swap, and EKF cold-start / dropout
//! behavior — to executable assertions.
//!
//! Each test docstring cites its LLR(s); cert/trace/tests.toml carries
//! the reciprocal `traces_to` link.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::checks::{InFlightFlags, PreArmFlags};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::ekf::{Ekf, EkfConfig, EkfState, Estimator};
use aviate_core::fault::FaultFlags;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorCmd, ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth,
    SensorReading, SensorSet,
};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Normalized, Pascals,
    Radians, RadiansPerSecond, Scalar, Seconds,
};
use aviate_core::{AviateKernel, ChannelId, InitState, TransitionError};

// --- fixtures ---------------------------------------------------------------

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn dt_100hz() -> TimeDelta {
    TimeDelta {
        dt_sec: Seconds(0.01),
        tick_delta: 10_000,
    }
}

fn make_kernel() -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
    let mixer = QuadXMixer {
        timestamp_source: timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let required = PreArmFlags::IMU_HEALTHY
        | PreArmFlags::IMU_CONVERGED
        | PreArmFlags::EKF_CONVERGED
        | PreArmFlags::THROTTLE_LOW
        | PreArmFlags::CONFIG_VALID;
    let mut kernel = AviateKernel::with_pre_arm_required(
        Ekf::default(),
        MultirotorController::default(),
        mixer,
        Sanitizer,
        mode_config,
        required,
    );
    kernel.state.checks.pre_arm.update_throttle(true);
    kernel
}

fn make_valid_sensors() -> SensorSet {
    let ts = timestamp();
    SensorSet {
        imus: [
            SensorReading {
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
            },
            SensorReading::default(),
            SensorReading::default(),
        ],
        baros: [
            SensorReading {
                value: BaroData {
                    altitude: Some(Meters(0.0)),
                    air: AirData {
                        static_pressure: Some(Pascals(101_325.0)),
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
            },
            SensorReading::default(),
        ],
        mags: [
            SensorReading {
                value: MagData {
                    field_ut: [Microtesla(20.0), Microtesla(0.0), Microtesla(40.0)],
                },
                valid: true,
                source_id: 0,
                timestamp: ts,
                health: SensorHealth::Good,
            },
            SensorReading::default(),
        ],
        gnss: [
            SensorReading {
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
            },
            SensorReading::default(),
        ],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

fn arm_kernel(kernel: &mut aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer>) {
    let sensors = make_valid_sensors();
    kernel.state.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
    assert_eq!(kernel.state.init_state, InitState::Ready);
    kernel
        .arm()
        .expect("kernel should arm with healthy sensors");
}

fn step(
    kernel: &mut aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer>,
    cmd: &Command,
    sensors: &SensorSet,
    command_age_ms: u32,
) -> ActuatorCmd {
    let actuator_state = kernel.state.actuator_state.clone();
    kernel
        .update(
            ChannelId::PRIMARY,
            dt_100hz(),
            sensors,
            cmd,
            command_age_ms,
            &actuator_state,
            None,
        )
        .actuator
}

fn zero_cmd() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    }
}

fn all_motors_zero(out: &ActuatorCmd) -> bool {
    (0..4).all(|i| out.outputs[i].0.abs() < 1e-5)
}

// --- LLR-FLT-202 ------------------------------------------------------------

/// LLR-FLT-202: a disarmed kernel emits the safe pattern regardless of
/// estimator state or commanded setpoint. Compares the output across
/// four configurations whose inputs span the meaningful axes (clean vs
/// extreme command; pristine vs corrupted estimator state) and asserts
/// every one returns the safe pattern.
#[test]
fn disarmed_safe_output_is_independent_of_inputs() {
    let mut kernel = make_kernel();
    let sensors = make_valid_sensors();

    let zero_cmd = zero_cmd();
    // Saturating: a non-level attitude, high angular rate, full thrust.
    let tilt = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 1.0);
    let saturating_cmd = Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(1.0),
            attitude: Some(tilt),
            angular_rate: Some([
                RadiansPerSecond(10.0),
                RadiansPerSecond(-10.0),
                RadiansPerSecond(10.0),
            ]),
            heading: Some(Radians(3.0)),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    };

    // (a) clean estimator, zero command.
    let out_a = step(&mut kernel, &zero_cmd, &sensors, 0);
    assert!(
        all_motors_zero(&out_a),
        "disarmed kernel + zero command must emit zeros"
    );

    // (b) clean estimator, saturating command.
    let out_b = step(&mut kernel, &saturating_cmd, &sensors, 0);
    assert!(
        all_motors_zero(&out_b),
        "disarmed kernel must ignore saturating command"
    );

    // (c) corrupted estimator (faults latched), zero command.
    kernel.state.faults.insert(FaultFlags::ESTIMATOR_DIVERGED);
    let out_c = step(&mut kernel, &zero_cmd, &sensors, 0);
    assert!(
        all_motors_zero(&out_c),
        "disarmed kernel must ignore latched estimator faults"
    );

    // (d) corrupted estimator, saturating command, stale command_age.
    let out_d = step(&mut kernel, &saturating_cmd, &sensors, u32::MAX);
    assert!(
        all_motors_zero(&out_d),
        "disarmed kernel must ignore every combination of pathological input"
    );
}

// --- LLR-FLT-205 ------------------------------------------------------------

/// LLR-FLT-205: NUMERIC_ERROR latched on an armed kernel forces the
/// update path to emit the safe pattern. Arms the kernel normally,
/// takes one healthy-input cycle that produces a non-zero command,
/// latches `NUMERIC_ERROR` into `KernelState.faults`, and asserts the
/// next cycle returns zeros.
///
/// The fault is staged through the public `state.faults` surface — the
/// same path the disarmed-output and mode-atomicity tests use, and
/// exactly what `AviateKernelTrait::inject_fault` does under the
/// `test-hooks` feature (`state.faults.insert(fault)`). Witnessing the
/// inhibition contract this way keeps the test in the default
/// `cargo test` surface, so the source-mode evidence check exercises it
/// without a `test-hooks` build.
#[test]
fn numeric_fault_latched_inhibits_actuator_output() {
    let mut kernel = make_kernel();
    arm_kernel(&mut kernel);

    let sensors = make_valid_sensors();
    let live_cmd = Command {
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

    // Sanity: armed kernel with hover throttle produces non-zero output.
    let before = step(&mut kernel, &live_cmd, &sensors, 0);
    let any_nonzero = (0..4).any(|i| before.outputs[i].0.abs() > 1e-3);
    assert!(
        any_nonzero,
        "armed hover step must produce non-zero motor commands as a precondition"
    );

    kernel.state.faults.insert(FaultFlags::NUMERIC_ERROR);

    let after = step(&mut kernel, &live_cmd, &sensors, 0);
    assert!(
        all_motors_zero(&after),
        "NUMERIC_ERROR must inhibit actuator output: motors {:?}",
        after.outputs
    );
}

// --- LLR-MORPH-201 ----------------------------------------------------------

/// LLR-MORPH-201: `request_config_mode` is rejected with the expected
/// typed error for each pre-condition violation, and the kernel's
/// observable mode does not move when the request is refused. This pins
/// the atomicity contract at the boundary: either the request succeeds
/// and the mode advances cleanly, or it fails and nothing changes.
#[test]
fn config_mode_request_atomicity_under_pre_conditions() {
    let mut kernel = make_kernel();

    // Pre-arm: NotArmed error path.
    assert_eq!(
        kernel.request_config_mode(ConfigMode::Cruise),
        Err(TransitionError::NotArmed),
        "config-mode change must be rejected while not armed"
    );
    assert_eq!(
        kernel.state.mode,
        ConfigMode::Hover,
        "mode must not change on a rejected request"
    );

    arm_kernel(&mut kernel);
    assert_eq!(kernel.state.mode, ConfigMode::Hover);

    // Same-mode: AlreadyInMode error path.
    assert_eq!(
        kernel.request_config_mode(ConfigMode::Hover),
        Err(TransitionError::AlreadyInMode),
        "requesting the current mode must be rejected"
    );
    assert_eq!(
        kernel.state.mode,
        ConfigMode::Hover,
        "mode must not change on an already-in-mode request"
    );

    // Fault state blocks transitions.
    kernel.state.faults.insert(FaultFlags::ALL_IMU_FAILED);
    assert_eq!(
        kernel.request_config_mode(ConfigMode::Cruise),
        Err(TransitionError::InFaultState),
        "config-mode change must be rejected with critical faults present"
    );
    assert_eq!(
        kernel.state.mode,
        ConfigMode::Hover,
        "mode must not change while critical faults are latched"
    );
}

// --- LLR-EST-201 ------------------------------------------------------------

/// LLR-EST-201: from a deliberately off-truth attitude seed, the
/// estimator's reported roll/pitch converges to within 2.0° of the
/// stationary ground truth within 10.0 s of simulated time when fed
/// healthy IMU + mag samples. The seed is 5° roll, 5° pitch.
#[test]
fn ekf_cold_start_converges_under_static_imu() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    // Seed with a 5° roll / 5° pitch offset. The estimator should pull
    // attitude back toward the true level via accel + mag fusion.
    let five_deg = 5.0_f32.to_radians();
    let q_roll = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), five_deg as Scalar);
    let q_pitch = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), five_deg as Scalar);
    let seed = q_roll.mul(&q_pitch);
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        seed,
    );
    assert!(state.is_initialized());

    let sensors = make_valid_sensors();
    let dt = 0.01_f32;

    // 10.0 s of healthy static input — convergence window from
    // HLR-EST-201.
    for _ in 0..1000 {
        ekf.observe(&mut state, &sensors, None, dt as Scalar);
    }

    let est = state.get_estimate();
    let (roll, pitch, _yaw) = est.attitude.to_euler();
    assert!(
        roll.abs() < 2.0_f32.to_radians(),
        "roll error {:.4} rad ({:.2}°) exceeds 2.0° bound after 10 s",
        roll,
        roll.to_degrees()
    );
    assert!(
        pitch.abs() < 2.0_f32.to_radians(),
        "pitch error {:.4} rad ({:.2}°) exceeds 2.0° bound after 10 s",
        pitch,
        pitch.to_degrees()
    );
    assert!(
        !state.has_numeric_fault(),
        "static healthy input must not produce numeric faults"
    );
}

// --- LLR-EST-206 ------------------------------------------------------------

/// LLR-EST-206: when GNSS health drops to Lost, the predict-only path
/// continues running and horizontal position drift over a 5.0 s window
/// remains ≤ 2.0 m. The vehicle is in steady-state hover with all other
/// sensors healthy.
#[test]
fn ekf_gnss_dropout_bounded_dead_reckoning_drift() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    // Seed at origin and identity orientation (matches the stationary
    // ground-truth implied by `make_valid_sensors()`).
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );

    let healthy = make_valid_sensors();
    let dt = 0.01_f32;

    // 10.0 s warm-up with all-healthy sensors to settle covariance.
    for _ in 0..1000 {
        ekf.observe(&mut state, &healthy, None, dt as Scalar);
    }
    assert!(state.is_initialized());
    let pos_before = state.get_estimate().position_ned;

    // 5.0 s of dropout: GNSS health goes Lost. IMU, baro, mag remain Good.
    let mut dropped = make_valid_sensors();
    for g in dropped.gnss.iter_mut() {
        g.value.health = GnssHealth::Lost;
    }
    for _ in 0..500 {
        ekf.observe(&mut state, &dropped, None, dt as Scalar);
    }

    let pos_after = state.get_estimate().position_ned;
    let dn = pos_after[0].0 - pos_before[0].0;
    let de = pos_after[1].0 - pos_before[1].0;
    let horizontal_drift = (dn * dn + de * de).sqrt();
    assert!(
        horizontal_drift <= 2.0,
        "horizontal drift over 5 s GNSS dropout was {} m (>2.0 m bound)",
        horizontal_drift
    );
    assert!(
        !state.has_numeric_fault(),
        "predict-only over dropout window must not produce numeric faults"
    );
}

// --- LLR-EST-203 ------------------------------------------------------------

/// LLR-EST-203: with GNSS and baro both `Good`, the fused position
/// estimate converges to within 0.5 m horizontal / 0.3 m vertical of
/// ground truth over a 30 s window. Seeds the filter 2 m off-truth in
/// every axis and runs `observe()` at 100 Hz against stationary
/// origin-truth sensors (GNSS at `[0,0,0]`, baro altitude 0); the GNSS
/// position/velocity and baro-altitude fusion must pull the estimate
/// back inside the bound.
#[test]
fn ekf_gnss_baro_fusion_converges_position_within_bounds() {
    let ekf = Ekf::new(EkfConfig::default());
    let mut state = EkfState::default();

    // Seed 2 m north, 2 m east, 2 m up of the origin ground truth.
    state.init(
        Vector3::new(Meters(2.0), Meters(2.0), Meters(-2.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    assert!(state.is_initialized());

    let sensors = make_valid_sensors();
    let dt = 0.01_f32;

    // 30.0 s of healthy GNSS + baro fusion (the window from HLR-EST-203).
    for _ in 0..3000 {
        ekf.observe(&mut state, &sensors, None, dt as Scalar);
    }

    let pos = state.get_estimate().position_ned;
    let horizontal = (pos[0].0 * pos[0].0 + pos[1].0 * pos[1].0).sqrt();
    let vertical = pos[2].0.abs();
    assert!(
        horizontal <= 0.5,
        "horizontal position error {} m exceeds the 0.5 m bound after 30 s fusion",
        horizontal
    );
    assert!(
        vertical <= 0.3,
        "vertical position error {} m exceeds the 0.3 m bound after 30 s fusion",
        vertical
    );
    assert!(
        !state.has_numeric_fault(),
        "healthy GNSS + baro fusion must not produce numeric faults"
    );
}

// --- Altitude geofence flag (issue #66) -------------------------------------

/// The `update()` cycle drives the ALTITUDE_OK geofence flag from the
/// vehicle's estimated altitude against the configured band. Within the
/// band the flag latches set; tightening the band above the vehicle
/// clears it. Witnesses that `update_altitude` is wired into the loop.
#[test]
fn altitude_ok_flag_tracks_geofence_through_update() {
    let mut kernel = make_kernel();
    arm_kernel(&mut kernel);
    let sensors = make_valid_sensors();

    let cmd = Command {
        mode: ControlMode::AltitudeHold,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            altitude: Some(Meters(5.0)),
            collective_thrust: Normalized(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Pilot,
    };

    // Default band [0, 100] m contains the ~0 m estimate → OK.
    let _ = step(&mut kernel, &cmd, &sensors, 0);
    assert!(
        kernel
            .state
            .checks
            .in_flight
            .current
            .contains(InFlightFlags::ALTITUDE_OK),
        "altitude within the geofence must set ALTITUDE_OK"
    );

    // Raise the floor above the vehicle → measured altitude is now below
    // the band and the flag clears.
    kernel.cfg.limits.min_altitude = Meters(50.0);
    let _ = step(&mut kernel, &cmd, &sensors, 0);
    assert!(
        !kernel
            .state
            .checks
            .in_flight
            .current
            .contains(InFlightFlags::ALTITUDE_OK),
        "altitude below the geofence floor must clear ALTITUDE_OK"
    );
}
