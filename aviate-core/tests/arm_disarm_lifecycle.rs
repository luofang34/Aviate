//! Lifecycle behaviour for the guarded disarm and the emergency
//! terminate (issues #293, #294).
//!
//! Every case drives the real path: the phase latch is only ever set by
//! running `update()` over an estimate that shows the vehicle climbing,
//! never by writing the latch. Deleting the per-cycle latch fold, or the
//! precondition in `disarm()`, fails these tests.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_core::checks::PreArmFlags;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlLawV1, ControlMode, Setpoint,
};
use aviate_core::ekf::Ekf;
use aviate_core::flight_phase::FlightPhase;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorState, ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::sensor::SensorSet;
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth, SensorReading,
};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, NormalizedThrust, Pascals,
    RadiansPerSecond, Seconds,
};
use aviate_core::{ArmError, ChannelId, DisarmError, InitState};

type TestKernel = aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer>;

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn time_delta() -> TimeDelta {
    TimeDelta {
        dt_sec: Seconds(0.0025),
        tick_delta: 2500,
    }
}

fn valid_sensors() -> SensorSet {
    use aviate_core::types::Celsius;
    let ts = timestamp();

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

fn make_kernel() -> TestKernel {
    let mut kernel = aviate_core::kernel::builder::AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(QuadXMixer {
            timestamp_source: timestamp,
        })
        .sanitizer(Sanitizer)
        .pre_arm_required(
            PreArmFlags::IMU_HEALTHY
                | PreArmFlags::IMU_CONVERGED
                | PreArmFlags::EKF_CONVERGED
                | PreArmFlags::THROTTLE_LOW
                | PreArmFlags::CONFIG_VALID,
        )
        .mode_config(ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        })
        .build()
        .expect("checked construction must accept the default binding");
    kernel.state.checks.pre_arm.update_throttle(true);
    kernel
}

fn hover_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    }
}

fn place_at(kernel: &mut TestKernel, altitude_m: f32, climb_rate: f32) {
    kernel.state.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(-altitude_m)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(-climb_rate),
        ),
        Quaternion::IDENTITY,
    );
}

fn run_cycle(kernel: &mut TestKernel) {
    let sensors = valid_sensors();
    let cmd = hover_command();
    let actuator_state = ActuatorState::default();
    kernel.update(
        ChannelId::PRIMARY,
        time_delta(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );
}

/// Bring a fresh kernel to `Armed` on the ground.
fn armed_on_ground() -> TestKernel {
    let mut kernel = make_kernel();
    let sensors = valid_sensors();
    place_at(&mut kernel, 0.0, 0.0);
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
    assert!(kernel.is_ready(), "kernel must reach Ready");
    kernel.arm().expect("a ready, fault-free kernel must arm");
    assert_eq!(kernel.state.init_state, InitState::Armed);
    kernel
}

/// Fly the kernel up to `altitude_m` and confirm the latch observed it.
fn fly_to(kernel: &mut TestKernel, altitude_m: f32) {
    place_at(kernel, altitude_m, 1.0);
    run_cycle(kernel);
    assert_eq!(
        kernel.state.flight_phase.phase(),
        FlightPhase::Airborne,
        "climbing to {altitude_m} m must latch airborne"
    );
}

// === Issue #293: an ordinary disarm must not drop a flying aircraft ===

#[test]
fn disarm_on_the_ground_is_accepted() {
    let mut kernel = armed_on_ground();
    run_cycle(&mut kernel);
    assert_eq!(kernel.state.flight_phase.phase(), FlightPhase::OnGround);
    assert_eq!(kernel.disarm(), Ok(()));
    assert_eq!(kernel.state.init_state, InitState::Disarmed);
}

#[test]
fn disarm_in_flight_is_refused() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);

    assert_eq!(kernel.disarm(), Err(DisarmError::Airborne));
}

#[test]
fn a_refused_disarm_changes_nothing() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    let law_before = kernel.state.control_law;

    assert!(kernel.disarm().is_err());

    // The whole point of the refusal is that the aircraft keeps flying.
    assert_eq!(kernel.state.init_state, InitState::Armed);
    assert_eq!(kernel.state.control_law, law_before);
    assert_eq!(kernel.state.flight_phase.phase(), FlightPhase::Airborne);
}

#[test]
fn repeated_refused_disarms_never_succeed_by_attrition() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    for _ in 0..50 {
        assert_eq!(kernel.disarm(), Err(DisarmError::Airborne));
        run_cycle(&mut kernel);
    }
    assert_eq!(kernel.state.init_state, InitState::Armed);
}

#[test]
fn disarm_is_accepted_again_after_landing() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    assert!(kernel.disarm().is_err());

    // Settle on the ground: the latch needs the landed condition held
    // continuously, so one cycle at zero is not enough.
    place_at(&mut kernel, 0.0, 0.0);
    for _ in 0..400 {
        run_cycle(&mut kernel);
    }
    assert_eq!(kernel.state.flight_phase.phase(), FlightPhase::OnGround);
    assert_eq!(kernel.disarm(), Ok(()));
}

// === Issue #293: the emergency path stays available ===

#[test]
fn terminate_cuts_outputs_in_flight() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    assert!(kernel.disarm().is_err(), "ordinary disarm refused");

    kernel.terminate();

    assert_eq!(kernel.state.init_state, InitState::Disarmed);
    assert_eq!(kernel.state.control_law, ControlLawV1::Backup);
    assert_eq!(
        kernel.state.flight_phase.phase(),
        FlightPhase::OnGround,
        "the flight period ended"
    );
}

#[test]
fn terminate_and_disarm_leave_identical_state_on_the_ground() {
    let mut a = armed_on_ground();
    run_cycle(&mut a);
    a.disarm().expect("grounded disarm accepted");

    let mut b = armed_on_ground();
    run_cycle(&mut b);
    b.terminate();

    // The two paths differ only in whether they are refused, never in
    // what they do once they act.
    assert_eq!(a.state.init_state, b.state.init_state);
    assert_eq!(a.state.control_law, b.state.control_law);
    assert_eq!(a.state.flight_phase, b.state.flight_phase);
}

// === Issue #294: the path back to armable, and observable refusals ===

#[test]
fn arm_after_a_grounded_disarm_succeeds_once_pre_arm_is_resatisfied() {
    let mut kernel = armed_on_ground();
    run_cycle(&mut kernel);
    kernel.disarm().expect("grounded disarm accepted");

    // Immediately after disarm the sample counts have been reset, so the
    // gate genuinely is unsatisfied — the refusal is real, not a bug.
    assert_eq!(kernel.arm(), Err(ArmError::NotReady));

    let sensors = valid_sensors();
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
    assert!(kernel.is_ready(), "the path back to Ready must terminate");
    assert_eq!(kernel.arm(), Ok(()));
}

#[test]
fn a_refused_arm_names_the_gate_and_the_missing_conditions() {
    let mut kernel = make_kernel();
    // Nothing has converged yet.
    let error = kernel.arm().expect_err("a cold kernel must not arm");
    assert_eq!(error, ArmError::NotReady);

    let missing = kernel.state.checks.pre_arm.missing();
    assert!(
        !missing.is_empty(),
        "a NotReady refusal must be able to say what is outstanding"
    );
    assert!(missing.contains(PreArmFlags::EKF_CONVERGED));
}

#[test]
fn arming_while_armed_is_refused_as_already_armed() {
    let mut kernel = armed_on_ground();
    assert_eq!(kernel.arm(), Err(ArmError::AlreadyArmed));
}

#[test]
fn re_arming_recaptures_the_ground_datum_at_the_new_altitude() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    kernel.terminate();

    // Sitting on a ledge 20 m above where it first armed. A datum that
    // survived the flight period would read this as airborne.
    place_at(&mut kernel, 20.0, 0.0);
    let sensors = valid_sensors();
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
    kernel.arm().expect("re-arm on the ledge");
    run_cycle(&mut kernel);
    assert_eq!(
        kernel.state.flight_phase.phase(),
        FlightPhase::OnGround,
        "height is measured from the new arm point"
    );
    assert_eq!(kernel.disarm(), Ok(()));
}

#[test]
fn ground_reset_clears_a_stale_flight_phase() {
    let mut kernel = armed_on_ground();
    fly_to(&mut kernel, 20.0);
    kernel.terminate();
    kernel.ground_reset();
    assert_eq!(kernel.state.flight_phase.phase(), FlightPhase::OnGround);
}
