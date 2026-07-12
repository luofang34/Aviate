//! Slew limiter integration tests (DRQ-FLT-001 / DRQ-MORPH-001).
//!
//! Drive the slew limiter through the full `kernel.update()` path
//! (public surface only) and verify that:
//!   - `slew_limit_per_cycle[i] == 0` reproduces the unconstrained
//!     baseline (existing airframe behavior).
//!   - a positive limit caps the per-cycle delta on actuator outputs
//!     even when the controller would command a large jump.
//!   - the limited output converges to the un-slewed value over
//!     multiple cycles (no permanent loss of authority).
//!
//! The standalone helper unit-tests in `aviate-core/src/kernel/slew.rs`
//! cover the math in isolation; this file pins the wiring through
//! `update()` and the `ResolvedKernelConfig.slew_limit_per_cycle`
//! field surface.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::checks::PreArmFlags;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::ekf::Ekf;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer, MAX_ACTUATORS};
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth,
    SensorReading, SensorSet,
};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Normalized, Pascals,
    RadiansPerSecond, Seconds,
};
use aviate_core::{ChannelId, InitState};

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

fn make_kernel_with_slew(
    slew: [Normalized; MAX_ACTUATORS],
) -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
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
    let mut kernel = aviate_core::kernel::builder::AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(mixer)
        .sanitizer(Sanitizer)
        .pre_arm_required(required)
        .config(aviate_core::kernel::config::ResolvedKernelConfig {
            mode_config,
            slew_limit_per_cycle: slew,
            ..Default::default()
        })
        .build()
        .expect("checked construction must accept the default binding");
    kernel.state.checks.pre_arm.update_throttle(true);
    kernel
}

fn make_kernel() -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
    make_kernel_with_slew(
        aviate_core::kernel::config::ResolvedKernelConfig::default().slew_limit_per_cycle,
    )
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

fn full_throttle_cmd() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: Normalized(1.0),
            attitude: Some(Quaternion::IDENTITY),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    }
}

fn step(
    kernel: &mut aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer>,
    cmd: &Command,
    sensors: &SensorSet,
) -> [Normalized; MAX_ACTUATORS] {
    let actuator_state = kernel.state.actuator_state.clone();
    let result = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        sensors,
        cmd,
        0,
        &actuator_state,
        None,
    );
    result.actuator.outputs
}

/// DRQ-FLT-001 wiring: zero slew_limit_per_cycle leaves the per-cycle
/// outputs identical to the unconstrained baseline. Sanity check that
/// the default config (all zeros) does not silently alter behavior.
#[test]
fn zero_limit_preserves_baseline_outputs() {
    let mut kernel_a = make_kernel();
    // Kernel B explicitly sets the slew limit to zero (same as
    // default, but pinning the contract).
    let mut kernel_b = make_kernel_with_slew([Normalized(0.0); MAX_ACTUATORS]);
    arm_kernel(&mut kernel_a);
    arm_kernel(&mut kernel_b);

    let sensors = make_valid_sensors();
    let cmd = full_throttle_cmd();

    for _ in 0..5 {
        let out_a = step(&mut kernel_a, &cmd, &sensors);
        let out_b = step(&mut kernel_b, &cmd, &sensors);
        for (i, (a, b)) in out_a.iter().zip(out_b.iter()).enumerate().take(4) {
            assert!(
                (a.0 - b.0).abs() < 1e-6,
                "ch {}: zero-limit kernel B {} diverges from default kernel A {}",
                i,
                b.0,
                a.0,
            );
        }
    }
}

/// DRQ-FLT-001: with a positive slew limit, the first-cycle delta on
/// each actuator channel cannot exceed the configured per-cycle limit.
/// Drives a freshly-armed kernel (commanded=0 from arm()) with full
/// throttle and asserts the first-cycle output is bounded by the
/// slew limit, not the controller's larger target.
#[test]
fn positive_limit_caps_first_cycle_delta() {
    let limit = 0.05;
    let mut kernel = make_kernel_with_slew([Normalized(limit); MAX_ACTUATORS]);
    arm_kernel(&mut kernel);

    let sensors = make_valid_sensors();
    let cmd = full_throttle_cmd();

    let outputs = step(&mut kernel, &cmd, &sensors);
    for (i, n) in outputs.iter().enumerate().take(4) {
        assert!(
            n.0 <= limit + 1e-6,
            "ch {}: first-cycle output {} exceeds slew limit {}",
            i,
            n.0,
            limit,
        );
        assert!(
            n.0 >= -limit - 1e-6,
            "ch {}: first-cycle output {} below -slew limit",
            i,
            n.0,
        );
    }
}

/// DRQ-FLT-001 / DRQ-MORPH-001: the slew limiter slows but does not
/// suppress an output change — over N cycles the output converges
/// to the un-slewed steady-state value. Pins that the limiter has
/// no permanent authority cost.
#[test]
fn slew_limited_output_converges_over_cycles() {
    let limit = 0.05;
    let mut limited = make_kernel_with_slew([Normalized(limit); MAX_ACTUATORS]);
    let mut baseline = make_kernel();
    arm_kernel(&mut limited);
    arm_kernel(&mut baseline);

    let sensors = make_valid_sensors();
    let cmd = full_throttle_cmd();

    // 30 cycles is comfortably more than the worst-case 1.0/0.05 = 20
    // cycles needed for the limited path to catch up to any
    // saturating target.
    let mut limited_final = [Normalized(0.0); MAX_ACTUATORS];
    let mut baseline_final = [Normalized(0.0); MAX_ACTUATORS];
    for _ in 0..30 {
        limited_final = step(&mut limited, &cmd, &sensors);
        baseline_final = step(&mut baseline, &cmd, &sensors);
    }

    for (i, (lim, base)) in limited_final
        .iter()
        .zip(baseline_final.iter())
        .enumerate()
        .take(4)
    {
        assert!(
            (lim.0 - base.0).abs() < 1e-4,
            "ch {}: limited final {} != baseline final {} after convergence",
            i,
            lim.0,
            base.0,
        );
    }
}
