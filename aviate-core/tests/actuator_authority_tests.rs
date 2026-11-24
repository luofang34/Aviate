//! TST-CTL-211: the kernel gives the actuator authority of each cycle to
//! the controller.
//!
//! A fixed-wing kernel must hand the discrete of the sensor set to its
//! law in each cycle. A multirotor kernel must not change with it.

#![allow(clippy::expect_used, clippy::panic)]
#![forbid(unsafe_code)]

#[path = "fixtures/fixed_wing.rs"]
mod fixed_wing_fixture;

use aviate_core::checks::PreArmFlags;
use aviate_core::control::fixed_wing::{FixedWingController, RateLoopState};
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{
    Command, CommandSource, ConfigMode, ControlMode, Setpoint, VehicleController,
};
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{MixerGeometry, ResolvedKernelConfig};
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::fixed_wing::{neutral_outputs, FixedWingMixer};
use aviate_core::mixer::{Mixer, ModeConfig, QuadXMixer, Sanitizer, MAX_ACTUATORS};
use aviate_core::sensor::{
    ActuatorAuthority, AirData, AirspeedData, BaroData, GnssData, GnssFix, GnssHealth, ImuData,
    MagData, SensorHealth, SensorReading, SensorSet,
};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, NormalizedThrust, Pascals,
    RadiansPerSecond, Seconds,
};
use aviate_core::ChannelId;

/// Cycles of 10 ms in each part of a test.
const CYCLES: u32 = 100;

fn ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn reading<T>(value: T) -> SensorReading<T> {
    SensorReading {
        value,
        valid: true,
        source_id: 0,
        timestamp: ts(),
        health: SensorHealth::Good,
    }
}

/// A vehicle at rest that turns at a constant body rate, with a probe
/// that reads 55 m/s: the rate loops of a law that asks for no rate see
/// a persistent error at a flying airspeed.
fn sensors(actuator_authority: ActuatorAuthority) -> SensorSet {
    SensorSet {
        imus: [
            reading(ImuData {
                accel: [
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(-9.81),
                ],
                gyro: [
                    RadiansPerSecond(0.05),
                    RadiansPerSecond(-0.03),
                    RadiansPerSecond(0.02),
                ],
            }),
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [
            reading(GnssData {
                position_ned: [Meters(0.0); 3],
                velocity_ned: [MetersPerSecond(0.0); 3],
                fix: GnssFix::ThreeD,
                health: GnssHealth::Good,
            }),
            SensorReading::default(),
        ],
        mags: [
            reading(MagData {
                field_ut: [Microtesla(20.0), Microtesla(0.0), Microtesla(40.0)],
            }),
            SensorReading::default(),
        ],
        baros: [
            reading(BaroData {
                altitude: Some(Meters(0.0)),
                air: AirData::default(),
            }),
            SensorReading::default(),
        ],
        airspeeds: [
            reading(AirspeedData {
                air: AirData {
                    dynamic_pressure: Some(Pascals(0.5 * 1.225 * 55.0 * 55.0)),
                    ..AirData::default()
                },
            }),
            SensorReading::default(),
        ],
        geometry: None,
        geomagnetic: None,
        actuator_authority,
    }
}

fn level_command(sequence: u32) -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.5),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence,
        source: CommandSource::Pilot,
    }
}

fn step<V: VehicleController, M: Mixer>(
    kernel: &mut DefaultAviateKernel<V, M>,
    sequence: u32,
    authority: ActuatorAuthority,
) -> [u32; MAX_ACTUATORS] {
    let actuator_state = kernel.state.actuator_state.clone();
    let result = kernel.update(
        ChannelId::PRIMARY,
        TimeDelta {
            dt_sec: Seconds(0.01),
            tick_delta: 10_000,
        },
        &sensors(authority),
        &level_command(sequence),
        0,
        &actuator_state,
        None,
    );
    result.actuator.outputs.map(|output| output.0.to_bits())
}

/// Make the kernel ready and arm it.
fn arm<V: VehicleController, M: Mixer>(kernel: &mut DefaultAviateKernel<V, M>) {
    kernel.state.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    kernel.state.checks.pre_arm.update_throttle(true);
    for _ in 0..150 {
        kernel.init_step(&sensors(ActuatorAuthority::Available), ts());
        if kernel.is_ready() {
            break;
        }
    }
    assert!(kernel.is_ready(), "kernel did not become ready");
    assert!(kernel.arm().is_ok(), "kernel did not arm");
}

const TEST_PRE_ARM: PreArmFlags = PreArmFlags::IMU_HEALTHY
    .union(PreArmFlags::IMU_CONVERGED)
    .union(PreArmFlags::EKF_CONVERGED)
    .union(PreArmFlags::THROTTLE_LOW)
    .union(PreArmFlags::CONFIG_VALID);

fn fixed_wing_kernel() -> DefaultAviateKernel<FixedWingController, FixedWingMixer> {
    let params = fixed_wing_fixture::reference_params();
    let mut kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(FixedWingController::new(params).expect("valid block"))
        .mixer(FixedWingMixer {
            timestamp_source: ts,
        })
        .sanitizer(Sanitizer)
        .pre_arm_required(TEST_PRE_ARM)
        .config(ResolvedKernelConfig {
            mode_config: ModeConfig {
                mode: ConfigMode::Cruise,
                groups: &[],
            },
            mixer_geometry: MixerGeometry::FixedWing,
            safe_output: neutral_outputs(),
            fixed_wing: Some(params),
            ..Default::default()
        })
        .build()
        .expect("the controller copies the block that the configuration holds");
    arm(&mut kernel);
    kernel
}

fn multirotor_kernel() -> DefaultAviateKernel<MultirotorController, QuadXMixer> {
    let mut kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(QuadXMixer {
            timestamp_source: ts,
        })
        .sanitizer(Sanitizer)
        .pre_arm_required(TEST_PRE_ARM)
        .config(ResolvedKernelConfig {
            mode_config: ModeConfig {
                mode: ConfigMode::Hover,
                groups: &[],
            },
            ..Default::default()
        })
        .build()
        .expect("the default binding is valid");
    arm(&mut kernel);
    kernel
}

#[test]
fn the_kernel_gives_the_authority_of_each_cycle_to_the_fixed_wing_law() {
    let mut kernel = fixed_wing_kernel();
    let empty = RateLoopState::default();

    for sequence in 0..CYCLES {
        let _ = step(&mut kernel, sequence, ActuatorAuthority::Available);
    }
    let law = &kernel.state.controller;
    assert_eq!(law.actuator_authority, ActuatorAuthority::Available);
    assert_ne!(law.rates, empty, "the rate error did not fill the memory");

    for sequence in CYCLES..2 * CYCLES {
        let _ = step(&mut kernel, sequence, ActuatorAuthority::Absent);
        let law = &kernel.state.controller;
        assert_eq!(law.actuator_authority, ActuatorAuthority::Absent);
        assert_eq!(law.rates, empty, "cycle {sequence}");
    }

    let _ = step(&mut kernel, 2 * CYCLES, ActuatorAuthority::Available);
    let law = &kernel.state.controller;
    assert_eq!(law.actuator_authority, ActuatorAuthority::Available);
    assert_ne!(law.rates, empty, "the law did not integrate again");
}

#[test]
fn a_fixed_wing_kernel_without_authority_still_gives_its_command() {
    let mut engaged = fixed_wing_kernel();
    let mut released = fixed_wing_kernel();
    // The first cycle starts from the same empty memory in both kernels,
    // so the command of the law is the same with and without authority.
    assert_eq!(
        step(&mut released, 0, ActuatorAuthority::Absent),
        step(&mut engaged, 0, ActuatorAuthority::Available)
    );
    let neutral = neutral_outputs().map(|output| output.0.to_bits());
    assert_ne!(step(&mut released, 1, ActuatorAuthority::Absent), neutral);
}

#[test]
fn a_multirotor_kernel_does_not_change_with_the_authority() {
    let mut engaged = multirotor_kernel();
    let mut released = multirotor_kernel();
    let mut moved = false;
    let mut first = None;
    for sequence in 0..CYCLES {
        let expected = step(&mut engaged, sequence, ActuatorAuthority::Available);
        let outputs = step(&mut released, sequence, ActuatorAuthority::Absent);
        assert_eq!(outputs, expected, "cycle {sequence}");
        moved |= *first.get_or_insert(outputs) != outputs;
    }
    assert!(
        moved,
        "the outputs never changed: the comparison proves nothing"
    );
}
