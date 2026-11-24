//! TST-STATE-110: a clone of a kernel is a snapshot of the kernel.
//!
//! A simulator makes a snapshot of a kernel with `Clone` and continues
//! from the snapshot later. These tests step a kernel, make a clone,
//! and step the two with the same inputs. The actuator outputs and the
//! canonical state bytes must be equal bit for bit in each cycle.

#![allow(clippy::expect_used, clippy::panic)]
#![forbid(unsafe_code)]

#[path = "fixtures/fixed_wing.rs"]
mod fixed_wing_fixture;

use aviate_core::checks::PreArmFlags;
use aviate_core::control::envelope::SimpleEnvelopeProtector;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::runtime::NoControllerState;
use aviate_core::control::spacecraft::SpacecraftController;
use aviate_core::control::vtol::VtolController;
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlMode, Setpoint, VehicleControlMode,
    VehicleController,
};
use aviate_core::ekf::{Ekf, Estimator};
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{MixerGeometry, ResolvedKernelConfig};
use aviate_core::kernel::pipeline::KernelPipeline;
use aviate_core::kernel::state::KernelState;
use aviate_core::kernel::{AviateKernelImpl, DefaultAviateKernel};
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::fixed_wing::{neutral_outputs, FixedWingMixer};
use aviate_core::mixer::spacecraft::SpacecraftMixer;
use aviate_core::mixer::{
    ActuatorCmd, Mixer, ModeConfig, QuadXMixer, QuadXMixerReversedSpin, QuadXMixerX500, Sanitizer,
    MAX_ACTUATORS,
};
use aviate_core::replicable::Replicable;
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth,
    SensorReading, SensorSet,
};
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, NormalizedSigned,
    NormalizedThrust, Pascals, RadiansPerSecond, Seconds,
};
use aviate_core::ChannelId;

/// Cycles before the snapshot.
const WARM_CYCLES: u32 = 40;
/// Cycles after the snapshot.
const COMPARE_CYCLES: u32 = 60;
/// Size of the buffer for the canonical state bytes.
const STATE_BUF: usize = 4096;

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

/// Sensor samples that change with `cycle`, so that the estimator and
/// the controller integrators hold a different state in each cycle.
fn sensors(cycle: u32) -> SensorSet {
    let phase = cycle as f32 * 0.21;
    let wave = triangle_wave(phase);
    SensorSet {
        imus: [
            reading(ImuData {
                accel: [
                    MetersPerSecondSquared(0.3 * wave),
                    MetersPerSecondSquared(-0.2 * wave),
                    MetersPerSecondSquared(-9.81),
                ],
                gyro: [
                    RadiansPerSecond(0.05 * wave),
                    RadiansPerSecond(-0.03 * wave),
                    RadiansPerSecond(0.02 * wave),
                ],
            }),
            SensorReading::default(),
            SensorReading::default(),
        ],
        gnss: [
            reading(GnssData {
                position_ned: [Meters(0.4 * wave), Meters(0.0), Meters(0.0)],
                velocity_ned: [MetersPerSecond(0.1 * wave); 3],
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
                altitude: Some(Meters(0.2 * wave)),
                air: AirData {
                    static_pressure: Some(Pascals(101_325.0)),
                    dynamic_pressure: None,
                    total_pressure: None,
                    temperature: Some(Celsius(20.0)),
                    indicated_airspeed: None,
                    true_airspeed: None,
                },
            }),
            SensorReading::default(),
        ],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
        geomagnetic: None,
    }
}

/// A bounded wave: the tests need a changing input, not an accurate
/// sine.
fn triangle_wave(phase: f32) -> f32 {
    let turn = phase % 2.0;
    if turn < 1.0 {
        2.0 * turn - 1.0
    } else {
        3.0 - 2.0 * turn
    }
}

fn command(cycle: u32) -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            attitude: Some(Quaternion::IDENTITY),
            collective_thrust: NormalizedThrust(0.4 + 0.002 * cycle as f32),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: cycle,
        source: CommandSource::Pilot,
    }
}

fn step<V: VehicleController, M: Mixer>(
    kernel: &mut DefaultAviateKernel<V, M>,
    cycle: u32,
) -> [u32; MAX_ACTUATORS] {
    let actuator_state = kernel.state.actuator_state.clone();
    let result = kernel.update(
        ChannelId::PRIMARY,
        TimeDelta {
            dt_sec: Seconds(0.01),
            tick_delta: 10_000,
        },
        &sensors(cycle),
        &command(cycle),
        0,
        &actuator_state,
        None,
    );
    output_bits(&result.actuator)
}

fn output_bits(cmd: &ActuatorCmd) -> [u32; MAX_ACTUATORS] {
    cmd.outputs.map(|output| output.0.to_bits())
}

/// Assert that two kernels agree on algorithm, configuration and the
/// canonical bytes of the full state.
fn assert_same_canonical_state<V: VehicleController, M: Mixer>(
    left: &DefaultAviateKernel<V, M>,
    right: &DefaultAviateKernel<V, M>,
    cycle: u32,
) {
    let len =
        <KernelState<<Ekf as Estimator>::RuntimeState, V::RuntimeState> as Replicable>::ENCODED_LEN;
    assert!(len <= STATE_BUF, "state buffer too small for {len} bytes");
    let mut left_buf = [0u8; STATE_BUF];
    let mut right_buf = [0u8; STATE_BUF];
    let left_snap = left.project_for_cross_channel(0, ChannelId::PRIMARY, &mut left_buf);
    let right_snap = right.project_for_cross_channel(0, ChannelId::PRIMARY, &mut right_buf);
    assert_eq!(left_snap.state_bytes.len(), len);
    assert!(
        left_snap.agrees_with(&right_snap),
        "canonical state differs at cycle {cycle}"
    );
}

/// Make the kernel ready, arm it and step it to the snapshot cycle.
fn warm_up<V: VehicleController, M: Mixer>(kernel: &mut DefaultAviateKernel<V, M>) {
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
        kernel.init_step(&sensors(0), ts());
        if kernel.is_ready() {
            break;
        }
    }
    assert!(kernel.is_ready(), "kernel did not become ready");
    assert!(kernel.arm().is_ok(), "kernel did not arm");
    for cycle in 0..WARM_CYCLES {
        step(kernel, cycle);
    }
}

/// Step a kernel and its clone in lockstep, then rewind the kernel to
/// the snapshot and replay the same cycles.
fn assert_clone_is_snapshot<V, M>(mut kernel: DefaultAviateKernel<V, M>)
where
    V: VehicleController + Clone,
    M: Mixer + Clone,
{
    warm_up(&mut kernel);
    let snapshot = kernel.clone();
    let mut twin = snapshot.clone();
    assert_same_canonical_state(&kernel, &twin, WARM_CYCLES);

    let mut first_pass = [[0u32; MAX_ACTUATORS]; COMPARE_CYCLES as usize];
    for (slot, cycle) in first_pass.iter_mut().zip(WARM_CYCLES..) {
        *slot = step(&mut kernel, cycle);
        assert_eq!(*slot, step(&mut twin, cycle), "outputs at cycle {cycle}");
        assert_same_canonical_state(&kernel, &twin, cycle);
    }
    assert!(
        first_pass
            .iter()
            .any(|outputs| Some(outputs) != first_pass.first()),
        "the outputs never changed: the comparison proves nothing"
    );

    kernel = snapshot.clone();
    assert_same_canonical_state(&kernel, &snapshot, WARM_CYCLES);
    for (expected, cycle) in first_pass.iter().zip(WARM_CYCLES..) {
        assert_eq!(
            *expected,
            step(&mut kernel, cycle),
            "replay at cycle {cycle}"
        );
    }
    assert_same_canonical_state(&kernel, &twin, WARM_CYCLES + COMPARE_CYCLES);
}

const TEST_PRE_ARM: PreArmFlags = PreArmFlags::IMU_HEALTHY
    .union(PreArmFlags::IMU_CONVERGED)
    .union(PreArmFlags::EKF_CONVERGED)
    .union(PreArmFlags::THROTTLE_LOW)
    .union(PreArmFlags::CONFIG_VALID);

#[test]
fn multirotor_kernel_clone_continues_bit_identically() {
    let kernel = AviateKernelBuilder::new()
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
    assert_clone_is_snapshot(kernel);
}

#[test]
fn fixed_wing_kernel_clone_continues_bit_identically() {
    let params = fixed_wing_fixture::reference_params();
    let kernel = AviateKernelBuilder::new()
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
    assert_clone_is_snapshot(kernel);
}

/// Assert that a clone of a mixer gives the outputs of the mixer.
fn assert_mixer_clone_mixes_identically<M: Mixer + Clone>(mixer: &M) {
    let twin = mixer.clone();
    for step in 0..8u8 {
        let level = f32::from(step) / 8.0;
        let axis = AxisCommand {
            roll: NormalizedSigned(level - 0.5),
            pitch: NormalizedSigned(0.3 - level),
            yaw: NormalizedSigned(0.25 * level),
            collective: NormalizedThrust(level),
        };
        assert_eq!(
            output_bits(&mixer.mix(&axis)),
            output_bits(&twin.mix(&axis)),
            "mixer clone differs at step {step}"
        );
    }
}

#[test]
fn mixer_clones_mix_bit_identically() {
    assert_mixer_clone_mixes_identically(&QuadXMixer {
        timestamp_source: ts,
    });
    assert_mixer_clone_mixes_identically(&QuadXMixerX500 {
        timestamp_source: ts,
    });
    assert_mixer_clone_mixes_identically(&QuadXMixerReversedSpin {
        timestamp_source: ts,
    });
    assert_mixer_clone_mixes_identically(&FixedWingMixer {
        timestamp_source: ts,
    });
    let mut spacecraft = SpacecraftMixer::new(ts);
    spacecraft.velocity_error_body_mps = [MetersPerSecond(0.4); 3];
    assert_mixer_clone_mixes_identically(&spacecraft);
}

#[test]
fn stateless_component_clones_step_identically() {
    let estimate = StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-10.0)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags: StateValidFlags::all(),
    };
    let limits = ResolvedKernelConfig::default().limits;
    let cmd = command(7);
    let axis_of = |controller: &VtolController| {
        controller.step(
            &mut NoControllerState,
            &estimate,
            &cmd,
            &VehicleControlMode::from_control_mode(cmd.mode),
            ConfigMode::Cruise,
            &limits,
        )
    };
    let vtol = VtolController;
    assert_eq!(axis_of(&vtol), axis_of(&vtol.clone()));

    let pipeline = KernelPipeline::new(Ekf::default(), vtol, SpacecraftMixer::new(ts), Sanitizer);
    assert_eq!(
        pipeline.algorithm_identity_hash(),
        pipeline.clone().algorithm_identity_hash()
    );
}

fn assert_clone<T: Clone>() {}

/// A field that is not `Clone` in one of these types stops this test at
/// compile time, with the name of the type in the error.
#[test]
fn kernel_object_graph_is_clone() {
    assert_clone::<DefaultAviateKernel<MultirotorController, QuadXMixer>>();
    assert_clone::<DefaultAviateKernel<MultirotorController, QuadXMixerX500>>();
    assert_clone::<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>>();
    assert_clone::<DefaultAviateKernel<FixedWingController, FixedWingMixer>>();
    assert_clone::<DefaultAviateKernel<VtolController, QuadXMixer>>();
    assert_clone::<DefaultAviateKernel<SpacecraftController, SpacecraftMixer>>();
    assert_clone::<AviateKernelImpl<Ekf, MultirotorController, QuadXMixer, Sanitizer>>();
    assert_clone::<KernelPipeline<Ekf, FixedWingController, FixedWingMixer, Sanitizer>>();
    assert_clone::<SimpleEnvelopeProtector>();
    assert_clone::<Sanitizer>();
}
