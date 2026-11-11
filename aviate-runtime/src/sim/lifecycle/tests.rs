//! World resets and host stalls, driven through `step_sim()`.

#![allow(clippy::expect_used)]

use aviate_core::checks::DegradationReason;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::fault::FaultFlags;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::kernel::config::{FixedWingParams, MixerGeometry, ResolvedKernelConfig};
use aviate_core::mixer::fixed_wing::FixedWingMixer;
use aviate_core::mixer::{QuadXMixer, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{MetersPerSecond, NormalizedThrust, Radians, Seconds};
use aviate_core::DefaultAviateKernel;
use aviate_core::InitState;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SimImuData, SimMagData, SimSensorPacket, SitlIO, XilConfig, XilNetConfig};

use super::super::{SitlRunner, SitlTime};

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn runner() -> SitlRunner<MultirotorController, QuadXMixer> {
    let kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(QuadXMixer {
            timestamp_source: timestamp,
        })
        .sanitizer(Sanitizer)
        .build()
        .expect("valid test kernel");
    SitlRunner::new(transport(), board_hal(), kernel)
}

/// The faults latched until ground reset. The sensor-health bits follow
/// live sensor health instead (these samples carry no baro or GNSS).
const LATCHED: FaultFlags = FaultFlags::ATTITUDE_LOST
    .union(FaultFlags::IMU_DEGRADED)
    .union(FaultFlags::POSITION_LOST)
    .union(FaultFlags::VELOCITY_LOST)
    .union(FaultFlags::BARO_DEGRADED)
    .union(FaultFlags::ENVELOPE_VIOLATION)
    .union(FaultFlags::RC_LOST)
    .union(FaultFlags::TIMING_PERSISTENT);

fn still_sample(t_us: u64, generation: u32) -> SimSensorPacket {
    SimSensorPacket::new(t_us)
        .with_imu(SimImuData {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0; 3],
            temperature: None,
        })
        .with_mag(SimMagData {
            field_ut: [20.0, 0.0, 45.0],
        })
        .with_reset_generation(generation)
}

/// A simulator that respawns the vehicle starts that vehicle's next
/// power cycle. A fault latched in the old world must not refuse every
/// arm in the new one, and the simulator's epoch change is the only
/// signal a board without a restart path has.
#[test]
fn a_new_world_generation_clears_a_latched_fault() {
    let mut runner = runner();
    for t in 1..=5u64 {
        runner
            .transport
            .feed_sensor_packet(&still_sample(t * 1_000, 1));
        runner.step_sim(1_000);
    }
    runner
        .kernel
        .handle_degradation(DegradationReason::AttitudeLost, timestamp());
    assert!(runner
        .kernel
        .state
        .faults
        .contains(FaultFlags::ATTITUDE_LOST));

    // More samples from the same world keep it.
    runner.transport.feed_sensor_packet(&still_sample(6_000, 1));
    runner.step_sim(1_000);
    assert!(runner
        .kernel
        .state
        .faults
        .contains(FaultFlags::ATTITUDE_LOST));

    // The first sample of a reset world clears it and restarts init.
    runner.transport.feed_sensor_packet(&still_sample(1_000, 2));
    runner.step_sim(1_000);
    assert!(
        !runner.kernel.state.faults.intersects(LATCHED),
        "{:?}",
        runner.kernel.state.faults
    );
    assert_ne!(runner.kernel.state.init_state, InitState::Ready);
    assert!(
        runner.sensor_cache.imu.is_some(),
        "the new world's first sample is used, not dropped"
    );
}

#[test]
fn the_first_world_a_simulator_states_is_not_a_reset() {
    let mut runner = runner();
    runner.transport.feed_sensor_packet(&still_sample(1_000, 7));
    runner.step_sim(1_000);
    runner
        .kernel
        .handle_degradation(DegradationReason::AttitudeLost, timestamp());
    runner.transport.feed_sensor_packet(&still_sample(2_000, 7));
    runner.step_sim(1_000);
    assert!(runner
        .kernel
        .state
        .faults
        .contains(FaultFlags::ATTITUDE_LOST));
}

/// In SITL the kernel cycle is clocked by simulation time. A host that
/// stalls between simulator steps — a loaded CI machine — must not look
/// like a missed deadline: no timing violation is recorded and nothing
/// latches.
#[test]
fn a_stalled_host_with_steady_sim_steps_latches_nothing() {
    let mut runner = runner();
    for t in 1..=200u64 {
        if t % 50 == 0 {
            // A deliberate host stall far longer than any cycle deadline.
            std::thread::sleep(std::time::Duration::from_millis(30));
        }
        runner
            .transport
            .feed_sensor_packet(&still_sample(t * 1_000, 1));
        runner.step_sim(1_000);
    }
    let stats = &runner.kernel.state.timing_stats;
    assert_eq!(stats.deadline_violations, 0);
    assert_eq!(stats.consecutive_violations, 0);
    assert_eq!(
        stats.max_cycle_us, 1_000,
        "the cycle is the simulation step"
    );
    assert!(
        !runner.kernel.state.faults.intersects(LATCHED),
        "{:?}",
        runner.kernel.state.faults
    );
}

fn transport() -> SitlIO {
    SitlIO::new(XilConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 16,
        },
    ))
    .expect("ephemeral SITL transport")
}

fn board_hal() -> super::super::SitlBoardHal {
    BoardHal::new(
        FakeImu::new(),
        FakeBaro::new(),
        FakeMag::new(),
        FakeGnss::new(),
        SitlTime::new(),
        FakeActuator::new(),
    )
}

/// A light single-engine aeroplane, built from the core types alone: the
/// runtime picks no airframe, so its tests do not depend on an airframe
/// crate.
fn fixed_wing_kernel() -> DefaultAviateKernel<FixedWingController, FixedWingMixer> {
    let params = FixedWingParams {
        stall_airspeed: MetersPerSecond(25.0),
        min_airspeed: MetersPerSecond(33.0),
        cruise_airspeed: MetersPerSecond(55.0),
        max_airspeed: MetersPerSecond(65.0),
        min_throttle: NormalizedThrust(0.0),
        cruise_throttle: NormalizedThrust(0.65),
        max_throttle: NormalizedThrust(1.0),
        max_climb_rate: MetersPerSecond(2.5),
        max_sink_rate: MetersPerSecond(5.0),
        trim_pitch: Radians(0.02),
        height_time_constant: Seconds(5.0),
        speed_time_constant: Seconds(5.0),
        energy_rate_p: 0.05,
        energy_rate_i: 0.02,
        balance_rate_p: 0.5,
        balance_rate_i: 0.2,
        route_bank_angle: Radians(0.44),
        course_time_constant: Seconds(5.0),
        course_integral_time: Seconds(20.0),
        track_lookahead_time: Seconds(8.0),
        max_intercept_angle: Radians(0.785),
        roll_time_constant: Seconds(0.5),
        pitch_time_constant: Seconds(0.5),
        rate_p: [0.4, 0.6, 0.5],
        rate_i: [0.2, 0.4, 0.1],
        rate_ff: [0.8, 1.0, 0.5],
        rate_integrator_limit: 0.3,
        airspeed_scale_min: 0.5,
        airspeed_scale_max: 2.0,
        airspeed_filter_time_constant: Seconds(0.5),
    };
    AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(FixedWingController::new(params).expect("valid fixed-wing parameters"))
        .mixer(FixedWingMixer {
            timestamp_source: timestamp,
        })
        .sanitizer(Sanitizer)
        .config(ResolvedKernelConfig {
            mixer_geometry: MixerGeometry::FixedWing,
            fixed_wing: Some(params),
            ..ResolvedKernelConfig::default()
        })
        .build()
        .expect("valid fixed-wing kernel")
}

/// The heartbeat names the airframe of the kernel configuration: a host
/// that checks the vehicle type refuses a rotorcraft report for a
/// configured fixed wing.
#[test]
fn the_heartbeat_type_follows_the_airframe_configuration() {
    use aviate_link::mavlink::MavType;

    assert_eq!(runner().transport.vehicle_type(), MavType::Quadrotor);

    let fixed_wing = SitlRunner::new(transport(), board_hal(), fixed_wing_kernel());
    assert_eq!(fixed_wing.transport.vehicle_type(), MavType::FixedWing);
}
