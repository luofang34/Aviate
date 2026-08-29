//! Arm-authorization and sensor-presence state-transition tests.

#![allow(clippy::expect_used)]

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::hal::{ActuatorHal as _, SensorHal as _};
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::mixer::{QuadXMixer, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::ArmError;
use aviate_hal_io::{
    BoardHal, CommandOutcome, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag, SystemCommand,
};
use aviate_hal_xil::{
    SimBaroData, SimImuData, SimMagData, SimSensorPacket, SitlIO, XilConfig, XilNetConfig,
};

use super::super::{ArmAuthorizer, SitlRunner, SitlTime};

struct RejectArm;

impl ArmAuthorizer for RejectArm {
    fn authorize_arm(&self) -> Result<(), ArmError> {
        Err(ArmError::NotReady)
    }
}

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
    let transport = SitlIO::new(XilConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 16,
        },
    ))
    .expect("ephemeral SITL transport");
    let board_hal = BoardHal::new(
        FakeImu::new(),
        FakeBaro::new(),
        FakeMag::new(),
        FakeGnss::new(),
        SitlTime::new(),
        FakeActuator::new(),
    );
    SitlRunner::new(transport, board_hal, kernel)
}

#[test]
fn inbound_arm_rejection_changes_no_arm_state() {
    let mut runner = runner();
    let kernel_state = runner.kernel.state.init_state;
    assert!(!runner.board_hal.is_armed());
    assert!(!runner.transport.is_armed());

    let outcome = runner.enact_discrete(&SystemCommand::Arm, &RejectArm);

    assert!(matches!(
        outcome,
        Some(CommandOutcome::ArmRejected {
            error: ArmError::NotReady,
            ..
        })
    ));
    assert_eq!(runner.kernel.state.init_state, kernel_state);
    assert!(!runner.board_hal.is_armed());
    assert!(!runner.transport.is_armed());
}

fn imu_sample() -> SimImuData {
    SimImuData {
        accel: [0.0, 0.0, -9.81],
        gyro: [0.0, 0.0, 0.0],
        temperature: None,
    }
}

#[test]
fn a_sample_without_a_pressure_lane_presents_no_pressure_reading() {
    let mut runner = runner();
    runner.transport.feed_sensor_packet(
        &SimSensorPacket::new(1)
            .with_imu(imu_sample())
            .with_baro(SimBaroData {
                pressure_pa: 100_000.0,
                differential_pressure_pa: None,
                pressure_altitude_m: None,
                temperature_c: 20.0,
            })
            .with_mag(SimMagData {
                field_ut: [20.0, 0.0, 45.0],
            }),
    );
    runner.feed_buffered_sensors();
    let complete = runner.board_hal.read_baro().expect("fed pressure reading");
    assert_eq!(
        complete
            .value
            .air
            .static_pressure
            .expect("static pressure")
            .0,
        100_000.0
    );
    assert_eq!(
        runner
            .board_hal
            .read_mag()
            .expect("fed magnetic reading")
            .value
            .field_ut[0]
            .0,
        20.0
    );

    runner
        .transport
        .feed_sensor_packet(&SimSensorPacket::new(2).with_imu(imu_sample()));
    runner.feed_buffered_sensors();

    assert!(
        runner.board_hal.read_baro().is_none(),
        "an absent pressure lane must present no reading, not a zero-pascal substitute"
    );
    assert!(
        runner.board_hal.read_mag().is_none(),
        "an absent magnetic lane must present no reading, not a zero-field substitute"
    );
    assert!(
        runner.board_hal.read_imu().is_some(),
        "the lane the sample carried must still reach the flight controller"
    );
}
