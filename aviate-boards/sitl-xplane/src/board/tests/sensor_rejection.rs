//! A sample the sensor contract refuses never reaches the flight controller.

use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use aviate_config::xplane_model::XPlaneSimulatorModel;
use aviate_hal_xil::perturbation::{
    ActuatorPerturbation, PerturbationConfig, PerturbationError, PerturbationIdentity, SensorLane,
    SensorNoise,
};
use aviate_hal_xil::sim_types::{SimImuData, SimSensorPacket};

use crate::{XPlaneBoard, XPlaneConfig};

use super::MODEL;

const DIFFERENTIAL_PRESSURE_BIT: u16 = 1 << 10;

type AliaBoard = XPlaneBoard<
    aviate_core::control::multirotor::MultirotorController,
    aviate_core::mixer::QuadXMixerReversedSpin,
>;

fn perturbation() -> PerturbationConfig {
    PerturbationConfig {
        identity: PerturbationIdentity {
            condition_digest: [7; 32],
            run_seed: 11,
        },
        sensor_noise: vec![SensorNoise {
            lane: SensorLane::AccelerometerX,
            peak_amplitude: 0.05,
            update_interval_samples: 10,
        }],
        actuator: ActuatorPerturbation::default(),
    }
}

fn board() -> AliaBoard {
    let model = XPlaneSimulatorModel::from_toml_str(MODEL).expect("valid model");
    let kernel =
        aviate_app_sitl_xplane_alia250_kernel::build_alia250_kernel().expect("Alia kernel builds");
    let address = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::LOCALHOST, 0));
    XPlaneBoard::with_config(
        kernel,
        XPlaneConfig::new(address, model).with_perturbation(perturbation()),
    )
    .expect("board builds")
}

fn imu() -> SimImuData {
    SimImuData {
        accel: [0.0, 0.0, -9.81],
        gyro: [0.0, 0.0, 0.0],
        temperature: None,
    }
}

/// A differential-pressure bit without its static-pressure bit is ambiguous.
fn ambiguous_pressure_sample(timestamp_us: u64) -> SimSensorPacket {
    let mut packet = SimSensorPacket::new(timestamp_us).with_imu(imu());
    packet.presence_mask |= DIFFERENTIAL_PRESSURE_BIT;
    packet
}

/// The simulator transport binds one fixed port, so the two boards this
/// case needs are built one after the other in a single test.
///
/// The cached inertial reading is the probe: the runner consumes the
/// transport buffer inside the same step, so only the cache still shows
/// whether the controller received the sample.
#[test]
fn only_a_sample_the_sensor_contract_accepts_reaches_the_flight_controller() {
    let _port = super::board_port_guard();
    {
        let mut refused = board();
        refused.process_sample(ambiguous_pressure_sample(1_000));
        assert!(
            matches!(
                refused.perturbation_failure(),
                Some(PerturbationError::SensorPresenceMismatch(_))
            ),
            "the sensor contract must refuse an ambiguous pressure sample"
        );
        assert!(
            refused.runner.sensor_cache.imu.is_none(),
            "a refused sample must not reach the flight controller"
        );
    }

    let mut accepted = board();
    accepted.process_sample(SimSensorPacket::new(1_000).with_imu(imu()));
    assert!(
        accepted.runner.sensor_cache.imu.is_some(),
        "an accepted sample must reach the flight controller"
    );
}
