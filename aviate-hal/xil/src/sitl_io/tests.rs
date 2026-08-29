//! Sensor-section presence tests for the simulator transport.

#![allow(clippy::expect_used)]

use aviate_hal_io::{RawBaroReading, RawImuReading, RawMagReading};

use crate::sim_types::{SimBaroData, SimImuData, SimMagData, SimSensorPacket};
use crate::{SitlIO, XilConfig, XilNetConfig};

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

fn imu() -> SimImuData {
    SimImuData {
        accel: [1.0, 2.0, 3.0],
        gyro: [4.0, 5.0, 6.0],
        temperature: None,
    }
}

fn baro() -> SimBaroData {
    SimBaroData {
        pressure_pa: 100_000.0,
        differential_pressure_pa: Some(500.0),
        pressure_altitude_m: Some(100.0),
        temperature_c: 20.0,
    }
}

fn mag() -> SimMagData {
    SimMagData {
        field_ut: [7.0, 8.0, 9.0],
    }
}

#[test]
fn an_absent_sensor_section_publishes_no_reading() {
    let mut transport = transport();
    transport.feed_sensor_packet(&SimSensorPacket::new(1).with_imu(imu()));
    let data = transport.take_sensor_data().expect("IMU-only sample");

    assert_eq!(
        data.imu.expect("IMU section").accel,
        [1.0, 2.0, 3.0],
        "the present section must carry the fed value"
    );
    assert!(
        data.baro.is_none(),
        "an absent pressure lane must not publish a zero-pascal reading"
    );
    assert!(
        data.mag.is_none(),
        "an absent magnetic lane must not publish a zero-field reading"
    );
}

#[test]
fn every_present_section_carries_its_own_value() {
    let mut transport = transport();
    transport.feed_sensor_packet(
        &SimSensorPacket::new(2)
            .with_imu(imu())
            .with_baro(baro())
            .with_mag(mag()),
    );
    let data = transport.take_sensor_data().expect("complete sample");

    let imu: RawImuReading = data.imu.expect("IMU section");
    let baro: RawBaroReading = data.baro.expect("pressure section");
    let mag: RawMagReading = data.mag.expect("magnetic section");
    assert_eq!(imu.gyro, [4.0, 5.0, 6.0]);
    assert_eq!(baro.pressure_pa, 100_000.0);
    assert_eq!(baro.differential_pressure_pa, Some(500.0));
    assert_eq!(baro.pressure_altitude_m, Some(100.0));
    assert_eq!(mag.field_ut, [7.0, 8.0, 9.0]);
}

#[test]
fn a_packet_without_any_sensor_section_buffers_nothing() {
    let mut transport = transport();
    transport.feed_sensor_packet(&SimSensorPacket::new(3));

    assert!(transport.take_sensor_data().is_none());
}
