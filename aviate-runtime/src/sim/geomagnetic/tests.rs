//! The runner's geomagnetic reference, driven through `step()`.

#![allow(clippy::expect_used)]

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::mixer::{QuadXMixer, Sanitizer};
use aviate_core::sensor::GeomagneticSource;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Radians;
use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{
    SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket, SitlIO, XilConfig,
    XilNetConfig,
};
use world_core::GeodeticPosition;
use world_geomag::wmm2025;

use super::super::{SitlRunner, SitlTime};
use super::decimal_year_now;

/// Calgary, at 51° N: a declination of about 13° east.
const LAT: f64 = 51.05;
const LON: f64 = -114.07;
const ALT_M: f32 = 1_045.0;

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

/// The model's declination at the test point, radians.
fn model_declination() -> f32 {
    wmm2025(
        GeodeticPosition::wgs84(LAT, LON, f64::from(ALT_M)),
        decimal_year_now(),
    )
    .expect("the test point and today are inside the model")
    .declination_rad() as f32
}

fn fix() -> SimGnssData {
    SimGnssData {
        lat_deg: LAT,
        lon_deg: LON,
        alt_m: ALT_M,
        position_ned: [0.0; 3],
        vel_ned: [0.0; 3],
        fix: SimGnssFix::ThreeD,
        h_acc: 0.5,
        v_acc: 0.8,
        satellites: 12,
    }
}

/// A level vehicle at true heading `yaw` under a field whose
/// declination is `declination`.
fn level_sample(t_us: u64, yaw: f32, declination: f32) -> SimSensorPacket {
    let (h, z) = (17.0_f32, 54.0_f32);
    let (n, e) = (h * declination.cos(), h * declination.sin());
    // The NED field seen from a body frame yawed by `yaw`.
    let (s, c) = yaw.sin_cos();
    SimSensorPacket::new(t_us)
        .with_imu(SimImuData {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0; 3],
            temperature: None,
        })
        .with_mag(SimMagData {
            field_ut: [c * n + s * e, -s * n + c * e, z],
        })
}

#[test]
fn a_fix_supplies_the_model_declination_and_a_dropout_keeps_it() {
    let mut runner = runner();
    assert!(runner.geomagnetic_reference().is_none());

    runner
        .transport
        .feed_sensor_packet(&level_sample(1_000, 0.0, 0.0).with_gnss(fix()));
    runner.step();
    let reference = runner
        .geomagnetic_reference()
        .expect("a 3-D fix gives a model reference");
    assert_eq!(reference.source, GeomagneticSource::Model);
    let expected = model_declination();
    assert!((reference.declination.0 - expected).abs() < 1e-5);
    assert!(
        expected.to_degrees() > 10.0,
        "the test point must have a large declination"
    );

    // GNSS lost: the declination is unchanged.
    for t in 2..10u64 {
        runner
            .transport
            .feed_sensor_packet(&level_sample(t * 1_000, 0.0, 0.0));
        runner.step();
    }
    assert_eq!(runner.geomagnetic_reference(), Some(reference));
}

#[test]
fn a_configured_declination_serves_until_a_fix_arrives() {
    let mut runner = runner();
    runner.set_configured_declination(Radians(0.1));
    runner
        .transport
        .feed_sensor_packet(&level_sample(1_000, 0.0, 0.0));
    runner.step();
    let configured = runner.geomagnetic_reference().expect("configured value");
    assert_eq!(configured.source, GeomagneticSource::Configured);
    assert_eq!(configured.declination, Radians(0.1));

    runner
        .transport
        .feed_sensor_packet(&level_sample(2_000, 0.0, 0.0).with_gnss(fix()));
    runner.step();
    let reference = runner.geomagnetic_reference().expect("model value");
    assert_eq!(reference.source, GeomagneticSource::Model);
}

/// The estimator is seeded from the first IMU and magnetometer sample.
/// That seed must use the same true-north reference as the update, or
/// the filter starts `declination` off and spends its first seconds
/// turning the vehicle to reconcile the two.
#[test]
fn the_initial_attitude_is_seeded_at_the_true_heading() {
    let mut runner = runner();
    let declination = model_declination();
    let true_yaw = 30.0_f32.to_radians();
    runner
        .transport
        .feed_sensor_packet(&level_sample(1_000, true_yaw, declination).with_gnss(fix()));
    runner.step();
    assert!(runner.ekf_initialized);
    let (_, _, yaw) = runner
        .kernel
        .state
        .estimator
        .get_estimate()
        .attitude
        .to_euler();
    assert!(
        (yaw - true_yaw).abs().to_degrees() < 0.5,
        "seeded yaw {:.2}° is not the true heading 30°",
        yaw.to_degrees()
    );
}
