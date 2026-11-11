//! Step-entry bookkeeping of `step_sensors`: time reference, sample stamps,
//! the airspeed lane, and the board's own declination.

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;
use aviate_core::sensor::{GeomagneticReference, GeomagneticSource, SensorHealth};
use aviate_core::types::Radians;
use aviate_hal_io::{
    GnssFix, RawAirspeedReading, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};
use world_core::{DecimalYear, GeodeticPosition};

use crate::host_sensors::{DirectHostSensors, DirectSensorSample};
use crate::synthesis::declination_rad_at;
use crate::{DirectBoardConfig, DirectNoiseTier, DirectXilBoard};

type Board = DirectXilBoard<
    aviate_core::control::multirotor::MultirotorController,
    aviate_core::mixer::QuadXMixerX500,
>;

const EPOCH: DecimalYear = DecimalYear(2026.5);

fn origin() -> GeodeticPosition {
    GeodeticPosition::wgs84(39.1582, -75.5244, 15.0)
}

fn board() -> Board {
    let config = DirectBoardConfig {
        origin: origin(),
        epoch: EPOCH,
        noise: DirectNoiseTier::Off,
        noise_seed: 1,
        sys_id: 1,
        comp_id: 1,
    };
    DirectXilBoard::new(build_x500_kernel().expect("X500 kernel"), config).expect("board")
}

fn level_imu() -> RawImuReading {
    RawImuReading {
        accel: [0.0, 0.0, -9.80],
        gyro: [0.0; 3],
        temperature: Some(25.0),
    }
}

fn imu_only(time_us: u64) -> DirectHostSensors {
    DirectHostSensors {
        time_us,
        imu: Some(DirectSensorSample::new(time_us, level_imu())),
        ..DirectHostSensors::default()
    }
}

fn fix_at(position: GeodeticPosition, fix: GnssFix) -> RawGnssReading {
    RawGnssReading {
        lat_deg: position.latitude_deg,
        lon_deg: position.longitude_deg,
        alt_m: position.ellipsoidal_height_m as f32,
        fix,
        ..RawGnssReading::default()
    }
}

#[test]
fn first_call_sets_the_time_reference_and_later_calls_run_one_cycle_each() {
    let mut board = board();

    let _ = board.step_sensors(&imu_only(10_000));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 0);
    assert!(
        board.sensor_cache.imu.is_none(),
        "the reference call must not use its samples"
    );

    let _ = board.step_sensors(&imu_only(12_500));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 1);
    assert_eq!(board.kernel().state.timing_stats.last_cycle_us, 2_500);

    // Time that does not increase runs no cycle.
    let _ = board.step_sensors(&imu_only(12_500));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 1);

    // Time that runs backwards becomes the new reference.
    let _ = board.step_sensors(&imu_only(4_000));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 1);
    let _ = board.step_sensors(&imu_only(5_000));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 2);
    assert_eq!(board.kernel().state.timing_stats.last_cycle_us, 1_000);
}

#[test]
fn a_new_generation_starts_a_new_time_reference() {
    let mut board = board();
    let _ = board.step_sensors(&imu_only(1_000));
    let _ = board.step_sensors(&imu_only(2_000));
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 1);

    let reset = DirectHostSensors {
        reset_generation: 1,
        ..imu_only(50_000)
    };
    let _ = board.step_sensors(&reset);
    // The world reset clears the kernel counters, and the call that carries
    // it only sets the new time reference.
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 0);
    assert_eq!(board.host_clock_us, Some(50_000));

    let next = DirectHostSensors {
        reset_generation: 1,
        ..imu_only(52_000)
    };
    let _ = board.step_sensors(&next);
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 1);
    assert_eq!(board.kernel().state.timing_stats.last_cycle_us, 2_000);
}

#[test]
fn each_reading_carries_the_measurement_time_of_its_sample() {
    let mut board = board();
    let _ = board.step_sensors(&imu_only(0));

    let sensors = DirectHostSensors {
        time_us: 20_000,
        imu: Some(DirectSensorSample::new(19_900, level_imu())),
        baro: Some(DirectSensorSample::new(
            14_000,
            RawBaroReading {
                pressure_pa: 101_000.0,
                temperature_c: 15.0,
                ..RawBaroReading::default()
            },
        )),
        mag: Some(DirectSensorSample::new(
            17_500,
            RawMagReading {
                field_ut: [20.0, -4.0, 45.0],
            },
        )),
        gnss: Some(DirectSensorSample::new(
            8_000,
            fix_at(origin(), GnssFix::ThreeD),
        )),
        airspeed: Some(DirectSensorSample::new(
            16_000,
            RawAirspeedReading {
                differential_pressure_pa: 1_800.0,
                static_pressure_pa: None,
                temperature_c: None,
            },
        )),
        ..DirectHostSensors::default()
    };
    let _ = board.step_sensors(&sensors);

    let cache = &board.sensor_cache;
    assert_eq!(cache.imu.expect("imu").timestamp.ticks, 19_900);
    assert_eq!(cache.baro.expect("baro").timestamp.ticks, 14_000);
    assert_eq!(cache.mag.expect("mag").timestamp.ticks, 17_500);
    assert_eq!(cache.gnss.expect("gnss").timestamp.ticks, 8_000);
    assert_eq!(cache.airspeed.expect("airspeed").timestamp.ticks, 16_000);
}

#[test]
fn airspeed_lane_reaches_the_sensor_set_and_holds_between_samples() {
    let mut board = board();
    let _ = board.step_sensors(&imu_only(0));

    let with_airspeed = DirectHostSensors {
        airspeed: Some(DirectSensorSample::new(
            2_500,
            RawAirspeedReading {
                differential_pressure_pa: 1_800.0,
                static_pressure_pa: Some(95_000.0),
                temperature_c: Some(9.0),
            },
        )),
        ..imu_only(2_500)
    };
    let _ = board.step_sensors(&with_airspeed);
    let lane = board.sensor_cache.to_sensor_set(None).airspeeds[0];
    assert!(lane.valid);
    assert_eq!(lane.health, SensorHealth::Good);
    assert_eq!(lane.value.air.dynamic_pressure.map(|p| p.0), Some(1_800.0));
    assert_eq!(lane.value.air.static_pressure.map(|p| p.0), Some(95_000.0));

    // No new sample: the lane stays healthy and offers no new measurement.
    let _ = board.step_sensors(&imu_only(5_000));
    let lane = board.sensor_cache.to_sensor_set(None).airspeeds[0];
    assert!(lane.valid);
    assert_eq!(lane.health, SensorHealth::Good);
    assert!(lane.value.air.dynamic_pressure.is_none());
}

#[test]
fn board_declination_follows_the_gnss_fix_and_survives_its_loss() {
    let mut board = board();
    let at_origin = declination_rad_at(origin(), EPOCH).expect("origin declination");
    assert_eq!(board.model_declination_rad, at_origin);

    // A position whose declination differs from the origin by several degrees.
    let elsewhere = GeodeticPosition::wgs84(47.0, 8.0, 500.0);
    let at_elsewhere = declination_rad_at(elsewhere, EPOCH).expect("declination");
    assert!((at_elsewhere - at_origin).abs() > 0.1);

    let _ = board.step_sensors(&imu_only(0));
    let with_fix = DirectHostSensors {
        gnss: Some(DirectSensorSample::new(
            2_500,
            fix_at(elsewhere, GnssFix::ThreeD),
        )),
        ..imu_only(2_500)
    };
    let _ = board.step_sensors(&with_fix);
    assert_eq!(board.model_declination_rad, at_elsewhere);

    // A reading without a fix, or with a malformed position, keeps it.
    let lost = DirectHostSensors {
        gnss: Some(DirectSensorSample::new(
            5_000,
            fix_at(origin(), GnssFix::None),
        )),
        ..imu_only(5_000)
    };
    let _ = board.step_sensors(&lost);
    assert_eq!(board.model_declination_rad, at_elsewhere);
    let malformed = DirectHostSensors {
        gnss: Some(DirectSensorSample::new(
            7_500,
            fix_at(GeodeticPosition::wgs84(f64::NAN, 8.0, 0.0), GnssFix::ThreeD),
        )),
        ..imu_only(7_500)
    };
    let _ = board.step_sensors(&malformed);
    assert_eq!(board.model_declination_rad, at_elsewhere);

    // A host-supplied reference takes precedence over the board model.
    let supplied = GeomagneticReference {
        declination: Radians(0.25),
        source: GeomagneticSource::Configured,
    };
    let overridden = DirectHostSensors {
        geomagnetic: Some(supplied),
        ..imu_only(10_000)
    };
    let _ = board.step_sensors(&overridden);
    assert_eq!(board.model_declination_rad, at_elsewhere);

    // A world reset returns the board model to the origin.
    let reset = DirectHostSensors {
        reset_generation: 1,
        ..imu_only(0)
    };
    let _ = board.step_sensors(&reset);
    assert_eq!(board.model_declination_rad, at_origin);
}

fn imu_and_mag(time_us: u64) -> DirectHostSensors {
    DirectHostSensors {
        mag: Some(DirectSensorSample::new(
            time_us,
            RawMagReading {
                field_ut: [20.0, -4.0, 45.0],
            },
        )),
        ..imu_only(time_us)
    }
}

#[test]
fn a_cycle_without_an_imu_sample_holds_the_last_one() {
    let mut board = board();
    let _ = board.step_sensors(&imu_only(0));
    let _ = board.step_sensors(&imu_only(2_500));

    let silent = DirectHostSensors {
        time_us: 5_000,
        ..DirectHostSensors::default()
    };
    let _ = board.step_sensors(&silent);
    assert_eq!(board.kernel().state.timing_stats.total_cycles, 2);
    assert_eq!(board.sensor_cache.imu.expect("imu").timestamp.ticks, 2_500);
    assert!(!board
        .kernel()
        .state
        .faults
        .contains(aviate_core::fault::FaultFlags::ALL_IMU_FAILED));
}

#[test]
fn estimator_seed_waits_for_a_fix_and_then_takes_it() {
    let mut board = board();
    let _ = board.step_sensors(&imu_and_mag(0));

    // A reading without a fix is not a seed, whatever position it carries.
    let no_fix = DirectHostSensors {
        gnss: Some(DirectSensorSample::new(
            2_500,
            RawGnssReading {
                position_ned: [900.0, 0.0, 0.0],
                ..fix_at(origin(), GnssFix::None)
            },
        )),
        ..imu_and_mag(2_500)
    };
    let _ = board.step_sensors(&no_fix);
    assert!(!board.ekf_initialized);

    let with_fix = DirectHostSensors {
        gnss: Some(DirectSensorSample::new(
            5_000,
            RawGnssReading {
                position_ned: [120.0, -30.0, -80.0],
                vel_ned: [40.0, 5.0, -1.0],
                ..fix_at(origin(), GnssFix::ThreeD)
            },
        )),
        ..imu_and_mag(5_000)
    };
    let _ = board.step_sensors(&with_fix);
    assert!(board.ekf_initialized);
    // The cycle that seeds the filter also propagates it once, by at most
    // 40 m/s over 2.5 ms, so the state is within 0.2 m and 0.2 m/s of the fix.
    let seeded = &board.kernel().state.estimator;
    assert!(
        (seeded.pos.x.0 - 120.0).abs() < 0.2,
        "north {}",
        seeded.pos.x.0
    );
    assert!(
        (seeded.vel.x.0 - 40.0).abs() < 0.2,
        "north rate {}",
        seeded.vel.x.0
    );
}

#[test]
fn estimator_seed_without_any_fix_is_at_rest_after_the_wait() {
    let mut board = board();
    let _ = board.step_sensors(&imu_and_mag(0));
    let _ = board.step_sensors(&imu_and_mag(2_500));
    assert!(!board.ekf_initialized);

    // 2 s after the attitude sources were complete, less one cycle.
    let _ = board.step_sensors(&imu_and_mag(2_000_000));
    assert!(!board.ekf_initialized);
    let _ = board.step_sensors(&imu_and_mag(2_002_500));
    assert!(board.ekf_initialized);
    assert_eq!(board.kernel().state.estimator.vel.x.0, 0.0);
}
