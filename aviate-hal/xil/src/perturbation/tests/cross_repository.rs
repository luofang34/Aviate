//! Values that Aviate and Pilotage must both produce for one input vector.
//!
//! Each expected value here is derived from the written domain contract, not
//! recorded from this implementation. A change to a preimage, to the digest
//! word, or to the value mapping fails one of these cases by value.

use super::super::actuator::{interval_digest, permutation_value};
use super::super::sensor::SensorLane;
use super::super::{ActuatorPerturbation, PerturbationConfig, PerturbationEngine, SensorNoise};
use super::{identity, GOLDEN_DIGEST, GOLDEN_RUN_SEED};
use crate::sim_types::{SimBaroData, SimImuData, SimSensorPacket};

/// The interval identity inputs that Pilotage pins for this domain.
const GOLDEN_EPOCH: u64 = 0x2122_2324_2526_2728;
const GOLDEN_INDEX: u64 = 0x3132_3334_3536_3738;
const GOLDEN_FIRST_SEQUENCE: u64 = 0x4142_4344_4546_4748;
const GOLDEN_CURSOR: u64 = 0x5152_5354_5556_5758;

fn hex(value: [u8; 32]) -> String {
    value.iter().map(|byte| format!("{byte:02x}")).collect()
}

#[test]
fn the_command_hold_interval_identity_matches_the_cross_repository_digest() {
    assert_eq!(
        hex(interval_digest(
            identity(),
            GOLDEN_EPOCH,
            GOLDEN_INDEX,
            GOLDEN_FIRST_SEQUENCE
        )),
        "94ab1093b990a952b30ec29395a88314347304a5b21ed170c862d2725d99bd6c"
    );
}

#[test]
fn the_command_hold_permutation_value_matches_the_cross_repository_word() {
    assert_eq!(
        permutation_value(
            identity(),
            GOLDEN_EPOCH,
            GOLDEN_INDEX,
            GOLDEN_FIRST_SEQUENCE,
            GOLDEN_CURSOR
        ),
        5_234_502_356_555_059_490
    );
}

fn sensor_config() -> PerturbationConfig {
    PerturbationConfig {
        identity: identity(),
        sensor_noise: vec![
            SensorNoise {
                lane: SensorLane::AccelerometerX,
                peak_amplitude: 2.0,
                update_interval_samples: 2,
            },
            SensorNoise {
                lane: SensorLane::DifferentialPressure,
                peak_amplitude: 200.0,
                update_interval_samples: 1,
            },
        ],
        actuator: ActuatorPerturbation::default(),
    }
}

fn sensor_sample() -> SimSensorPacket {
    SimSensorPacket::new(10)
        .with_imu(SimImuData {
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: None,
        })
        .with_baro(SimBaroData {
            pressure_pa: 100_000.0,
            differential_pressure_pa: Some(500.0),
            pressure_altitude_m: None,
            temperature_c: 20.0,
        })
}

#[test]
fn each_sensor_lane_offset_matches_the_cross_repository_value() {
    let mut engine = PerturbationEngine::new(sensor_config()).expect("engine");
    let mut sample = sensor_sample();

    let application = engine.apply_sensor(10, &mut sample).expect("application");

    // Accelerometer X: lane tag 0, update bucket 10 / 2 = 5, peak 2 m/s².
    assert_eq!(application.update_buckets[0], Some(5));
    assert_eq!(application.raw_value_bits[0], Some(0x3f80_0000));
    assert_eq!(application.effective_value_bits[0], Some(0x403c_4a40));

    // Differential pressure: lane tag 10, update bucket 10 / 1 = 10, peak 200 Pa.
    assert_eq!(application.update_buckets[10], Some(10));
    assert_eq!(application.raw_value_bits[10], Some(0x43fa_0000));
    assert_eq!(application.effective_value_bits[10], Some(0x43c4_6996));
}

#[test]
fn a_changed_run_seed_changes_every_sensor_lane_offset() {
    let mut config = sensor_config();
    config.identity.run_seed = GOLDEN_RUN_SEED.wrapping_add(1);
    let mut engine = PerturbationEngine::new(config).expect("engine");
    let mut sample = sensor_sample();

    let application = engine.apply_sensor(10, &mut sample).expect("application");

    assert_eq!(application.update_buckets[0], Some(5));
    assert_ne!(application.effective_value_bits[0], Some(0x403c_4a40));
    assert_ne!(application.effective_value_bits[10], Some(0x43c4_6996));
}

#[test]
fn a_changed_condition_digest_changes_every_sensor_lane_offset() {
    let mut config = sensor_config();
    let mut changed = GOLDEN_DIGEST;
    changed[31] = GOLDEN_DIGEST[31].wrapping_add(1);
    config.identity.condition_digest = changed;
    let mut engine = PerturbationEngine::new(config).expect("engine");
    let mut sample = sensor_sample();

    let application = engine.apply_sensor(10, &mut sample).expect("application");

    assert_ne!(application.effective_value_bits[0], Some(0x403c_4a40));
    assert_ne!(application.effective_value_bits[10], Some(0x43c4_6996));
}
