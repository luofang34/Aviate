#![allow(clippy::expect_used)]

use std::path::PathBuf;

use super::*;
use crate::perturbation::actuator::schedule;
use crate::sim_types::{SimBaroData, SimImuData, SimMagData, SimSensorPacket};

const CONDITION: &str = r#"{"schema_version":4,"id":"condition-a","revision":1,"seed":7,"wind":{"steady":{"speed_mps":5.0,"direction_deg":90.0},"gusts":[],"turbulence":{"kind":"none"}},"timing":{"estimate_delay_ns":0,"update_jitter":{"kind":"none"}},"sensor":{"kind":"bounded_noise","lanes":[{"sensor":"magnetometer","axis":"x","peak_amplitude_gauss":0.5,"update_interval_samples":10},{"sensor":"differential_pressure","peak_amplitude_hpa":1.5,"update_interval_samples":5}]},"actuator":{"authority_scale_basis_points":9000,"command_loss":{"kind":"seeded_zero_order_hold","fraction_basis_points":100,"decision_interval_samples":100}},"controller_initialization":{"hover_thrust_force":{"kind":"scale_baseline","scale_basis_points":11000}},"plant":{"payload_mass_delta_kg":120.0,"longitudinal_cg_offset_m":0.1,"lateral_cg_offset_m":0.0,"hover_thrust_expectation":{"kind":"measured_weight_ratio"}}}"#;

fn capabilities() -> [PerturbationCapability; 4] {
    [
        PerturbationCapability::SensorPerturbation,
        PerturbationCapability::ActuatorAuthority,
        PerturbationCapability::CommandHold,
        PerturbationCapability::HoverTrimUncertainty,
    ]
}

fn artifact_path(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!(
        "aviate-condition-artifact-{}-{name}.json",
        std::process::id()
    ))
}

fn write_condition(name: &str, contents: &str) -> PathBuf {
    let path = artifact_path(name);
    std::fs::write(&path, contents).expect("write condition fixture");
    path
}

#[test]
fn strict_load_binds_bytes_canonical_identity_seed_and_owned_capabilities() {
    let path = write_condition("load", CONDITION);
    let expected = digest(CONDITION.as_bytes());
    let loaded = LoadedPerturbationArtifact::load(&path, expected, expected, 99, &capabilities())
        .expect("strict artifact");

    assert_eq!(loaded.identity().artifact_sha256, expected);
    assert_eq!(loaded.identity().condition_digest, expected);
    assert_eq!(loaded.identity().run_seed, 99);
    assert_eq!(
        loaded.identity().required_capabilities,
        capabilities().to_vec()
    );
    assert_eq!(loaded.hover_scale_basis_points(), 11_000);
    assert_eq!(loaded.config().sensor_noise[0].peak_amplitude, 50.0);
    assert_eq!(loaded.config().sensor_noise[1].peak_amplitude, 150.0);
    assert_eq!(loaded.config().actuator.authority_scale_basis_points, 9_000);
    std::fs::remove_file(path).ok();
}

#[test]
fn simulator_owned_wind_and_plant_do_not_enter_the_aviate_capability_set() {
    let path = write_condition("capabilities", CONDITION);
    let expected = digest(CONDITION.as_bytes());

    assert!(
        LoadedPerturbationArtifact::load(&path, expected, expected, 99, &capabilities()).is_ok()
    );
    assert!(matches!(
        LoadedPerturbationArtifact::load(
            &path,
            expected,
            expected,
            99,
            &[PerturbationCapability::SensorPerturbation]
        ),
        Err(ArtifactError::CapabilityMismatch)
    ));
    std::fs::remove_file(path).ok();
}

#[test]
fn a_live_file_change_fails_the_arm_time_guard() {
    let path = write_condition("live-change", CONDITION);
    let expected = digest(CONDITION.as_bytes());
    let loaded = LoadedPerturbationArtifact::load(&path, expected, expected, 99, &capabilities())
        .expect("strict artifact");
    let guard = loaded.live_guard();
    guard.verify().expect("unchanged artifact");

    std::fs::write(&path, format!("{CONDITION}\n")).expect("change condition fixture");
    assert!(matches!(
        guard.verify(),
        Err(ArtifactError::LiveArtifactChanged { .. })
    ));
    std::fs::remove_file(path).ok();
}

#[test]
fn unknown_fields_and_digest_mismatches_fail_closed() {
    let unknown = CONDITION.replacen("\"revision\":1", "\"revision\":1,\"extra\":0", 1);
    let path = write_condition("unknown", &unknown);
    let unknown_digest = digest(unknown.as_bytes());
    assert!(matches!(
        LoadedPerturbationArtifact::load(
            &path,
            unknown_digest,
            unknown_digest,
            99,
            &capabilities()
        ),
        Err(ArtifactError::Json(_))
    ));
    assert!(matches!(
        LoadedPerturbationArtifact::load(&path, [1; 32], unknown_digest, 99, &capabilities()),
        Err(ArtifactError::ArtifactDigestMismatch)
    ));
    std::fs::remove_file(path).ok();
}

#[cfg(unix)]
#[test]
fn the_loader_does_not_follow_the_final_symbolic_link() {
    use std::os::unix::fs::symlink;

    let target = write_condition("link-target", CONDITION);
    let link = artifact_path("link");
    symlink(&target, &link).expect("create symbolic link");
    let expected = digest(CONDITION.as_bytes());

    assert!(matches!(
        LoadedPerturbationArtifact::load(&link, expected, expected, 99, &capabilities()),
        Err(ArtifactError::Open { .. })
    ));
    std::fs::remove_file(link).ok();
    std::fs::remove_file(target).ok();
}

#[test]
fn pilotage_condition_v4_bytes_and_decisions_match_the_cross_repository_golden() {
    const GOLDEN: &[u8] = include_bytes!("../../../fixtures/condition-v4.golden.json");
    const RUN_SEED: u64 = 0x1112_1314_1516_1718;

    let raw_digest = digest(GOLDEN);
    let canonical_bytes = GOLDEN.strip_suffix(b"\n").expect("fixture terminal LF");
    let condition: ConditionSet = serde_json::from_slice(canonical_bytes).expect("golden JSON");
    let canonical = serde_json::to_vec(&condition).expect("canonical JSON");
    let condition_digest = digest(&canonical);
    assert_eq!(
        hex(raw_digest),
        "84d57e0b7da52d6f935bdbb29b540772169b719fe1b5fac1311461ddbf62028d"
    );
    assert_eq!(
        hex(condition_digest),
        "ecca66bfcc8bf95fd9bbdf663add8ae3f6aef2c765325d3bc3e288716f7ba763"
    );
    assert_eq!(canonical, canonical_bytes);

    let path = artifact_path("pilotage-golden");
    std::fs::write(&path, GOLDEN).expect("write golden artifact");
    let loaded = LoadedPerturbationArtifact::load(
        &path,
        raw_digest,
        condition_digest,
        RUN_SEED,
        &capabilities(),
    )
    .expect("load Pilotage golden");
    let mut engine = PerturbationEngine::new(loaded.config().clone()).expect("golden engine");
    let mut packet = SimSensorPacket::new(123)
        .with_imu(SimImuData {
            accel: [1.0, 2.0, 3.0],
            gyro: [4.0, 5.0, 6.0],
            temperature: None,
        })
        .with_mag(SimMagData {
            field_ut: [7.0, 8.0, 9.0],
        })
        .with_baro(SimBaroData {
            pressure_pa: 100_000.0,
            differential_pressure_pa: Some(500.0),
            pressure_altitude_m: Some(100.0),
            temperature_c: 20.0,
        });
    let sensor_application = engine
        .apply_sensor(123, &mut packet)
        .expect("golden sensor decision");
    assert_eq!(sensor_application.raw_value_bits[0], Some(0x3f80_0000));
    assert_eq!(
        sensor_application.effective_value_bits[0],
        Some(0x3f7a_9bbb)
    );
    assert_eq!(packet.imu.expect("IMU").accel[0].to_bits(), 0x3f7a_9bbb);

    let hold = loaded.config().actuator.command_hold.expect("golden hold");
    let decisions =
        schedule(loaded.config().identity, 2, 3, 1_001, hold).expect("golden hold schedule");
    let held = decisions
        .iter()
        .enumerate()
        .filter_map(|(index, selected)| selected.then_some(index))
        .collect::<Vec<_>>();
    assert_eq!(held, vec![89]);
    std::fs::remove_file(path).ok();
}

#[test]
fn golden_command_hold_stream_is_repeatable_and_run_seed_bound() {
    const GOLDEN: &[u8] = include_bytes!("../../../fixtures/condition-v4.golden.json");
    const RUN_SEED: u64 = 0x1112_1314_1516_1718;

    let raw_digest = digest(GOLDEN);
    let canonical_bytes = GOLDEN.strip_suffix(b"\n").expect("fixture terminal LF");
    let condition: ConditionSet = serde_json::from_slice(canonical_bytes).expect("golden JSON");
    let condition_digest = digest(&serde_json::to_vec(&condition).expect("canonical JSON"));
    let path = artifact_path("pilotage-golden-run-seed");
    std::fs::write(&path, GOLDEN).expect("write golden artifact");
    let load = |run_seed| {
        LoadedPerturbationArtifact::load(
            &path,
            raw_digest,
            condition_digest,
            run_seed,
            &capabilities(),
        )
        .expect("load Pilotage golden")
    };
    let first = load(RUN_SEED);
    let repeated = load(RUN_SEED);
    let changed = load(RUN_SEED.wrapping_add(1));
    let mut expected_changed = first.config().clone();
    expected_changed.identity.run_seed = RUN_SEED.wrapping_add(1);
    assert_eq!(first.config(), repeated.config());
    assert_eq!(changed.config(), &expected_changed);

    let hold = first.config().actuator.command_hold.expect("golden hold");
    let stream = |artifact: &LoadedPerturbationArtifact| {
        schedule(artifact.config().identity, 2, 3, 1_001, hold).expect("golden hold schedule")
    };
    assert_eq!(stream(&first), stream(&repeated));
    assert_ne!(stream(&first), stream(&changed));
    std::fs::remove_file(path).ok();
}

fn hex(value: [u8; 32]) -> String {
    value.iter().map(|byte| format!("{byte:02x}")).collect()
}
