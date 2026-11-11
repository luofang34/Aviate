//! Runtime state: kernel hooks, reset and canonical encoding.

use super::super::air_data::AirspeedSource;
use super::super::{AirSample, FixedWingRuntimeState, StaticAir};
use super::{command, cruise_state, pitot, Rig};
use crate::control::runtime::ControllerRuntimeState;
use crate::control::{ControlMode, Setpoint};
use crate::replicable::Replicable;
use crate::sensor::{AirData, AirspeedData, SensorHealth, SensorReading, SensorSet, MAX_AIRSPEED};
use crate::types::Seconds;

#[test]
fn only_a_valid_cycle_interval_is_stored() {
    let mut runtime = FixedWingRuntimeState::default();
    runtime.set_cycle_interval(Seconds(0.004));
    assert_eq!(runtime.dt_sec, 0.004);
    for invalid in [0.0, -0.004, f32::NAN, f32::INFINITY] {
        runtime.set_cycle_interval(Seconds(invalid));
        assert_eq!(runtime.dt_sec, 0.0);
    }
}

#[test]
fn air_data_hook_keeps_one_sample_until_the_step_takes_it() {
    let mut runtime = FixedWingRuntimeState::default();
    runtime.set_air_data(Some(&pitot(40.0)));
    let sample = runtime.air_sample.expect("sample offered");
    assert!((sample.indicated_mps - 40.0).abs() < 1.0e-3);

    // A lane without a new measurement offers nothing.
    runtime.set_air_data(Some(&AirData::default()));
    assert!(runtime.air_sample.is_none());
    runtime.set_air_data(Some(&pitot(40.0)));
    runtime.set_air_data(None);
    assert!(runtime.air_sample.is_none());

    // The step consumes the sample.
    let mut rig = Rig::new();
    let _ = rig.step(
        &cruise_state(),
        &command(ControlMode::Attitude, Setpoint::default()),
    );
    assert!(rig.runtime.air_sample.is_none());
    assert!(rig.runtime.air.is_fresh());
    assert_eq!(
        rig.runtime.targets.airspeed_source,
        AirspeedSource::Measured
    );
}

#[test]
fn a_probe_that_stops_is_dropped_after_one_second() {
    let mut rig = Rig::new();
    let level = command(ControlMode::Attitude, Setpoint::default());
    let _ = rig.step(&cruise_state(), &level);
    rig.probe = None;
    // 100 cycles of 10 ms: the last sample is exactly at the limit.
    let _ = rig.run(99, &cruise_state(), &level);
    assert_eq!(
        rig.runtime.targets.airspeed_source,
        AirspeedSource::Measured
    );
    let _ = rig.run(3, &cruise_state(), &level);
    assert_eq!(
        rig.runtime.targets.airspeed_source,
        AirspeedSource::GroundSpeed
    );
}

#[test]
fn reset_returns_the_power_on_state() {
    let mut rig = Rig::new();
    let climb = command(
        ControlMode::AltitudeHold,
        Setpoint {
            vertical_speed: Some(crate::types::MetersPerSecond(-1.0)),
            ..Setpoint::default()
        },
    );
    let _ = rig.run(20, &cruise_state(), &climb);
    assert_ne!(rig.runtime, FixedWingRuntimeState::default());
    rig.runtime.reset();
    assert_eq!(rig.runtime, FixedWingRuntimeState::default());
}

fn encoded(runtime: &FixedWingRuntimeState) -> [u8; FixedWingRuntimeState::ENCODED_LEN] {
    let mut buf = [0_u8; FixedWingRuntimeState::ENCODED_LEN];
    assert_eq!(
        runtime.encode_canonical(&mut buf),
        FixedWingRuntimeState::ENCODED_LEN
    );
    buf
}

#[test]
fn every_persistent_field_is_in_the_canonical_encoding() {
    let mutations: [fn(&mut FixedWingRuntimeState); 31] = [
        |s| s.dt_sec = 0.5,
        |s| s.air_sample = Some(AirSample::default()),
        |s| {
            s.air_sample = Some(AirSample {
                indicated_mps: 1.0,
                ..AirSample::default()
            });
        },
        |s| {
            s.air_sample = Some(AirSample {
                true_mps: 1.0,
                ..AirSample::default()
            });
        },
        |s| {
            s.air_sample = Some(AirSample {
                static_air: Some(StaticAir::default()),
                ..AirSample::default()
            });
        },
        |s| {
            s.air_sample = Some(AirSample {
                static_air: Some(StaticAir {
                    pressure_pa: 1.0,
                    scale_height_m: 0.0,
                }),
                ..AirSample::default()
            });
        },
        |s| {
            s.air_sample = Some(AirSample {
                static_air: Some(StaticAir {
                    pressure_pa: 0.0,
                    scale_height_m: 1.0,
                }),
                ..AirSample::default()
            });
        },
        |s| s.air.indicated_mps = 1.0,
        |s| s.air.true_mps = 1.0,
        |s| s.air.true_rate_mps2 = 1.0,
        |s| s.air.static_pa = 1.0,
        |s| s.air.static_rate_paps = 1.0,
        |s| s.air.scale_height_m = 1.0,
        |s| s.air.age_s = 1.0,
        |s| s.air.primed = true,
        |s| s.air.static_primed = true,
        |s| s.held_static_pa = Some(0.0),
        |s| s.held_static_pa = Some(1.0),
        |s| s.lateral.turn_rate_integrator = 1.0,
        |s| s.energy.throttle_integrator = 1.0,
        |s| s.energy.pitch_integrator = 1.0,
        |s| s.energy.underspeed = true,
        |s| s.rates.integrators[0] = 1.0,
        |s| s.rates.integrators[1] = 1.0,
        |s| s.rates.integrators[2] = 1.0,
        |s| s.targets.bank = 1.0,
        |s| s.targets.pitch = 1.0,
        |s| s.targets.rates[0] = 1.0,
        |s| s.targets.rates[1] = 1.0,
        |s| s.targets.rates[2] = 1.0,
        |s| s.targets.airspeed_source = AirspeedSource::Measured,
    ];
    let baseline = encoded(&FixedWingRuntimeState::default());
    let mut seen = [baseline; 32];
    for (index, mutate) in mutations.into_iter().enumerate() {
        let mut runtime = FixedWingRuntimeState::default();
        mutate(&mut runtime);
        let bytes = encoded(&runtime);
        assert!(
            !seen[..=index].contains(&bytes),
            "field {index} is missing from the encoding or aliases another"
        );
        seen[index + 1] = bytes;
    }

    let mut ground_speed = FixedWingRuntimeState::default();
    ground_speed.targets.airspeed_source = AirspeedSource::GroundSpeed;
    assert!(!seen.contains(&encoded(&ground_speed)));
}

#[test]
fn a_short_buffer_takes_a_prefix_of_the_encoding() {
    let runtime = FixedWingRuntimeState {
        dt_sec: 0.25,
        ..FixedWingRuntimeState::default()
    };
    let full = encoded(&runtime);
    let mut short = [0_u8; 10];
    assert_eq!(runtime.encode_canonical(&mut short), 10);
    assert_eq!(short, full[..10]);
}

#[test]
fn the_kernel_offers_the_first_usable_airspeed_sensor() {
    let reading = |valid: bool, health: SensorHealth, pressure: f32| SensorReading {
        value: AirspeedData {
            air: AirData {
                dynamic_pressure: Some(crate::types::Pascals(pressure)),
                ..AirData::default()
            },
        },
        valid,
        health,
        ..SensorReading::default()
    };
    let set = |airspeeds: [SensorReading<AirspeedData>; MAX_AIRSPEED]| SensorSet {
        imus: Default::default(),
        gnss: Default::default(),
        mags: Default::default(),
        baros: Default::default(),
        airspeeds,
        geometry: None,
        geomagnetic: None,
    };
    let pressure_of = |sensors: &SensorSet| {
        sensors
            .primary_air_data()
            .and_then(|air| air.dynamic_pressure)
            .map(|pressure| pressure.0)
    };

    let none = set([SensorReading::default(), SensorReading::default()]);
    assert_eq!(pressure_of(&none), None);
    let first = set([
        reading(true, SensorHealth::Good, 100.0),
        reading(true, SensorHealth::Good, 200.0),
    ]);
    assert_eq!(pressure_of(&first), Some(100.0));
    // An invalid or unhealthy first sensor gives way to the second.
    for bad in [
        reading(false, SensorHealth::Good, 100.0),
        reading(true, SensorHealth::Degraded, 100.0),
    ] {
        let second = set([bad, reading(true, SensorHealth::Good, 200.0)]);
        assert_eq!(pressure_of(&second), Some(200.0));
    }
}
