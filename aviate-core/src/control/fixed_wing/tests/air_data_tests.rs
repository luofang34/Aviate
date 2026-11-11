//! The pitot-static latch and the airspeed fallback chain.

use super::super::air_data::{
    airspeeds_of, AirDataLatch, Airspeed, AirspeedSource, AIRSPEED_STALE_AFTER_S,
};
use super::super::AirSample;
use super::{cruise_state, pitot};
use crate::kernel::config::fixed_wing::tests::reference_params;
use crate::sensor::AirData;
use crate::state::StateValidFlags;
use crate::types::{Celsius, Pascals};

const TAU: f32 = 0.5;

fn sample(speed: f32) -> Option<AirSample> {
    airspeeds_of(&pitot(speed))
}

#[test]
fn dynamic_pressure_becomes_indicated_airspeed() {
    let air = airspeeds_of(&pitot(40.0)).expect("usable sample");
    assert!((air.indicated_mps - 40.0).abs() < 1.0e-3);
    // Without static pressure and temperature there is no density.
    assert_eq!(air.true_mps, air.indicated_mps);
}

#[test]
fn thin_air_makes_true_airspeed_exceed_indicated_airspeed() {
    // 3000 m in the standard atmosphere: 70.1 kPa and -4.5 C, a density
    // of 0.909 kg/m3 and a true-to-indicated ratio of 1.161.
    let air = airspeeds_of(&AirData {
        static_pressure: Some(Pascals(70_108.0)),
        temperature: Some(Celsius(-4.5)),
        ..pitot(50.0)
    })
    .expect("usable sample");
    assert!((air.indicated_mps - 50.0).abs() < 1.0e-3);
    assert!((air.true_mps / air.indicated_mps - 1.161).abs() < 0.002);
}

#[test]
fn unusable_samples_are_refused_or_repaired() {
    assert!(airspeeds_of(&AirData::default()).is_none());
    let not_a_number = AirData {
        dynamic_pressure: Some(Pascals(f32::NAN)),
        ..AirData::default()
    };
    assert!(airspeeds_of(&not_a_number).is_none());

    // A negative reading is zero airspeed, not the root of a negative.
    let negative = AirData {
        dynamic_pressure: Some(Pascals(-3.0)),
        ..AirData::default()
    };
    assert_eq!(
        airspeeds_of(&negative).map(|air| air.indicated_mps),
        Some(0.0)
    );

    // A static pressure that gives no density leaves true = indicated.
    for static_pressure in [0.0, f32::NAN] {
        let air = airspeeds_of(&AirData {
            static_pressure: Some(Pascals(static_pressure)),
            temperature: Some(Celsius(15.0)),
            ..pitot(30.0)
        })
        .expect("usable sample");
        assert_eq!(air.true_mps, air.indicated_mps);
    }
}

#[test]
fn latch_filters_fresh_samples_and_restarts_after_an_outage() {
    let mut latch = AirDataLatch::default();
    assert!(!latch.is_fresh());

    latch.observe(sample(50.0), 0.05, TAU);
    assert!(latch.is_fresh());
    assert!((latch.indicated_mps - 50.0).abs() < 1.0e-3);
    assert_eq!(latch.true_rate_mps2, 0.0);

    // A second sample 50 ms later moves the value by dt/(tau+dt) = 1/11
    // of the 11 m/s step. That move over the interval is 20 m/s2, and
    // the rate filter takes 1/11 of it.
    latch.observe(sample(61.0), 0.05, TAU);
    assert!((latch.indicated_mps - 51.0).abs() < 1.0e-3);
    assert!((latch.true_rate_mps2 - 20.0 / 11.0).abs() < 0.01);
    assert_eq!(latch.age_s, 0.0);

    // Silence ages the latch; past the limit it is not fresh.
    for _ in 0..20 {
        latch.observe(None, 0.05, TAU);
    }
    assert!((latch.age_s - 1.0).abs() < 1.0e-4);
    latch.observe(None, 0.05, TAU);
    assert!(!latch.is_fresh());
    // The age counter stops at twice the limit.
    for _ in 0..100 {
        latch.observe(None, 0.05, TAU);
    }
    assert_eq!(latch.age_s, 2.0 * AIRSPEED_STALE_AFTER_S);

    // The first sample after the outage is taken whole.
    latch.observe(sample(30.0), 0.05, TAU);
    assert!(latch.is_fresh());
    assert!((latch.indicated_mps - 30.0).abs() < 1.0e-3);
    assert_eq!(latch.true_rate_mps2, 0.0);
}

#[test]
fn a_sample_with_no_elapsed_time_replaces_the_value() {
    let mut latch = AirDataLatch::default();
    latch.observe(sample(50.0), 0.0, TAU);
    // No interval means no filter step and no rate: the value is taken.
    latch.observe(sample(52.0), 0.0, TAU);
    assert!((latch.indicated_mps - 52.0).abs() < 1.0e-3);
    assert_eq!(latch.true_rate_mps2, 0.0);
}

#[test]
fn airspeed_comes_from_the_best_source_that_is_available() {
    let params = reference_params();
    let mut latch = AirDataLatch::default();
    latch.observe(sample(48.0), 0.05, TAU);

    let measured = Airspeed::resolve(&latch, &cruise_state(), &params);
    assert_eq!(measured.source, AirspeedSource::Measured);
    assert!(measured.is_measured());
    assert!((measured.indicated - 48.0).abs() < 1.0e-3);

    // No probe: the speed over the ground, climb included.
    let mut climbing = cruise_state();
    climbing.velocity_ned[0].0 = 40.0;
    climbing.velocity_ned[2].0 = -30.0;
    let ground = Airspeed::resolve(&AirDataLatch::default(), &climbing, &params);
    assert_eq!(ground.source, AirspeedSource::GroundSpeed);
    assert!(!ground.is_measured());
    assert!((ground.true_speed - 50.0).abs() < 1.0e-3);

    // No probe and no velocity: the cruise airspeed of the description.
    let mut blind = cruise_state();
    blind.valid_flags.remove(StateValidFlags::VELOCITY);
    let assumed = Airspeed::resolve(&AirDataLatch::default(), &blind, &params);
    assert_eq!(assumed.source, AirspeedSource::Assumed);
    assert_eq!(assumed.indicated, params.cruise_airspeed.0);
    assert_eq!(AirspeedSource::default(), AirspeedSource::Assumed);
}

#[test]
fn a_slow_vehicle_divides_by_the_stall_speed() {
    let params = reference_params();
    let mut stopped = cruise_state();
    stopped.velocity_ned[0].0 = 3.0;
    let air = Airspeed::resolve(&AirDataLatch::default(), &stopped, &params);
    assert_eq!(air.divisor(&params), params.stall_airspeed.0);
    let fast = Airspeed::resolve(&AirDataLatch::default(), &cruise_state(), &params);
    assert_eq!(fast.divisor(&params), 55.0);
}

/// A sample of 50 m/s with a static port at `pressure` Pa and 15 C.
fn with_static(pressure: f32) -> Option<AirSample> {
    airspeeds_of(&AirData {
        static_pressure: Some(Pascals(pressure)),
        temperature: Some(Celsius(15.0)),
        ..pitot(50.0)
    })
}

#[test]
fn static_port_gives_a_height_scale_from_its_temperature() {
    // R * T / g at 15 C is 287.05 * 288.15 / 9.81 = 8432 m.
    let port = with_static(95_000.0)
        .and_then(|sample| sample.static_air)
        .expect("static port");
    assert_eq!(port.pressure_pa, 95_000.0);
    assert!((port.scale_height_m - 8_432.0).abs() < 2.0);

    // A probe that reports no temperature gets the standard one, and
    // its true airspeed stays the indicated airspeed.
    let untempered = airspeeds_of(&AirData {
        static_pressure: Some(Pascals(70_000.0)),
        ..pitot(50.0)
    })
    .expect("usable sample");
    let port = untempered.static_air.expect("static port");
    assert!((port.scale_height_m - 8_432.0).abs() < 2.0);
    assert_eq!(untempered.true_mps, untempered.indicated_mps);

    // A temperature that is not a number is not used either.
    let broken = airspeeds_of(&AirData {
        static_pressure: Some(Pascals(95_000.0)),
        temperature: Some(Celsius(f32::NAN)),
        ..pitot(50.0)
    })
    .expect("usable sample");
    assert!(broken
        .static_air
        .is_some_and(|port| (port.scale_height_m - 8_432.0).abs() < 2.0));

    assert!(sample(50.0).is_some_and(|sample| sample.static_air.is_none()));
}

#[test]
fn pressure_height_follows_the_static_port_through_the_filters() {
    let mut latch = AirDataLatch::default();
    assert!(latch.pressure_height(95_000.0).is_none());

    latch.observe(with_static(95_000.0), 0.04, TAU);
    let (height, climb) = latch.pressure_height(95_000.0).expect("static port in use");
    assert_eq!((height, climb), (0.0, 0.0));

    // The pressure falls by 12 Pa each second: at 95 kPa and 15 C that
    // is a climb of 8432 * 12 / 95000 = 1.065 m/s. After twenty filter
    // time constants the filtered rate has settled on it, and the
    // height lags the true one by one time constant of climb.
    let mut pressure = 95_000.0_f32;
    for _ in 0..250 {
        pressure -= 12.0 * 0.04;
        latch.observe(with_static(pressure), 0.04, TAU);
    }
    let (height, climb) = latch.pressure_height(95_000.0).expect("static port in use");
    assert!((climb - 1.065).abs() < 0.02, "climb {climb}");
    let true_height = 8_432.0 * (95_000.0_f32 / pressure).ln();
    assert!(
        (true_height - height - 1.065 * TAU).abs() < 0.1,
        "height {height}"
    );

    // A sample without the port ends the pressure height at once.
    latch.observe(sample(50.0), 0.04, TAU);
    assert!(latch.pressure_height(95_000.0).is_none());
    // The port restarts without a rate.
    latch.observe(with_static(94_000.0), 0.04, TAU);
    let (_, climb) = latch.pressure_height(95_000.0).expect("static port in use");
    assert_eq!(climb, 0.0);

    // A stale latch gives no height.
    for _ in 0..30 {
        latch.observe(None, 0.04, TAU);
    }
    assert!(latch.pressure_height(95_000.0).is_none());
}
