//! Simulated Earth-field tests.

use super::*;

const CALGARY: GeodeticPoint = GeodeticPoint {
    latitude_deg: 51.05,
    longitude_deg: -114.07,
    height_m: 1_045.0,
};

#[test]
fn the_field_at_the_origin_is_the_model_field_in_microtesla() {
    let Ok(earth) = EarthField::at_origin(CALGARY) else {
        panic!("today is inside the model validity");
    };
    let Ok(model) = wmm2025(CALGARY, earth.decimal_year) else {
        panic!("inside the model");
    };
    let [n, e, d] = earth.ned_ut();
    assert!((f64::from(n) - model.north_nt / 1_000.0).abs() < 1e-4);
    assert!((f64::from(e) - model.east_nt / 1_000.0).abs() < 1e-4);
    assert!((f64::from(d) - model.down_nt / 1_000.0).abs() < 1e-4);
    assert!((f64::from(earth.declination_deg()) - model.declination_deg).abs() < 1e-3);
}

#[test]
fn the_field_is_re_evaluated_only_after_a_long_move() {
    let Ok(mut earth) = EarthField::at_origin(CALGARY) else {
        panic!("today is inside the model validity");
    };
    let start = earth.ned_ut();
    earth.follow([500.0, 0.0, -50.0]);
    assert_eq!(earth.ned_ut(), start, "500 m keeps the field");
    earth.follow([50_000.0, 0.0, 0.0]);
    assert_ne!(earth.ned_ut(), start, "50 km north meets a different field");
}
