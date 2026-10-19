//! Evaluator domain and pole tests; the published test values are in
//! `tests/wmm2025_test_values.rs`.

use super::*;

fn point(latitude_deg: f64, longitude_deg: f64) -> GeodeticPoint {
    GeodeticPoint {
        latitude_deg,
        longitude_deg,
        height_m: 0.0,
    }
}

#[test]
fn dates_outside_the_validity_window_are_refused() {
    for year in [2024.99, 2030.01, f64::NAN] {
        assert!(matches!(
            wmm2025(point(47.4, 8.5), year),
            Err(GeomagError::OutsideValidity { .. })
        ));
    }
    assert!(wmm2025(point(47.4, 8.5), 2025.0).is_ok());
    assert!(wmm2025(point(47.4, 8.5), 2030.0).is_ok());
}

#[test]
fn points_outside_the_domain_are_refused() {
    let bad = [
        point(90.5, 0.0),
        point(f64::NAN, 0.0),
        point(0.0, f64::INFINITY),
        GeodeticPoint {
            height_m: 851_000.0,
            ..point(0.0, 0.0)
        },
        GeodeticPoint {
            height_m: -1_001.0,
            ..point(0.0, 0.0)
        },
    ];
    for p in bad {
        assert_eq!(wmm2025(p, 2026.0), Err(GeomagError::InvalidPoint), "{p:?}");
    }
}

#[test]
fn the_geographic_poles_give_a_finite_field() {
    for lat in [90.0, -90.0] {
        let field = wmm2025(point(lat, 0.0), 2027.5);
        assert!(field.is_ok());
        let Ok(field) = field else {
            return;
        };
        for v in [
            field.north_nt,
            field.east_nt,
            field.down_nt,
            field.declination_deg,
        ] {
            assert!(v.is_finite(), "lat {lat}: {field:?}");
        }
        // Near the north pole the field points steeply down, near the
        // south pole steeply up.
        assert!(field.inclination_deg.abs() > 70.0);
    }
}
