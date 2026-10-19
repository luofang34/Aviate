//! WMM2025 against NOAA's published test values.
//!
//! `data/WMM2025_TestValues.txt` is the test-value file of the official
//! WMM2025 release (WMM2025COF.zip from
//! https://www.ncei.noaa.gov/products/world-magnetic-model), unmodified,
//! sha256 `e6975b093dddeb6153e0b23cc418425c438167e7c5b1dd795da379cb654f5819`.
//! Its rows cover the epoch (2025.0) and every half year after it
//! through 2029.5, so they exercise the secular-variation terms too.

#![allow(clippy::expect_used)]

use aviate_geomag::{wmm2025, GeodeticPoint, WMM2025};

const TEST_VALUES: &str = include_str!("data/WMM2025_TestValues.txt");

/// Published angles carry two decimals.
const ANGLE_TOL_DEG: f64 = 0.01;
/// Published components carry six decimals; the tolerance covers the
/// difference between this f64 synthesis and the reference build.
const COMPONENT_TOL_NT: f64 = 0.001;

struct Row {
    year: f64,
    height_km: f64,
    lat: f64,
    lon: f64,
    decl: f64,
    incl: f64,
    h: f64,
    x: f64,
    y: f64,
    z: f64,
    f: f64,
}

fn rows() -> Vec<Row> {
    TEST_VALUES
        .lines()
        .filter(|l| !l.trim_start().starts_with('#') && !l.trim().is_empty())
        .map(|l| {
            let v: Vec<f64> = l
                .split_whitespace()
                .map(|t| t.parse().expect("numeric field"))
                .collect();
            Row {
                year: v[0],
                height_km: v[1],
                lat: v[2],
                lon: v[3],
                decl: v[4],
                incl: v[5],
                h: v[6],
                x: v[7],
                y: v[8],
                z: v[9],
                f: v[10],
            }
        })
        .collect()
}

#[test]
fn every_published_test_value_is_reproduced() {
    let rows = rows();
    assert_eq!(rows.len(), 100, "the official file has 100 test points");
    assert!(rows.iter().any(|r| r.year == 2025.5));
    for r in &rows {
        let field = wmm2025(
            GeodeticPoint {
                latitude_deg: r.lat,
                longitude_deg: r.lon,
                height_m: r.height_km * 1_000.0,
            },
            r.year,
        )
        .expect("test points are inside the model domain");
        let at = format!("{} {} km {} {}", r.year, r.height_km, r.lat, r.lon);
        assert!(
            (field.declination_deg - r.decl).abs() <= ANGLE_TOL_DEG,
            "D at {at}: {field:?}"
        );
        assert!(
            (field.inclination_deg - r.incl).abs() <= ANGLE_TOL_DEG,
            "I at {at}: {field:?}"
        );
        for (name, got, want) in [
            ("X", field.north_nt, r.x),
            ("Y", field.east_nt, r.y),
            ("Z", field.down_nt, r.z),
            ("H", field.horizontal_nt, r.h),
            ("F", field.total_nt, r.f),
        ] {
            assert!(
                (got - want).abs() <= COMPONENT_TOL_NT,
                "{name} at {at}: got {got}, published {want}"
            );
        }
        assert_eq!(field.model, WMM2025);
    }
}
