//! Origin parsing tests.

use super::*;

#[test]
fn a_full_origin_parses() {
    assert_eq!(
        GeoOrigin::parse("47.3977420, 8.5459921, 500.5"),
        Ok(GeoOrigin {
            latitude_deg: 47.397_742,
            longitude_deg: 8.545_992_1,
            elevation_m: 500.5,
        })
    );
}

#[test]
fn a_malformed_origin_is_refused_not_defaulted() {
    assert_eq!(
        GeoOrigin::parse("47.39,8.54"),
        Err(OriginError::FieldCount(2))
    );
    assert!(matches!(
        GeoOrigin::parse("47.39,east,488"),
        Err(OriginError::NotANumber("longitude", _))
    ));
    assert!(matches!(
        GeoOrigin::parse("NaN,8.54,488"),
        Err(OriginError::NotANumber("latitude", _))
    ));
    assert!(matches!(
        GeoOrigin::parse("91,8.54,488"),
        Err(OriginError::OutOfRange("latitude", _))
    ));
}
