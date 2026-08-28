//! Sensor presence tests.

use super::SensorFields;

#[test]
fn complete_vectors_are_required() {
    let imu_only = SensorFields::from_bits(0b11_1111);
    assert!(imu_only.imu());
    assert!(!imu_only.mag());
    assert!(!imu_only.baro());

    assert!(!SensorFields::from_bits(0b11_1110).imu());
    assert!(!SensorFields::from_bits(0b10_1111).imu());
    assert!(!SensorFields::from_bits(0b110 << 6).mag());
}

#[test]
fn pressure_presence_bits_are_independent() {
    let static_only = SensorFields::from_bits(1 << 9);
    let dynamic_only = SensorFields::from_bits(1 << 10);
    let altitude_only = SensorFields::from_bits(1 << 11);

    assert!(static_only.baro());
    assert!(!static_only.differential_pressure());
    assert!(!static_only.pressure_altitude());
    assert!(!dynamic_only.baro());
    assert!(dynamic_only.differential_pressure());
    assert!(!dynamic_only.pressure_altitude());
    assert!(!altitude_only.baro());
    assert!(!altitude_only.differential_pressure());
    assert!(altitude_only.pressure_altitude());
}

#[test]
fn zero_and_full_bitmaps_declare_all_known_lanes() {
    for fields in [
        SensorFields::from_bits(0),
        SensorFields::from_bits(u32::MAX),
    ] {
        assert!(fields.imu());
        assert!(fields.mag());
        assert!(fields.baro());
        assert!(fields.differential_pressure());
        assert!(fields.pressure_altitude());
        assert_eq!(fields.known_presence_mask(), 0x0fff);
    }
}
