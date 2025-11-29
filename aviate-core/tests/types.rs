//! Tests for §6 Numeric Types
//!
//! Covers:
//! - Validated trait (is_valid, sanitize_or_default)
//! - Arithmetic operations (Add, Sub, Mul, Div, Neg)
//! - NaN/Inf handling across all newtypes

use aviate_core::types::{
    Celsius, Degrees, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Normalized,
    NormalizedSigned, Pascals, Radians, RadiansPerSecond, Scalar, Seconds, Validated,
};

// =============================================================================
// Validated Trait - is_valid()
// =============================================================================

#[test]
fn validated_accepts_finite_values() {
    assert!(Scalar::is_valid(&1.0));
    assert!(Scalar::is_valid(&0.0));
    assert!(Scalar::is_valid(&-1.0));
    assert!(Scalar::is_valid(&1e10));
    assert!(Scalar::is_valid(&1e-10));
}

#[test]
fn validated_rejects_nan() {
    assert!(!Scalar::is_valid(&Scalar::NAN));
    assert!(!Meters(Scalar::NAN).is_valid());
    assert!(!Radians(Scalar::NAN).is_valid());
    assert!(!Normalized(Scalar::NAN).is_valid());
}

#[test]
fn validated_rejects_infinity() {
    assert!(!Scalar::is_valid(&Scalar::INFINITY));
    assert!(!Scalar::is_valid(&Scalar::NEG_INFINITY));
    assert!(!MetersPerSecond(Scalar::INFINITY).is_valid());
    assert!(!RadiansPerSecond(Scalar::NEG_INFINITY).is_valid());
}

// =============================================================================
// Validated Trait - sanitize_or_default()
// =============================================================================

#[test]
fn sanitize_returns_value_when_valid() {
    let value: Scalar = 42.0;
    assert_eq!(value.sanitize_or_default(0.0), 42.0);

    let meters = Meters(100.0);
    assert_eq!(meters.sanitize_or_default(Meters(0.0)).0, 100.0);
}

#[test]
fn sanitize_returns_default_when_nan() {
    let nan: Scalar = Scalar::NAN;
    assert_eq!(nan.sanitize_or_default(0.0), 0.0);

    let nan_meters = Meters(Scalar::NAN);
    assert_eq!(nan_meters.sanitize_or_default(Meters(-1.0)).0, -1.0);
}

#[test]
fn sanitize_returns_default_when_infinite() {
    let inf: Scalar = Scalar::INFINITY;
    assert_eq!(inf.sanitize_or_default(99.0), 99.0);

    let neg_inf = RadiansPerSecond(Scalar::NEG_INFINITY);
    assert_eq!(neg_inf.sanitize_or_default(RadiansPerSecond(0.0)).0, 0.0);
}

// =============================================================================
// Arithmetic - Addition
// =============================================================================

#[test]
fn add_meters() {
    let a = Meters(10.0);
    let b = Meters(3.5);
    let result = a + b;
    assert!((result.0 - 13.5).abs() < 1e-6);
}

#[test]
fn add_negative_values() {
    let a = Radians(0.5);
    let b = Radians(-0.3);
    let result = a + b;
    assert!((result.0 - 0.2).abs() < 1e-6);
}

// =============================================================================
// Arithmetic - Subtraction
// =============================================================================

#[test]
fn sub_meters_per_second() {
    let a = MetersPerSecond(10.0);
    let b = MetersPerSecond(3.0);
    let result = a - b;
    assert!((result.0 - 7.0).abs() < 1e-6);
}

#[test]
fn sub_yields_negative() {
    let a = Meters(5.0);
    let b = Meters(10.0);
    let result = a - b;
    assert!((result.0 - (-5.0)).abs() < 1e-6);
}

// =============================================================================
// Arithmetic - Multiplication by Scalar
// =============================================================================

#[test]
fn mul_by_positive_scalar() {
    let value = Meters(5.0);
    let result = value * 3.0;
    assert!((result.0 - 15.0).abs() < 1e-6);
}

#[test]
fn mul_by_negative_scalar() {
    let value = RadiansPerSecond(2.0);
    let result = value * -0.5;
    assert!((result.0 - (-1.0)).abs() < 1e-6);
}

#[test]
fn mul_by_zero() {
    let value = Pascals(101325.0);
    let result = value * 0.0;
    assert!((result.0).abs() < 1e-6);
}

// =============================================================================
// Arithmetic - Division by Scalar
// =============================================================================

#[test]
fn div_by_positive_scalar() {
    let value = Meters(10.0);
    let result = value / 2.0;
    assert!((result.0 - 5.0).abs() < 1e-6);
}

#[test]
fn div_by_negative_scalar() {
    let value = Celsius(100.0);
    let result = value / -4.0;
    assert!((result.0 - (-25.0)).abs() < 1e-6);
}

#[test]
fn div_by_zero_yields_infinity() {
    let value = Meters(1.0);
    let result = value / 0.0;
    assert!(result.0.is_infinite());
    assert!(!result.is_valid());
}

// =============================================================================
// Arithmetic - Negation
// =============================================================================

#[test]
fn neg_positive_becomes_negative() {
    let value = Meters(5.0);
    let result = -value;
    assert!((result.0 - (-5.0)).abs() < 1e-6);
}

#[test]
fn neg_negative_becomes_positive() {
    let value = RadiansPerSecond(-3.0);
    let result = -value;
    assert!((result.0 - 3.0).abs() < 1e-6);
}

#[test]
fn neg_zero_stays_zero() {
    let value = Normalized(0.0);
    let result = -value;
    assert!((result.0).abs() < 1e-6);
}

// =============================================================================
// Edge Cases - Chained Operations
// =============================================================================

#[test]
fn chained_arithmetic_operations() {
    let a = Meters(10.0);
    let b = Meters(5.0);
    // (a + b) * 2 - a = (15) * 2 - 10 = 20
    let result = (a + b) * 2.0 - a;
    assert!((result.0 - 20.0).abs() < 1e-6);
}

// =============================================================================
// All Newtypes - Validated Coverage
// =============================================================================

#[test]
fn all_newtypes_implement_validated() {
    // Ensure all newtypes have Validated implemented
    assert!(Meters(1.0).is_valid());
    assert!(MetersPerSecond(1.0).is_valid());
    assert!(MetersPerSecondSquared(1.0).is_valid());
    assert!(RadiansPerSecond(1.0).is_valid());
    assert!(Radians(1.0).is_valid());
    assert!(Seconds(1.0).is_valid());
    assert!(Normalized(0.5).is_valid());
    assert!(NormalizedSigned(0.0).is_valid());
    assert!(Pascals(101325.0).is_valid());
    assert!(Celsius(20.0).is_valid());
    assert!(Degrees(45.0).is_valid());
    assert!(Microtesla(50.0).is_valid());
}

// =============================================================================
// FloatExt Trait Tests - Use explicit trait calls to ensure coverage
// =============================================================================

use aviate_core::types::{FloatExt, KilogramMeterSquared, Kilograms};
use core::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};

#[test]
fn floatext_sin_explicit() {
    // Explicit trait method calls ensure FloatExt is covered, not inherent f32 methods
    let val: Scalar = 0.0;
    assert!((FloatExt::sin(val) - 0.0).abs() < 1e-6);

    let val: Scalar = FRAC_PI_2;
    assert!((FloatExt::sin(val) - 1.0).abs() < 1e-6);

    let val: Scalar = PI;
    assert!(FloatExt::sin(val).abs() < 1e-5);
}

#[test]
fn floatext_cos_explicit() {
    let val: Scalar = 0.0;
    assert!((FloatExt::cos(val) - 1.0).abs() < 1e-6);

    let val: Scalar = FRAC_PI_2;
    assert!(FloatExt::cos(val).abs() < 1e-5);

    let val: Scalar = PI;
    assert!((FloatExt::cos(val) - (-1.0)).abs() < 1e-5);
}

#[test]
fn floatext_asin_explicit() {
    let val: Scalar = 0.0;
    assert!((FloatExt::asin(val) - 0.0).abs() < 1e-6);

    let val: Scalar = 1.0;
    assert!((FloatExt::asin(val) - FRAC_PI_2).abs() < 1e-5);

    let val: Scalar = -1.0;
    assert!((FloatExt::asin(val) - (-FRAC_PI_2)).abs() < 1e-5);
}

#[test]
fn floatext_sqrt_explicit() {
    let val: Scalar = 4.0;
    assert!((FloatExt::sqrt(val) - 2.0).abs() < 1e-6);

    let val: Scalar = 0.0;
    assert!((FloatExt::sqrt(val) - 0.0).abs() < 1e-6);
}

#[test]
fn floatext_atan2_explicit() {
    // atan2(0, 1) = 0
    let y: Scalar = 0.0;
    let x: Scalar = 1.0;
    assert!((FloatExt::atan2(y, x) - 0.0).abs() < 1e-6);

    // atan2(1, 0) = π/2
    let y: Scalar = 1.0;
    let x: Scalar = 0.0;
    assert!((FloatExt::atan2(y, x) - FRAC_PI_2).abs() < 1e-5);

    // atan2(1, 1) = π/4
    let y: Scalar = 1.0;
    let x: Scalar = 1.0;
    assert!((FloatExt::atan2(y, x) - FRAC_PI_4).abs() < 1e-5);
}

#[test]
fn floatext_powf_explicit() {
    let base: Scalar = 2.0;
    assert!((FloatExt::powf(base, 3.0) - 8.0).abs() < 1e-5);

    let base: Scalar = 4.0;
    assert!((FloatExt::powf(base, 0.5) - 2.0).abs() < 1e-5);
}

// =============================================================================
// Additional Arithmetic Operations - Cover missing types
// =============================================================================

#[test]
fn neg_degrees() {
    let d = Degrees(45.0);
    let neg_d = -d;
    assert!((neg_d.0 - (-45.0)).abs() < 1e-6);
}

#[test]
fn neg_microtesla() {
    let m = Microtesla(50.0);
    let neg_m = -m;
    assert!((neg_m.0 - (-50.0)).abs() < 1e-6);
}

#[test]
fn neg_kilograms() {
    let k = Kilograms(10.0);
    let neg_k = -k;
    assert!((neg_k.0 - (-10.0)).abs() < 1e-6);
}

#[test]
fn neg_kilogram_meter_squared() {
    let kms = KilogramMeterSquared(5.0);
    let neg_kms = -kms;
    assert!((neg_kms.0 - (-5.0)).abs() < 1e-6);
}

#[test]
fn neg_meters_per_second_squared() {
    let a = MetersPerSecondSquared(9.8);
    let neg_a = -a;
    assert!((neg_a.0 - (-9.8)).abs() < 1e-6);
}

#[test]
fn neg_normalized_signed() {
    let ns = NormalizedSigned(0.5);
    let neg_ns = -ns;
    assert!((neg_ns.0 - (-0.5)).abs() < 1e-6);
}

#[test]
fn neg_meters_per_second() {
    let v = MetersPerSecond(10.0);
    let neg_v = -v;
    assert!((neg_v.0 - (-10.0)).abs() < 1e-6);
}

#[test]
fn mul_microtesla() {
    let m = Microtesla(25.0);
    let result = m * 2.0;
    assert!((result.0 - 50.0).abs() < 1e-6);
}

#[test]
fn mul_kilograms() {
    let k = Kilograms(5.0);
    let result = k * 3.0;
    assert!((result.0 - 15.0).abs() < 1e-6);
}

#[test]
fn mul_kilogram_meter_squared() {
    let kms = KilogramMeterSquared(2.0);
    let result = kms * 4.0;
    assert!((result.0 - 8.0).abs() < 1e-6);
}

#[test]
fn mul_meters_per_second_squared() {
    let a = MetersPerSecondSquared(5.0);
    let result = a * 2.0;
    assert!((result.0 - 10.0).abs() < 1e-6);
}

#[test]
fn div_degrees() {
    let d = Degrees(90.0);
    let result = d / 2.0;
    assert!((result.0 - 45.0).abs() < 1e-6);
}

#[test]
fn div_kilograms() {
    let k = Kilograms(10.0);
    let result = k / 2.0;
    assert!((result.0 - 5.0).abs() < 1e-6);
}

#[test]
fn div_kilogram_meter_squared() {
    let kms = KilogramMeterSquared(8.0);
    let result = kms / 4.0;
    assert!((result.0 - 2.0).abs() < 1e-6);
}

#[test]
fn add_kilograms() {
    let k1 = Kilograms(5.0);
    let k2 = Kilograms(3.0);
    let result = k1 + k2;
    assert!((result.0 - 8.0).abs() < 1e-6);
}

#[test]
fn add_radians_per_second() {
    let r1 = RadiansPerSecond(1.0);
    let r2 = RadiansPerSecond(0.5);
    let result = r1 + r2;
    assert!((result.0 - 1.5).abs() < 1e-6);
}

#[test]
fn sub_normalized() {
    let n1 = Normalized(0.8);
    let n2 = Normalized(0.3);
    let result = n1 - n2;
    assert!((result.0 - 0.5).abs() < 1e-6);
}

#[test]
fn sub_seconds() {
    let s1 = Seconds(10.0);
    let s2 = Seconds(3.0);
    let result = s1 - s2;
    assert!((result.0 - 7.0).abs() < 1e-6);
}

#[test]
fn sub_meters_per_second_explicit() {
    let v1 = MetersPerSecond(20.0);
    let v2 = MetersPerSecond(5.0);
    let result = v1 - v2;
    assert!((result.0 - 15.0).abs() < 1e-6);
}

// =============================================================================
// Validated sanitize_or_default for specific types
// =============================================================================

#[test]
fn sanitize_radians() {
    let valid = Radians(1.0);
    assert_eq!(valid.sanitize_or_default(Radians(0.0)).0, 1.0);

    let nan = Radians(Scalar::NAN);
    assert_eq!(nan.sanitize_or_default(Radians(0.0)).0, 0.0);
}

#[test]
fn sanitize_normalized_signed() {
    let valid = NormalizedSigned(0.5);
    assert_eq!(valid.sanitize_or_default(NormalizedSigned(0.0)).0, 0.5);

    let nan = NormalizedSigned(Scalar::NAN);
    assert_eq!(nan.sanitize_or_default(NormalizedSigned(0.0)).0, 0.0);
}

#[test]
fn sanitize_degrees() {
    let valid = Degrees(45.0);
    assert_eq!(valid.sanitize_or_default(Degrees(0.0)).0, 45.0);

    let inf = Degrees(Scalar::INFINITY);
    assert_eq!(inf.sanitize_or_default(Degrees(0.0)).0, 0.0);
}

#[test]
fn sanitize_seconds() {
    let valid = Seconds(1.0);
    assert_eq!(valid.sanitize_or_default(Seconds(0.0)).0, 1.0);

    let nan = Seconds(Scalar::NAN);
    assert_eq!(nan.sanitize_or_default(Seconds(0.0)).0, 0.0);
}

#[test]
fn validated_scalar_is_valid() {
    // Test the is_valid associated function on Scalar (f32)
    assert!(Scalar::is_valid(&1.0));
    assert!(!Scalar::is_valid(&Scalar::NAN));
    assert!(!Scalar::is_valid(&Scalar::INFINITY));
}

#[test]
fn validated_celsius_is_valid() {
    assert!(Celsius(20.0).is_valid());
    assert!(!Celsius(Scalar::NAN).is_valid());
    assert!(!Celsius(Scalar::INFINITY).is_valid());
}
