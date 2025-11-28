//! Tests for §6 Numeric Types
//!
//! Covers:
//! - Validated trait (is_valid, sanitize_or_default)
//! - Arithmetic operations (Add, Sub, Mul, Div, Neg)
//! - NaN/Inf handling across all newtypes

use aviate_core::types::{
    Scalar, Meters, MetersPerSecond, MetersPerSecondSquared,
    RadiansPerSecond, Radians, Seconds, Normalized, NormalizedSigned,
    Pascals, Celsius, Degrees, Microtesla, Validated,
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
