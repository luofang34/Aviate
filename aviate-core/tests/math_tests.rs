//! Tests for math module
//!
//! Covers uncovered lines in math.rs:
//! - Vector3::zero
//! - Matrix::sub
//! - Matrix::to_vec3
//! - Quaternion::normalize identity fallback
//! - Quaternion::to_euler gimbal lock handling
//! - Quaternion::default

use aviate_core::math::{Matrix, Quaternion, Vector3};
use aviate_core::types::Scalar;
use core::f32::consts::FRAC_PI_2;

// =============================================================================
// Vector3 Tests
// =============================================================================

#[test]
fn vector3_zero() {
    let v = Vector3::<Scalar>::zero();
    assert_eq!(v.x, 0.0);
    assert_eq!(v.y, 0.0);
    assert_eq!(v.z, 0.0);
}

#[test]
fn vector3_new() {
    let v = Vector3::new(1.0, 2.0, 3.0);
    assert_eq!(v.x, 1.0);
    assert_eq!(v.y, 2.0);
    assert_eq!(v.z, 3.0);
}

// =============================================================================
// Matrix Tests
// =============================================================================

#[test]
fn matrix_sub() {
    let mut a = Matrix::<2, 2>::zero();
    a.data[0][0] = 5.0;
    a.data[0][1] = 3.0;
    a.data[1][0] = 2.0;
    a.data[1][1] = 1.0;

    let mut b = Matrix::<2, 2>::zero();
    b.data[0][0] = 2.0;
    b.data[0][1] = 1.0;
    b.data[1][0] = 1.0;
    b.data[1][1] = 1.0;

    let c = a.sub(&b);

    assert_eq!(c.data[0][0], 3.0);
    assert_eq!(c.data[0][1], 2.0);
    assert_eq!(c.data[1][0], 1.0);
    assert_eq!(c.data[1][1], 0.0);
}

#[test]
fn matrix_sub_negative_result() {
    let a = Matrix::<2, 2>::zero();
    let mut b = Matrix::<2, 2>::zero();
    b.data[0][0] = 5.0;

    let c = a.sub(&b);
    assert_eq!(c.data[0][0], -5.0);
}

#[test]
fn matrix_to_vec3() {
    let mut m = Matrix::<3, 1>::zero();
    m.data[0][0] = 1.0;
    m.data[1][0] = 2.0;
    m.data[2][0] = 3.0;

    let v = m.to_vec3();

    assert_eq!(v.x, 1.0);
    assert_eq!(v.y, 2.0);
    assert_eq!(v.z, 3.0);
}

// =============================================================================
// Quaternion Tests
// =============================================================================

#[test]
fn quaternion_default() {
    let q = Quaternion::default();
    assert_eq!(q, Quaternion::IDENTITY);
    assert_eq!(q.w, 1.0);
    assert_eq!(q.x, 0.0);
    assert_eq!(q.y, 0.0);
    assert_eq!(q.z, 0.0);
}

#[test]
fn quaternion_normalize_identity_fallback() {
    // Create a zero-length quaternion
    let q = Quaternion {
        w: 0.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    // normalize() should return IDENTITY when magnitude is too small
    let normalized = q.normalize();

    assert_eq!(normalized, Quaternion::IDENTITY);
}

#[test]
fn quaternion_normalize_near_zero() {
    // Create a very small quaternion
    let q = Quaternion {
        w: 1e-10,
        x: 1e-10,
        y: 1e-10,
        z: 1e-10,
    };

    let normalized = q.normalize();

    // Should fall back to identity
    assert_eq!(normalized, Quaternion::IDENTITY);
}

#[test]
fn quaternion_to_euler_gimbal_lock_positive() {
    // Create a quaternion that forces sinp > 1.0 (or exactly 1.0 via clamp)
    // For gimbal lock at +90° pitch: q = (cos(45°), 0, sin(45°), 0) = (0.707, 0, 0.707, 0)
    // sinp = 2 * (w * y - z * x) = 2 * 0.707 * 0.707 = 1.0
    let q = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), FRAC_PI_2);

    let (roll, pitch, yaw) = q.to_euler();

    // At gimbal lock, pitch should be ±π/2
    assert!(
        (pitch.abs() - FRAC_PI_2).abs() < 0.1,
        "Pitch {} should be near ±π/2",
        pitch
    );
    let _ = (roll, yaw); // Suppress warnings
}

#[test]
fn quaternion_to_euler_gimbal_lock_negative() {
    // Create a quaternion at negative gimbal lock (pitch = -90 degrees)
    let q = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), -FRAC_PI_2);

    let (_roll, pitch, _yaw) = q.to_euler();

    // Should be near -π/2
    assert!(
        (pitch.abs() - FRAC_PI_2).abs() < 0.1,
        "Pitch {} should be near ±π/2",
        pitch
    );
}

#[test]
fn quaternion_to_euler_gimbal_lock_positive_explicit() {
    // Construct a quaternion where sinp is forced to be > 1.0 (will be clamped)
    // We artificially create non-normalized quaternion that when calculated
    // gives sinp = 2 * (w * y - z * x) >= 1.0
    // Pure pitch +90°: w=cos(45°)≈0.707, y=sin(45°)≈0.707, x=z=0
    // sinp = 2 * 0.707 * 0.707 ≈ 1.0
    // To guarantee > 1.0, we slightly increase y
    let q = Quaternion {
        w: 0.707,
        x: 0.0,
        y: 0.71, // Slightly larger than normalized
        z: 0.0,
    };

    let (_roll, pitch, _yaw) = q.to_euler();

    // sinp = 2 * 0.707 * 0.71 = 1.004 > 1.0, triggers positive gimbal lock
    // Pitch should be exactly +π/2
    assert!(
        (pitch - FRAC_PI_2).abs() < 0.01,
        "Pitch {} should be +π/2",
        pitch
    );
}

#[test]
fn quaternion_to_euler_gimbal_lock_negative_explicit() {
    // Construct quaternion where sinp <= -1.0
    // Pure pitch -90°: w=cos(-45°)=cos(45°)≈0.707, y=sin(-45°)=-0.707
    let q = Quaternion {
        w: 0.707,
        x: 0.0,
        y: -0.71, // Slightly more negative
        z: 0.0,
    };

    let (_roll, pitch, _yaw) = q.to_euler();

    // sinp = 2 * 0.707 * (-0.71) = -1.004 < -1.0, triggers negative gimbal lock
    // Pitch should be exactly -π/2
    assert!(
        (pitch + FRAC_PI_2).abs() < 0.01,
        "Pitch {} should be -π/2",
        pitch
    );
}

#[test]
fn quaternion_to_euler_normal() {
    // Normal rotation, no gimbal lock
    let q = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), 0.5);

    let (roll, pitch, yaw) = q.to_euler();

    // Roll should be ~0.5
    assert!((roll - 0.5).abs() < 0.05, "Roll {} should be ~0.5", roll);
    // Pitch and yaw should be ~0
    assert!(pitch.abs() < 0.1);
    assert!(yaw.abs() < 0.1);
}

#[test]
fn quaternion_identity_to_euler() {
    let q = Quaternion::IDENTITY;

    let (roll, pitch, yaw) = q.to_euler();

    assert!(roll.abs() < 0.001);
    assert!(pitch.abs() < 0.001);
    assert!(yaw.abs() < 0.001);
}

// =============================================================================
// Matrix Add and Mul Scalar (already covered but adding for completeness)
// =============================================================================

#[test]
fn matrix_add() {
    let mut a = Matrix::<2, 2>::zero();
    a.data[0][0] = 1.0;

    let mut b = Matrix::<2, 2>::zero();
    b.data[0][0] = 2.0;

    let c = a.add(&b);
    assert_eq!(c.data[0][0], 3.0);
}

#[test]
fn matrix_mul_scalar() {
    let mut m = Matrix::<2, 2>::zero();
    m.data[0][0] = 2.0;
    m.data[1][1] = 3.0;

    let scaled = m.mul_scalar(2.0);

    assert_eq!(scaled.data[0][0], 4.0);
    assert_eq!(scaled.data[1][1], 6.0);
}

#[test]
fn matrix_mat_mul() {
    let mut a = Matrix::<2, 2>::zero();
    a.data[0][0] = 1.0;
    a.data[0][1] = 2.0;
    a.data[1][0] = 3.0;
    a.data[1][1] = 4.0;

    let mut b = Matrix::<2, 2>::zero();
    b.data[0][0] = 1.0;
    b.data[1][1] = 1.0;

    let c = a.mat_mul(&b);

    // A * I = A
    assert_eq!(c.data[0][0], 1.0);
    assert_eq!(c.data[0][1], 2.0);
    assert_eq!(c.data[1][0], 3.0);
    assert_eq!(c.data[1][1], 4.0);
}
