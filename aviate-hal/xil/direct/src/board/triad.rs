//! Closed-form `no_std` TRIAD attitude quaternion seed from gravity and Earth field.

use aviate_core::math::{Quaternion, Vector3};

/// Compute the initial body-FRD-to-world-NED quaternion from accelerometer and magnetometer samples.
///
/// Roll and pitch align the measured reaction force (`accel_body`, `[0, 0, -g]` at level rest)
/// with the local NED vertical. Yaw uses the tilt-compensated horizontal magnetic field plus
/// `declination_rad` (positive east) so it agrees with `Ekf::update_mag_state`.
pub(super) fn triad_init_quat(
    accel_body: [f32; 3],
    mag_body: [f32; 3],
    declination_rad: f32,
) -> Quaternion {
    let [ax, ay, az] = accel_body;
    let a_norm = libm::sqrtf(ax * ax + ay * ay + az * az);
    if !a_norm.is_finite() || a_norm < 1.0 {
        return Quaternion::IDENTITY;
    }

    let [mx, my, mz] = mag_body;
    let m_norm = libm::sqrtf(mx * mx + my * my + mz * mz);
    if !m_norm.is_finite() || m_norm < 1.0 {
        return Quaternion::IDENTITY;
    }

    let roll = libm::atan2f(-ay, -az);
    let pitch = libm::atan2f(ax, libm::sqrtf(ay * ay + az * az));

    let (sin_r, cos_r) = (libm::sinf(roll), libm::cosf(roll));
    let (sin_p, cos_p) = (libm::sinf(pitch), libm::cosf(pitch));
    let mag_n_level = cos_p * mx + sin_p * sin_r * my + sin_p * cos_r * mz;
    let mag_e_level = cos_r * my - sin_r * mz;
    let yaw = libm::atan2f(-mag_e_level, mag_n_level) + declination_rad;

    let qz = Quaternion::from_axis_angle(Vector3::new(0.0, 0.0, 1.0), yaw);
    let qy = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), pitch);
    let qx = Quaternion::from_axis_angle(Vector3::new(1.0, 0.0, 0.0), roll);
    qz.mul(&qy).mul(&qx)
}
