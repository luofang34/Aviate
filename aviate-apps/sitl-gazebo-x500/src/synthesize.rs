//! Ground-truth → sensor synthesis for the X500 Gazebo SITL FC.
//!
//! Reads the Gazebo plugin's `AviateModelState` (ENU world, FLU body
//! pose / velocity / angular velocity), converts to Aviate's NED+FRD
//! conventions, and builds a `SimSensorPacket` (IMU + baro + mag +
//! GNSS) for the kernel. Applies tier-matched Gaussian noise on top
//! when `AVIATE_SENSOR_NOISE` is set.
//!
//! Frame conversion math is unit-tested at the bottom of this file —
//! a sign error here would produce a vehicle that flies but with a
//! subtly-wrong attitude estimate the kernel can't tell from real
//! physics.

use aviate_backend_gz::{enu_to_ned_f32, AviateModelState};
use aviate_core::kernel::config::ActuatorCurveKind;
use aviate_core::types::NormalizedThrust;
use aviate_hal_xil::sim_types::{
    SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket, SimTimestampUs,
};

use crate::noise::{
    apply_baro_noise, apply_gnss_noise, apply_imu_noise, apply_mag_noise, NoiseRng, NoiseTier,
};

/// X500 rotor `maxRotVelocity` from the PX4-gazebo-models SDF
/// (`motorConstant` 8.55e-6 N/(rad/s)² × 1000² × 4 motors ≈ 34 N max
/// thrust against ≈20 N weight, so a thrust-to-weight ratio of ~1.7
/// at full motor output). Maps `Normalized([0.0, 1.0])` actuator
/// output linearly to rotor speed.
pub const MOTOR_MAX_RPS: f64 = 1000.0;

/// Zurich reference for the auto-generated SITL world.
pub const REF_LATITUDE_DEG: f64 = 47.3977419;
pub const REF_LONGITUDE_DEG: f64 = 8.5455938;
pub const REF_ELEVATION_M: f32 = 488.0;
/// Magnetic field at Zurich, NED in microtesla (approximate).
/// Reference magnetic field in NED, microtesla.
///
/// The east component is zero on purpose: the kernel's mag heading
/// update doesn't compensate for declination, so any non-zero E
/// component produces a persistent mag innovation that the EKF
/// chases indefinitely and the controller perceives as a slow
/// yaw drift after takeoff. SITL is the wrong place to import a
/// realistic Earth-field model — there is no GPS-driven IGRF
/// correction in the loop. Cert work will land that pairing, then
/// this constant can be replaced by a runtime lookup.
pub const MAG_NED_UT: [f32; 3] = [21.0, 0.0, 43.0];
/// ISA sea-level pressure (Pa) and temperature (Celsius).
pub const ISA_P0_PA: f32 = 101_325.0;
pub const ISA_T0_C: f32 = 15.0;
/// Standard gravity (m/s², NED-down convention so accel under hover ≈ -9.81 on z).
pub const GRAVITY: f32 = 9.81;

/// Apply tier-matched additive Gaussian noise to every populated
/// sensor channel in `packet` in place. No-op for `NoiseTier::Off`.
/// Reference latitude for GNSS noise is the SITL world origin.
pub fn apply_packet_noise(packet: &mut SimSensorPacket, tier: NoiseTier, rng: &mut NoiseRng) {
    if tier == NoiseTier::Off {
        return;
    }
    if let Some(ref mut imu) = packet.imu {
        apply_imu_noise(imu, tier, rng);
    }
    if let Some(ref mut baro) = packet.baro {
        apply_baro_noise(baro, tier, rng);
    }
    if let Some(ref mut mag) = packet.mag {
        apply_mag_noise(mag, tier, rng);
    }
    if let Some(ref mut gnss) = packet.gnss {
        apply_gnss_noise(gnss, tier, rng, REF_LATITUDE_DEG);
    }
}

/// Build a `SimSensorPacket` from the latest ground-truth model state.
/// Coordinate conventions match the rest of aviate-core: NED for
/// position / velocity / accel; body frame for gyro.
///
/// `prev_ned_vel` is the NED-frame velocity this function returned
/// on its previous call (so the caller — typically the FC main
/// loop — passes its own state forward). The argument is required
/// because gz's `WorldLinearVelocity` component returns zero on
/// macOS for the same reason `WorldAngularVelocity` does; reading
/// `state.vel` directly leaves the EKF with no velocity signal at
/// all and the controller flying blind during fast transients.
/// Computing velocity from a position derivative and acceleration
/// from a velocity derivative keeps the controllers sim-agnostic
/// — they receive the same shape of sensor data they would from a
/// real IMU + GNSS pair.
///
/// Returns `(packet, current_ned_vel)`; the caller threads the
/// velocity through to the next call.
pub fn synthesize_packet(
    state: &AviateModelState,
    prev: Option<&AviateModelState>,
    prev_t_us: u64,
    prev_ned_vel: [f32; 3],
) -> (SimSensorPacket, [f32; 3]) {
    let now_ned_pos = enu_to_ned_f32(state.pos);
    let now_us = state.time_us;
    let dt = match prev {
        Some(_) if now_us > prev_t_us => (now_us - prev_t_us) as f32 * 1e-6,
        _ => 0.001,
    };

    // Velocity from finite difference of position (the gz
    // `state.vel` field is reported as zero on macOS — see
    // doc-comment above).
    let now_ned_vel = match prev {
        Some(p) if now_us > prev_t_us && dt > 1e-6 => {
            let prev_ned_pos = enu_to_ned_f32(p.pos);
            [
                (now_ned_pos[0] - prev_ned_pos[0]) / dt,
                (now_ned_pos[1] - prev_ned_pos[1]) / dt,
                (now_ned_pos[2] - prev_ned_pos[2]) / dt,
            ]
        }
        _ => [0.0, 0.0, 0.0],
    };

    // gz's `WorldAngularVelocity` component reports 0 in our tests
    // even when the model is actually rotating (visible in the
    // attitude quaternion changing between cycles). Empirically
    // the FC reading `state.ang_vel` returned `(0, 0, 0)` while
    // `state.quat` showed sustained drift in y, so we cannot trust
    // that component as a gyro source.
    //
    // Instead, compute the body-frame angular velocity from
    // successive attitudes:
    //
    //   q_delta = q_new ⊗ q_old.conjugate
    //   ω_world ≈ 2 · q_delta.imag / dt    (small-angle approx)
    //
    // Then rotate ω_world (in NED+FRD world after the ENU→NED
    // swap) into the body frame via `q_ned^T`.
    let q_enu = [
        state.quat[0] as f32,
        state.quat[1] as f32,
        state.quat[2] as f32,
        state.quat[3] as f32,
    ];
    let q_ned = enu_quat_to_ned(q_enu);
    let gyro = match prev {
        Some(p) if now_us > prev_t_us => {
            let q_enu_prev = [
                p.quat[0] as f32,
                p.quat[1] as f32,
                p.quat[2] as f32,
                p.quat[3] as f32,
            ];
            let q_ned_prev = enu_quat_to_ned(q_enu_prev);
            // q_delta = q_ned * q_ned_prev.conjugate
            let qd_w = q_ned[0] * q_ned_prev[0]
                + q_ned[1] * q_ned_prev[1]
                + q_ned[2] * q_ned_prev[2]
                + q_ned[3] * q_ned_prev[3];
            let qd_x = -q_ned[0] * q_ned_prev[1] + q_ned[1] * q_ned_prev[0]
                - q_ned[2] * q_ned_prev[3]
                + q_ned[3] * q_ned_prev[2];
            let qd_y =
                -q_ned[0] * q_ned_prev[2] + q_ned[1] * q_ned_prev[3] + q_ned[2] * q_ned_prev[0]
                    - q_ned[3] * q_ned_prev[1];
            let qd_z = -q_ned[0] * q_ned_prev[3] - q_ned[1] * q_ned_prev[2]
                + q_ned[2] * q_ned_prev[1]
                + q_ned[3] * q_ned_prev[0];
            // Force shortest-arc (qd and -qd are the same rotation;
            // we want the imaginary part in the +w hemisphere).
            let sign = if qd_w >= 0.0 { 1.0 } else { -1.0 };
            let omega_world_ned = [
                2.0 * sign * qd_x / dt,
                2.0 * sign * qd_y / dt,
                2.0 * sign * qd_z / dt,
            ];
            rotate_world_to_body(q_ned, omega_world_ned)
        }
        _ => [0.0; 3],
    };

    // Inertial acceleration from the velocity finite difference —
    // the same baseline the velocity above is derived over. gz
    // positions are exact doubles (no measurement noise), so the
    // second difference is clean; the ±3 g clamp guards the spikes
    // a physics-step time hiccup can produce. Feeding zero here
    // instead (gravity-only IMU) starves the EKF predict step of
    // every vertical transient: the velocity estimate then moves
    // only at the GNSS fusion gain, lags hard braking by seconds,
    // and a braked climb descends into the ground while the filter
    // still believes it is climbing.
    let accel_inertial_ned = match prev {
        Some(_) if dt > 1e-6 => {
            let lim = 3.0 * GRAVITY;
            [
                ((now_ned_vel[0] - prev_ned_vel[0]) / dt).clamp(-lim, lim),
                ((now_ned_vel[1] - prev_ned_vel[1]) / dt).clamp(-lim, lim),
                ((now_ned_vel[2] - prev_ned_vel[2]) / dt).clamp(-lim, lim),
            ]
        }
        _ => [0.0, 0.0, 0.0],
    };
    // q_ned was computed above for the gyro conversion; re-use it.
    let accel_ned = [
        accel_inertial_ned[0],
        accel_inertial_ned[1],
        accel_inertial_ned[2] - GRAVITY,
    ];
    let accel_body = rotate_world_to_body(q_ned, accel_ned);

    let imu = SimImuData {
        accel: accel_body,
        gyro,
        temperature: Some(25.0),
    };

    // Barometric pressure from NED-Z (down-positive): altitude_msl =
    // ref_elev - down. Use ISA isothermal approximation; good to
    // hundreds of meters for SITL purposes.
    let altitude_msl_m = REF_ELEVATION_M - now_ned_pos[2];
    let baro = SimBaroData {
        pressure_pa: isa_pressure(altitude_msl_m),
        temperature_c: ISA_T0_C,
    };

    // Magnetometer: constant Earth field rotated into body frame.
    let mag_body = rotate_world_to_body(q_ned, MAG_NED_UT);
    let mag = SimMagData { field_ut: mag_body };

    // GNSS: convert NED offset to lat/lon delta around Zurich
    // reference. Flat-earth approximation, more than accurate enough
    // for sub-100m XIL flights.
    let lat_per_m = 1.0 / 111_111.0;
    let lon_per_m = 1.0 / (111_111.0 * (REF_LATITUDE_DEG.to_radians()).cos());
    let gnss = SimGnssData {
        lat_deg: REF_LATITUDE_DEG + (now_ned_pos[0] as f64) * lat_per_m,
        lon_deg: REF_LONGITUDE_DEG + (now_ned_pos[1] as f64) * lon_per_m,
        alt_m: altitude_msl_m,
        // Aviate kernel uses local NED — pass ground-truth NED
        // position directly so the GNSS update agrees with the EKF
        // frame; the lat/lon/alt fields stay for telemetry parity
        // with real receivers.
        position_ned: now_ned_pos,
        vel_ned: now_ned_vel,
        fix: SimGnssFix::ThreeD,
        satellites: 14,
        h_acc: 0.5,
        v_acc: 0.8,
    };

    let packet = SimSensorPacket {
        timestamp_us: now_us as SimTimestampUs,
        imu: Some(imu),
        baro: Some(baro),
        mag: Some(mag),
        gnss: Some(gnss),
    };
    (packet, now_ned_vel)
}

/// Convert an attitude quaternion produced by gz-sim (ENU world,
/// FLU body: forward-left-up) to Aviate's convention (NED world,
/// FRD body: forward-right-down). Both encode the rotation from
/// body to world; world AND body frames differ.
///
/// Two basis swaps are needed:
///
/// * **World ENU → NED**: change-of-basis `[[0,1,0],[1,0,0],
///   [0,0,-1]]` — det +1, a 180° rotation about `(1,1,0)/√2`,
///   rotor `q_ENU→NED = (0, 1/√2, 1/√2, 0)`.
/// * **Body FRD → FLU**: 180° rotation about the forward (X)
///   axis (negates Y and Z), rotor `q_FRD→FLU = (0, 1, 0, 0)`.
///
/// Composition for the same physical attitude:
///   `q_NED_FRD = q_ENU→NED · q_ENU_FLU · q_FRD→FLU`
///
/// Expanding the Hamilton product (`s = 1/√2`):
/// ```text
///   w_out = s·(w_in + z_in)
///   x_out = s·(x_in + y_in)
///   y_out = s·(x_in − y_in)
///   z_out = s·(w_in − z_in)
/// ```
///
/// For `q_ENU_FLU = identity` (vehicle in FLU body aligned with
/// ENU world, the gz-sim default), the result is `(s, 0, 0, s)` —
/// a 90° yaw about NED-Down, i.e. body-X points East in NED. That
/// matches the gz model's default pose (model frame X = world X =
/// East in ENU = +Y = East in NED). See `tests::enu_quat_to_ned_*`.
pub fn enu_quat_to_ned(q_enu_flu: [f32; 4]) -> [f32; 4] {
    let [w, x, y, z] = q_enu_flu;
    let s = core::f32::consts::FRAC_1_SQRT_2;
    [s * (w + z), s * (x + y), s * (x - y), s * (w - z)]
}

/// Convert a body-frame vector from gz-sim's FLU convention
/// (forward, left, up) to Aviate's FRD (forward, right, down).
/// `Y` and `Z` flip; `X` (forward) is unchanged.
///
/// Currently unused in the production synthesize path — gz's
/// `WorldAngularVelocity` returns ENU-world ang vel, so the
/// gyro pipeline goes through the full world→NED→body rotation
/// rather than a simple body-frame sign swap. Kept for the
/// rotation unit-test suite and for the day a different gz
/// component provides body-frame data directly.
#[allow(dead_code)]
pub fn flu_to_frd_body(v_flu: [f32; 3]) -> [f32; 3] {
    [v_flu[0], -v_flu[1], -v_flu[2]]
}

/// Rotate a world-frame vector into body frame via the inverse of `q`.
/// `q = [w, x, y, z]` rotates body→world; the conjugate rotates back.
pub fn rotate_world_to_body(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let [w, x, y, z] = q;
    // Inverse rotation = conjugate for a unit quaternion. The matrix
    // form below is the body-from-world DCM.
    let r00 = 1.0 - 2.0 * (y * y + z * z);
    let r01 = 2.0 * (x * y + w * z);
    let r02 = 2.0 * (x * z - w * y);
    let r10 = 2.0 * (x * y - w * z);
    let r11 = 1.0 - 2.0 * (x * x + z * z);
    let r12 = 2.0 * (y * z + w * x);
    let r20 = 2.0 * (x * z + w * y);
    let r21 = 2.0 * (y * z - w * x);
    let r22 = 1.0 - 2.0 * (x * x + y * y);
    [
        r00 * v[0] + r01 * v[1] + r02 * v[2],
        r10 * v[0] + r11 * v[1] + r12 * v[2],
        r20 * v[0] + r21 * v[1] + r22 * v[2],
    ]
}

/// Convert a mixer output — normalized per-motor THRUST, force
/// domain (#140) — into the quad rotor angular-velocity setpoint
/// (rad/s) for the X500's `MulticopterMotorModelSystem`.
///
/// This is the one place the resolved actuator curve is applied:
/// the controller and mixer reason purely in force, and
/// `ResolvedKernelConfig::actuator_curve` describes the plant. The
/// gz rotor produces thrust `T = motorConstant · ω²` (quadratic),
/// so the X500 configuration resolves
/// `ActuatorCurveKind::QuadraticRotor` and the boundary command is
/// `√thrust`: `ω = √thrust · MAX_RPS`, making physical thrust
/// linear in the kernel's command. At the X500's force-domain
/// hover trim (20.25 N weight / 34.19 N max = 0.5929) this
/// commands `√0.5929 · MAX_RPS = 0.77 · MAX_RPS` — the rotor speed
/// at which the X500 model's total thrust equals its weight.
pub fn cmd_to_omega(curve: ActuatorCurveKind, thrust: f32) -> f64 {
    f64::from(curve.boundary_command(NormalizedThrust(thrust)).0) * MOTOR_MAX_RPS
}

/// ISA pressure (Pa) at a given altitude MSL (m), troposphere model.
pub fn isa_pressure(altitude_msl_m: f32) -> f32 {
    let t0 = 288.15_f32;
    let l = 0.0065_f32;
    let exponent = 5.2561_f32;
    ISA_P0_PA * (1.0 - l * altitude_msl_m / t0).powf(exponent)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-5;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() <= TOL
    }
    fn vec_close(a: [f32; 3], b: [f32; 3]) -> bool {
        close(a[0], b[0]) && close(a[1], b[1]) && close(a[2], b[2])
    }
    fn quat_close(a: [f32; 4], b: [f32; 4]) -> bool {
        // Quaternions q and -q represent the same rotation; accept
        // either sign by checking both.
        let same = close(a[0], b[0]) && close(a[1], b[1]) && close(a[2], b[2]) && close(a[3], b[3]);
        let neg =
            close(a[0], -b[0]) && close(a[1], -b[1]) && close(a[2], -b[2]) && close(a[3], -b[3]);
        same || neg
    }
    fn quat_norm(q: [f32; 4]) -> f32 {
        (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
    }

    /// Reference brute-force ENU-vector to NED-vector swap, used to
    /// independently derive the expected quaternion behavior.
    fn enu_vec_to_ned(v: [f32; 3]) -> [f32; 3] {
        // E-N-U → N-E-D: swap X/Y, negate Z.
        [v[1], v[0], -v[2]]
    }

    /// Brute-force body→world DCM from a quaternion `[w, x, y, z]`,
    /// computed via the standard formula. Used to cross-check the
    /// closed-form `rotate_world_to_body` (which applies the
    /// transpose / conjugate).
    fn dcm_world_from_body(q: [f32; 4]) -> [[f32; 3]; 3] {
        let [w, x, y, z] = q;
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - w * z),
                2.0 * (x * z + w * y),
            ],
            [
                2.0 * (x * y + w * z),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - w * x),
            ],
            [
                2.0 * (x * z - w * y),
                2.0 * (y * z + w * x),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ]
    }

    #[test]
    fn enu_quat_to_ned_preserves_unit_norm() {
        let cases: &[[f32; 4]] = &[
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.5, 0.5, 0.5, 0.5],
            [0.6, 0.0, 0.8, 0.0],
        ];
        for &q in cases {
            let out = enu_quat_to_ned(q);
            assert!(
                (quat_norm(out) - 1.0).abs() < 1e-4,
                "non-unit norm {:?} -> {:?} (|q|={})",
                q,
                out,
                quat_norm(out)
            );
        }
    }

    #[test]
    fn enu_quat_to_ned_identity_input_matches_closed_form() {
        // q_enu_flu = identity means body (FLU) axes are aligned
        // with ENU world axes: body-X (Forward) = East, body-Y
        // (Left) = North, body-Z (Up) = Up. The equivalent NED+FRD
        // attitude: body-X (Forward) still East = NED-Y, body-Y
        // (Right) = South = -NED-X, body-Z (Down) = Down = +NED-Z.
        // That is a +90° yaw rotation about NED-Down, rotor
        // `(cos 45°, 0, 0, sin 45°) = (1/√2, 0, 0, 1/√2)`.
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let expected = [s, 0.0, 0.0, s];
        let got = enu_quat_to_ned([1.0, 0.0, 0.0, 0.0]);
        assert!(quat_close(got, expected), "identity ENU: got {:?}", got);
    }

    #[test]
    fn enu_quat_to_ned_consistent_under_frame_swap() {
        let cases: &[[f32; 4]] = &[
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.5, 0.5, 0.5, 0.5],
        ];
        let v_flu = [0.7_f32, -0.2, 0.5];
        let v_frd = flu_to_frd_body(v_flu);
        for &q_enu_flu in cases {
            let q_ned_frd = enu_quat_to_ned(q_enu_flu);
            assert!(
                (quat_norm(q_ned_frd) - 1.0).abs() < 1e-4,
                "non-unit norm for q={:?}: {:?}",
                q_enu_flu,
                q_ned_frd
            );

            let dcm_enu = dcm_world_from_body(q_enu_flu);
            let v_enu_world = [
                dcm_enu[0][0] * v_flu[0] + dcm_enu[0][1] * v_flu[1] + dcm_enu[0][2] * v_flu[2],
                dcm_enu[1][0] * v_flu[0] + dcm_enu[1][1] * v_flu[1] + dcm_enu[1][2] * v_flu[2],
                dcm_enu[2][0] * v_flu[0] + dcm_enu[2][1] * v_flu[1] + dcm_enu[2][2] * v_flu[2],
            ];
            let dcm_ned = dcm_world_from_body(q_ned_frd);
            let v_ned_world = [
                dcm_ned[0][0] * v_frd[0] + dcm_ned[0][1] * v_frd[1] + dcm_ned[0][2] * v_frd[2],
                dcm_ned[1][0] * v_frd[0] + dcm_ned[1][1] * v_frd[1] + dcm_ned[1][2] * v_frd[2],
                dcm_ned[2][0] * v_frd[0] + dcm_ned[2][1] * v_frd[1] + dcm_ned[2][2] * v_frd[2],
            ];
            let v_ned_via_swap = enu_vec_to_ned(v_enu_world);
            assert!(
                vec_close(v_ned_via_swap, v_ned_world),
                "frame-swap mismatch for q={:?}:\n  ENU→swap = {:?}\n  NED      = {:?}",
                q_enu_flu,
                v_ned_via_swap,
                v_ned_world
            );
        }
    }

    #[test]
    fn flu_to_frd_body_flips_y_and_z() {
        assert_eq!(flu_to_frd_body([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0]);
        assert_eq!(flu_to_frd_body([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0]);
        assert_eq!(flu_to_frd_body([-4.5, 0.0, 7.1]), [-4.5, 0.0, -7.1]);
    }

    #[test]
    fn rotate_world_to_body_identity_is_passthrough() {
        let v = [0.3_f32, -0.7, 1.5];
        let out = rotate_world_to_body([1.0, 0.0, 0.0, 0.0], v);
        assert!(
            vec_close(out, v),
            "identity rotation changed {:?} -> {:?}",
            v,
            out
        );
    }

    #[test]
    fn rotate_world_to_body_90_about_z_swaps_xy() {
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let q = [s, 0.0, 0.0, s];
        assert!(vec_close(
            rotate_world_to_body(q, [1.0, 0.0, 0.0]),
            [0.0, -1.0, 0.0]
        ));
        assert!(vec_close(
            rotate_world_to_body(q, [0.0, 1.0, 0.0]),
            [1.0, 0.0, 0.0]
        ));
        assert!(vec_close(
            rotate_world_to_body(q, [0.0, 0.0, 1.0]),
            [0.0, 0.0, 1.0]
        ));
    }

    #[test]
    fn rotate_world_to_body_90_about_x_swaps_yz() {
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let q = [s, s, 0.0, 0.0];
        assert!(vec_close(
            rotate_world_to_body(q, [1.0, 0.0, 0.0]),
            [1.0, 0.0, 0.0]
        ));
        assert!(vec_close(
            rotate_world_to_body(q, [0.0, 1.0, 0.0]),
            [0.0, 0.0, -1.0]
        ));
        assert!(vec_close(
            rotate_world_to_body(q, [0.0, 0.0, 1.0]),
            [0.0, 1.0, 0.0]
        ));
    }

    #[test]
    fn rotate_world_to_body_matches_brute_force_dcm() {
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let half = 0.5_f32;
        let cos_22_5 = (core::f32::consts::PI / 8.0).cos();
        let sin_22_5 = (core::f32::consts::PI / 8.0).sin();
        let third_axis = 1.0_f32 / (3.0_f32).sqrt();
        let sin_22_5_third = sin_22_5 * third_axis;
        let cases: &[[f32; 4]] = &[
            [s, s, 0.0, 0.0],
            [s, 0.0, s, 0.0],
            [s, 0.0, 0.0, s],
            [half, half, half, half],
            [cos_22_5, sin_22_5_third, sin_22_5_third, sin_22_5_third],
            [cos_22_5, sin_22_5 * 0.6, sin_22_5 * 0.8, 0.0],
        ];
        let v = [0.3_f32, -0.7, 1.5];

        for &q in cases {
            let norm = quat_norm(q);
            assert!(
                (norm - 1.0).abs() < 1e-4,
                "case {:?} not unit (n={})",
                q,
                norm
            );

            let dcm_bw = dcm_world_from_body(q);
            let dcm_wb = [
                [dcm_bw[0][0], dcm_bw[1][0], dcm_bw[2][0]],
                [dcm_bw[0][1], dcm_bw[1][1], dcm_bw[2][1]],
                [dcm_bw[0][2], dcm_bw[1][2], dcm_bw[2][2]],
            ];
            let expected = [
                dcm_wb[0][0] * v[0] + dcm_wb[0][1] * v[1] + dcm_wb[0][2] * v[2],
                dcm_wb[1][0] * v[0] + dcm_wb[1][1] * v[1] + dcm_wb[1][2] * v[2],
                dcm_wb[2][0] * v[0] + dcm_wb[2][1] * v[1] + dcm_wb[2][2] * v[2],
            ];
            let got = rotate_world_to_body(q, v);
            assert!(
                vec_close(got, expected),
                "q={:?}: closed-form={:?} brute-force={:?}",
                q,
                got,
                expected
            );
        }
    }
}
