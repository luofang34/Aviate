//! X500 Gazebo SITL FC binary.
//!
//! Architecture:
//!
//! ```text
//! gz sim ── AviateGzPlugin.dylib ── POSIX shm /aviate_gz_bridge ── this binary
//!                model state (ENU)                    │
//!                                                     │ synthesize NED
//!                                                     ▼
//!                                  SitlIO.feed_sensor_packet(...)
//!                                                     │
//!                                                     ▼
//!                                  GazeboSitlBoard.step() → ActuatorCmd
//!                                                     │
//!                                                     ▼
//!                                  plugin.set_motor_speeds(...)
//! ```
//!
//! The `AviateGzPlugin` writes pose / velocity / angular-velocity into
//! shared memory each `PostUpdate` tick. This binary reads that ground
//! truth, synthesizes IMU + baro + mag + GNSS readings, feeds them into
//! the kernel via the SITL transport, runs one kernel cycle, and writes
//! the resulting motor commands back to the plugin.

use std::time::{Duration, Instant};

use aviate_backend_gz::{enu_to_ned_f32, enu_vel_to_ned_f32, AviateModelState, GzPluginBridge};
use aviate_board_sitl_gazebo::GazeboSitlBoard;
use aviate_hal_xil::sim_types::{
    SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket, SimTimestampUs,
};

/// Cycle period for the FC loop (1 kHz, matching loop_periods::GAZEBO_US).
const CYCLE_PERIOD_US: u64 = 1_000;

/// X500 rotor `maxRotVelocity` from the PX4-gazebo-models SDF
/// (`motorConstant` 8.55e-6 N/(rad/s)² × 1000² × 4 motors ≈ 34 N max
/// thrust against ≈20 N weight, so a thrust-to-weight ratio of ~1.7
/// at full motor output). Maps `Normalized([0.0, 1.0])` actuator
/// output linearly to rotor speed.
const MOTOR_MAX_RPS: f64 = 1000.0;

/// Zurich reference for the auto-generated SITL world.
const REF_LATITUDE_DEG: f64 = 47.3977419;
const REF_LONGITUDE_DEG: f64 = 8.5455938;
const REF_ELEVATION_M: f32 = 488.0;
/// Magnetic field at Zurich, NED in microtesla (approximate).
const MAG_NED_UT: [f32; 3] = [21.0, 1.5, 43.0];
/// ISA sea-level pressure (Pa) and temperature (Celsius).
const ISA_P0_PA: f32 = 101_325.0;
const ISA_T0_C: f32 = 15.0;
/// Standard gravity (m/s², NED-down convention so accel under hover ≈ -9.81 on z).
const GRAVITY: f32 = 9.81;

fn main() -> std::io::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("aviate sitl-gazebo-x500 starting");

    let mut board = GazeboSitlBoard::new_with_retry(10, 200)?;
    log::info!("board constructed");

    // Connect to the gz-sim system plugin via shared memory. The plugin
    // initializes the shared region as soon as gz-sim loads the SDF, so
    // a short retry loop is plenty.
    let plugin = GzPluginBridge::connect_with_retry(20, 250)
        .map_err(|e| std::io::Error::other(format!("gz plugin: {e:?}")))?;
    log::info!("connected to AviateGzPlugin");

    let mut last_state: Option<AviateModelState> = None;
    let mut last_t_us: u64 = 0;

    let cycle = Duration::from_micros(CYCLE_PERIOD_US);
    let mut next_tick = Instant::now() + cycle;

    loop {
        // 1. Read the latest ground-truth model state from the plugin.
        if let Some(state) = plugin.get_model_state() {
            // 2. Compute body-frame accelerometer from velocity delta in
            //    world frame, then project gravity through the body
            //    attitude. The kernel sees a "perfect IMU" — no noise,
            //    no bias.
            //
            // Capture `time_us` BEFORE `last_state = Some(state)` so
            // the sequence does not silently break if a future
            // `AviateModelState` field becomes non-Copy.
            let packet = synthesize_packet(&state, last_state.as_ref(), last_t_us);
            let state_time_us = state.time_us;
            last_state = Some(state);
            last_t_us = state_time_us;

            board.transport_mut().feed_sensor_packet(&packet);
        }

        // 3. Run one kernel cycle.
        let cmd = board.step();

        // 4. Forward actuator outputs to gz-sim as rotor velocities.
        //
        // Aviate's mixer produces normalized [0, 1] outputs whose
        // semantics the kernel treats as normalized **thrust** (the
        // mixer's additive corrections compose meaningfully in
        // thrust units; in motor-speed units, mid-throttle would
        // produce only `cmd²` of max thrust). The X500 rotor model
        // in PX4-gazebo-models implements quadratic thrust:
        // `thrust = motorConstant · ω²`. So normalized-thrust input
        // maps to motor angular velocity as `ω = MAX · √cmd`.
        // Without the sqrt, "0.65 hover" actually produces only
        // 0.42 of max thrust — well below the X500's 0.57 weight-
        // to-max-thrust ratio, and the vehicle sinks.
        let motor_speeds = [
            cmd_to_omega(cmd.outputs[0].0),
            cmd_to_omega(cmd.outputs[1].0),
            cmd_to_omega(cmd.outputs[2].0),
            cmd_to_omega(cmd.outputs[3].0),
        ];
        if let Err(e) = plugin.set_motor_speeds(&motor_speeds) {
            log::warn!("set_motor_speeds failed: {e:?}");
        }

        // 5. Pace the loop. We do not lock to gz sim_step here — the
        //    plugin's `lockstep` setting (off by default in our smoke
        //    world) decides whether gz advances independently.
        let now = Instant::now();
        if now < next_tick {
            std::thread::sleep(next_tick - now);
        }
        next_tick += cycle;
    }
}

/// Build a `SimSensorPacket` from the latest ground-truth model state.
/// Coordinate conventions match the rest of aviate-core: NED for
/// position / velocity / accel; body frame for gyro.
fn synthesize_packet(
    state: &AviateModelState,
    prev: Option<&AviateModelState>,
    prev_t_us: u64,
) -> SimSensorPacket {
    let now_ned_pos = enu_to_ned_f32(state.pos);
    let now_ned_vel = enu_vel_to_ned_f32(state.vel);
    let now_us = state.time_us;
    let dt = match prev {
        Some(_) if now_us > prev_t_us => (now_us - prev_t_us) as f32 * 1e-6,
        _ => 0.001,
    };

    // Gz reports angular velocity in body frame, but with FLU body
    // convention; aviate-core expects FRD. Flip Y and Z.
    let gyro = flu_to_frd_body([
        state.ang_vel[0] as f32,
        state.ang_vel[1] as f32,
        state.ang_vel[2] as f32,
    ]);

    // Accelerometer reading: specific force = inertial acceleration −
    // gravity, expressed in body frame. For a multirotor in steady
    // hover, accel_body ≈ (0, 0, −g) (g pulls Down in NED, IMU
    // reports the reaction; the kernel's EKF expects accel_body ≈
    // (0, 0, −9.81) at rest).
    let accel_inertial_ned = match prev {
        Some(p) => {
            let prev_vel = enu_vel_to_ned_f32(p.vel);
            [
                (now_ned_vel[0] - prev_vel[0]) / dt,
                (now_ned_vel[1] - prev_vel[1]) / dt,
                (now_ned_vel[2] - prev_vel[2]) / dt,
            ]
        }
        None => [0.0, 0.0, 0.0],
    };
    // Project (accel - g) into body frame. The plugin gives us the
    // model's world-frame quaternion in ENU `[w, x, y, z]`.
    let q_enu = [
        state.quat[0] as f32,
        state.quat[1] as f32,
        state.quat[2] as f32,
        state.quat[3] as f32,
    ];
    let q_ned = enu_quat_to_ned(q_enu);
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
        vel_ned: now_ned_vel,
        fix: SimGnssFix::ThreeD,
        satellites: 14,
        h_acc: 0.5,
        v_acc: 0.8,
    };

    SimSensorPacket {
        timestamp_us: now_us as SimTimestampUs,
        imu: Some(imu),
        baro: Some(baro),
        mag: Some(mag),
        gnss: Some(gnss),
    }
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
fn enu_quat_to_ned(q_enu_flu: [f32; 4]) -> [f32; 4] {
    let [w, x, y, z] = q_enu_flu;
    let s = core::f32::consts::FRAC_1_SQRT_2;
    [s * (w + z), s * (x + y), s * (x - y), s * (w - z)]
}

/// Convert a body-frame vector from gz-sim's FLU convention
/// (forward, left, up) to Aviate's FRD (forward, right, down).
/// `Y` and `Z` flip; `X` (forward) is unchanged.
fn flu_to_frd_body(v_flu: [f32; 3]) -> [f32; 3] {
    [v_flu[0], -v_flu[1], -v_flu[2]]
}

/// Rotate a world-frame vector into body frame via the inverse of `q`.
/// `q = [w, x, y, z]` rotates body→world; the conjugate rotates back.
fn rotate_world_to_body(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
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

/// Convert an Aviate `Normalized` [0, 1] actuator command into a
/// quad rotor angular-velocity setpoint (rad/s) for the X500's
/// `MulticopterMotorModelSystem`. Linearizes thrust against the
/// rotor's quadratic `motorConstant · ω²` law via `√cmd`. Inputs
/// outside [0, 1] (which the mixer should already clamp) are
/// saturated at 0 so we never command a negative motor speed.
fn cmd_to_omega(normalized: f32) -> f64 {
    let clamped = normalized.clamp(0.0, 1.0);
    (clamped as f64).sqrt() * MOTOR_MAX_RPS
}

/// ISA pressure (Pa) at a given altitude MSL (m), troposphere model.
fn isa_pressure(altitude_msl_m: f32) -> f32 {
    let t0 = 288.15_f32;
    let l = 0.0065_f32;
    let exponent = 5.2561_f32;
    ISA_P0_PA * (1.0 - l * altitude_msl_m / t0).powf(exponent)
}

// ---------------------------------------------------------------------------
// Unit tests for the rotation helpers. The math is load-bearing for
// every synthesized IMU sample fed into the kernel — a sign error
// here would produce a vehicle that flies but with a subtly-wrong
// attitude estimate the kernel can't tell from real physics.
// ---------------------------------------------------------------------------

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
        let neg = close(a[0], -b[0])
            && close(a[1], -b[1])
            && close(a[2], -b[2])
            && close(a[3], -b[3]);
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

    // --- enu_quat_to_ned ----------------------------------------------------

    #[test]
    fn enu_quat_to_ned_preserves_unit_norm() {
        // Pick a handful of unit quaternions; converted result must
        // stay unit-norm (within float tolerance).
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
        // For an arbitrary FLU body vector `v_flu`, both attitude
        // representations must take it to the same physical
        // world-frame vector — after the appropriate basis swaps.
        //
        //   q_ENU_FLU · v_flu                 → v_ENU
        //   q_NED_FRD · (flu_to_frd v_flu)    → v_NED
        //   v_NED == enu_vec_to_ned(v_ENU)
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
                dcm_enu[0][0] * v_flu[0]
                    + dcm_enu[0][1] * v_flu[1]
                    + dcm_enu[0][2] * v_flu[2],
                dcm_enu[1][0] * v_flu[0]
                    + dcm_enu[1][1] * v_flu[1]
                    + dcm_enu[1][2] * v_flu[2],
                dcm_enu[2][0] * v_flu[0]
                    + dcm_enu[2][1] * v_flu[1]
                    + dcm_enu[2][2] * v_flu[2],
            ];
            let dcm_ned = dcm_world_from_body(q_ned_frd);
            let v_ned_world = [
                dcm_ned[0][0] * v_frd[0]
                    + dcm_ned[0][1] * v_frd[1]
                    + dcm_ned[0][2] * v_frd[2],
                dcm_ned[1][0] * v_frd[0]
                    + dcm_ned[1][1] * v_frd[1]
                    + dcm_ned[1][2] * v_frd[2],
                dcm_ned[2][0] * v_frd[0]
                    + dcm_ned[2][1] * v_frd[1]
                    + dcm_ned[2][2] * v_frd[2],
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

    // --- rotate_world_to_body -----------------------------------------------

    #[test]
    fn rotate_world_to_body_identity_is_passthrough() {
        let v = [0.3_f32, -0.7, 1.5];
        let out = rotate_world_to_body([1.0, 0.0, 0.0, 0.0], v);
        assert!(vec_close(out, v), "identity rotation changed {:?} -> {:?}", v, out);
    }

    #[test]
    fn rotate_world_to_body_90_about_z_swaps_xy() {
        // q = cos(45°) + k·sin(45°) is a +90° rotation about world-Z.
        // body→world rotates body-X to world-Y. Therefore world-Y
        // arrives at body-X, world-X arrives at body-(-Y).
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let q = [s, 0.0, 0.0, s];
        assert!(vec_close(rotate_world_to_body(q, [1.0, 0.0, 0.0]), [0.0, -1.0, 0.0]));
        assert!(vec_close(rotate_world_to_body(q, [0.0, 1.0, 0.0]), [1.0, 0.0, 0.0]));
        assert!(vec_close(rotate_world_to_body(q, [0.0, 0.0, 1.0]), [0.0, 0.0, 1.0]));
    }

    #[test]
    fn rotate_world_to_body_90_about_x_swaps_yz() {
        // +90° about world-X: world-Y → world-Z (in body frame, the
        // body sees world-Y arrive at body-Z).
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let q = [s, s, 0.0, 0.0];
        assert!(vec_close(rotate_world_to_body(q, [1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]));
        // world-Y body-projection lands on body-(-Z) under the
        // inverse rotation (body→world rotates body-Y to world-Z,
        // so the inverse rotates world-Y to body-(-Z)).
        assert!(vec_close(rotate_world_to_body(q, [0.0, 1.0, 0.0]), [0.0, 0.0, -1.0]));
        assert!(vec_close(rotate_world_to_body(q, [0.0, 0.0, 1.0]), [0.0, 1.0, 0.0]));
    }

    #[test]
    fn rotate_world_to_body_matches_brute_force_dcm() {
        // Compare against a transposed brute-force DCM for a handful
        // of representative unit quaternions. The closed-form
        // `rotate_world_to_body` is `DCM_world_from_body^T · v`;
        // verify that equality directly.
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let half = 0.5_f32;
        // 45° about (1,1,1)/√3 axis: w = cos(22.5°), xyz = sin(22.5°)·(1/√3, 1/√3, 1/√3)
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
            assert!((norm - 1.0).abs() < 1e-4, "case {:?} not unit (n={})", q, norm);

            let dcm_bw = dcm_world_from_body(q);
            // Transpose: brute-force world→body matrix.
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
