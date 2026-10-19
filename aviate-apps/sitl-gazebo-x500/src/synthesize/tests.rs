//! Gazebo sensor-synthesis frame tests.

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
    let neg = close(a[0], -b[0]) && close(a[1], -b[1]) && close(a[2], -b[2]) && close(a[3], -b[3]);
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

/// East speed (ENU x, m/s) and yaw rate (about ENU up, rad/s) of the
/// steady motion the step-sequence tests replay.
const EAST_SPEED: f64 = 2.0;
const YAW_RATE: f64 = 0.5;

/// The model state of the steady motion at simulation time `t_us`.
fn steady_state(t_us: u64, reset_generation: u32) -> AviateModelState {
    let t = t_us as f64 * 1e-6;
    let half_yaw = 0.5 * YAW_RATE * t;
    AviateModelState {
        pos: [EAST_SPEED * t, 0.0, 5.0],
        quat: [half_yaw.cos(), 0.0, 0.0, half_yaw.sin()],
        time_us: t_us,
        sim_step: t_us / 1_000,
        reset_generation,
        valid: 1,
        ..AviateModelState::default()
    }
}

/// Every sample the synthesizer gives for `times` (one world epoch).
fn replay(synth: &mut StepSynthesizer, times: &[u64]) -> Vec<StepSample> {
    times
        .iter()
        .filter_map(|&t| synth.next(&steady_state(t, 1)))
        .collect()
}

/// The FC reads shared memory on its own clock, so it sees some
/// states twice and misses others. Each new step must give exactly
/// one sample whose period is the simulation time since the previous
/// state, and steady motion must read as steady: no zero-velocity
/// sample and no acceleration swing around a repeated state.
#[test]
fn repeated_and_skipped_states_give_one_steady_sample_per_new_step() {
    let mut synth = synth_at(crate::origin::GeoOrigin::DEFAULT);
    let times = [
        1_000, 2_000, 3_000, 3_000, 3_000, 4_000, 6_000, 6_000, 7_000,
    ];
    let samples = replay(&mut synth, &times);

    let deltas: Vec<u64> = samples.iter().map(|s| s.delta_us).collect();
    assert_eq!(deltas, [1_000, 1_000, 1_000, 2_000, 1_000]);
    let steps: Vec<u64> = samples.iter().map(|s| s.sim_step).collect();
    assert_eq!(steps, [2, 3, 4, 6, 7]);

    for sample in &samples {
        let t = sample.packet.timestamp_us;
        let gnss = sample.packet.gnss.expect("gnss lane");
        let imu = sample.packet.imu.expect("imu lane");
        assert!(
            vec_close_within(gnss.vel_ned, [0.0, EAST_SPEED as f32, 0.0], 1e-3),
            "t={t}: velocity {:?} is not the steady east speed",
            gnss.vel_ned
        );
        assert!(
            vec_close_within(imu.accel, [0.0, 0.0, -GRAVITY], 0.05),
            "t={t}: specific force {:?} is not gravity alone",
            imu.accel
        );
        assert!(
            vec_close_within(imu.gyro, [0.0, 0.0, -YAW_RATE as f32], 1e-3),
            "t={t}: body rate {:?} is not the steady yaw rate",
            imu.gyro
        );
    }
}

#[test]
fn the_first_state_and_a_repeat_of_it_give_no_sample() {
    let mut synth = synth_at(crate::origin::GeoOrigin::DEFAULT);
    assert!(replay(&mut synth, &[5_000, 5_000]).is_empty());
}

/// A world reset rewinds simulation time and starts a new epoch. The
/// synthesizer must start over from the new epoch rather than treat
/// every later state as older than the last one and stop for good.
#[test]
fn a_world_reset_reseeds_the_step_sequence() {
    let mut synth = synth_at(crate::origin::GeoOrigin::DEFAULT);
    assert_eq!(replay(&mut synth, &[1_000, 2_000]).len(), 1);

    assert!(synth.next(&steady_state(0, 2)).is_none());
    let sample = synth
        .next(&steady_state(1_000, 2))
        .expect("the second state of the new epoch is a new step");
    assert_eq!(sample.delta_us, 1_000);
    let imu = sample.packet.imu.expect("imu lane");
    assert!(
        vec_close_within(imu.accel, [0.0, 0.0, -GRAVITY], 0.05),
        "the first step of a new epoch has no velocity history: {:?}",
        imu.accel
    );
}

#[test]
fn reset_forgets_the_previous_attachment() {
    let mut synth = synth_at(crate::origin::GeoOrigin::DEFAULT);
    assert_eq!(replay(&mut synth, &[1_000, 2_000]).len(), 1);
    synth.reset();
    assert!(synth.next(&steady_state(3_000, 1)).is_none());
}

fn vec_close_within(a: [f32; 3], b: [f32; 3], tol: f32) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= tol)
}

/// A launcher that moves the world origin moves the GNSS fix with it.
#[test]
fn gnss_and_baro_are_made_around_the_stated_origin() {
    let origin = crate::origin::GeoOrigin {
        latitude_deg: 47.397_742,
        longitude_deg: 8.545_992_1,
        elevation_m: 500.0,
    };
    let mut synth = synth_at(origin);
    let at_rest = |t_us| AviateModelState {
        quat: [1.0, 0.0, 0.0, 0.0],
        time_us: t_us,
        reset_generation: 1,
        valid: 1,
        ..AviateModelState::default()
    };
    assert!(synth.next(&at_rest(1_000)).is_none());
    let sample = synth.next(&at_rest(2_000)).expect("a new step");
    let gnss = sample.packet.gnss.expect("gnss lane");
    assert_eq!(gnss.lat_deg, origin.latitude_deg);
    assert_eq!(gnss.lon_deg, origin.longitude_deg);
    assert_eq!(gnss.alt_m, origin.elevation_m);
    let baro = sample.packet.baro.expect("baro lane");
    assert_eq!(baro.pressure_pa, isa_pressure(origin.elevation_m));
}

fn synth_at(origin: crate::origin::GeoOrigin) -> StepSynthesizer {
    let earth = aviate_hal_xil::world::EarthField::at_origin(origin.geodetic_point())
        .expect("today and the origin are inside the model");
    StepSynthesizer::new(origin, earth)
}

/// The simulated magnetometer and the controller share one Earth field:
/// at 51° N the magnetic heading of the synthesized reading plus the
/// model's declination is the vehicle's true heading.
#[test]
fn the_magnetometer_reads_the_model_field_at_the_vehicle() {
    let origin = crate::origin::GeoOrigin {
        latitude_deg: 51.05,
        longitude_deg: -114.07,
        elevation_m: 1_045.0,
    };
    let earth = aviate_hal_xil::world::EarthField::at_origin(origin.geodetic_point())
        .expect("inside the model");
    let declination = earth.declination_deg();
    assert!(
        declination > 10.0,
        "51.05 N 114.07 W has a large declination"
    );
    let mut synth = StepSynthesizer::new(origin, earth);

    // Facing true heading 30°: ENU yaw is measured from east, counter-
    // clockwise, so a 30° heading is an ENU yaw of 60°.
    let enu_yaw = 60.0_f64.to_radians();
    let at = |t_us| AviateModelState {
        quat: [(enu_yaw / 2.0).cos(), 0.0, 0.0, (enu_yaw / 2.0).sin()],
        time_us: t_us,
        reset_generation: 1,
        valid: 1,
        ..AviateModelState::default()
    };
    assert!(synth.next(&at(1_000)).is_none());
    let sample = synth.next(&at(2_000)).expect("a new step");
    let [mx, my, _] = sample.packet.mag.expect("mag lane").field_ut;
    let magnetic = (-my).atan2(mx).to_degrees();
    let true_heading = magnetic + declination;
    assert!(
        (true_heading - 30.0).abs() < 0.1,
        "magnetic {magnetic:.2}° + declination {declination:.2}° is not 30°"
    );
}
