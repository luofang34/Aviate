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
