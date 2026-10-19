//! Truth-synthesis tests: rotations and the step sequence, in NED.

use super::*;
use world_atmosphere::{Altitude, Isa};
use world_core::{GeodeticPosition, Ned};

/// A fixed date, so every run evaluates the same Earth field.
const EPOCH: DecimalYear = DecimalYear(2026.8);

const TOL: f32 = 1e-5;

fn close(a: f32, b: f32) -> bool {
    (a - b).abs() <= TOL
}
fn vec_close(a: [f32; 3], b: [f32; 3]) -> bool {
    close(a[0], b[0]) && close(a[1], b[1]) && close(a[2], b[2])
}
fn quat_norm(q: [f32; 4]) -> f32 {
    (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt()
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

/// North-east-down speed (m/s) and yaw rate (about NED down, rad/s)
/// of the steady motion the step-sequence tests replay: flying east
/// while turning left.
const EAST_SPEED: f32 = 2.0;
const YAW_RATE: f32 = -0.5;

/// The truth of the steady motion at simulation time `t_us`.
fn steady_state(t_us: u64, reset_generation: u32) -> TruthSample {
    let t = t_us as f32 * 1e-6;
    let half_yaw = 0.5 * YAW_RATE * t;
    TruthSample {
        time_us: t_us,
        reset_generation,
        step: t_us / 1_000,
        position_ned_m: [0.0, EAST_SPEED * t, -5.0],
        attitude: [half_yaw.cos(), 0.0, 0.0, half_yaw.sin()],
    }
}

/// Every sample the synthesizer gives for `times` (one world epoch).
fn replay(synth: &mut StepSynthesizer, times: &[u64]) -> Vec<StepSample> {
    times
        .iter()
        .filter_map(|&t| synth.next(&steady_state(t, 1)))
        .collect()
}

fn synth_at(origin: GeoOrigin) -> StepSynthesizer {
    StepSynthesizer::new(origin, EPOCH).expect("the origin and date are inside the models")
}

/// The specific force a level vehicle at rest reads: the normal gravity
/// at its position, upward.
fn at_rest_force(synth: &StepSynthesizer) -> [f32; 3] {
    [0.0, 0.0, -(synth.environment().gravity_mps2 as f32)]
}

fn vec_close_within(a: [f32; 3], b: [f32; 3], tol: f32) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() <= tol)
}

/// A board reads the simulator on its own clock, so it sees some
/// states twice and misses others. Each new step must give exactly
/// one sample whose period is the simulation time since the previous
/// state, and steady motion must read as steady: no zero-velocity
/// sample and no acceleration swing around a repeated state.
#[test]
fn repeated_and_skipped_states_give_one_steady_sample_per_new_step() {
    let mut synth = synth_at(GeoOrigin::DEFAULT);
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
            vec_close_within(gnss.vel_ned, [0.0, EAST_SPEED, 0.0], 1e-3),
            "t={t}: velocity {:?} is not the steady east speed",
            gnss.vel_ned
        );
        assert!(
            vec_close_within(imu.accel, at_rest_force(&synth), 0.05),
            "t={t}: specific force {:?} is not gravity alone",
            imu.accel
        );
        assert!(
            vec_close_within(imu.gyro, [0.0, 0.0, YAW_RATE], 1e-3),
            "t={t}: body rate {:?} is not the steady yaw rate",
            imu.gyro
        );
        assert_eq!(sample.packet.reset_generation, Some(1));
    }
}

#[test]
fn the_first_state_and_a_repeat_of_it_give_no_sample() {
    let mut synth = synth_at(GeoOrigin::DEFAULT);
    assert!(replay(&mut synth, &[5_000, 5_000]).is_empty());
}

/// A world reset rewinds simulation time and starts a new epoch. The
/// synthesizer must start over from the new epoch rather than treat
/// every later state as older than the last one and stop for good.
#[test]
fn a_world_reset_reseeds_the_step_sequence() {
    let mut synth = synth_at(GeoOrigin::DEFAULT);
    assert_eq!(replay(&mut synth, &[1_000, 2_000]).len(), 1);

    assert!(synth.next(&steady_state(0, 2)).is_none());
    let sample = synth
        .next(&steady_state(1_000, 2))
        .expect("the second state of the new epoch is a new step");
    assert_eq!(sample.delta_us, 1_000);
    assert_eq!(sample.packet.reset_generation, Some(2));
    let imu = sample.packet.imu.expect("imu lane");
    assert!(
        vec_close_within(imu.accel, at_rest_force(&synth), 0.05),
        "the first step of a new epoch has no velocity history: {:?}",
        imu.accel
    );
}

#[test]
fn reset_forgets_the_previous_attachment() {
    let mut synth = synth_at(GeoOrigin::DEFAULT);
    assert_eq!(replay(&mut synth, &[1_000, 2_000]).len(), 1);
    synth.reset();
    assert!(synth.next(&steady_state(3_000, 1)).is_none());
}

fn at_rest(t_us: u64, attitude: [f32; 4]) -> TruthSample {
    TruthSample {
        time_us: t_us,
        reset_generation: 1,
        step: t_us / 1_000,
        position_ned_m: [0.0; 3],
        attitude,
    }
}

/// A launcher that moves the world origin moves the GNSS fix with it,
/// and the baro reads the standard atmosphere at the origin's elevation.
#[test]
fn gnss_and_baro_are_made_around_the_stated_origin() {
    let origin = GeoOrigin {
        position: GeodeticPosition::wgs84(47.397_742, 8.545_992_1, 500.0),
    };
    let mut synth = synth_at(origin);
    let level = [1.0, 0.0, 0.0, 0.0];
    assert!(synth.next(&at_rest(1_000, level)).is_none());
    let sample = synth.next(&at_rest(2_000, level)).expect("a new step");
    let gnss = sample.packet.gnss.expect("gnss lane");
    assert!((gnss.lat_deg - 47.397_742).abs() < 1e-12);
    assert!((gnss.lon_deg - 8.545_992_1).abs() < 1e-12);
    assert!((gnss.alt_m - 500.0).abs() < 1e-3);
    let isa = Isa
        .at(Altitude::GeometricMsl(500.0))
        .expect("inside the standard");
    let baro = sample.packet.baro.expect("baro lane");
    assert_eq!(baro.pressure_pa, isa.pressure_pa as f32);
    assert_eq!(baro.temperature_c, isa.temperature_c() as f32);
}

/// Away from the origin every lane follows the vehicle: the fix is the
/// tangent-plane position, the baro the standard atmosphere at its
/// height, and gravity the normal gravity there.
#[test]
fn the_environment_follows_the_vehicle_through_the_world_models() {
    let origin = GeoOrigin::DEFAULT;
    let mut synth = synth_at(origin);
    let level = [1.0, 0.0, 0.0, 0.0];
    let at_ground = synth.environment().gravity_mps2;
    let away = |t_us| TruthSample {
        position_ned_m: [3_000.0, -4_000.0, -1_000.0],
        ..at_rest(t_us, level)
    };
    assert!(synth.next(&away(1_000)).is_none());
    let sample = synth.next(&away(2_000)).expect("a new step");

    let expected = origin.plane().expect("a valid origin").to_geodetic(Ned {
        north_m: 3_000.0,
        east_m: -4_000.0,
        down_m: -1_000.0,
    });
    let gnss = sample.packet.gnss.expect("gnss lane");
    assert_eq!(
        (gnss.lat_deg, gnss.lon_deg),
        (expected.latitude_deg, expected.longitude_deg)
    );
    assert!((f64::from(gnss.alt_m) - expected.ellipsoidal_height_m).abs() < 1e-3);
    // Curvature: the tangent plane rises above the ellipsoid away from
    // the origin, by about d² / 2R (2 m at 5 km).
    let rise = expected.ellipsoidal_height_m - (488.0 + 1_000.0);
    assert!((rise - 1.96).abs() < 0.05, "{rise}");

    let isa = Isa
        .at(Altitude::GeometricMsl(expected.ellipsoidal_height_m))
        .expect("inside the standard");
    let baro = sample.packet.baro.expect("baro lane");
    assert_eq!(baro.pressure_pa, isa.pressure_pa as f32);
    // 1 km up, gravity is about 3 mm/s² weaker.
    let aloft = synth.environment().gravity_mps2;
    assert!(
        (at_ground - aloft - 3.086e-3).abs() < 1e-4,
        "{at_ground} {aloft}"
    );
}

/// The simulated magnetometer and the controller share one Earth field:
/// at 51° N the magnetic heading of the synthesized reading plus the
/// model's declination is the vehicle's true heading.
#[test]
fn the_magnetometer_reads_the_model_field_at_the_vehicle() {
    let origin = GeoOrigin {
        position: GeodeticPosition::wgs84(51.05, -114.07, 1_045.0),
    };
    let mut synth = synth_at(origin);
    let declination = synth.environment().declination_deg();
    assert!(
        declination > 10.0,
        "51.05 N 114.07 W has a large declination"
    );

    // Facing true heading 30°.
    let half = 15.0_f32.to_radians();
    let heading_30 = [half.cos(), 0.0, 0.0, half.sin()];
    assert!(synth.next(&at_rest(1_000, heading_30)).is_none());
    let sample = synth.next(&at_rest(2_000, heading_30)).expect("a new step");
    let [mx, my, _] = sample.packet.mag.expect("mag lane").field_ut;
    let magnetic = (-my).atan2(mx).to_degrees();
    let true_heading = magnetic + declination;
    assert!(
        (true_heading - 30.0).abs() < 0.1,
        "magnetic {magnetic:.2}° + declination {declination:.2}° is not 30°"
    );
}

/// A world model's refusal at the origin refuses the synthesizer rather
/// than substituting an environment.
#[test]
fn an_origin_outside_the_models_is_refused() {
    let too_deep = GeoOrigin {
        position: GeodeticPosition::wgs84(0.0, 0.0, -6_000.0),
    };
    assert!(matches!(
        StepSynthesizer::new(too_deep, EPOCH),
        Err(SynthesisError::EarthField(_) | SynthesisError::Atmosphere(_))
    ));
    assert!(matches!(
        StepSynthesizer::new(GeoOrigin::DEFAULT, DecimalYear(2031.0)),
        Err(SynthesisError::EarthField(_))
    ));
}
