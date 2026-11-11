//! The Jacobians of the joint measurements against finite differences
//! of their predicted values, with the error-state conventions of the
//! filter: additive velocity and wind, and an attitude error in the
//! navigation frame that multiplies from the left.

use super::{heading_measurement, JointMeasurement};
use crate::ekf::air_aiding::{airspeed_measurement, sideslip_measurement, AirAidingConfig};
use crate::ekf::update::heading_innovation;
use crate::ekf::{EkfState, IDX_ATT, IDX_VEL};
use crate::math::{Quaternion, Vector3};
use crate::types::{Meters, MetersPerSecond, Scalar};

const STEP: Scalar = 1e-3;

fn config() -> AirAidingConfig {
    AirAidingConfig {
        meas_noise_tas: 1.0,
        sideslip_sigma: 0.05,
        wind_prior_var: 64.0,
        process_noise_wind: 0.05,
        min_true_airspeed: 15.0,
    }
}

/// A banked, pitched, yawed vehicle climbing through a crosswind.
fn state() -> EkfState {
    let mut state = EkfState::new();
    let attitude = Quaternion::from_axis_angle(Vector3::new(0.3, 0.2, 0.9), 0.8).normalize();
    state.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(30.0),
            MetersPerSecond(40.0),
            MetersPerSecond(-2.0),
        ),
        attitude,
    );
    state.air.active = true;
    state.air.wind_ne = [4.0, -3.0];
    state
}

/// Perturbation `which` of size `step`: 0..3 velocity, 3..6 attitude,
/// 6..8 wind.
fn perturbed(base: &EkfState, which: usize, step: Scalar) -> EkfState {
    let mut state = base.clone();
    let mut delta = [0.0; 3];
    match which {
        0..=2 => {
            let v = [&mut state.vel.x, &mut state.vel.y, &mut state.vel.z];
            v.into_iter().nth(which).unwrap().0 += step;
        }
        3..=5 => {
            delta[which - 3] = step;
            let small = Quaternion::new(1.0, 0.5 * delta[0], 0.5 * delta[1], 0.5 * delta[2]);
            state.quat = small.normalize().mul(&state.quat).normalize();
        }
        _ => state.air.wind_ne[which - 6] += step,
    }
    state
}

fn jacobian_entry(measurement: &JointMeasurement, which: usize) -> Scalar {
    match which {
        0..=2 => measurement.h_x[IDX_VEL + which],
        3..=5 => measurement.h_x[IDX_ATT + which - 3],
        _ => measurement.h_w[which - 6],
    }
}

/// Compare each Jacobian entry with a central difference of `predicted`.
fn check(
    predicted: impl Fn(&EkfState) -> Scalar,
    measured: impl Fn(&EkfState) -> JointMeasurement,
) {
    let base = state();
    let measurement = measured(&base);
    for which in 0..8 {
        let plus = predicted(&perturbed(&base, which, STEP));
        let minus = predicted(&perturbed(&base, which, -STEP));
        let numeric = (plus - minus) / (2.0 * STEP);
        let analytic = jacobian_entry(&measurement, which);
        let tolerance = 2e-2 * numeric.abs().max(1.0);
        assert!(
            (numeric - analytic).abs() < tolerance,
            "entry {which}: numeric {numeric}, analytic {analytic}"
        );
    }
}

#[test]
fn airspeed_jacobian_matches_the_predicted_airspeed() {
    let config = config();
    check(
        |state| 50.0 - airspeed_measurement(state, 50.0, &config).innovation,
        |state| airspeed_measurement(state, 50.0, &config),
    );
}

#[test]
fn sideslip_jacobian_matches_the_predicted_lateral_air_velocity() {
    let config = config();
    check(
        |state| -sideslip_measurement(state, 50.0, &config).innovation,
        |state| sideslip_measurement(state, 50.0, &config),
    );
}

#[test]
fn heading_jacobian_sees_tilt_through_the_levelled_field() {
    // A field with a strong vertical part, as at mid latitudes.
    let field_nav = Vector3::new(20.0, -2.0, 45.0);
    let base = state();
    let q = base.quat;
    let field_body = Quaternion::new(q.w, -q.x, -q.y, -q.z).rotate_vector(field_nav);
    let body = [field_body.x, field_body.y, field_body.z];
    let measurement = |state: &EkfState| {
        let innovation = heading_innovation(&state.quat, body, 0.0);
        heading_measurement(state.quat.rotate_vector(field_body), innovation, 0.05).unwrap()
    };
    check(
        |state| -heading_innovation(&state.quat, body, 0.0),
        measurement,
    );
    // The tilt entries are not zero: a roll or pitch error moves the heading.
    let row = measurement(&base).h_x;
    assert!(row[IDX_ATT].abs() > 0.1);
    assert!(row[IDX_ATT + 1].abs() > 0.1);
    assert!(heading_measurement(Vector3::new(0.0, 0.0, 45.0), 0.0, 0.05).is_none());
    assert!(heading_measurement(Vector3::new(Scalar::NAN, 0.0, 45.0), 0.0, 0.05).is_none());
}

#[test]
fn air_aiding_fuses_pitot_propagates_wind_and_tracks_validity() {
    use crate::ekf::{Ekf, EkfConfig, Estimator, STATE_DIM};
    use crate::sensor::{
        AirData, AirspeedData, GnssData, GnssFix, GnssHealth, ImuData, SensorHealth, SensorReading,
        SensorSet, SEA_LEVEL_AIR_DENSITY_KGPM3,
    };
    use crate::state::StateValidFlags;
    use crate::time::{TimeSource, Timestamp};
    use crate::types::{MetersPerSecondSquared, Pascals, RadiansPerSecond};

    let cfg = config();
    // Near-zero airspeed falls back to the body nose direction.
    let mut parked = EkfState::new();
    parked.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(1.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    let slow_meas = airspeed_measurement(&parked, 20.0, &cfg);
    assert!((slow_meas.h_x[IDX_VEL] - 1.0).abs() < 1e-6);

    fn good_reading<T>(value: T) -> SensorReading<T> {
        SensorReading {
            value,
            valid: true,
            source_id: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Good,
        }
    }
    let blank_sensors = || SensorSet {
        imus: [SensorReading::default(); 3],
        gnss: [SensorReading::default(); 2],
        mags: [SensorReading::default(); 2],
        baros: [SensorReading::default(); 2],
        airspeeds: [SensorReading::default(); 2],
        geometry: None,
        geomagnetic: None,
    };
    let pitot_pa = |tas: Scalar| 0.5 * SEA_LEVEL_AIR_DENSITY_KGPM3 * tas * tas;
    let sensors_with_pitot = |dyn_pa: Option<Scalar>, gnss_good: bool| {
        let mut s = blank_sensors();
        s.imus[0] = good_reading(ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [RadiansPerSecond(0.0); 3],
        });
        if let Some(pa) = dyn_pa {
            s.airspeeds[0] = good_reading(AirspeedData {
                air: AirData {
                    dynamic_pressure: Some(Pascals(pa)),
                    ..AirData::default()
                },
            });
        }
        if gnss_good {
            s.gnss[0] = good_reading(GnssData {
                position_ned: [Meters(0.0), Meters(0.0), Meters(-100.0)],
                velocity_ned: [
                    MetersPerSecond(50.0),
                    MetersPerSecond(4.0),
                    MetersPerSecond(0.0),
                ],
                fix: GnssFix::ThreeD,
                health: GnssHealth::Good,
            });
        }
        s
    };

    // Without air_aiding configured, pitot data leaves the 15-state filter untouched.
    let plain_ekf = Ekf::default();
    assert!(!plain_ekf.has_air_aiding());
    let mut s_with = state();
    s_with.air = Default::default();
    let mut s_without = s_with.clone();
    plain_ekf.observe(
        &mut s_with,
        &sensors_with_pitot(Some(pitot_pa(50.0)), false),
        None,
        0.01,
    );
    plain_ekf.observe(&mut s_without, &sensors_with_pitot(None, false), None, 0.01);
    assert_eq!(s_with.vel, s_without.vel);
    assert!(!s_with.air.active);

    // With air_aiding configured: uninitialized no-op, activation, slow/missing pitot,
    // accepted pitot, outlier pitot rejection, GNSS cross-update, and validity flags.
    let ekf = Ekf::new(EkfConfig {
        air_aiding: Some(AirAidingConfig {
            wind_prior_var: 1.0,
            ..cfg
        }),
        ..EkfConfig::default()
    });
    assert!(ekf.has_air_aiding());

    let mut unseeded = EkfState::new();
    let mut empty_sensors = blank_sensors();
    empty_sensors.airspeeds = sensors_with_pitot(Some(pitot_pa(50.0)), false).airspeeds;
    ekf.observe(&mut unseeded, &empty_sensors, None, 0.01);
    assert!(!unseeded.air.active);

    let mut st = EkfState::new();
    st.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(-100.0)),
        Vector3::new(
            MetersPerSecond(50.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    // Activates wind block; missing and sub-threshold pitot samples do not reset age_s.
    ekf.observe(&mut st, &sensors_with_pitot(None, false), None, 0.01);
    assert!(st.air.active);
    assert!(st.air.age_s > 1.0);
    ekf.observe(
        &mut st,
        &sensors_with_pitot(Some(pitot_pa(5.0)), false),
        None,
        0.01,
    );
    assert!(st.air.age_s > 1.0);

    // Valid pitot sample fuses and asserts AIRSPEED validity.
    ekf.observe(
        &mut st,
        &sensors_with_pitot(Some(pitot_pa(50.0)), true),
        None,
        0.01,
    );
    assert_eq!(st.air.age_s, 0.0);
    assert!(st
        .get_estimate()
        .valid_flags
        .contains(StateValidFlags::AIRSPEED));

    // Out-of-bounds state_idx guard on follow_state_update leaves wind unchanged.
    let wind_before = st.air.wind_ne;
    st.air
        .follow_state_update(STATE_DIM, &[0.0; STATE_DIM], 1.0, 1.0);
    assert_eq!(st.air.wind_ne, wind_before);

    // Outlier pitot sample is rejected by the innovation gate, leaving age_s > 0.
    ekf.observe(
        &mut st,
        &sensors_with_pitot(Some(pitot_pa(400.0)), false),
        None,
        0.05,
    );
    assert!(st.air.age_s > 0.0);

    // Unbounded air-velocity variance and stale air.age_s each clear AIRSPEED validity.
    let mut unbounded = st.clone();
    unbounded.air.p_ww[0][0] = 500.0;
    assert!(!unbounded
        .get_estimate()
        .valid_flags
        .contains(StateValidFlags::AIRSPEED));
    st.air.age_s = 10.0;
    assert!(!st
        .get_estimate()
        .valid_flags
        .contains(StateValidFlags::AIRSPEED));
}
