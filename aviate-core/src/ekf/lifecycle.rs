//! Construction, seeding, reset and numeric-fault latches of [`EkfState`].

use super::{air_aiding, rebase, seed, validity, EkfState};
use crate::math::{Quaternion, Vector3, QUAT_NORM_EPS};
#[cfg(feature = "test-hooks")]
use crate::state::{StateEstimate, StateValidFlags};
use crate::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond};

impl EkfState {
    /// Construct a fresh state with the initial-uncertainty
    /// covariance from `initial_covariance`. All other fields are
    /// zero / identity.
    pub fn new() -> Self {
        Self {
            quat: Quaternion::IDENTITY,
            pos: Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            vel: Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            gyro_bias: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            accel_bias: Vector3::new(
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
            ),
            last_gyro_body: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            p_cov: seed::initial_covariance(),
            initialized: false,
            quat_fault: false,
            baro_ref: None,
            baro_ref_var: 0.0,
            gnss_pos_age_s: validity::AGE_SATURATION_S,
            gnss_vel_age_s: validity::AGE_SATURATION_S,
            baro_age_s: validity::AGE_SATURATION_S,
            imu_age_s: 0.0,
            accel_clip_s: 0.0,
            air: air_aiding::AirAidingState::default(),
            frame: rebase::NavFrame::default(),
        }
    }

    /// Seed pos/vel/quat and clear bias states; mark initialized.
    /// Any prior numeric-fault latch is cleared on init (INV-27).
    pub fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion) {
        self.pos = pos;
        self.vel = vel;
        self.quat = quat;
        self.gyro_bias = Vector3::new(
            RadiansPerSecond(0.0), // COV:EXCL_BR(inlined-callee fold)
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        );
        self.accel_bias = Vector3::new(
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
        );
        self.p_cov = seed::initial_covariance();
        self.initialized = true;
        self.quat_fault = false;
        // A fresh init/arm re-latches the baro datum on the next
        // accepted sample so field elevation is captured at the
        // current site rather than a stale reference.
        self.baro_ref = None;
        self.baro_ref_var = 0.0;
        // A fresh init/arm has no aiding history yet — any fusion
        // accepted before this init() call must not make the new
        // filter look pre-aided.
        self.gnss_pos_age_s = validity::AGE_SATURATION_S;
        self.gnss_vel_age_s = validity::AGE_SATURATION_S;
        self.baro_age_s = validity::AGE_SATURATION_S;
        self.imu_age_s = 0.0;
        self.accel_clip_s = 0.0;
        // A seed is expressed in the configured frame.
        self.frame = rebase::NavFrame::default();
        self.air = air_aiding::AirAidingState::default();
    }

    /// Whether `init()` has run successfully since construction or `reset()`.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Whether a numeric fault has latched (INV-27 quaternion normalization).
    pub fn has_numeric_fault(&self) -> bool {
        self.quat_fault
    }
    // COV:EXCL_START(phantom DA: grcov attributes a debug-info region onto this doc comment; reset() is exercised by the ground-reset tests)
    /// Ground reset — clear all filter state to a factory
    /// un-initialized posture. Caller (kernel `ground_reset`) is
    /// responsible for ensuring the vehicle is on the ground and
    /// disarmed.
    // COV:EXCL_STOP
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// INV-27: Normalize quaternion and validate result.
    /// Returns IDENTITY and sets quat_fault if normalization fails.
    pub(crate) fn sanitize_quat(&mut self, q: Quaternion) -> Quaternion {
        let q = q.normalize();
        // COV:EXCL_START(DEFENSIVE: INV-27 numerical-corruption guard;
        // is_normalized(QUAT_NORM_EPS) fails only when the normalize()
        // output is NaN/Inf, which requires a corrupted input quaternion.
        // Not reachable from finite sensor paths.)
        if !q.is_normalized(QUAT_NORM_EPS) {
            self.quat_fault = true;
            Quaternion::IDENTITY
        } else {
            q
        }
        // COV:EXCL_STOP
    }

    // COV:EXCL_START(phantom DA: grcov attributes non-executable coverage
    // regions onto the doc-comment and `#[cfg(feature = "test-hooks")]`
    // attribute lines; the fn body below stays covered by test-hooks exercises.)
    /// Inject state for testing (spec §20 test-hooks).
    ///
    /// Directly sets the EKF internal state from an external
    /// `StateEstimate`. Only available with the `test-hooks` feature
    /// enabled.
    #[cfg(feature = "test-hooks")]
    // COV:EXCL_STOP
    pub fn set_state(&mut self, state: &StateEstimate) {
        self.quat = state.attitude;
        self.last_gyro_body = Vector3 {
            x: state.angular_velocity[0],
            y: state.angular_velocity[1],
            z: state.angular_velocity[2],
        };
        self.pos = Vector3 {
            x: state.position_ned[0],
            y: state.position_ned[1],
            z: state.position_ned[2],
        };
        self.vel = Vector3 {
            x: state.velocity_ned[0],
            y: state.velocity_ned[1],
            z: state.velocity_ned[2],
        };
        self.initialized = state.valid_flags.contains(
            StateValidFlags::ATTITUDE
                | StateValidFlags::ANGULAR_RATE
                | StateValidFlags::POSITION
                | StateValidFlags::VELOCITY,
        );
    }
}
