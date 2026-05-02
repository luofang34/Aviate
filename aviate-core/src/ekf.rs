//! Error-state EKF: state container + algorithm-identity object.
//!
//! Phase 4 split this module into two roles:
//!
//!   - `EstimatorState` — the persistent 18-state filter contents
//!     (position, velocity, attitude, biases, covariance, init/fault
//!     latches). Lives under `KernelState.estimator` so the kernel
//!     has a single owner for safety-relevant state. Defines the
//!     pure-state operations (init, reset, get_estimate,
//!     is_initialized, has_numeric_fault, test-hook set_state) as
//!     inherent methods.
//!
//!   - `Ekf` — the algorithm-identity object carrying tuning
//!     parameters (`EkfConfig`). The `Estimator` trait's mutation
//!     methods (predict, update_gnss, update_baro, update_mag) take
//!     `&self` for config plus `&mut EstimatorState` to write
//!     filter state. This is the "every safety-relevant persistent
//!     state field has exactly one owner" hard rule established by
//!     SYS-STATE-001 / HLR-STATE-003.
//!
//! Submodules carry the math:
//!   - `ekf/predict.rs` — IMU-driven state and covariance prediction.
//!   - `ekf/update.rs`  — GNSS / baro / mag fusion entry points.
//!   - `ekf/scalar.rs`  — scalar Kalman update kernel + heading
//!     specialization, shared by the fusion entry points.
//!
//! Submodules carry no re-exports to sidestep rustc's coverage
//! phantom-DA issue (see `aviate-core/src/lib.rs` for context); every
//! `aviate_core::ekf::Ekf::X` still resolves from the parent module.

mod predict;
mod scalar;
mod update;

use crate::math::{Matrix, Quaternion, Vector3, QUAT_NORM_EPS};
use crate::sensor::{BaroData, GnssData, ImuData, MagData, SensorReading};
use crate::state::{EstimateQuality, StateEstimate, StateValidFlags};
use crate::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, RadiansPerSecond, Scalar,
};

/// State estimator contract (LLR-EST-101..108, LLR-STATE-105).
///
/// Phase 4: the trait surface is split into algorithm operations
/// (this trait — predict/update_*; take `&self` + `&mut EstimatorState`)
/// and pure-state operations (inherent methods on `EstimatorState`
/// — init/reset/get_estimate/etc.). The implementor (`Ekf`) carries
/// only configuration / tuning; persistent filter state lives in
/// `EstimatorState`. This makes "every safety-relevant persistent
/// state field has exactly one owner (`KernelState`)" structurally
/// enforced — the hard rule for redundant-channel snapshot
/// replication, voting, and hot-spare takeover.
pub trait Estimator {
    /// IMU-driven state and covariance propagation. Bails on
    /// non-finite dt or invalid IMU samples without touching state.
    fn predict(&self, state: &mut EstimatorState, imu: &ImuData, dt: Scalar);

    /// Fuse a GNSS reading. Health-gated: drops Suspect/Lost or no-fix.
    fn update_gnss(&self, state: &mut EstimatorState, gnss_reading: &SensorReading<GnssData>);

    /// Fuse a barometric pressure reading into the altitude channel.
    fn update_baro(&self, state: &mut EstimatorState, baro_reading: &SensorReading<BaroData>);

    /// Fuse a magnetometer reading into the heading channel.
    fn update_mag(&self, state: &mut EstimatorState, mag_reading: &SensorReading<MagData>);
}

// State dimension: 3 pos, 3 vel, 3 att_err, 3 gyro_bias, 3 accel_bias, 3 mag_bias = 18
pub const STATE_DIM: usize = 18;

// State indices — shared with predict/update/scalar submodules.
pub(crate) const IDX_POS: usize = 0;
pub(crate) const IDX_VEL: usize = 3;
pub(crate) const IDX_ATT: usize = 6;
pub(crate) const IDX_GB: usize = 9;
pub(crate) const IDX_AB: usize = 12;
pub(crate) const IDX_MB: usize = 15;

#[derive(Clone, Copy, Debug)]
pub struct EkfConfig {
    pub process_noise_gyro: Scalar,
    pub process_noise_accel: Scalar,
    pub process_noise_gyro_bias: Scalar,
    pub process_noise_accel_bias: Scalar,
    pub meas_noise_gnss_pos: Scalar,
    pub meas_noise_gnss_vel: Scalar,
    pub meas_noise_baro: Scalar,
    /// Heading measurement noise \[rad²\]
    pub meas_noise_mag: Scalar,
    /// Innovation gate threshold (sigma)
    pub innovation_gate: Scalar,

    // Magnetometer fusion config
    /// Inclination vertical ratio at which weight decay begins (default 0.80)
    pub mag_inclination_decay_start: Scalar,
    /// Inclination vertical ratio at which fusion stops (default 0.95)
    pub mag_inclination_limit: Scalar,
    /// Minimum valid field strength \[μT\] (default 20.0)
    pub mag_field_min: Scalar,
    /// Maximum valid field strength \[μT\] (default 70.0)
    pub mag_field_max: Scalar,
    /// Mag bias random walk process noise [μT²/s] (default 1e-5)
    pub process_noise_mag_bias: Scalar,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            process_noise_gyro: 1e-3,
            process_noise_accel: 1e-2,
            process_noise_gyro_bias: 1e-4,
            process_noise_accel_bias: 1e-4,
            meas_noise_gnss_pos: 0.5, // m^2
            meas_noise_gnss_vel: 0.1, // (m/s)^2
            meas_noise_baro: 2.0,     // m^2
            meas_noise_mag: 0.05,     // rad^2 (heading noise)
            innovation_gate: 5.0,     // 5-sigma gate
            // Magnetometer config
            mag_inclination_decay_start: 0.80, // Start weight decay
            mag_inclination_limit: 0.95,       // Stop fusion
            mag_field_min: 20.0,               // μT
            mag_field_max: 70.0,               // μT
            process_noise_mag_bias: 1e-5,      // μT²/s
        }
    }
}

/// Persistent filter state — the 18-state error-state EKF contents
/// plus initialization and numeric-fault latches. Lives under
/// `KernelState.estimator` (single safety-relevant-state owner).
///
/// Cross-channel redundancy (spec §16) hashes / votes / replicates
/// this struct; downstream Phase 5 will add `encode_canonical()` for
/// deterministic byte serialization.
// COV:EXCL_START(phantom DA: struct-init lines for the `Default`-impl
// chain have no executable code beyond the struct literal; rustc's
// coverage attribution places phantom DAs on the field declarations
// under grcov — same artifact class documented at the head of
// `aviate-core/src/kernel/config.rs` and `aviate-core/src/kernel/state.rs`.)
#[derive(Clone, Debug)]
pub struct EstimatorState {
    /// Body→Earth (NED) attitude quaternion.
    pub quat: Quaternion,
    /// Position in NED frame.
    pub pos: Vector3<Meters>,
    /// Velocity in NED frame.
    pub vel: Vector3<MetersPerSecond>,
    /// Gyro bias (body frame).
    pub gyro_bias: Vector3<RadiansPerSecond>,
    /// Accelerometer bias (body frame).
    pub accel_bias: Vector3<MetersPerSecondSquared>,
    /// Magnetometer bias (body frame).
    pub mag_bias: Vector3<Microtesla>,
    /// Last bias-corrected gyro sample, exposed via `get_estimate`'s
    /// `angular_velocity` field. Persisted across cycles because the
    /// caller observes it on cycles between predict() invocations.
    pub last_gyro_body: Vector3<RadiansPerSecond>,
    /// Covariance matrix P (18×18).
    pub p_cov: Matrix<STATE_DIM, STATE_DIM>,
    /// True after a successful `init()`; cleared by `reset()`.
    pub initialized: bool,
    /// INV-27 quaternion-normalization fault latch. Set when a
    /// quaternion mul produces non-finite output; cleared only by
    /// `reset()` or a fresh `init()`.
    pub quat_fault: bool,
}
// COV:EXCL_STOP

impl EstimatorState {
    /// Construct a fresh state with the same initial-uncertainty
    /// covariance the EKF used pre-Phase-4 (`I * 0.1`). All other
    /// fields are zero / identity.
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
            mag_bias: Vector3::new(Microtesla(0.0), Microtesla(0.0), Microtesla(0.0)),
            last_gyro_body: Vector3::new(
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
            ),
            p_cov: Matrix::identity().mul_scalar(0.1),
            initialized: false,
            quat_fault: false,
        }
    }

    /// Seed pos/vel/quat and clear bias states; mark initialized.
    /// Any prior numeric-fault latch is cleared on init (INV-27).
    pub fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion) {
        self.pos = pos;
        self.vel = vel;
        self.quat = quat;
        self.gyro_bias = Vector3::new(
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        );
        self.accel_bias = Vector3::new(
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
            MetersPerSecondSquared(0.0),
        );
        self.mag_bias = Vector3::new(Microtesla(0.0), Microtesla(0.0), Microtesla(0.0));
        self.p_cov = Matrix::identity().mul_scalar(0.1);
        self.initialized = true;
        self.quat_fault = false;
    }

    /// Snapshot the current state estimate for downstream consumers.
    pub fn get_estimate(&self) -> StateEstimate {
        StateEstimate {
            attitude: self.quat,
            angular_velocity: [
                self.last_gyro_body.x,
                self.last_gyro_body.y,
                self.last_gyro_body.z,
            ],
            position_ned: [self.pos.x, self.pos.y, self.pos.z],
            velocity_ned: [self.vel.x, self.vel.y, self.vel.z],
            quality: if self.initialized {
                EstimateQuality::Good
            } else {
                EstimateQuality::Unusable
            },
            valid_flags: if self.initialized {
                StateValidFlags::all()
            } else {
                StateValidFlags::empty()
            },
        }
    }

    /// Whether `init()` has run successfully since construction or `reset()`.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Whether a numeric fault has latched (INV-27 quaternion normalization).
    pub fn has_numeric_fault(&self) -> bool {
        self.quat_fault
    }

    /// Ground reset — clear all filter state to a factory
    /// un-initialized posture. Caller (kernel `ground_reset`) is
    /// responsible for ensuring the vehicle is on the ground and
    /// disarmed.
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

    /// Inject state for testing (spec §20 test-hooks).
    ///
    /// Directly sets the EKF internal state from an external
    /// `StateEstimate`. Only available with the `test-hooks` feature
    /// enabled.
    #[cfg(feature = "test-hooks")]
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
        self.initialized = state.valid_flags.contains(StateValidFlags::all());
    }
}

impl Default for EstimatorState {
    fn default() -> Self {
        Self::new()
    }
}

/// EKF algorithm identity — carries tuning configuration only.
/// Persistent filter state lives in `EstimatorState`, which
/// `predict` / `update_*` mutate via `&mut state` arguments.
#[derive(Clone, Copy, Debug)]
pub struct Ekf {
    pub(crate) config: EkfConfig,
}

impl Ekf {
    pub fn new(config: EkfConfig) -> Self {
        Self { config }
    }
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new(EkfConfig::default())
    }
}
