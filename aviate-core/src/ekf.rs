//! Error-state EKF for attitude, position, velocity, and bias estimation.
//!
//! 18-state filter: position, velocity, attitude error, gyro bias, accel
//! bias, mag bias (3 axes each). The predict step and per-sensor updates
//! are split into submodules to keep each file under the 500-line cap;
//! each submodule carries `impl Ekf { ... }` blocks that Rust merges:
//!
//! - `ekf.rs`         — state shape, constants, constructors, accessors.
//! - `ekf/predict.rs` — IMU-driven state and covariance prediction.
//! - `ekf/update.rs`  — sensor fusion entry points (GNSS, baro, mag).
//! - `ekf/scalar.rs`  — the scalar EKF update kernel and its heading
//!   specialization, shared by the fusion entry points.
//!
//! Submodules carry no re-exports to sidestep rustc's coverage phantom-DA
//! issue (see PR for control.rs split for context); every
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

/// State estimator contract (LLR-EST-101..108).
///
/// The kernel consumes estimators only through this trait — concrete
/// implementations (the 18-state error-state EKF in this module today;
/// alternates such as UKF / IEKF / factor-graph backends in future)
/// stay swappable without touching kernel control flow. The surface is
/// intentionally narrow: it covers exactly what `kernel_update.rs` and
/// `kernel_logic.rs` call. Implementation-specific knobs (covariance
/// matrices, tuning parameters, diagnostic accessors) stay on the
/// concrete type.
pub trait Estimator {
    /// Seed pos/vel/quat and clear bias states; mark initialized.
    fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion);

    /// IMU-driven state and covariance propagation. Bails on
    /// non-finite dt or invalid IMU samples without touching state.
    fn predict(&mut self, imu: &ImuData, dt: Scalar);

    /// Fuse a GNSS reading. Health-gated: drops Suspect/Lost or no-fix.
    fn update_gnss(&mut self, gnss_reading: &SensorReading<GnssData>);

    /// Fuse a barometric pressure reading into the altitude channel.
    fn update_baro(&mut self, baro_reading: &SensorReading<BaroData>);

    /// Fuse a magnetometer reading into the heading channel.
    fn update_mag(&mut self, mag_reading: &SensorReading<MagData>);

    /// Snapshot the current state estimate for downstream consumers.
    fn get_estimate(&self) -> StateEstimate;

    /// Whether `init()` has run successfully since construction or fault.
    fn is_initialized(&self) -> bool;

    /// Whether a numeric fault has latched (e.g. INV-27 quat normalization).
    fn has_numeric_fault(&self) -> bool;

    /// Ground reset — clear all filter state to a factory un-initialized
    /// posture. Called from `AviateKernelImpl::ground_reset` to restart
    /// the lifecycle without re-constructing the kernel.
    fn reset(&mut self);

    /// Test-only state injection (spec §20). Bypasses init gate.
    #[cfg(feature = "test-hooks")]
    fn set_state(&mut self, state: &StateEstimate);
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

pub struct Ekf {
    // Core state — pub(crate) so predict/update/scalar submodules can
    // operate on the filter directly without needing accessor methods.
    pub(crate) quat: Quaternion,
    pub(crate) pos: Vector3<Meters>,
    pub(crate) vel: Vector3<MetersPerSecond>,
    pub(crate) gyro_bias: Vector3<RadiansPerSecond>,
    pub(crate) accel_bias: Vector3<MetersPerSecondSquared>,
    pub(crate) mag_bias: Vector3<Microtesla>,

    pub(crate) last_gyro_body: Vector3<RadiansPerSecond>,

    // Covariance P (18x18)
    pub(crate) p_cov: Matrix<STATE_DIM, STATE_DIM>,

    // COV:EXCL_START(phantom DA from ekf/ submodule impl blocks; rustc
    // attributes some submodule-impl coverage back to these struct
    // field declaration lines instead of method bodies. Same artifact
    // class as the previous COV:EXCL markers below the methods —
    // adding the Estimator trait shifted the phantom-DA targets onto
    // the field declarations. No executable code on any of these
    // lines.)
    // Configuration
    pub(crate) config: EkfConfig,

    pub(crate) initialized: bool,

    /// INV-27: Quaternion normalization fault flag (latches until init())
    pub(crate) quat_fault: bool,
    // COV:EXCL_STOP
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new(EkfConfig::default())
    }
}

impl Ekf {
    pub fn new(config: EkfConfig) -> Self {
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
            p_cov: Matrix::identity().mul_scalar(0.1), // Initial uncertainty
            config,
            initialized: false,
            quat_fault: false,
        }
    }

    pub fn is_initialized(&self) -> bool {
        self.initialized
    }
    // COV:EXCL(phantom DA from ekf/ submodule impl blocks; blank line)

    /// Returns true if a quaternion normalization fault has occurred (INV-27).
    /// Fault latches until init() is called.
    // COV:EXCL(phantom DA from ekf/ submodule impl blocks; doc comment)
    pub fn has_numeric_fault(&self) -> bool {
        self.quat_fault
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

    /// Reset filter to factory un-initialized state, preserving the
    /// originally-configured tuning parameters. Caller (kernel
    /// `ground_reset`) is responsible for ensuring the vehicle is on
    /// the ground and disarmed.
    pub fn reset(&mut self) {
        *self = Self::new(self.config);
    }

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
        self.quat_fault = false; // Clear latch on re-init (INV-27)
    }

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

    /// Inject state for testing (spec §20 test-hooks)
    ///
    /// Directly sets the EKF internal state from an external StateEstimate.
    /// Only available with the `test-hooks` feature enabled.
    #[cfg(feature = "test-hooks")]
    pub fn set_state(&mut self, state: &StateEstimate) {
        self.quat = state.attitude;
        self.last_gyro_body = crate::math::Vector3 {
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

// COV:EXCL_START(DELEGATE: every body in this impl forwards to the
// equivalent inherent Ekf method, which carries the actual logic and is
// directly tested by aviate-core/tests/ekf_tests.rs. Covering this block
// would only prove that trait dispatch resolves, not that the EKF math
// is correct.)
impl Estimator for Ekf {
    fn init(&mut self, pos: Vector3<Meters>, vel: Vector3<MetersPerSecond>, quat: Quaternion) {
        Ekf::init(self, pos, vel, quat)
    }

    fn predict(&mut self, imu: &ImuData, dt: Scalar) {
        Ekf::predict(self, imu, dt)
    }

    fn update_gnss(&mut self, gnss_reading: &SensorReading<GnssData>) {
        Ekf::update_gnss(self, gnss_reading)
    }

    fn update_baro(&mut self, baro_reading: &SensorReading<BaroData>) {
        Ekf::update_baro(self, baro_reading)
    }

    fn update_mag(&mut self, mag_reading: &SensorReading<MagData>) {
        Ekf::update_mag(self, mag_reading)
    }

    fn get_estimate(&self) -> StateEstimate {
        Ekf::get_estimate(self)
    }

    fn is_initialized(&self) -> bool {
        Ekf::is_initialized(self)
    }

    fn has_numeric_fault(&self) -> bool {
        Ekf::has_numeric_fault(self)
    }

    fn reset(&mut self) {
        Ekf::reset(self)
    }

    #[cfg(feature = "test-hooks")]
    fn set_state(&mut self, state: &StateEstimate) {
        Ekf::set_state(self, state)
    }
}
// COV:EXCL_STOP
