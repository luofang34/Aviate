//! Error-state EKF: state container + algorithm-identity object.
//!
//! This module hosts the default state estimator (`Ekf` +
//! `EkfState`). The `Estimator` trait surface is generic over an
//! associated `RuntimeState` so non-EKF estimators (MEKF, UKF,
//! complementary filter, particle filter, VIO graph backends) can
//! plug in with their own state shape — a 6-state attitude-only
//! cubesat ADCS filter, an N-particle cloud, or a sliding-window
//! graph all need different on-disk shapes than the 15-state ESKF
//! the EKF uses today.
//!
//! Roles:
//!
//!   - `EkfState` — the persistent 15-state filter contents
//!     (position, velocity, attitude, biases, 15×15 covariance,
//!     init/fault latches). Implements `EstimatorRuntimeState`.
//!     Lives under `KernelState.estimator` (single safety-relevant-
//!     state owner). Pure-state operations (`init`, `reset`,
//!     `get_estimate`, `is_initialized`, `has_numeric_fault`,
//!     test-hook `set_state`) are inherent methods on `EkfState`
//!     — they're EKF-specific implementation details that don't
//!     belong on the generic trait surface.
//!
//!   - `Ekf` — the algorithm-identity object carrying tuning
//!     parameters (`EkfConfig`). Implements `Estimator` with
//!     `type RuntimeState = EkfState`. Trait methods (`predict`,
//!     `update_gnss`, `update_baro`, `update_mag`, `estimate`,
//!     `reset`) take `&self` for config plus
//!     `&mut Self::RuntimeState` to write filter state.
//!
//! Submodules carry the math:
//!   - `ekf/predict.rs` — IMU-driven state and covariance prediction.
//!   - `ekf/update.rs`  — GNSS / baro / mag fusion entry points.
//!   - `ekf/scalar.rs`  — scalar Kalman update kernel + heading
//!     specialization, shared by the fusion entry points.
//!   - `ekf/runtime.rs` — `EstimatorRuntimeState` trait surface.
//!   - `ekf/replicable.rs` — canonical byte encoding for lockstep
//!     cross-channel replication.
//!   - `ekf/validity.rs` — per-source aiding-freshness bookkeeping
//!     and the `get_estimate()` validity/quality derivation.
//!
//! Submodules carry no re-exports to sidestep rustc's coverage
//! phantom-DA issue (see `aviate-core/src/lib.rs` for context); every
//! `aviate_core::ekf::Ekf::X` still resolves from the parent module.

mod predict;
mod replicable;
pub mod runtime;
mod scalar;
mod update;
mod validity;

use crate::control::SensorOverrides;
use crate::ekf::runtime::EstimatorRuntimeState;
use crate::math::{Matrix, Quaternion, Vector3, QUAT_NORM_EPS};
use crate::sensor::SensorSet;
use crate::state::StateEstimate;
#[cfg(feature = "test-hooks")]
use crate::state::StateValidFlags;
use crate::types::{Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Scalar};

/// State estimator contract (LLR-EST-110, LLR-EST-111, LLR-STATE-105).
///
/// Algorithm/state split: the trait carries algorithm identity on
/// `&self` (tuning, configuration) and per-call runtime state on
/// `&mut Self::RuntimeState`. The associated type lets non-EKF
/// estimators plug in with their own state shape; today's only impl
/// (`Ekf`) selects `RuntimeState = EkfState`.
///
/// **Single-cycle observation entry point.** The trait exposes one
/// `observe()` method that consumes a complete `SensorSet` snapshot
/// (plus optional `SensorOverrides`) and advances state by `dt`.
/// Inside `observe()`, each estimator decides which sensor channels
/// it uses, in what order, and how to gate them. EKF-style
/// "predict per IMU then update per sensor" is one shape; UKF
/// batch updates, particle-filter resampling, and graph-keyframe
/// triggers all fit through the same trait surface. Per-sensor
/// methods (`predict_state`, `update_gnss_state`,
/// `update_baro_state`, `update_mag_state`) remain as **inherent
/// helpers** on `Ekf` for direct unit-test access; the kernel
/// calls only `observe()`.
///
/// EKF-specific operations that don't generalize to non-Kalman
/// backends (`init` from raw position/velocity/quaternion,
/// `set_state` from a StateEstimate summary, `is_initialized`,
/// `has_numeric_fault`, `get_estimate`) are inherent methods on
/// `EkfState`. The kernel reads only `estimate(state).quality`
/// (which every implementation must produce), so non-EKF estimators
/// participate without exposing those EKF-shape predicates.
pub trait Estimator {
    /// Persistent runtime state owned by `KernelState.estimator`.
    type RuntimeState: EstimatorRuntimeState;

    /// 64-bit algorithm-identity constant, fixed at the impl site.
    /// Two channels with byte-identical firmware produce the same
    /// `ALGORITHM_ID`; cross-channel mismatch SHALL block lockstep
    /// entry (spec §16). The constant is independent of compiler
    /// version, target triple, and `core::any::type_name` symbol
    /// formatting — those are best-effort and not deterministic.
    /// Allocate from `cert/algorithm_id_registry.toml` to keep IDs
    /// globally unique across estimator implementations.
    const ALGORITHM_ID: u64;

    /// Drive the estimator forward by one cycle, consuming the
    /// kernel's complete `SensorSet` snapshot. Implementations
    /// decide which channels they use (IMU-only attitude filter,
    /// IMU+GNSS+baro+mag tightly-coupled, IMU+VIO loosely-coupled,
    /// …), in what order, and how to gate them; the kernel does
    /// not pre-process or pre-select.
    ///
    /// `overrides` carries kernel-applied test/command overrides
    /// (e.g. forcing GNSS health for failsafe scenarios).
    /// Pass-through estimators may ignore it.
    ///
    /// `dt` is the cycle period in seconds. Implementations bail on
    /// non-finite or non-positive `dt` without touching state.
    fn observe(
        &self,
        state: &mut Self::RuntimeState,
        sensors: &SensorSet,
        overrides: Option<&SensorOverrides>,
        dt: Scalar,
    );

    /// Project the runtime state onto the kernel-facing
    /// `StateEstimate` summary (attitude / angular_velocity /
    /// position_NED / velocity_NED / quality / valid_flags).
    /// Pure: no state mutation.
    fn estimate(&self, state: &Self::RuntimeState) -> StateEstimate;

    /// Return the runtime state to its post-power-on baseline.
    /// Default impl delegates to `runtime.reset()`. Override only if
    /// the algorithm needs to reset additional state outside the
    /// runtime struct.
    fn reset(&self, state: &mut Self::RuntimeState) {
        <Self::RuntimeState as EstimatorRuntimeState>::reset(state);
    }

    /// Optional test-only state injection from a `StateEstimate`
    /// summary. Default impl is a no-op — non-Kalman estimators
    /// (complementary filter, particle filter, graph backends) cannot
    /// generally reconstruct internal state from the kernel-facing
    /// `StateEstimate` projection, so they ignore the call. EKF-shape
    /// estimators (`Ekf`, future MEKF) override to forward the
    /// injection into their concrete state.
    #[cfg(feature = "test-hooks")]
    fn inject_state(&self, state: &mut Self::RuntimeState, est: &StateEstimate) {
        let _ = (state, est);
    }
}

// State dimension: 3 pos, 3 vel, 3 att_err, 3 gyro_bias, 3 accel_bias = 15
pub const STATE_DIM: usize = 15;

// State indices — shared with predict/update/scalar submodules.
// COV:EXCL_START(phantom DA: const decl lines carry coverage
// attribution but have no executable code beyond the literal eval.)
pub(crate) const IDX_POS: usize = 0;
pub(crate) const IDX_VEL: usize = 3;
pub(crate) const IDX_ATT: usize = 6;
pub(crate) const IDX_GB: usize = 9;
pub(crate) const IDX_AB: usize = 12;
// COV:EXCL_STOP

// COV:EXCL_START(phantom DA: struct-field declaration lines for
// EkfConfig and its Default impl carry coverage attributions from
// grcov even though the lines have no executable code beyond the
// struct/literal layout. Same artifact class as the EkfState wrap.)
#[derive(Clone, Copy, Debug)]
pub struct EkfConfig {
    pub process_noise_gyro: Scalar,
    pub process_noise_accel: Scalar,
    pub process_noise_gyro_bias: Scalar,
    pub process_noise_accel_bias: Scalar,
    /// Hard bound on each gyro-bias state component \[rad/s\].
    /// MEMS gyro bias is tens of mrad/s; any estimate beyond this is
    /// the filter mis-attributing attitude motion to bias (the #123
    /// pump: vehicle oscillation + GNSS innovations walked the bias
    /// to whole rad/s, and predict then integrated the phantom rate).
    pub gyro_bias_limit: Scalar,
    /// Hard bound on each accel-bias state component \[m/s²\].
    pub accel_bias_limit: Scalar,
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
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            process_noise_gyro: 1e-3,
            process_noise_accel: 1e-2,
            // Bias states are slow random walks: covariance grows by
            // `noise·dt` per cycle, so these are sized for drift over
            // minutes (σ ≈ 8 mrad/s of gyro-bias walk per minute),
            // not for tracking per-second dynamics — a bias state
            // that can move fast enough to absorb attitude motion
            // recreates the #123 pump.
            process_noise_gyro_bias: 1e-6,
            process_noise_accel_bias: 1e-5,
            gyro_bias_limit: 0.1,
            accel_bias_limit: 0.5,
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
        }
    }
}
// COV:EXCL_STOP
// COV:EXCL_START(phantom DA: grcov attributes non-executable coverage
// regions onto the doc-comment, blank, and struct-field-declaration lines
// of this `Default`/struct-literal chain; no executable code lives here
// beyond the struct literal — same artifact class documented at the head
// of `aviate-core/src/kernel/config.rs` and `aviate-core/src/kernel/state.rs`.)
/// Persistent filter state — the 15-state error-state EKF contents
/// plus initialization and numeric-fault latches. Lives under
/// `KernelState.estimator` (single safety-relevant-state owner).
///
/// Cross-channel redundancy (spec §16) hashes / votes / replicates
/// this struct; downstream Phase 5 will add `encode_canonical()` for
/// deterministic byte serialization.
#[derive(Clone, Debug)]
pub struct EkfState {
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
    /// Last bias-corrected gyro sample, exposed via `get_estimate`'s
    /// `angular_velocity` field. Persisted across cycles because the
    /// caller observes it on cycles between predict() invocations.
    pub last_gyro_body: Vector3<RadiansPerSecond>,
    /// Covariance matrix P (15×15).
    pub p_cov: Matrix<STATE_DIM, STATE_DIM>,
    /// True after a successful `init()`; cleared by `reset()`.
    pub initialized: bool,
    /// INV-27 quaternion-normalization fault latch. Set when a
    /// quaternion mul produces non-finite output; cleared only by
    /// `reset()` or a fresh `init()`.
    pub quat_fault: bool,
    /// QFE origin-referencing datum for baro fusion. The ISA formula
    /// yields absolute MSL pressure altitude, but `pos.z` is
    /// local-origin-relative (as is fused GNSS height). This latches
    /// the NED-Z offset on the first accepted baro sample so the
    /// initial innovation is ≈0 and later samples measure height
    /// change relative to the origin datum. `None` until the first
    /// accepted sample; cleared by `reset()` and `init()`.
    pub baro_ref: Option<Scalar>,
    /// Variance \[m²\] of the QFE `baro_ref` datum for its scalar
    /// random-walk estimator. Seeded when the datum is first latched;
    /// GNSS-anchored height shrinks it and the process-noise floor lets
    /// the datum track slow ground-pressure drift.
    pub baro_ref_var: Scalar,
    /// Seconds since the last ACCEPTED GNSS position fusion (any of the
    /// three axes passing the innovation gate resets this). Drives
    /// `get_estimate()`'s POSITION validity — see `ekf/validity.rs`.
    /// Starts (and is reset by `init()`/`reset()`) at
    /// `validity::AGE_SATURATION_S`, i.e. "never fused", so a freshly
    /// initialized filter does not claim position validity before any
    /// aiding has actually landed.
    pub gnss_pos_age_s: Scalar,
    /// Seconds since the last ACCEPTED GNSS velocity fusion. Same
    /// semantics as `gnss_pos_age_s`, drives VELOCITY validity.
    pub gnss_vel_age_s: Scalar,
    /// Seconds since the last ACCEPTED barometer height fusion. Feeds
    /// `EstimateQuality` (not the POSITION flag itself — horizontal
    /// position is unobservable from baro alone, so only GNSS position
    /// freshness gates that flag; a stale baro still backs `Good` off
    /// to `Degraded` as the vertical-redundancy signal it is).
    pub baro_age_s: Scalar,
}
// COV:EXCL_STOP

/// Initial covariance: position/velocity/attitude blocks start loose
/// (0.1), bias blocks start at realistic MEMS scales — gyro
/// (0.02 rad/s)², accel (0.2 m/s²)². Seeding the gyro-bias variance
/// at 0.1 (σ ≈ 0.32 rad/s, ~30× a real bias) invites the filter to
/// explain early attitude motion as bias: with oscillating flight
/// the GNSS cross-terms walk the bias state to whole rad/s and
/// predict integrates the phantom rate (the #123 pump).
fn initial_covariance() -> Matrix<STATE_DIM, STATE_DIM> {
    let mut p = Matrix::identity().mul_scalar(0.1);
    for i in 0..3 {
        p.set(IDX_GB + i, IDX_GB + i, 4e-4);
        p.set(IDX_AB + i, IDX_AB + i, 0.04);
    }
    p
}

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
            p_cov: initial_covariance(),
            initialized: false,
            quat_fault: false,
            baro_ref: None,
            baro_ref_var: 0.0,
            gnss_pos_age_s: validity::AGE_SATURATION_S,
            gnss_vel_age_s: validity::AGE_SATURATION_S,
            baro_age_s: validity::AGE_SATURATION_S,
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
        self.p_cov = initial_covariance();
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
        self.initialized = state.valid_flags.contains(StateValidFlags::all());
    }
}

impl EstimatorRuntimeState for EkfState {
    fn reset(&mut self) {
        // Delegate to the inherent ground-reset path, which restores
        // the filter to its factory un-initialized posture.
        EkfState::reset(self);
    }
}

impl Default for EkfState {
    fn default() -> Self {
        Self::new()
    }
}

/// EKF algorithm identity — carries tuning configuration only.
/// Persistent filter state lives in `EkfState`, which
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
