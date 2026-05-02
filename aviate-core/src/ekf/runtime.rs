//! Estimator runtime-state contract — the persistent-state side of the
//! `Estimator` trait pair.
//!
//! Companion to `Estimator::predict / update_* / estimate / reset`,
//! which all take `&mut Self::RuntimeState`. Implementors split their
//! on-disk shape into two halves:
//!
//!   - **Algorithm identity / tuning** — `&self` on the estimator
//!     (e.g. `Ekf { config: EkfConfig }`). Read-only across the flight
//!     loop; mutated only at construction. Lives inside
//!     `KernelPipeline` (no per-cycle mutation).
//!   - **Persistent runtime state** — `&mut Self::RuntimeState`. Owns
//!     the filter's actual numerical contents: state vector,
//!     covariance (or whatever the algorithm uses — particle cloud,
//!     attitude quaternion + 3-vec MEKF error, sigma-point cache,
//!     graph-based sliding window), bias terms, init/fault latches.
//!     Lives inside `KernelState.estimator` so the kernel's "exactly
//!     one safety-relevant-state owner" rule covers estimator state
//!     too.
//!
//! `EkfState` (the 18-state ESKF) is the default implementor and
//! today's only one. The associated-type design lets a future MEKF /
//! complementary filter / particle filter / VIO sliding window use
//! its own runtime-state type without forcing every alternative into
//! the EKF's fixed shape (`pos / vel / quat-error / 3-bias / 18×18 P`).
//!
//! Phantom-DA note: this module avoids `pub use submodule::Trait`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale.

/// Persistent runtime state of a state estimator.
///
/// Implementors SHALL provide a `Default` value representing the
/// post-power-on / post-ground-reset baseline (un-initialized filter,
/// zero biases, factory covariance) and a `reset` method that returns
/// the runtime to that baseline without re-allocating.
///
/// Trait bounds:
///
///   - `Default` — required by `KernelState::new` /
///     `KernelState::default` for construction.
///   - `Clone` — Phase-5 cross-channel snapshot replication
///     prerequisite. `KernelState: Clone` requires every field
///     including the estimator's runtime state.
///   - `Debug` — `KernelState` derives `Debug` for diagnostic dumps
///     (`tracing` events, post-mortem panics in tests).
pub trait EstimatorRuntimeState: Default + Clone + core::fmt::Debug {
    /// Return the runtime state to its post-power-on baseline.
    /// Equivalent to `*self = Self::default()` for simple cases;
    /// implementors with allocated buffers may zero-fill in place.
    fn reset(&mut self);
}
