//! Controller runtime state — the persistent-state side of the
//! `VehicleController` trait pair (LLR-CTL-102, LLR-CTL-103).
//!
//! Companion to `VehicleController::step(&self, runtime: &mut
//! Self::RuntimeState, ...)`. Implementors split their on-disk shape
//! into two halves:
//!
//!   - **Algorithm identity / tuning gains** — `&self` on the
//!     controller (e.g. `MultirotorController { pos_ctrl, vel_ctrl,
//!     rate_ctrl, att_ctrl }`). Read-only across the flight loop.
//!     Lives inside `KernelPipeline` (no per-cycle mutation).
//!   - **Persistent runtime state** — `&mut Self::RuntimeState`.
//!     Owns rate-loop integrators, anti-windup memories, derivative
//!     filters, command slew-limiter accumulators, VTOL transition
//!     blend coefficients, fixed-wing energy-loop memory, mode-latch
//!     flip-flops, etc. Lives inside `KernelState.control` so the
//!     kernel's "exactly one safety-relevant-state owner" rule
//!     covers controller state too — making it amenable to the same
//!     snapshot / hash / vote / hot-spare-takeover machinery as the
//!     EKF and sanitizer fallback states.
//!
//! Today's implementors (P-only multirotor / fixed-wing / VTOL
//! placeholders) are gains-only and use the unit-struct
//! [`NoControllerState`] sentinel — the trait's `&mut RuntimeState`
//! borrow is then a no-op, but the structural contract is in place
//! so tomorrow's controllers grow integrators without a second
//! refactor.
//!
//! Phantom-DA note: this module avoids `pub use submodule::Trait`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale.

/// Persistent runtime state of a vehicle controller.
///
/// Implementors SHALL provide a `Default` value representing the
/// post-power-on / post-ground-reset baseline (zero integrators,
/// zero filter histories, neutral mode latches), and a `reset`
/// method that returns the runtime to that baseline without
/// re-allocating. `reset` is called by the kernel on transitions
/// that invalidate accumulated controller memory (`ground_reset`,
/// `disarm`, control-law change to `Backup`).
pub trait ControllerRuntimeState: Default {
    /// Return the runtime state to its post-construction baseline.
    /// Equivalent to `*self = Self::default()` for simple cases;
    /// implementors with allocated buffers may zero-fill in place.
    fn reset(&mut self);
}

/// Zero-size placeholder for controllers with no persistent runtime
/// state. Used by today's gains-only implementors
/// (`MultirotorController`, `FixedWingController`,
/// `VtolController`); a controller that grows persistent state
/// swaps `type RuntimeState = NoControllerState` for its own
/// `ControllerRuntimeState`-implementing struct.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NoControllerState;

impl ControllerRuntimeState for NoControllerState {
    fn reset(&mut self) {
        // Unit struct: no per-instance state to clear.
    }
}
