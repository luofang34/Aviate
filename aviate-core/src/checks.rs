//! Unified check system.
//!
//! Provides a consistent pattern for all safety checks in aviate-core.
//! Each check flag is traceable to spec requirements for formal validation.
//!
//! ## Check Categories
//!
//! - [`pre_arm`]   — conditions required before arming (§17 InitState).
//! - [`in_flight`] — continuous monitoring during flight (§14, §15).
//! - [`transition`] — safety checks for config mode changes (§4.5).
//!
//! ## Design Philosophy
//!
//! - Proactive checks (pre-conditions) vs reactive faults (`FaultFlags`).
//! - Each bit traceable to a spec section.
//! - Configurable `required` flags per vehicle type.
//! - `missing()` reports exactly what failed for diagnostics.
//!
//! `KernelChecks` (in [`kernel_checks`]) bundles all three categories as
//! the single value owned by the flight kernel.

pub mod in_flight;
pub mod invariants;
pub mod kernel_checks;
pub mod pre_arm;
pub mod transition;

pub use in_flight::{DegradationReason, InFlightFlags, InFlightStatus};
pub use invariants::CheckInvariants;
pub use kernel_checks::KernelChecks;
pub use pre_arm::{PreArmFlags, PreArmStatus, SampleCounts};
pub use transition::{TransitionFailure, TransitionFlags, TransitionLimits, TransitionStatus};
