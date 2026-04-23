//! Aviate: minimal deterministic hard-real-time motion control kernel.
//!
//! This crate implements the three responsibilities in `docs/AVIATE_SPEC.md`
//! §2.1 — state estimation, stabilization control, actuation — and the
//! `AviateKernelTrait` spec §20 surface. Navigation, mission management,
//! networking, logging, and UI live outside this crate by design.

#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod airframe;
pub mod checks;
pub mod control;
pub mod ekf;
pub mod fault;
pub mod hal;
pub mod kernel;
pub mod kernel_logic;
pub mod kernel_trait;
pub mod kernel_types;
pub mod kernel_update;
pub mod math;
pub mod mixer;
pub mod sensor;
pub mod state;
pub mod time;
pub mod types;

pub use airframe::Airframe;

pub use crate::checks::{
    DegradationReason, InFlightFlags, TransitionFailure, TransitionFlags, TransitionLimits,
};

pub use crate::kernel::{init_core, AviateKernel, AviateKernelImpl, InitState, Watchdog};
// AviateKernelTrait is deliberately NOT re-exported at the crate root:
// rustc's coverage debug info attributes re-exported trait-impl items to
// SF:lib.rs with the defining file's line numbers, producing phantom DA
// entries and phantom FN entries that can't be silenced by COV:EXCL on
// the defining file. Consumers import via `aviate_core::kernel_trait::
// AviateKernelTrait` — no external callers use the short path today.
pub use crate::kernel_types::{
    ArmError, ChannelHealthV1, ChannelId, ChannelStatus, Config, ConfigBlock, ConfigError,
    ConfigTransitionState, CrossChannelData, CycleTiming, DegradationEvent, EnumValidationError,
    EnvelopeMargin, HealthReport, InitResult, TimingStats, TransitionError, UpdateResult,
    CONTROL_LOOP_DEADLINE_US, CONTROL_LOOP_PERIOD_US, CRITICAL_FAULTS, DEFAULT_COMMAND_TIMEOUT_MS,
    TIMING_VIOLATION_THRESHOLD,
};
