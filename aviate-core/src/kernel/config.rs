//! `ResolvedKernelConfig` — validated, flight-period-immutable kernel
//! configuration (LLR-CFG-101..103).
//!
//! Every field here is set ONCE during construction (via the builder)
//! and never mutated during the flight loop. This is what
//! `AviateKernelTrait::get_config()` exposes — and what redundant
//! channels exchange-and-equality-check at startup to confirm they're
//! running the same firmware (same algorithm identity AND same tuning).
//!
//! What goes here:
//!   - `limits`             — flight envelope hard limits (spec §13)
//!   - `mode_config`        — per-mode mixer + actuator group config (spec §4, §9)
//!   - `fault_table`        — fault → degradation lookup (spec §15)
//!   - `command_timeout_ms` — uplink-command staleness threshold (spec §12)
//!   - `safe_output`        — last-ditch fallback actuator pattern (spec §10.5)
//!
//! What does NOT go here (intentional):
//!   - `mode` (current `ConfigMode`) — runtime state, transitions during flight
//!   - any per-cycle counters / fault flags / lifecycle state — those go to
//!     `KernelState` (Phase 3)
//!   - any algorithm identity (estimator, controller, mixer, sanitizer) —
//!     those live on `KernelPipeline` (Phase 2)
//!
//! See `docs/AVIATE_SPEC.md` §19 (Configuration) for the spec contract.

use crate::control::{ConfigMode, Limits};
use crate::fault::FaultHandlingTable;
use crate::kernel_types::DEFAULT_COMMAND_TIMEOUT_MS;
use crate::mixer::ModeConfig;
use crate::types::Normalized;

/// Maximum number of actuator channels the kernel can drive.
/// Mirrors `crate::mixer::MAX_ACTUATORS` — duplicated here as a const
/// to avoid a circular dep on the mixer module just to size a field.
pub const MAX_ACTUATORS: usize = 16;

/// Validated, flight-period-immutable kernel configuration.
///
/// Constructed via `AviateKernelBuilder` — direct field assignment is
/// allowed today but flagged for review post-Phase-5 once the
/// `load_config()` parser lands (DRQ-CFG-001).
// COV:EXCL_START(phantom DA: struct-init lines for Default impl have no
// executable code beyond the literal; rustc's coverage attribution
// places phantom DAs on the field declarations under grcov.)
#[derive(Clone, Debug)]
pub struct ResolvedKernelConfig {
    /// Flight envelope hard limits (spec §13).
    pub limits: Limits,

    /// Per-mode actuator group + mixer configuration (spec §4, §9).
    pub mode_config: ModeConfig,

    /// Fault → degradation policy table (spec §15.2).
    pub fault_table: FaultHandlingTable,

    /// Pilot-command staleness threshold (spec §12). Beyond this, the
    /// kernel synthesizes a failsafe command instead of the last
    /// received one.
    pub command_timeout_ms: u32,

    /// Last-ditch safe actuator output (spec §10.5). Used when
    /// estimator divergence / numeric fault forces a non-controlled
    /// shutdown. Phase 1 keeps this as a single global pattern;
    /// per-mode safe patterns live on `ActuatorGroupConfig.safe_pattern`
    /// inside `mode_config` and supersede this for normal sanitization.
    /// See DRQ-MIX-001 for the full per-mode migration.
    pub safe_output: [Normalized; MAX_ACTUATORS],
}
// COV:EXCL_STOP

impl Default for ResolvedKernelConfig {
    fn default() -> Self {
        Self {
            limits: default_limits(),
            mode_config: ModeConfig {
                mode: ConfigMode::Hover,
                groups: &[],
            },
            fault_table: FaultHandlingTable::DEFAULT,
            command_timeout_ms: DEFAULT_COMMAND_TIMEOUT_MS,
            safe_output: [Normalized(0.0); MAX_ACTUATORS],
        }
    }
}

/// Default flight-envelope limits used by the builder when the caller
/// hasn't supplied custom limits. Mirrors the literal that lived
/// inline in `AviateKernelImpl::new()` pre-Phase-1.
fn default_limits() -> Limits {
    Limits {
        max_roll: crate::types::Radians(0.78), // ~45 deg
        max_pitch: crate::types::Radians(0.78),
        max_roll_rate: crate::types::RadiansPerSecond(3.0),
        max_pitch_rate: crate::types::RadiansPerSecond(3.0),
        max_yaw_rate: crate::types::RadiansPerSecond(3.0),
        max_horizontal_speed: crate::types::MetersPerSecond(10.0),
        max_climb_rate: crate::types::MetersPerSecond(2.0),
        max_descent_rate: crate::types::MetersPerSecond(2.0),
        max_altitude: crate::types::Meters(100.0),
        min_altitude: crate::types::Meters(0.0),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 2.0,
        min_load_factor: 0.0,
    }
}
