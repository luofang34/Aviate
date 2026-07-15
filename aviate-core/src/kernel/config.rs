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

use crate::control::cascade_gains::CascadeGains;
use crate::control::{ConfigMode, Limits};
use crate::fault::FaultHandlingTable;
use crate::kernel_types::DEFAULT_COMMAND_TIMEOUT_MS;
use crate::mixer::ModeConfig;
use crate::types::{Normalized, NormalizedThrust};

pub mod actuation;
mod canonical;

pub use actuation::{ActuatorCurveKind, MixerGeometry};

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

    /// Per-actuator per-cycle slew limit (DRQ-FLT-001 / DRQ-MORPH-001).
    ///
    /// `slew_limit_per_cycle[i] > 0`: the per-cycle delta on channel
    /// `i` is clamped to `±slew_limit_per_cycle[i]` of the previous
    /// cycle's output. `<= 0` or non-finite: channel unconstrained
    /// (default — preserves existing airframe behavior).
    ///
    /// Applies only in the normal control path; severe-fault
    /// early-return paths (numeric error, enum corruption) bypass
    /// this and emit the safe pattern immediately (LLR-FLT-205).
    pub slew_limit_per_cycle: [Normalized; MAX_ACTUATORS],

    /// Cascade tuning — every PID gain and limit the multirotor
    /// controller reads. Owned here, not on the controller struct,
    /// so `canonical_hash` covers tuning (DRQ-CTL-001). Before this
    /// landed, gains lived as constructor arguments on
    /// `MultirotorController` and were invisible to lockstep:
    /// two channels could disagree on tuning silently because
    /// `algorithm_identity_hash` only sees algorithm classes.
    pub cascade_gains: CascadeGains,

    /// Per-airframe hover-thrust trim in the FORCE domain (#140):
    /// the fraction of maximum total thrust at which motor thrust
    /// equals airframe weight, `weight / max_thrust`. For the X500
    /// (2.06 kg mass, 34.19 N max thrust) this is 20.25/34.19
    /// ≈ 0.59.
    ///
    /// The closed-loop velocity controller uses this as the offset
    /// around which it commands collective-thrust corrections; wrong
    /// value here means the closed loop saturates trying to hold
    /// altitude. The rotor-speed / PWM shape of the plant is NOT
    /// this field's concern — `actuator_curve` converts force to the
    /// boundary command exactly once at the board/simulator edge.
    ///
    /// Default 0.5: safe for builds whose airframe has not yet been
    /// calibrated — the closed loop will be sluggish but will not
    /// destabilize at full saturation.
    pub hover_thrust_norm: NormalizedThrust,

    /// Registered mixer geometry this configuration was resolved
    /// for (#140). Declaration-side witness folded into
    /// `canonical_hash`; the compiled mixer TYPE is separately
    /// witnessed by `algorithm_identity_hash`. The app is
    /// responsible for mapping its preset's geometry onto the
    /// compiled mixer it injects.
    pub mixer_geometry: MixerGeometry,

    /// Actuator curve between the cascade's force-domain collective
    /// and the boundary command (#140). Resolved from the airframe
    /// preset, folded into `canonical_hash`, and applied EXACTLY
    /// ONCE at the board/simulator boundary via
    /// [`ActuatorCurveKind::boundary_command`].
    pub actuator_curve: ActuatorCurveKind,
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
            slew_limit_per_cycle: [Normalized(0.0); MAX_ACTUATORS],
            cascade_gains: CascadeGains::default(),
            hover_thrust_norm: NormalizedThrust(0.5),
            mixer_geometry: MixerGeometry::QuadX,
            actuator_curve: ActuatorCurveKind::Linear,
        }
    }
}

impl ResolvedKernelConfig {
    /// Deterministic 64-bit canonical hash over the entire flight-period
    /// configuration (LLR-CFG-104).
    ///
    /// Companion to `KernelPipeline::algorithm_identity_hash`
    /// (HLR-PIPE-002). Spec §16 lockstep entry verifies firmware-bundle
    /// agreement across channels via TWO orthogonal witnesses:
    ///
    ///   - `algorithm_identity_hash` — same algorithm classes
    ///     (estimator / controller / mixer / sanitizer types).
    ///   - `canonical_hash` — same flight-envelope limits, same mode
    ///     config, same fault table, same command-timeout threshold,
    ///     same last-ditch safe output.
    ///
    /// Two channels with byte-identical firmware AND byte-identical
    /// resolved configuration produce the same `(identity, canonical)`
    /// pair; mismatch on either witness blocks lockstep entry. This is
    /// the orthogonal half the user-tunable parameters need that
    /// `algorithm_identity_hash` deliberately doesn't cover (it sees
    /// only types).
    ///
    /// FNV-1a 64-bit fold over a tagged byte serialization of every
    /// field in declaration order. Field separator `0xff` between
    /// fields prevents concatenation aliasing across boundaries; enums
    /// are folded by tag byte; floats by `f32::to_le_bytes` (target-
    /// endian-independent); options by a discriminant byte plus
    /// payload; slices by length-prefix plus per-element folding.
    ///
    /// Determinism boundary: the hash is exact across any two
    /// invocations producing structurally-equal `ResolvedKernelConfig`
    /// values. Static-slice fields (`mode_config.groups`,
    /// `fault_table.entries`) participate by content, not by pointer
    /// identity — two channels with separately-built but content-
    /// equal slices hash equal.
    ///
    /// Not yet covered (deferred to Phase-5): a serialized form
    /// suitable for transmission across the cross-channel link
    /// (`encode_canonical`); this method only produces the hash, not
    /// the bytes.
    pub fn canonical_hash(&self) -> u64 {
        canonical::canonical_hash(self)
    }

    /// Canonical identity over the slice of this configuration a
    /// multirotor controller copies (`cascade_gains` plus
    /// `hover_thrust_norm`). The builder rejects a kernel whose
    /// controller reports a different value (LLR-CFG-103 binding).
    pub fn controller_tuning_identity(&self) -> u64 {
        canonical::controller_tuning_identity(&self.cascade_gains, self.hover_thrust_norm.0)
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

/// Canonical identity over a gains/hover pair. Controller
/// construction and `ResolvedKernelConfig::controller_tuning_identity`
/// both call this one function, so their encodings cannot drift.
///
/// Only the multirotor controller constructs through this wrapper, so it
/// is gated to the `mc` feature to stay dead-code-free when `mc` is off.
#[cfg(feature = "mc")]
pub(crate) fn canonical_controller_tuning_identity(
    gains: &CascadeGains,
    hover_thrust_norm: crate::types::Scalar,
) -> u64 {
    canonical::controller_tuning_identity(gains, hover_thrust_norm)
}
