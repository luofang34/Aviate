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

use crate::control::{ConfigMode, ControlLawV1, Limits};
use crate::fault::{FaultAction, FaultCategory, FaultHandlingTable, FaultResponse};
use crate::kernel_types::DEFAULT_COMMAND_TIMEOUT_MS;
use crate::mixer::{
    ActuatorGroupConfig, CouplingKind, FallbackPolicy, GroupKind, GroupVector, ModeConfig,
};
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
        let mut h = Fnv1a64::new();
        feed_limits(&mut h, &self.limits);
        h.feed_separator();
        feed_mode_config(&mut h, &self.mode_config);
        h.feed_separator();
        feed_fault_table(&mut h, &self.fault_table);
        h.feed_separator();
        h.feed_u32(self.command_timeout_ms);
        h.feed_separator();
        for n in &self.safe_output {
            h.feed_f32(n.0);
        }
        h.finish()
    }
}

/// Internal FNV-1a 64-bit folder. Same constants as
/// `KernelPipeline::algorithm_identity_hash` (LLR-PIPE-103).
struct Fnv1a64 {
    hash: u64,
}

impl Fnv1a64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;

    fn new() -> Self {
        Self { hash: Self::OFFSET }
    }

    fn feed_byte(&mut self, byte: u8) {
        self.hash ^= u64::from(byte);
        self.hash = self.hash.wrapping_mul(Self::PRIME);
    }

    fn feed_bytes(&mut self, bytes: &[u8]) {
        for b in bytes {
            self.feed_byte(*b);
        }
    }

    fn feed_separator(&mut self) {
        // Distinct from any UTF-8 byte and any plausible payload byte
        // adjacency. Same sentinel used by LLR-PIPE-103.
        self.feed_byte(0xff);
    }

    fn feed_u8(&mut self, x: u8) {
        self.feed_byte(x);
    }

    fn feed_u16(&mut self, x: u16) {
        self.feed_bytes(&x.to_le_bytes());
    }

    fn feed_u32(&mut self, x: u32) {
        self.feed_bytes(&x.to_le_bytes());
    }

    fn feed_usize(&mut self, x: usize) {
        // 8-byte fixed width so the same content hashes identically on
        // 32-bit and 64-bit targets (lockstep peers may differ in
        // pointer width even when the firmware is otherwise byte-
        // identical for the kernel state surface).
        self.feed_bytes(&(x as u64).to_le_bytes());
    }

    fn feed_f32(&mut self, x: f32) {
        self.feed_bytes(&x.to_le_bytes());
    }

    fn feed_bool(&mut self, b: bool) {
        self.feed_byte(if b { 1 } else { 0 });
    }

    fn finish(self) -> u64 {
        self.hash
    }
}

fn feed_option_f32(h: &mut Fnv1a64, opt: Option<f32>) {
    match opt {
        None => h.feed_byte(0),
        Some(v) => {
            h.feed_byte(1);
            h.feed_f32(v);
        }
    }
}

fn feed_limits(h: &mut Fnv1a64, l: &Limits) {
    // Field order mirrors the struct declaration in control.rs.
    h.feed_f32(l.max_roll.0);
    h.feed_f32(l.max_pitch.0);
    h.feed_f32(l.max_roll_rate.0);
    h.feed_f32(l.max_pitch_rate.0);
    h.feed_f32(l.max_yaw_rate.0);
    h.feed_f32(l.max_horizontal_speed.0);
    h.feed_f32(l.max_climb_rate.0);
    h.feed_f32(l.max_descent_rate.0);
    h.feed_f32(l.max_altitude.0);
    h.feed_f32(l.min_altitude.0);
    feed_option_f32(h, l.min_airspeed.map(|v| v.0));
    feed_option_f32(h, l.max_airspeed.map(|v| v.0));
    h.feed_f32(l.max_load_factor);
    h.feed_f32(l.min_load_factor);
}

fn feed_config_mode(h: &mut Fnv1a64, mode: ConfigMode) {
    h.feed_u8(config_mode_tag(mode));
}

fn config_mode_tag(mode: ConfigMode) -> u8 {
    match mode {
        ConfigMode::Hover => 0,
        ConfigMode::Cruise => 1,
        ConfigMode::Transition => 2,
        ConfigMode::Degraded => 3,
    }
}

fn feed_mode_config(h: &mut Fnv1a64, m: &ModeConfig) {
    feed_config_mode(h, m.mode);
    h.feed_usize(m.groups.len());
    for g in m.groups {
        feed_actuator_group_config(h, g);
    }
}

fn feed_actuator_group_config(h: &mut Fnv1a64, g: &ActuatorGroupConfig) {
    feed_group_kind(h, g.kind);
    feed_coupling_kind(h, g.coupling);
    feed_fallback_policy(h, g.fallback);
    h.feed_usize(g.members.len());
    h.feed_bytes(g.members);
    feed_group_vector(h, &g.safe_pattern);
}

fn feed_group_kind(h: &mut Fnv1a64, k: GroupKind) {
    match k {
        GroupKind::Multirotor => h.feed_u8(0),
        GroupKind::DistributedThrust => h.feed_u8(1),
        GroupKind::ControlSurfaces => h.feed_u8(2),
        GroupKind::Morphing => h.feed_u8(3),
        GroupKind::Auxiliary => h.feed_u8(4),
        GroupKind::Custom(id) => {
            h.feed_u8(5);
            h.feed_u8(id);
        }
    }
}

fn feed_coupling_kind(h: &mut Fnv1a64, k: CouplingKind) {
    h.feed_u8(match k {
        CouplingKind::Strong => 0,
        CouplingKind::Weak => 1,
    });
}

fn feed_fallback_policy(h: &mut Fnv1a64, p: FallbackPolicy) {
    match p {
        FallbackPolicy::HoldLastGood => h.feed_u8(0),
        FallbackPolicy::DecayToSafe { tau_ms } => {
            h.feed_u8(1);
            h.feed_u16(tau_ms);
        }
        FallbackPolicy::SafePattern => h.feed_u8(2),
    }
}

fn feed_group_vector(h: &mut Fnv1a64, v: &GroupVector) {
    for n in &v.outputs {
        h.feed_f32(n.0);
    }
    h.feed_u16(v.mask);
    h.feed_bool(v.valid);
}

fn feed_fault_table(h: &mut Fnv1a64, t: &FaultHandlingTable) {
    h.feed_usize(t.entries.len());
    for e in t.entries {
        feed_fault_response(h, e);
    }
}

fn feed_fault_response(h: &mut Fnv1a64, r: &FaultResponse) {
    feed_fault_category(h, r.fault);
    feed_fault_action(h, r.action);
    match r.degrade_to {
        None => h.feed_byte(0),
        Some(law) => {
            h.feed_byte(1);
            feed_control_law(h, law);
        }
    }
    h.feed_u32(r.max_response_time_ms);
}

fn feed_fault_category(h: &mut Fnv1a64, c: FaultCategory) {
    // Tag byte order mirrors the enum declaration in fault.rs;
    // adding a new variant requires extending this match (the
    // exhaustiveness check fails CI when this lags behind).
    h.feed_u8(match c {
        FaultCategory::ImuFailed => 0,
        FaultCategory::ImuAllFailed => 1,
        FaultCategory::GnssLost => 2,
        FaultCategory::GnssAllLost => 3,
        FaultCategory::BaroFailed => 4,
        FaultCategory::MagFailed => 5,
        FaultCategory::AirspeedFailed => 6,
        FaultCategory::ActuatorFailed => 7,
        FaultCategory::ActuatorSaturated => 8,
        FaultCategory::ActuatorDisagreement => 9,
        FaultCategory::ActuatorNumericError => 10,
        FaultCategory::ActuatorFallbackPersistent => 11,
        FaultCategory::EstimatorDiverged => 12,
        FaultCategory::AttitudeUncertain => 13,
        FaultCategory::PositionUncertain => 14,
        FaultCategory::NumericError => 15,
        FaultCategory::CommandTimeout => 16,
        FaultCategory::CommandInvalid => 17,
        FaultCategory::TimingViolation => 18,
        FaultCategory::TimingViolationPersistent => 19,
        FaultCategory::ConfigInvalid => 20,
        FaultCategory::ConfigTransitionFailed => 21,
        FaultCategory::MemoryError => 22,
        FaultCategory::EnumInvalid => 23,
    });
}

fn feed_fault_action(h: &mut Fnv1a64, a: FaultAction) {
    h.feed_u8(match a {
        FaultAction::Monitor => 0,
        FaultAction::Isolate => 1,
        FaultAction::Degrade => 2,
        FaultAction::Emergency => 3,
    });
}

fn feed_control_law(h: &mut Fnv1a64, law: ControlLawV1) {
    h.feed_u8(match law {
        ControlLawV1::Primary => 0,
        ControlLawV1::Alternate => 1,
        ControlLawV1::Direct => 2,
        ControlLawV1::Backup => 3,
    });
}

#[cfg(test)]
mod tests {
    // TST-CFG-104: structural witness for LLR-CFG-104.
    use super::*;

    #[test]
    fn canonical_hash_is_deterministic() {
        let cfg_a = ResolvedKernelConfig::default();
        let cfg_b = ResolvedKernelConfig::default();
        assert_eq!(
            cfg_a.canonical_hash(),
            cfg_b.canonical_hash(),
            "two structurally-equal configs must hash equal"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_command_timeout() {
        let mut cfg = ResolvedKernelConfig::default();
        let baseline = cfg.canonical_hash();
        cfg.command_timeout_ms = cfg.command_timeout_ms.wrapping_add(1);
        assert_ne!(
            baseline,
            cfg.canonical_hash(),
            "changing command_timeout_ms must change the canonical hash"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_limits() {
        let mut cfg = ResolvedKernelConfig::default();
        let baseline = cfg.canonical_hash();
        cfg.limits.max_roll = crate::types::Radians(cfg.limits.max_roll.0 + 0.1);
        assert_ne!(
            baseline,
            cfg.canonical_hash(),
            "changing a Limits field must change the canonical hash"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_safe_output() {
        let mut cfg = ResolvedKernelConfig::default();
        let baseline = cfg.canonical_hash();
        cfg.safe_output[0] = Normalized(0.5);
        assert_ne!(
            baseline,
            cfg.canonical_hash(),
            "changing safe_output must change the canonical hash"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_optional_airspeed_limits() {
        // Option<f32> discriminant must contribute. None vs Some(0.0)
        // would otherwise collide.
        let mut cfg_none = ResolvedKernelConfig::default();
        cfg_none.limits.min_airspeed = None;
        let mut cfg_some_zero = ResolvedKernelConfig::default();
        cfg_some_zero.limits.min_airspeed = Some(crate::types::MetersPerSecond(0.0));
        assert_ne!(
            cfg_none.canonical_hash(),
            cfg_some_zero.canonical_hash(),
            "Option<f32> discriminant must contribute to the hash"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_mode_config() {
        let cfg_hover = ResolvedKernelConfig::default();
        let mut cfg_cruise = ResolvedKernelConfig::default();
        cfg_cruise.mode_config.mode = ConfigMode::Cruise;
        assert_ne!(
            cfg_hover.canonical_hash(),
            cfg_cruise.canonical_hash(),
            "changing mode_config.mode must change the canonical hash"
        );
    }

    #[test]
    fn canonical_hash_distinguishes_fault_table() {
        // Default fault_table vs an empty one: different entry counts
        // → different hashes.
        let cfg_default = ResolvedKernelConfig::default();
        let mut cfg_empty = ResolvedKernelConfig::default();
        cfg_empty.fault_table = FaultHandlingTable { entries: &[] };
        assert_ne!(
            cfg_default.canonical_hash(),
            cfg_empty.canonical_hash(),
            "fault_table content must contribute to the hash"
        );
    }

    #[test]
    fn canonical_hash_is_nonzero() {
        // FNV-1a starts from a nonzero offset basis; collapsing to 0
        // would indicate a coding error (e.g. an early return or a
        // skipped fold step).
        assert_ne!(ResolvedKernelConfig::default().canonical_hash(), 0);
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
