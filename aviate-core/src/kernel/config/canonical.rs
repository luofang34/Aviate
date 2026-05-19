//! `ResolvedKernelConfig::canonical_hash` machinery (LLR-CFG-104).
//!
//! Extracted from `kernel/config.rs` so the parent file stays under
//! the 500-line cap. Holds the FNV-1a 64-bit folder, every per-field
//! / per-variant feeder, and the unit-test suite that pins the
//! encoding rules (TST-CFG-104).
//!
//! See `kernel/config.rs::ResolvedKernelConfig::canonical_hash` for
//! the user-facing entry point.

use crate::control::{ConfigMode, ControlLawV1, Limits};
use crate::fault::{FaultAction, FaultCategory, FaultHandlingTable, FaultResponse};
use crate::mixer::{
    ActuatorGroupConfig, CouplingKind, FallbackPolicy, GroupKind, GroupVector, ModeConfig,
};

use super::ResolvedKernelConfig;

/// FNV-1a 64-bit fold over the entire flight-period configuration.
/// Same constants as `KernelPipeline::algorithm_identity_hash`
/// (LLR-PIPE-103).
pub(super) fn canonical_hash(cfg: &ResolvedKernelConfig) -> u64 {
    let mut h = Fnv1a64::new();
    feed_limits(&mut h, &cfg.limits);
    h.feed_separator();
    feed_mode_config(&mut h, &cfg.mode_config);
    h.feed_separator();
    feed_fault_table(&mut h, &cfg.fault_table);
    h.feed_separator();
    h.feed_u32(cfg.command_timeout_ms);
    h.feed_separator();
    for n in &cfg.safe_output {
        h.feed_f32(n.0);
    }
    h.feed_separator();
    for n in &cfg.slew_limit_per_cycle {
        h.feed_f32(n.0);
    }
    h.feed_separator();
    h.feed_f32(cfg.hover_thrust_norm.0);
    h.finish()
}

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

fn feed_mode_config(h: &mut Fnv1a64, m: &ModeConfig) {
    feed_config_mode(h, m.mode);
    h.feed_usize(m.groups.len());
    for g in m.groups {
        feed_actuator_group_config(h, g);
    }
}

fn feed_config_mode(h: &mut Fnv1a64, mode: ConfigMode) {
    h.feed_u8(match mode {
        ConfigMode::Hover => 0,
        ConfigMode::Cruise => 1,
        ConfigMode::Transition => 2,
        ConfigMode::Degraded => 3,
    });
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
    // the exhaustiveness check fails CI when this lags behind.
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
mod tests;
