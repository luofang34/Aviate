// TST-CFG-104: structural witness for LLR-CFG-104.
use super::super::MAX_ACTUATORS;
use super::*;
use crate::fault::FaultHandlingTable;
use crate::types::Normalized;

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
    assert_ne!(baseline, cfg.canonical_hash());
}

#[test]
fn canonical_hash_distinguishes_limits() {
    let mut cfg = ResolvedKernelConfig::default();
    let baseline = cfg.canonical_hash();
    cfg.limits.max_roll = crate::types::Radians(cfg.limits.max_roll.0 + 0.1);
    assert_ne!(baseline, cfg.canonical_hash());
}

#[test]
fn canonical_hash_distinguishes_safe_output() {
    let mut cfg = ResolvedKernelConfig::default();
    let baseline = cfg.canonical_hash();
    cfg.safe_output[0] = Normalized(0.5);
    assert_ne!(baseline, cfg.canonical_hash());
}

#[test]
fn canonical_hash_distinguishes_optional_airspeed_limits() {
    // Option<f32> discriminant must contribute. None vs Some(0.0)
    // would otherwise collide.
    let mut cfg_none = ResolvedKernelConfig::default();
    cfg_none.limits.min_airspeed = None;
    let mut cfg_some_zero = ResolvedKernelConfig::default();
    cfg_some_zero.limits.min_airspeed = Some(crate::types::MetersPerSecond(0.0));
    assert_ne!(cfg_none.canonical_hash(), cfg_some_zero.canonical_hash());
}

#[test]
fn canonical_hash_distinguishes_mode_config() {
    let cfg_hover = ResolvedKernelConfig::default();
    let mut cfg_cruise = ResolvedKernelConfig::default();
    cfg_cruise.mode_config.mode = ConfigMode::Cruise;
    assert_ne!(cfg_hover.canonical_hash(), cfg_cruise.canonical_hash());
}

#[test]
fn canonical_hash_distinguishes_fault_table() {
    let cfg_default = ResolvedKernelConfig::default();
    let mut cfg_empty = ResolvedKernelConfig::default();
    cfg_empty.fault_table = FaultHandlingTable { entries: &[] };
    assert_ne!(cfg_default.canonical_hash(), cfg_empty.canonical_hash());
}

#[test]
fn canonical_hash_is_nonzero() {
    // FNV-1a starts from a nonzero offset basis; collapsing to 0
    // would indicate a coding error.
    assert_ne!(ResolvedKernelConfig::default().canonical_hash(), 0);
}

#[test]
fn canonical_hash_distinguishes_all_config_modes() {
    // Exercises every ConfigMode match arm; each mode produces a
    // distinct tag byte, so all four hashes SHALL be pairwise
    // distinct.
    let modes = [
        ConfigMode::Hover,
        ConfigMode::Cruise,
        ConfigMode::Transition,
        ConfigMode::Degraded,
    ];
    let hashes: [u64; 4] = core::array::from_fn(|i| {
        let mut cfg = ResolvedKernelConfig::default();
        cfg.mode_config.mode = modes[i];
        cfg.canonical_hash()
    });
    // Drop the format-arg failure message: the cold-panic argument
    // expression is reported as uncovered by LLVM coverage
    // instrumentation. Default `assert_ne!` failure prints the
    // hash values, which is enough to debug a regression — and the
    // test fn name names the enum.
    let _ = modes;
    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(hashes[i], hashes[j]);
        }
    }
}

/// Hash a single `feed_*` invocation in isolation. Used by the
/// per-tag-match exhaustive tests below.
fn isolated_hash<F: FnOnce(&mut Fnv1a64)>(f: F) -> u64 {
    let mut h = Fnv1a64::new();
    f(&mut h);
    h.finish()
}

#[test]
fn feed_group_kind_covers_all_variants() {
    // Drives every match arm of `feed_group_kind`. Distinct tag
    // bytes per variant mean the 6 isolated hashes SHALL be
    // pairwise distinct.
    let kinds = [
        GroupKind::Multirotor,
        GroupKind::DistributedThrust,
        GroupKind::ControlSurfaces,
        GroupKind::Morphing,
        GroupKind::Auxiliary,
        GroupKind::Custom(42),
    ];
    let hashes: [u64; 6] =
        core::array::from_fn(|i| isolated_hash(|h| feed_group_kind(h, kinds[i])));
    let _ = kinds;
    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(hashes[i], hashes[j]);
        }
    }
}

#[test]
fn feed_coupling_kind_covers_all_variants() {
    let strong = isolated_hash(|h| feed_coupling_kind(h, CouplingKind::Strong));
    let weak = isolated_hash(|h| feed_coupling_kind(h, CouplingKind::Weak));
    assert_ne!(strong, weak);
}

#[test]
fn feed_fallback_policy_covers_all_variants() {
    // HoldLastGood / DecayToSafe / SafePattern — three distinct
    // tag bytes; DecayToSafe additionally folds `tau_ms`.
    let hold = isolated_hash(|h| feed_fallback_policy(h, FallbackPolicy::HoldLastGood));
    let decay =
        isolated_hash(|h| feed_fallback_policy(h, FallbackPolicy::DecayToSafe { tau_ms: 100 }));
    let safe = isolated_hash(|h| feed_fallback_policy(h, FallbackPolicy::SafePattern));
    assert_ne!(hold, decay);
    assert_ne!(decay, safe);
    assert_ne!(hold, safe);
}

#[test]
fn feed_fault_category_covers_all_variants() {
    // 24 variants — exhausting all `feed_fault_category` tag
    // bytes. Pairwise distinctness verifies no two variants
    // collapse to the same tag (which would silently weaken
    // cross-channel verification).
    let cats = [
        FaultCategory::ImuFailed,
        FaultCategory::ImuAllFailed,
        FaultCategory::GnssLost,
        FaultCategory::GnssAllLost,
        FaultCategory::BaroFailed,
        FaultCategory::MagFailed,
        FaultCategory::AirspeedFailed,
        FaultCategory::ActuatorFailed,
        FaultCategory::ActuatorSaturated,
        FaultCategory::ActuatorDisagreement,
        FaultCategory::ActuatorNumericError,
        FaultCategory::ActuatorFallbackPersistent,
        FaultCategory::EstimatorDiverged,
        FaultCategory::AttitudeUncertain,
        FaultCategory::PositionUncertain,
        FaultCategory::NumericError,
        FaultCategory::CommandTimeout,
        FaultCategory::CommandInvalid,
        FaultCategory::TimingViolation,
        FaultCategory::TimingViolationPersistent,
        FaultCategory::ConfigInvalid,
        FaultCategory::ConfigTransitionFailed,
        FaultCategory::MemoryError,
        FaultCategory::EnumInvalid,
    ];
    let hashes: [u64; 24] =
        core::array::from_fn(|i| isolated_hash(|h| feed_fault_category(h, cats[i])));
    let _ = cats;
    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(hashes[i], hashes[j]);
        }
    }
}

#[test]
fn feed_fault_action_covers_all_variants() {
    let actions = [
        FaultAction::Monitor,
        FaultAction::Isolate,
        FaultAction::Degrade,
        FaultAction::Emergency,
    ];
    let hashes: [u64; 4] =
        core::array::from_fn(|i| isolated_hash(|h| feed_fault_action(h, actions[i])));
    let _ = actions;
    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(hashes[i], hashes[j]);
        }
    }
}

#[test]
fn feed_control_law_covers_all_variants() {
    let laws = [
        ControlLawV1::Primary,
        ControlLawV1::Alternate,
        ControlLawV1::Direct,
        ControlLawV1::Backup,
    ];
    let hashes: [u64; 4] =
        core::array::from_fn(|i| isolated_hash(|h| feed_control_law(h, laws[i])));
    let _ = laws;
    for i in 0..hashes.len() {
        for j in (i + 1)..hashes.len() {
            assert_ne!(hashes[i], hashes[j]);
        }
    }
}

#[test]
fn canonical_hash_exercises_non_empty_groups() {
    // Drives the helper paths the default config (empty `groups`
    // slice) cannot reach: per-element fold over
    // `mode_config.groups`, `feed_group_vector` (which calls
    // `feed_f32` + `feed_u16` + `feed_bool`), and the
    // `FallbackPolicy::DecayToSafe { tau_ms }` arm.
    static GROUPS: [ActuatorGroupConfig; 1] = [ActuatorGroupConfig {
        kind: GroupKind::Custom(7),
        coupling: CouplingKind::Weak,
        fallback: FallbackPolicy::DecayToSafe { tau_ms: 250 },
        members: &[0u8, 1, 2],
        safe_pattern: GroupVector {
            outputs: [Normalized(0.25); MAX_ACTUATORS],
            mask: 0x000F,
            valid: true,
        },
    }];

    let mut cfg = ResolvedKernelConfig::default();
    cfg.mode_config.groups = &GROUPS;
    let h_with = cfg.canonical_hash();
    assert_ne!(h_with, 0);
    assert_ne!(
        h_with,
        ResolvedKernelConfig::default().canonical_hash(),
        "non-empty groups slice must change the canonical hash"
    );
}
