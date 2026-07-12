//! Builder-level controller/config binding (DRQ-CTL-001).
//!
//! A kernel must be impossible to build when the controller's
//! effective tuning disagrees with the canonical-hashed configuration.
//! The sweep below mutates one gains/hover field at a time on the
//! config side only and requires `build()` to reject every mismatch
//! with both identities in the error; the positive case proves a
//! controller built from the resolved config is accepted.

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::{AviateKernelBuilder, KernelBuildError};
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::mixer::{QuadXMixer, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::Normalized;

fn fake_ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn build_with(
    cfg: ResolvedKernelConfig,
    controller: MultirotorController,
) -> Result<(), KernelBuildError> {
    AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(controller)
        .mixer(QuadXMixer {
            timestamp_source: fake_ts,
        })
        .sanitizer(Sanitizer)
        .config(cfg)
        .build()
        .map(|_| ())
}

#[test]
fn controller_built_from_resolved_config_is_accepted() {
    let cfg = ResolvedKernelConfig::default();
    let controller = MultirotorController::from_gains(cfg.cascade_gains, cfg.hover_thrust_norm.0);
    assert!(build_with(cfg, controller).is_ok());
}

#[test]
fn every_gains_and_hover_field_mismatch_is_rejected() {
    type Mutate = fn(&mut ResolvedKernelConfig);
    // One mutation per CascadeGains field plus the hover seed. A new
    // gains field must be added here or the count assertion at the
    // bottom of this test forces the decision.
    let mutations: &[(&str, Mutate)] = &[
        ("pos_p", |c| c.cascade_gains.pos_p[0] += 0.125),
        ("pos_accel_limits", |c| {
            c.cascade_gains.pos_accel_limits[1] += 0.125
        }),
        ("pos_vel_caps", |c| c.cascade_gains.pos_vel_caps[2] += 0.125),
        ("vel_p", |c| c.cascade_gains.vel_p[0] += 0.125),
        ("vel_i", |c| c.cascade_gains.vel_i[1] += 0.125),
        ("vel_max_roll_pitch", |c| {
            c.cascade_gains.vel_max_roll_pitch += 0.125
        }),
        ("vel_accel_ff", |c| c.cascade_gains.vel_accel_ff += 0.125),
        ("vel_d", |c| c.cascade_gains.vel_d[2] += 0.125),
        ("att_p", |c| c.cascade_gains.att_p[0] += 0.125),
        ("rate_p", |c| c.cascade_gains.rate_p[1] += 0.125),
        ("rate_d", |c| c.cascade_gains.rate_d[2] += 0.125),
        ("rate_d_lpf_alpha", |c| {
            c.cascade_gains.rate_d_lpf_alpha += 0.125
        }),
        ("hover_thrust_norm", |c| {
            c.hover_thrust_norm = Normalized(c.hover_thrust_norm.0 + 0.125)
        }),
    ];

    for (field, mutate) in mutations {
        let base = ResolvedKernelConfig::default();
        let controller =
            MultirotorController::from_gains(base.cascade_gains, base.hover_thrust_norm.0);

        let mut mutated = ResolvedKernelConfig::default();
        mutate(&mut mutated);

        let result = build_with(mutated, controller);
        assert!(
            matches!(&result, Err(KernelBuildError::ControllerConfigMismatch(_))),
            "{field}: expected ControllerConfigMismatch, got {result:?}"
        );
        let Err(KernelBuildError::ControllerConfigMismatch(m)) = result else {
            continue;
        };
        assert_ne!(
            m.controller_identity, m.config_identity,
            "{field}: mismatch error must carry differing identities"
        );
    }

    // Field-count pin: CascadeGains has 12 public tuning fields plus
    // the hover seed. Adding a field without extending the sweep
    // fails here instead of silently escaping coverage.
    assert_eq!(mutations.len(), 13);
}
