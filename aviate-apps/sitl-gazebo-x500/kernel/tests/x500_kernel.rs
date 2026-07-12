//! The lockstep-hashed `ResolvedKernelConfig` must carry the SAME
//! tuning the flying controller was constructed from, and app-owned
//! builder construction must be indistinguishable on the wire from
//! the construction it replaced: the resolved-config hash and the
//! algorithm-identity hash are pinned to the retired factory's values.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;

#[test]
fn config_and_controller_share_one_gains_source() {
    let kernel = build_x500_kernel().expect("binding check must accept the single-source build");
    assert_eq!(
        kernel.cfg.cascade_gains, kernel.pipeline.controller.vel_ctrl.gains,
        "velocity loop must fly the hashed gains"
    );
    assert_eq!(
        kernel.cfg.cascade_gains, kernel.pipeline.controller.rate_ctrl.gains,
        "rate loop must fly the hashed gains"
    );
    assert!(
        (kernel.cfg.hover_thrust_norm.0 - kernel.pipeline.controller.vel_ctrl.hover_thrust_norm)
            .abs()
            < f32::EPSILON,
        "hover trim must fly the hashed value"
    );
}

#[test]
fn app_built_kernel_matches_pre_change_identity() {
    let kernel = build_x500_kernel().expect("binding check must accept the single-source build");
    assert_eq!(kernel.cfg.canonical_hash(), 0xbb2e_268f_867c_9e9c);
    assert_eq!(
        kernel.pipeline.algorithm_identity_hash(),
        0x20ce_8c48_7287_24d5
    );
}
