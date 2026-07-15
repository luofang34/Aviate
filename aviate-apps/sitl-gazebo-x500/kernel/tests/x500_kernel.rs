//! The lockstep-hashed `ResolvedKernelConfig` must carry the SAME
//! tuning the flying controller was constructed from, and the wire
//! identity of the X500 build is pinned: the resolved-config hash and
//! the algorithm-identity hash may move only as a reviewed decision
//! (a tuning change, or an extension of the hashed tuning surface),
//! never as a construction-path accident.

#![allow(clippy::expect_used, clippy::panic)]

use aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel;

#[test]
fn config_and_controller_share_one_gains_source() {
    let kernel = build_x500_kernel().expect("binding check must accept the single-source build");
    assert_eq!(
        kernel.cfg().cascade_gains,
        *kernel.pipeline().controller.velocity_gains(),
        "velocity loop must fly the hashed gains"
    );
    assert_eq!(
        kernel.cfg().cascade_gains,
        *kernel.pipeline().controller.rate_gains(),
        "rate loop must fly the hashed gains"
    );
    assert!(
        (kernel.cfg().hover_thrust_norm.0 - kernel.pipeline().controller.hover_thrust_norm()).abs()
            < f32::EPSILON,
        "hover trim must fly the hashed value"
    );
}

#[test]
fn app_built_kernel_matches_pre_change_identity() {
    let kernel = build_x500_kernel().expect("binding check must accept the single-source build");
    // Canonical-hash pin. Moves when the hashed tuning surface or its
    // encoding changes — e.g. a `CascadeGains` field addition — even
    // when every flown value is unchanged; the algorithm-identity pin
    // below is the witness that the control law itself is the same.
    assert_eq!(kernel.cfg().canonical_hash(), 0x1b84_cab8_60c4_5a0c);
    assert_eq!(
        kernel.pipeline().algorithm_identity_hash(),
        0x20ce_8c48_7287_24d5
    );
}
