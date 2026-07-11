//! #114 regression: the lockstep-hashed `ResolvedKernelConfig` must
//! carry the SAME tuning the flying controller was constructed from.
//! Two independently initialized copies agree only by coincidence;
//! this pins the bijection for the SITL kernel factory.

#![allow(clippy::expect_used, clippy::panic)]
#![cfg(feature = "env-sitl")]

use aviate_runtime::sim::create_kernel;

#[test]
fn config_and_controller_share_one_gains_source() {
    let kernel = create_kernel();
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
        "hover trim must match the hashed value"
    );
}
