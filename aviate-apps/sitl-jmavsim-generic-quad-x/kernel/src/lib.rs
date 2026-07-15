//! App-owned X500 kernel construction.
//!
//! Airframe selection is an application decision: this app states that
//! it flies the X500 controller/mixer pair, builds the full resolved
//! configuration, and constructs through `AviateKernelBuilder` — so
//! the controller/config binding check guards the production path. The
//! board receives the kernel by injection and never chooses an
//! airframe.

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::{AviateKernelBuilder, KernelBuildError};
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry};
use aviate_core::mixer::{ModeConfig, QuadXMixerX500, Sanitizer};
use aviate_core::types::NormalizedThrust;
use aviate_core::DefaultAviateKernel;
use aviate_runtime::sitl_timestamp;

/// Build the X500 kernel this app flies. One tuning source: the same
/// gains and hover seed land in the lockstep-hashed configuration and
/// construct the controller, and the builder refuses the kernel if
/// they ever disagree.
pub fn build_x500_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerX500>, KernelBuildError> {
    let gains = CascadeGains::x500_defaults();
    // Force-domain hover trim (#140): the explicit migration of the
    // X500's legacy SPEED-domain seed 0.77 through the quadratic
    // rotor curve, thrust = speed², so 0.77² = 0.5929 — equivalently
    // weight / max_thrust = 20.25 N / 34.19 N. The boundary curve
    // maps it back to the identical 0.77 rotor-speed command at trim
    // (X500 parity pinned by
    // `v1_speed_seed_squared_round_trips_through_the_quadratic_curve`).
    // The formal trim sweep (#140) re-derives the value with saved
    // evidence before the preset may change it.
    let hover = NormalizedThrust(0.5929);
    let cfg = ResolvedKernelConfig {
        cascade_gains: gains,
        hover_thrust_norm: hover,
        mixer_geometry: MixerGeometry::QuadXX500,
        actuator_curve: ActuatorCurveKind::QuadraticRotor,
        mode_config: ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        },
        ..ResolvedKernelConfig::default()
    };

    let mut kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::from_gains(gains, hover.0))
        .mixer(QuadXMixerX500 {
            timestamp_source: sitl_timestamp,
        })
        .sanitizer(Sanitizer)
        .config(cfg)
        .build()?;

    // Default command carries low throttle, so the throttle pre-arm
    // check starts satisfied.
    kernel.state.checks.pre_arm.update_throttle(true);

    Ok(kernel)
}
