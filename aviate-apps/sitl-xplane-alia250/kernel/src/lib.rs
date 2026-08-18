//! App-owned kernel construction for the Alia-250 lift rotors.
//!
//! Airframe selection is an application decision: this app states that
//! it flies the four lift rotors of the simulator's Alia-250 as a
//! quad-X, builds the resolved configuration, and constructs through
//! the checked builder. The board receives the kernel by injection and
//! never chooses an airframe.
//!
//! The rotor arrangement comes from the simulated airframe: lift rotors
//! at ±3.0 m longitudinally and ±2.5 m laterally, with the front-right
//! and rear-left pair spinning opposite the other two — the diagonal
//! pattern `MixerGeometry::QuadXX500` closes yaw correctly for.
//!
//! Tuning status, stated plainly: the cascade gains below are the X500
//! defaults. They are the STARTING point for a vehicle two orders of
//! magnitude heavier, not a tuned solution — this app exists to fly the
//! link, the estimate stream and the payload view end to end, and its
//! attitude tuning is open work.

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

/// Force-domain hover trim. The simulated airframe carries far more
/// thrust margin than an X500, so the collective sits low at hover.
const HOVER_TRIM: f32 = 0.35;

/// Builds the kernel this app flies.
///
/// # Errors
///
/// Returns [`KernelBuildError`] when the controller and the resolved
/// configuration disagree — the builder refuses rather than flying a
/// kernel whose tuning does not match its declared config.
pub fn build_alia250_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerX500>, KernelBuildError> {
    let gains = CascadeGains::x500_defaults();
    let hover = NormalizedThrust(HOVER_TRIM);
    let cfg = ResolvedKernelConfig {
        cascade_gains: gains,
        hover_thrust_norm: hover,
        mixer_geometry: MixerGeometry::QuadXX500,
        // The bridge scales the command straight onto the simulator's
        // throttle lane, whose thrust rises with the square of the
        // command — the same plant shape as a rotor-speed command.
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

    // The default command carries low throttle, so the throttle pre-arm
    // check starts satisfied.
    kernel.state.checks.pre_arm.update_throttle(true);
    Ok(kernel)
}
