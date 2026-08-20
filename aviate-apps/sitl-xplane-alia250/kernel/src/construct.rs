//! Kernel construction through the checked builder: the flight
//! cascade, the identification cascade, and the shared resolved
//! configuration both build through.

use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::{AviateKernelBuilder, KernelBuildError};
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry};
use aviate_core::mixer::{ModeConfig, QuadXMixerReversedSpin, Sanitizer};
use aviate_core::types::NormalizedThrust;
use aviate_core::DefaultAviateKernel;
use aviate_runtime::sitl_timestamp;

use crate::tuning::{alia250_gains, hover_trim};

/// Builds the kernel the identification experiment flies: the X500
/// default cascade, which is marginal on this airframe but demonstrably
/// holds a short hop. Identification must not fly the gains it exists
/// to derive — a bang-bang cascade tips the vehicle before the first
/// excitation window opens.
///
/// # Errors
///
/// Returns [`KernelBuildError`] as [`build_alia250_kernel`] does.
pub fn build_alia250_identification_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, KernelBuildError> {
    build_with(CascadeGains::x500_defaults())
}

/// Builds the kernel this app flies.
///
/// # Errors
///
/// Returns [`KernelBuildError`] when the controller and the resolved
/// configuration disagree — the builder refuses rather than flying a
/// kernel whose tuning does not match its declared config.
pub fn build_alia250_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, KernelBuildError> {
    // `AVIATE_CASCADE=x500` flies the stock X500 cascade instead of
    // the Alia derivation — the right tuning when the simulator is
    // pointed at an X500-class airframe for a demo or a tuning
    // session. SITL-only affordance, like the plant overrides above.
    if std::env::var("AVIATE_CASCADE").as_deref() == Ok("x500") {
        return build_with(CascadeGains::x500_defaults());
    }
    build_with(alia250_gains())
}

fn build_with(
    gains: CascadeGains,
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, KernelBuildError> {
    let hover = NormalizedThrust(hover_trim());
    let cfg = ResolvedKernelConfig {
        cascade_gains: gains,
        hover_thrust_norm: hover,
        mixer_geometry: MixerGeometry::QuadXX500ReversedSpin,
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
        .mixer(QuadXMixerReversedSpin {
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
