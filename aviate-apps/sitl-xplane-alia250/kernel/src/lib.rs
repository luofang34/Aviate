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
//! Tuning status, stated plainly: the attitude cascade is scaled from
//! the X500 derivation by this airframe's estimated plant authority
//! (see `alia250_gains`) and validated by flying it, not by a rig. It
//! holds takeoff, hover and gentle translation; aggressive maneuvering
//! is untested and the outer position loops still carry X500 numbers.

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

/// Force-domain hover trim, MEASURED by the grounded collective sweep
/// (`--sweep`): the simulator's own prop-force dataref crosses the
/// vehicle's ~30.6 kN weight at force ~0.78, and the vehicle lifts off
/// between the 0.73 and 0.82 steps. The 0.35 guess this replaces made
/// the vertical loop operate saturated at all times — every takeoff
/// slammed the collective from idle to maximum, which excites the prop
/// model's spool-up transient instead of flying through it.
const HOVER_TRIM: f32 = 0.78;

/// Measured plant authority per axis, rad/s^2 per unit normalized
/// force-domain torque: the `--identify` flight's output (correlation
/// at the 2.5 rad/s probe, virtual-test-stand run). Roll and yaw are
/// as measured; the pitch channel's measurement was contaminated by
/// cross-axis coupling, so pitch carries roll scaled by the arm ratio
/// (2.5 m lateral / 3.0 m longitudinal). These are the ONLY
/// airframe-specific inputs to the attitude cascade; re-run the
/// experiment and update them when the airframe (or the simulator's
/// model of it) changes.
const PLANT_K: [f32; 3] = [4.5, 3.5, 2.7];

/// Attitude-cascade gains derived from the measured plant.
///
/// Design targets, per the LLR-CTL-202/205 formulation the X500
/// derivation uses (closed-loop wn = sqrt(att_p * K * rate_p), zeta =
/// 0.5 * sqrt(K * rate_p / att_p), separation S = K * rate_p / att_p):
///
/// - zeta = 1.25 (overdamped, no overshoot), hence S = 4 * zeta^2 =
///   6.25 — above the 5x separation floor with margin.
/// - wn = 2 rad/s roll/pitch, 1.2 rad/s yaw: a heavy payload vehicle's
///   response (settle ~1.2 s / ~2 s). Deliberately below the probe
///   frequency where the measured phase already shows the rotors'
///   spool lag — crossing over into that lag is what a hotter target
///   would do, and what the first hand-scaled guess died of.
///
/// Which gives, per axis: att_p = wn / sqrt(S), rate_p = S * att_p / K.
/// The rate command cap keeps a six-ton vehicle from being asked to
/// servo fighter rates; the tilt cap bounds horizontal acceleration to
/// what a payload flight needs.
fn alia250_gains() -> CascadeGains {
    const ZETA: f32 = 1.25;
    const SEPARATION: f32 = 4.0 * ZETA * ZETA;
    const WN: [f32; 3] = [2.0, 2.0, 1.2];
    let mut att_p = [0.0_f32; 3];
    let mut rate_p = [0.0_f32; 3];
    for axis in 0..3 {
        att_p[axis] = WN[axis] / SEPARATION.sqrt();
        rate_p[axis] = SEPARATION * att_p[axis] / PLANT_K[axis];
    }
    CascadeGains {
        att_p,
        att_max_rate_cmd: 1.0,
        rate_p,
        // Damping the long arms would ring with; kept small relative
        // to P so derivative noise cannot dominate the torque demand.
        rate_d: [0.05, 0.05, 0.05],
        vel_max_roll_pitch: 0.25,
        ..CascadeGains::x500_defaults()
    }
}


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
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerX500>, KernelBuildError> {
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
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerX500>, KernelBuildError> {
    build_with(alia250_gains())
}

fn build_with(
    gains: CascadeGains,
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerX500>, KernelBuildError> {
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
