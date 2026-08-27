//! Checked Alia 250 kernel construction from one validated preset.

use core::fmt;

use aviate_config::airframe_preset::{
    preset_from_toml_str, resolve_candidate, ActuatorCurve, AirframePreset, CandidateError,
    CandidateIdentity, ContentDigest, MixerKind,
};
use aviate_config::xplane_model::XPlaneSimulatorModel;
use aviate_core::control::cascade_gains::CascadeGains;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::ConfigMode;
use aviate_core::ekf::Ekf;
use aviate_core::kernel::builder::{AviateKernelBuilder, KernelBuildError};
use aviate_core::kernel::config::{ActuatorCurveKind, MixerGeometry, ResolvedKernelConfig};
use aviate_core::mixer::{ModeConfig, QuadXMixerReversedSpin, Sanitizer};
use aviate_core::types::NormalizedThrust;
use aviate_core::DefaultAviateKernel;
use aviate_runtime::sitl_timestamp;

use crate::tuning::{gains_from_preset, gains_to_preset, limits_from_preset};

pub(crate) const ALIA250_PRESET: &str = include_str!("../../../../presets/alia250.toml");

/// The concrete Alia kernel type.
pub type AliaKernel = DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>;

const NOMINAL_HOVER_SCALE_BASIS_POINTS: u16 = 10_000;

/// Online hover estimator state for the immutable Alia initialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HoverEstimatorMode {
    /// No online component changes the hover force.
    Disabled,
}

impl HoverEstimatorMode {
    /// Return the stable manifest value.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Disabled => "disabled",
        }
    }
}

/// Immutable evidence for one force-domain hover initialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HoverInitializationEvidence {
    /// Preset hover-force baseline as IEEE-754 bits.
    pub baseline_force_bits: u32,
    /// Effective hover force as IEEE-754 bits.
    pub effective_force_bits: u32,
    /// Applied scale in basis points.
    pub scale_basis_points: u16,
    /// Online estimator mode for this run.
    pub estimator_mode: HoverEstimatorMode,
    /// Canonical hash of the effective kernel configuration.
    pub effective_kernel_config_hash: u64,
}

/// A checked Alia kernel with its hover initialization evidence.
pub struct InitializedAliaKernel {
    /// Checked flight kernel.
    pub kernel: AliaKernel,
    /// Immutable hover initialization.
    pub hover_initialization: HoverInitializationEvidence,
}

/// Provenance for one candidate-resolved kernel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CalibrationRunManifest {
    /// Stable candidate name.
    pub candidate_id: String,
    /// Preset, candidate, and plant artifact identities.
    pub identity: CandidateIdentity,
    /// Simulator model identity verified by the plant artifact.
    pub simulator_model: ContentDigest,
    /// Canonical identity of the resolved flight-period configuration.
    pub kernel_config_hash: u64,
}

/// A candidate-resolved kernel and its trial provenance.
pub struct CalibratedAliaKernel {
    /// Checked flight kernel.
    pub kernel: AliaKernel,
    /// Immutable run provenance.
    pub manifest: CalibrationRunManifest,
    /// Immutable hover initialization.
    pub hover_initialization: HoverInitializationEvidence,
}

/// An Alia kernel construction error.
#[derive(Debug)]
pub enum AliaKernelBuildError {
    /// The embedded preset cannot be parsed or validated.
    InvalidPreset(String),
    /// The preset names a mixer that this application did not compile.
    UnsupportedMixer(MixerKind),
    /// The checked kernel builder rejected the resolved configuration.
    Kernel(KernelBuildError),
    /// The calibration candidate is invalid or does not match its base.
    Candidate(CandidateError),
    /// The hover-force scale is outside the supported range.
    InvalidHoverScale(u16),
    /// The scaled hover force is outside the valid open normalized interval.
    InvalidEffectiveHover,
}

impl fmt::Display for AliaKernelBuildError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidPreset(error) => write!(formatter, "invalid Alia 250 preset: {error}"),
            Self::UnsupportedMixer(mixer) => {
                write!(formatter, "unsupported Alia 250 mixer: {mixer:?}")
            }
            Self::Kernel(error) => write!(formatter, "kernel construction refused: {error:?}"),
            Self::Candidate(error) => write!(formatter, "calibration candidate refused: {error}"),
            Self::InvalidHoverScale(value) => {
                write!(
                    formatter,
                    "hover-force scale {value} is outside 8000 through 12000"
                )
            }
            Self::InvalidEffectiveHover => {
                formatter.write_str("effective hover force is outside the open interval (0, 1)")
            }
        }
    }
}

impl std::error::Error for AliaKernelBuildError {}

impl From<KernelBuildError> for AliaKernelBuildError {
    fn from(error: KernelBuildError) -> Self {
        Self::Kernel(error)
    }
}

impl From<CandidateError> for AliaKernelBuildError {
    fn from(error: CandidateError) -> Self {
        Self::Candidate(error)
    }
}

/// Build the identification kernel with conservative gains.
///
/// # Errors
///
/// Returns an error when the preset or checked kernel is invalid.
pub fn build_alia250_identification_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, AliaKernelBuildError>
{
    build_alia250_identification_kernel_with_hover_scale(NOMINAL_HOVER_SCALE_BASIS_POINTS)
        .map(|built| built.kernel)
}

/// Build the identification kernel with one checked hover-force scale.
///
/// # Errors
///
/// Returns an error when the scale, preset, or checked kernel is invalid.
pub fn build_alia250_identification_kernel_with_hover_scale(
    scale_basis_points: u16,
) -> Result<InitializedAliaKernel, AliaKernelBuildError> {
    let preset = load_preset()?;
    build_with(preset, CascadeGains::x500_defaults(), scale_basis_points)
}

/// Build the normal Alia 250 kernel from the embedded preset.
///
/// # Errors
///
/// Returns an error when the preset or checked kernel is invalid.
pub fn build_alia250_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, AliaKernelBuildError>
{
    build_alia250_kernel_with_hover_scale(NOMINAL_HOVER_SCALE_BASIS_POINTS)
        .map(|built| built.kernel)
}

/// Build the normal Alia kernel with one checked hover-force scale.
///
/// # Errors
///
/// Returns an error when the scale, preset, or checked kernel is invalid.
pub fn build_alia250_kernel_with_hover_scale(
    scale_basis_points: u16,
) -> Result<InitializedAliaKernel, AliaKernelBuildError> {
    let preset = load_preset()?;
    let gains = gains_from_preset(preset.gains);
    build_with(preset, gains, scale_basis_points)
}

/// Build one kernel from an immutable calibration candidate document.
///
/// # Errors
///
/// Returns an error when the candidate, preset, or checked kernel is invalid.
pub fn build_alia250_kernel_with_candidate(
    candidate_text: &str,
    plant_artifact_text: &str,
    simulator_model: &XPlaneSimulatorModel,
) -> Result<CalibratedAliaKernel, AliaKernelBuildError> {
    build_alia250_kernel_with_candidate_and_hover_scale(
        candidate_text,
        plant_artifact_text,
        simulator_model,
        NOMINAL_HOVER_SCALE_BASIS_POINTS,
    )
}

/// Build one candidate kernel with one checked hover-force scale.
///
/// # Errors
///
/// Returns an error when an input identity, scale, preset, or kernel is invalid.
pub fn build_alia250_kernel_with_candidate_and_hover_scale(
    candidate_text: &str,
    plant_artifact_text: &str,
    simulator_model: &XPlaneSimulatorModel,
    scale_basis_points: u16,
) -> Result<CalibratedAliaKernel, AliaKernelBuildError> {
    let resolved = resolve_candidate(
        ALIA250_PRESET,
        candidate_text,
        plant_artifact_text,
        simulator_model,
    )?;
    let (preset, candidate_id, identity, plant_artifact) = resolved.into_parts();
    let gains = gains_from_preset(preset.gains);
    let built = build_with(preset, gains, scale_basis_points)?;
    let manifest = CalibrationRunManifest {
        candidate_id,
        identity,
        simulator_model: ContentDigest::from_hex(&plant_artifact.simulator_model_digest)?,
        kernel_config_hash: built.kernel.cfg().canonical_hash(),
    };
    Ok(CalibratedAliaKernel {
        kernel: built.kernel,
        manifest,
        hover_initialization: built.hover_initialization,
    })
}

fn load_preset() -> Result<AirframePreset, AliaKernelBuildError> {
    let preset =
        preset_from_toml_str(ALIA250_PRESET).map_err(AliaKernelBuildError::InvalidPreset)?;
    if preset.schema_version != 2 || preset.name != "alia250" {
        return Err(AliaKernelBuildError::InvalidPreset(
            "expected schema 2 preset named alia250".to_owned(),
        ));
    }
    if preset.mixer != MixerKind::QuadXX500ReversedSpin {
        return Err(AliaKernelBuildError::UnsupportedMixer(preset.mixer));
    }
    Ok(preset)
}

fn build_with(
    preset: AirframePreset,
    gains: CascadeGains,
    scale_basis_points: u16,
) -> Result<InitializedAliaKernel, AliaKernelBuildError> {
    let gains = gains_from_preset(gains_to_preset(gains));
    let baseline = preset.hover_thrust_force_seed();
    let hover = scaled_hover_force(baseline, scale_basis_points)?;
    let actuator_curve = match preset.actuator_curve {
        ActuatorCurve::Linear => ActuatorCurveKind::Linear,
        ActuatorCurve::Quadratic => ActuatorCurveKind::QuadraticRotor,
    };
    let cfg = ResolvedKernelConfig {
        limits: limits_from_preset(preset.limits),
        cascade_gains: gains,
        hover_thrust_norm: hover,
        mixer_geometry: MixerGeometry::QuadXX500ReversedSpin,
        actuator_curve,
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
    kernel.state.checks.pre_arm.update_throttle(true);
    let hover_initialization = HoverInitializationEvidence {
        baseline_force_bits: baseline.to_bits(),
        effective_force_bits: hover.0.to_bits(),
        scale_basis_points,
        estimator_mode: HoverEstimatorMode::Disabled,
        effective_kernel_config_hash: kernel.cfg().canonical_hash(),
    };
    Ok(InitializedAliaKernel {
        kernel,
        hover_initialization,
    })
}

fn scaled_hover_force(
    baseline: f32,
    scale_basis_points: u16,
) -> Result<NormalizedThrust, AliaKernelBuildError> {
    if !(8_000..=12_000).contains(&scale_basis_points) {
        return Err(AliaKernelBuildError::InvalidHoverScale(scale_basis_points));
    }
    let effective = (f64::from(baseline) * f64::from(scale_basis_points)
        / f64::from(NOMINAL_HOVER_SCALE_BASIS_POINTS)) as f32;
    if !effective.is_finite() || !(0.0..1.0).contains(&effective) || effective == 0.0 {
        Err(AliaKernelBuildError::InvalidEffectiveHover)
    } else {
        Ok(NormalizedThrust(effective))
    }
}

#[cfg(test)]
mod tests;
