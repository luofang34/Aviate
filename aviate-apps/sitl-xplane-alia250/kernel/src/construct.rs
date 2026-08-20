//! Checked Alia 250 kernel construction from one validated preset.

use core::fmt;

use aviate_config::airframe_preset::{
    preset_from_toml_str, resolve_candidate, ActuatorCurve, AirframePreset, CandidateError,
    CandidateIdentity, MixerKind,
};
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

use crate::tuning::{gains_from_preset, limits_from_preset};

const ALIA250_PRESET: &str = include_str!("../../../../presets/alia250.toml");

/// The concrete Alia kernel type.
pub type AliaKernel = DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>;

/// Provenance for one candidate-resolved kernel.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CalibrationRunManifest {
    /// Stable candidate name.
    pub candidate_id: String,
    /// Preset, candidate, and plant artifact identities.
    pub identity: CandidateIdentity,
    /// Canonical identity of the resolved flight-period configuration.
    pub kernel_config_hash: u64,
}

/// A candidate-resolved kernel and its trial provenance.
pub struct CalibratedAliaKernel {
    /// Checked flight kernel.
    pub kernel: AliaKernel,
    /// Immutable run provenance.
    pub manifest: CalibrationRunManifest,
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
    let preset = load_preset()?;
    build_with(preset, CascadeGains::x500_defaults())
}

/// Build the normal Alia 250 kernel from the embedded preset.
///
/// # Errors
///
/// Returns an error when the preset or checked kernel is invalid.
pub fn build_alia250_kernel(
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, AliaKernelBuildError>
{
    let preset = load_preset()?;
    let gains = gains_from_preset(preset.gains);
    build_with(preset, gains)
}

/// Build one kernel from an immutable calibration candidate document.
///
/// # Errors
///
/// Returns an error when the candidate, preset, or checked kernel is invalid.
pub fn build_alia250_kernel_with_candidate(
    candidate_text: &str,
) -> Result<CalibratedAliaKernel, AliaKernelBuildError> {
    let resolved = resolve_candidate(ALIA250_PRESET, candidate_text)?;
    let gains = gains_from_preset(resolved.preset.gains);
    let kernel = build_with(resolved.preset, gains)?;
    let manifest = CalibrationRunManifest {
        candidate_id: resolved.candidate_id,
        identity: resolved.identity,
        kernel_config_hash: kernel.cfg().canonical_hash(),
    };
    Ok(CalibratedAliaKernel { kernel, manifest })
}

fn load_preset() -> Result<AirframePreset, AliaKernelBuildError> {
    let preset =
        preset_from_toml_str(ALIA250_PRESET).map_err(AliaKernelBuildError::InvalidPreset)?;
    if preset.mixer != MixerKind::QuadXX500ReversedSpin {
        return Err(AliaKernelBuildError::UnsupportedMixer(preset.mixer));
    }
    Ok(preset)
}

fn build_with(
    preset: AirframePreset,
    gains: CascadeGains,
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, AliaKernelBuildError>
{
    let hover = NormalizedThrust(preset.hover_thrust_force_seed());
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
    Ok(kernel)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::expect_used)]

    use super::*;

    #[test]
    fn normal_kernel_uses_the_shipped_preset() {
        let preset = load_preset().expect("valid preset");
        let kernel = build_alia250_kernel().expect("valid kernel");
        assert_eq!(kernel.cfg().cascade_gains, gains_from_preset(preset.gains));
        assert_eq!(kernel.cfg().hover_thrust_norm.0, 0.43);
        assert_eq!(
            kernel.cfg().mixer_geometry,
            MixerGeometry::QuadXX500ReversedSpin
        );
    }

    #[test]
    fn identification_kernel_has_a_matching_config_witness() {
        let kernel = build_alia250_identification_kernel().expect("valid kernel");
        assert_eq!(kernel.cfg().cascade_gains, CascadeGains::x500_defaults());
    }

    #[test]
    fn candidate_build_reports_all_input_identities() {
        let base =
            aviate_config::airframe_preset::ContentDigest::calculate(ALIA250_PRESET.as_bytes());
        let text = format!(
            "schema_version = 1\ncandidate_id = \"candidate-a\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\"\n[gains]\natt_p = [0.7, 0.7, 0.3]\n"
        );
        let built = build_alia250_kernel_with_candidate(&text).expect("valid candidate kernel");
        assert_eq!(built.manifest.candidate_id, "candidate-a");
        assert_eq!(built.kernel.cfg().cascade_gains.att_p, [0.7, 0.7, 0.3]);
        assert_eq!(
            built.manifest.kernel_config_hash,
            built.kernel.cfg().canonical_hash()
        );
    }
}
