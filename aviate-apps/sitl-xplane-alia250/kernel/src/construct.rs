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
    plant_artifact_text: &str,
    simulator_model: &XPlaneSimulatorModel,
) -> Result<CalibratedAliaKernel, AliaKernelBuildError> {
    let resolved = resolve_candidate(
        ALIA250_PRESET,
        candidate_text,
        plant_artifact_text,
        simulator_model,
    )?;
    let (preset, candidate_id, identity, plant_artifact) = resolved.into_parts();
    let gains = gains_from_preset(preset.gains);
    let kernel = build_with(preset, gains)?;
    let manifest = CalibrationRunManifest {
        candidate_id,
        identity,
        simulator_model: ContentDigest::from_hex(&plant_artifact.simulator_model_digest)?,
        kernel_config_hash: kernel.cfg().canonical_hash(),
    };
    Ok(CalibratedAliaKernel { kernel, manifest })
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
) -> Result<DefaultAviateKernel<MultirotorController, QuadXMixerReversedSpin>, AliaKernelBuildError>
{
    let gains = gains_from_preset(gains_to_preset(gains));
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

    const XPLANE_MODEL: &str = include_str!("../../../../presets/alia250-xplane.toml");

    fn model_digest() -> ContentDigest {
        let model = aviate_config::xplane_model::XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL)
            .expect("valid model");
        ContentDigest::from_hex(&model.canonical_digest().expect("model digest").to_string())
            .expect("compatible digest")
    }

    fn plant_artifact() -> String {
        format!(
            "schema_version = 1\nartifact_id = \"plant-a\"\nairframe_id = \"alia250\"\nsimulator_model_digest = \"{}\"\nrun_manifest_digest = \"{}\"\ntrace_digest = \"{}\"\nsample_clock = \"simulator-microseconds\"\noperating_hover_force = 0.43\nprobe_rad_s = [1.0, 2.5]\nsample_rate_hz = 100.0\nauthority_k = [5.3, 3.1, 1.0]\ndelay_s = [0.02, 0.02, 0.03]\ndelay_ci95_s = [0.005, 0.005, 0.005]\nr_squared = [0.96, 0.95, 0.94]\nauthority_ci95 = [0.2, 0.15, 0.05]\ncoherence = [0.96, 0.95, 0.94]\napplied_input_max = [0.2, 0.2, 0.2]\nsample_count = [500, 500, 500]\nsaturation_fraction = [0.0, 0.0, 0.0]\nresponse_sign = [1, 1, 1]\n",
            model_digest(),
            "a".repeat(64),
            "b".repeat(64),
        )
    }

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
        let plant = plant_artifact();
        let plant_digest = ContentDigest::calculate(plant.as_bytes());
        let text = format!(
            "schema_version = 1\ncandidate_id = \"candidate-a\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"inner-loop\"\n[inner_loop]\nnatural_frequency_rad_s = [1.75, 1.75, 0.75]\nloop_separation = [6.25, 6.25, 6.25]\n"
        );
        let model = aviate_config::xplane_model::XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL)
            .expect("valid model");
        let built = build_alia250_kernel_with_candidate(&text, &plant, &model)
            .expect("valid candidate kernel");
        assert_eq!(built.manifest.candidate_id, "candidate-a");
        assert_eq!(built.kernel.cfg().cascade_gains.att_p, [0.7, 0.7, 0.3]);
        assert_eq!(built.manifest.identity.plant_artifact, plant_digest);
        assert_eq!(built.manifest.simulator_model, model_digest());
        assert_eq!(
            built.manifest.kernel_config_hash,
            built.kernel.cfg().canonical_hash()
        );
    }
}
