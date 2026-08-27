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
        assert_eq!(
            kernel.cfg().hover_kernel_prefix_hash(),
            0x6007_fa96_434b_827d
        );
    }

    #[test]
    fn hover_scale_changes_only_the_force_domain_initialization() {
        let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");
        let model_identity = model.canonical_digest().expect("model identity");
        let low = build_alia250_kernel_with_hover_scale(9_000).expect("0.9 hover scale");
        let nominal = build_alia250_kernel_with_hover_scale(10_000).expect("nominal hover scale");
        let high = build_alia250_kernel_with_hover_scale(11_000).expect("1.1 hover scale");

        assert_eq!(
            f32::from_bits(low.hover_initialization.baseline_force_bits),
            0.43
        );
        assert_eq!(
            low.hover_initialization.baseline_force_bits,
            nominal.hover_initialization.baseline_force_bits
        );
        assert_eq!(
            nominal.hover_initialization.baseline_force_bits,
            high.hover_initialization.baseline_force_bits
        );
        assert_eq!(
            low.kernel.cfg().hover_thrust_norm.0,
            (f64::from(0.43_f32) * 0.9) as f32
        );
        assert_eq!(nominal.kernel.cfg().hover_thrust_norm.0, 0.43);
        assert_eq!(
            high.kernel.cfg().hover_thrust_norm.0,
            (f64::from(0.43_f32) * 1.1) as f32
        );
        assert_eq!(
            low.hover_initialization.estimator_mode,
            HoverEstimatorMode::Disabled
        );
        assert_eq!(
            low.hover_initialization.effective_kernel_config_hash,
            low.kernel.cfg().canonical_hash()
        );
        assert_eq!(
            model.canonical_digest().expect("unchanged model identity"),
            model_identity
        );
    }

    #[test]
    fn hover_scale_bounds_fail_before_kernel_construction() {
        for scale in [7_999, 12_001] {
            assert!(matches!(
                build_alia250_kernel_with_hover_scale(scale),
                Err(AliaKernelBuildError::InvalidHoverScale(value)) if value == scale
            ));
        }
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

    #[test]
    fn allowed_candidate_field_changes_both_resolved_identities() {
        let base = ContentDigest::calculate(ALIA250_PRESET.as_bytes());
        let plant = plant_artifact();
        let plant_digest = ContentDigest::calculate(plant.as_bytes());
        let original = format!(
            "schema_version = 1\ncandidate_id = \"candidate-identity\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"rate-integral-derivative\"\n[gains]\nrate_d_lpf_alpha = 0.45\n"
        );
        let changed = original.replace("rate_d_lpf_alpha = 0.45", "rate_d_lpf_alpha = 0.55");
        let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");

        let first = build_alia250_kernel_with_candidate(&original, &plant, &model)
            .expect("valid original candidate");
        let repeated = build_alia250_kernel_with_candidate(&original, &plant, &model)
            .expect("repeat original candidate");
        let mutated = build_alia250_kernel_with_candidate(&changed, &plant, &model)
            .expect("valid mutated candidate");

        assert_eq!(first.manifest.identity, repeated.manifest.identity);
        assert_eq!(
            first.manifest.kernel_config_hash,
            repeated.manifest.kernel_config_hash
        );
        assert_ne!(
            first.manifest.identity.candidate,
            mutated.manifest.identity.candidate
        );
        assert_ne!(
            first.manifest.kernel_config_hash,
            mutated.manifest.kernel_config_hash
        );
    }

    #[test]
    fn pilotage_candidate_prefix_has_a_pinned_vector() {
        let base = ContentDigest::calculate(ALIA250_PRESET.as_bytes());
        let plant = plant_artifact();
        let plant_digest = ContentDigest::calculate(plant.as_bytes());
        let candidate = format!(
            "schema_version = 1\ncandidate_id = \"pilotage-vector\"\nbase_preset_digest = \"{base}\"\nplant_artifact_digest = \"{plant_digest}\"\nstage = \"inner-loop\"\n[inner_loop]\nnatural_frequency_rad_s = [1.7, 1.8, 0.8]\nloop_separation = [6.0, 6.0, 6.2]\n"
        );
        let model = XPlaneSimulatorModel::from_toml_str(XPLANE_MODEL).expect("valid model");
        let built = build_alia250_kernel_with_candidate(&candidate, &plant, &model)
            .expect("valid candidate kernel");
        assert_eq!(
            built.kernel.cfg().hover_kernel_prefix_hash(),
            0x262e_ca7c_3a3b_18bd
        );
    }
}
