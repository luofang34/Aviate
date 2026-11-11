//! One fixed-wing airframe and the kernel parts that follow from it.

use aviate_core::checks::PreArmFlags;
use aviate_core::control::fixed_wing::FixedWingController;
use aviate_core::control::{ConfigMode, Limits};
use aviate_core::ekf::air_aiding::AirAidingConfig;
use aviate_core::ekf::{Ekf, EkfConfig};
use aviate_core::kernel::builder::{AviateKernelBuilder, KernelBuildError};
use aviate_core::kernel::config::{
    FixedWingParams, FixedWingParamsError, MixerGeometry, ResolvedKernelConfig,
};
use aviate_core::kernel::DefaultAviateKernel;
use aviate_core::mixer::fixed_wing::{
    neutral_outputs, FixedWingMixer, AILERON_LANE, ELEVATOR_LANE, RUDDER_LANE, THROTTLE_LANE,
};
use aviate_core::mixer::{
    ActuatorGroupConfig, CouplingKind, FallbackPolicy, GroupKind, GroupVector, ModeConfig,
    Sanitizer, MAX_ACTUATORS,
};
use aviate_core::time::Timestamp;
use aviate_core::types::Normalized;

/// One fixed-wing vehicle: its identifier, its description for the
/// control law, and its envelope limits.
#[derive(Clone, Debug)]
pub struct FixedWingAirframe {
    /// Stable identifier, lowercase and hyphen-separated.
    pub id: &'static str,
    /// Airframe description and loop tuning for the control law.
    pub params: FixedWingParams,
    /// Envelope limits for the kernel.
    pub limits: Limits,
    /// Air-data aiding of the estimator: airspeed, zero sideslip, wind.
    pub air_aiding: AirAidingConfig,
}

/// Why [`FixedWingAirframe::kernel`] cannot build a kernel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FixedWingKernelError {
    /// The description cannot fly.
    Params(FixedWingParamsError),
    /// The kernel parts do not agree.
    Build(KernelBuildError),
}

/// Safe pattern of the surface group: every surface at neutral.
const fn neutral_surfaces() -> GroupVector {
    let mut outputs = [Normalized(0.0); MAX_ACTUATORS];
    let mut lane = AILERON_LANE;
    while lane <= RUDDER_LANE {
        outputs[lane] = Normalized(0.5);
        lane += 1;
    }
    GroupVector {
        outputs,
        mask: 0b1110,
        valid: true,
    }
}

/// Safe pattern of the throttle group: idle.
const fn idle_throttle() -> GroupVector {
    GroupVector {
        outputs: [Normalized(0.0); MAX_ACTUATORS],
        mask: 0b0001,
        valid: true,
    }
}

/// Actuator groups of a conventional fixed wing.
///
/// A surface command that is not a number must not move one surface
/// alone, so the three surfaces fall back together. The throttle is its
/// own group: a bad throttle command must not freeze the surfaces.
static GROUPS: [ActuatorGroupConfig; 2] = [
    ActuatorGroupConfig {
        kind: GroupKind::DistributedThrust,
        coupling: CouplingKind::Strong,
        fallback: FallbackPolicy::SafePattern,
        members: &[THROTTLE_LANE as u8],
        safe_pattern: idle_throttle(),
    },
    ActuatorGroupConfig {
        kind: GroupKind::ControlSurfaces,
        coupling: CouplingKind::Strong,
        fallback: FallbackPolicy::HoldLastGood,
        members: &[AILERON_LANE as u8, ELEVATOR_LANE as u8, RUDDER_LANE as u8],
        safe_pattern: neutral_surfaces(),
    },
];

impl FixedWingAirframe {
    /// Construct the controller for this airframe.
    ///
    /// # Errors
    ///
    /// Returns [`FixedWingParamsError`] when the description cannot fly.
    pub fn controller(&self) -> Result<FixedWingController, FixedWingParamsError> {
        FixedWingController::new(self.params)
    }

    /// Construct the estimator for this airframe.
    ///
    /// It is the estimator of every other vehicle with air-data aiding
    /// switched on, so that attitude, height and airspeed stay usable
    /// without a position fix.
    #[must_use]
    pub fn estimator(&self) -> Ekf {
        Ekf::new(EkfConfig {
            air_aiding: Some(self.air_aiding),
            ..EkfConfig::default()
        })
    }

    /// Construct the whole kernel of this airframe: the estimator with
    /// air-data aiding, the law, the mixer and the fixed-wing pre-arm
    /// checks. A host that assembles the parts itself can miss one of
    /// them, and a fixed wing without its estimator block or pre-arm
    /// set cannot start without a position fix.
    ///
    /// # Errors
    ///
    /// Returns [`FixedWingKernelError`] when the description cannot fly
    /// or its parts do not agree.
    pub fn kernel(
        &self,
        timestamp_source: fn() -> Timestamp,
    ) -> Result<DefaultAviateKernel<FixedWingController, FixedWingMixer>, FixedWingKernelError>
    {
        AviateKernelBuilder::new()
            .estimator(self.estimator())
            .controller(self.controller().map_err(FixedWingKernelError::Params)?)
            .mixer(Self::mixer(timestamp_source))
            .sanitizer(Sanitizer)
            .pre_arm_required(Self::pre_arm_required())
            .config(self.kernel_config())
            .build()
            .map_err(FixedWingKernelError::Build)
    }

    /// Pre-arm checks for a fixed wing: no position fix is required.
    #[must_use]
    pub const fn pre_arm_required() -> PreArmFlags {
        PreArmFlags::FIXED_WING_MINIMUM
    }

    /// Construct the four-lane mixer.
    #[must_use]
    pub fn mixer(timestamp_source: fn() -> Timestamp) -> FixedWingMixer {
        FixedWingMixer { timestamp_source }
    }

    /// Resolved kernel configuration for this airframe.
    ///
    /// The safe output is idle throttle with neutral surfaces: the
    /// kernel emits it when it cannot fly the control law.
    #[must_use]
    pub fn kernel_config(&self) -> ResolvedKernelConfig {
        ResolvedKernelConfig {
            limits: self.limits.clone(),
            mode_config: ModeConfig {
                mode: ConfigMode::Cruise,
                groups: &GROUPS,
            },
            safe_output: neutral_outputs(),
            mixer_geometry: MixerGeometry::FixedWing,
            fixed_wing: Some(self.params),
            ..ResolvedKernelConfig::default()
        }
    }
}
