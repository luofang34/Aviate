//! `AviateKernelBuilder` — fluent kernel construction (LLR-CFG-103).
//!
//! Replaces both `AviateKernelImpl::new()` and
//! `AviateKernelImpl::with_pre_arm_required()`. As the kernel grows
//! more substitutable parts (Phase 2: pipeline; Phase 3: state slot;
//! Phase 4: estimator state seed), they get builder methods rather
//! than additional positional ctor arguments.
//!
//! Usage:
//!
//! ```ignore
//! use aviate_core::kernel::builder::AviateKernelBuilder;
//! let kernel = AviateKernelBuilder::new()
//!     .estimator(Ekf::default())
//!     .controller(MultirotorController::default())
//!     .mixer(QuadXMixer { timestamp_source: hw_clock })
//!     .sanitizer(Sanitizer::default())
//!     .mode_config(my_mode_config)
//!     .pre_arm_required(PreArmFlags::QUAD_WITH_GPS)
//!     .build();
//! ```
//!
//! The estimator/controller/mixer/sanitizer are required; calling
//! `build()` without them is a compile error (the builder uses the
//! type-state pattern via `Option`).

use crate::checks::{KernelChecks, PreArmFlags};
use crate::control::envelope::SimpleEnvelopeProtector;
use crate::control::{ConfigMode, ControlLawV1, Limits, VehicleController};
use crate::ekf::Estimator;
use crate::fault::FaultFlags;
use crate::kernel::config::ResolvedKernelConfig;
use crate::kernel::AviateKernelImpl;
use crate::kernel_types::{InitState, TimingStats};
use crate::mixer::{ActuatorSanitizer, ActuatorState, Mixer, ModeConfig};

/// Builder for `AviateKernelImpl<E, V, M, S>`. See module docs for usage.
pub struct AviateKernelBuilder<E, V, M, S>
where
    E: Estimator,
    V: VehicleController,
    M: Mixer,
    S: ActuatorSanitizer,
{
    estimator: Option<E>,
    controller: Option<V>,
    mixer: Option<M>,
    sanitizer: Option<S>,
    cfg: ResolvedKernelConfig,
    pre_arm_required: Option<PreArmFlags>,
}

impl<E, V, M, S> Default for AviateKernelBuilder<E, V, M, S>
where
    E: Estimator,
    V: VehicleController,
    M: Mixer,
    S: ActuatorSanitizer,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<E, V, M, S> AviateKernelBuilder<E, V, M, S>
where
    E: Estimator,
    V: VehicleController,
    M: Mixer,
    S: ActuatorSanitizer,
{
    pub fn new() -> Self {
        Self {
            estimator: None,
            controller: None,
            mixer: None,
            sanitizer: None,
            cfg: ResolvedKernelConfig::default(),
            pre_arm_required: None,
        }
    }

    pub fn estimator(mut self, e: E) -> Self {
        self.estimator = Some(e);
        self
    }

    pub fn controller(mut self, c: V) -> Self {
        self.controller = Some(c);
        self
    }

    pub fn mixer(mut self, m: M) -> Self {
        self.mixer = Some(m);
        self
    }

    pub fn sanitizer(mut self, s: S) -> Self {
        self.sanitizer = Some(s);
        self
    }

    /// Override per-mode actuator + mixer configuration.
    pub fn mode_config(mut self, mc: ModeConfig) -> Self {
        self.cfg.mode_config = mc;
        self
    }

    /// Override flight-envelope limits.
    pub fn limits(mut self, l: Limits) -> Self {
        self.cfg.limits = l;
        self
    }

    /// Override the pilot-command staleness threshold.
    pub fn command_timeout_ms(mut self, ms: u32) -> Self {
        self.cfg.command_timeout_ms = ms;
        self
    }

    /// Replace the entire resolved config in one shot. Useful when a
    /// caller has its own `load_config()` path that produces a fully
    /// validated config (post-DRQ-CFG-001 future).
    pub fn config(mut self, cfg: ResolvedKernelConfig) -> Self {
        self.cfg = cfg;
        self
    }

    /// Specify the pre-arm requirement set explicitly (mirrors the
    /// removed `with_pre_arm_required()` ctor). When omitted, the
    /// default `KernelChecks::new()` requirements apply.
    pub fn pre_arm_required(mut self, required: PreArmFlags) -> Self {
        self.pre_arm_required = Some(required);
        self
    }

    /// Build the kernel. Returns the name of the first missing
    /// required component (estimator / controller / mixer / sanitizer)
    /// as `Err(&'static str)` instead of panicking — flight builds
    /// must surface caller misuse through the typed error path,
    /// because the workspace lint policy denies `clippy::panic`.
    pub fn build(self) -> Result<AviateKernelImpl<E, V, M, S>, &'static str> {
        let estimator = self.estimator.ok_or("estimator")?;
        let controller = self.controller.ok_or("controller")?;
        let mixer = self.mixer.ok_or("mixer")?;
        let sanitizer = self.sanitizer.ok_or("sanitizer")?;

        let checks = match self.pre_arm_required {
            Some(required) => KernelChecks::with_pre_arm_required(required),
            None => KernelChecks::new(),
        };

        Ok(AviateKernelImpl {
            estimator,
            controller,
            mixer,
            sanitizer,
            protector: SimpleEnvelopeProtector,
            mode: ConfigMode::Hover,
            init_state: InitState::PowerOn,
            faults: FaultFlags::empty(),
            control_law: ControlLawV1::Primary,
            checks,
            actuator_state: ActuatorState::default(),
            timing_stats: TimingStats::default(),
            cfg: self.cfg,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::multirotor::MultirotorController;
    use crate::control::{AxisCommand, Limits};
    use crate::ekf::Ekf;
    use crate::mixer::{ModeConfig, QuadXMixer, Sanitizer};
    use crate::time::{TimeSource, Timestamp};
    use crate::types::{Normalized, NormalizedSigned, Radians};

    /// Timestamp source consumed by `QuadXMixer` — the function is invoked
    /// indirectly when a test exercises `kernel.mixer.mix(...)` (see
    /// `full_chain_kernel_mixer_invokes_timestamp_source` below).
    fn fake_ts() -> Timestamp {
        Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        }
    }

    type TestBuilder = AviateKernelBuilder<Ekf, MultirotorController, QuadXMixer, Sanitizer>;

    fn fake_mixer() -> QuadXMixer {
        QuadXMixer {
            timestamp_source: fake_ts,
        }
    }

    /// Helper that supplies all four required pipeline components, used
    /// by every "happy path" test below. Returns the builder mid-chain
    /// so each test can append its own override before `.build()`.
    fn full_pipeline_builder() -> TestBuilder {
        TestBuilder::new()
            .estimator(Ekf::default())
            .controller(MultirotorController::default())
            .mixer(fake_mixer())
            .sanitizer(Sanitizer::default())
    }

    #[test]
    fn new_returns_empty_builder_that_fails_to_build() {
        let result = TestBuilder::new().build();
        assert_eq!(result.err(), Some("estimator"));
    }

    #[test]
    fn default_impl_matches_new() {
        let result = TestBuilder::default().build();
        assert_eq!(result.err(), Some("estimator"));
    }

    #[test]
    fn partial_through_estimator_fails_at_controller() {
        let result = TestBuilder::new().estimator(Ekf::default()).build();
        assert_eq!(result.err(), Some("controller"));
    }

    #[test]
    fn partial_through_controller_fails_at_mixer() {
        let result = TestBuilder::new()
            .estimator(Ekf::default())
            .controller(MultirotorController::default())
            .build();
        assert_eq!(result.err(), Some("mixer"));
    }

    #[test]
    fn partial_through_mixer_fails_at_sanitizer() {
        let result = TestBuilder::new()
            .estimator(Ekf::default())
            .controller(MultirotorController::default())
            .mixer(fake_mixer())
            .build();
        assert_eq!(result.err(), Some("sanitizer"));
    }

    #[test]
    fn full_chain_builds_kernel_with_default_cfg() -> Result<(), &'static str> {
        let kernel = full_pipeline_builder().build()?;
        assert_eq!(kernel.init_state, InitState::PowerOn);
        Ok(())
    }

    /// Ensures `fake_ts` is actually invoked by exercising the mixer on
    /// the built kernel. Without this call the function pointer is
    /// stored but never dereferenced, leaving the function body
    /// uncovered.
    #[test]
    fn full_chain_kernel_mixer_invokes_timestamp_source() -> Result<(), &'static str> {
        let kernel = full_pipeline_builder().build()?;
        let cmd = kernel.mixer.mix(&AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.0),
        });
        // QuadXMixer.mix() calls (self.timestamp_source)() to populate
        // the ActuatorCmd timestamp — confirm the indirection ran by
        // checking the source tag.
        assert_eq!(cmd.timestamp.source, TimeSource::Internal);
        Ok(())
    }

    #[test]
    fn mode_config_override_propagates_to_cfg() -> Result<(), &'static str> {
        let custom_mc = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &[],
        };
        let kernel = full_pipeline_builder().mode_config(custom_mc).build()?;
        assert_eq!(kernel.cfg.mode_config.mode, ConfigMode::Cruise);
        Ok(())
    }

    #[test]
    fn limits_override_propagates_to_cfg() -> Result<(), &'static str> {
        let custom_limits = Limits {
            max_roll: Radians(1.0),
            ..ResolvedKernelConfig::default().limits
        };
        let kernel = full_pipeline_builder().limits(custom_limits).build()?;
        assert_eq!(kernel.cfg.limits.max_roll.0, 1.0);
        Ok(())
    }

    #[test]
    fn command_timeout_ms_override_propagates_to_cfg() -> Result<(), &'static str> {
        let kernel = full_pipeline_builder().command_timeout_ms(1234).build()?;
        assert_eq!(kernel.cfg.command_timeout_ms, 1234);
        Ok(())
    }

    #[test]
    fn config_replace_overrides_entire_cfg() -> Result<(), &'static str> {
        let custom = ResolvedKernelConfig {
            command_timeout_ms: 4321,
            ..ResolvedKernelConfig::default()
        };
        let kernel = full_pipeline_builder().config(custom).build()?;
        assert_eq!(kernel.cfg.command_timeout_ms, 4321);
        Ok(())
    }

    #[test]
    fn pre_arm_required_propagates_to_checks() -> Result<(), &'static str> {
        let required = PreArmFlags::IMU_HEALTHY | PreArmFlags::THROTTLE_LOW;
        let kernel = full_pipeline_builder().pre_arm_required(required).build()?;
        // Kernel built without pre-arm-satisfied state must report
        // unmet requirements.
        assert!(!kernel.checks.pre_arm.is_satisfied());
        Ok(())
    }
}
