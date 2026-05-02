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
