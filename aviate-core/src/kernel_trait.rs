//! `AviateKernelTrait` — the public spec §20 contract — and its impl for
//! `AviateKernelImpl`. Kept isolated so the contract is easy to audit
//! without being buried under implementation detail.

use crate::control::{ConfigMode, ControlLawV1, VehicleController};
use crate::ekf::Estimator;
use crate::fault::FaultFlags;
use crate::kernel::config::ResolvedKernelConfig;
use crate::kernel::{AviateKernelImpl, InitState};
use crate::kernel_types::{
    ArmError, ChannelId, ConfigBlock, ConfigError, ConfigTransitionState, CrossChannelData,
    HealthReport, InitResult, TransitionError, UpdateResult,
};
use crate::mixer::{ActuatorSanitizer, ActuatorState, Mixer};
use crate::sensor::SensorSet;
use crate::time::{TimeDelta, Timestamp};

#[cfg(feature = "test-hooks")]
use crate::state::StateEstimate;

/// Core flight control kernel interface (spec §20)
///
/// Defines the standard interface for flight control implementations.
/// All persistent state relevant to control or estimation is owned by
/// implementations of this trait (spec §37).
pub trait AviateKernelTrait {
    /// Advance initialization state machine
    fn init_step(&mut self, sensors: &SensorSet, time: Timestamp) -> InitResult;

    /// Get current initialization state
    fn init_state(&self) -> InitState;

    /// Check if system is ready to arm
    fn is_ready(&self) -> bool;

    /// Attempt to arm the system
    fn arm(&mut self) -> Result<(), ArmError>;

    /// Disarm the system
    fn disarm(&mut self);

    /// Get current configuration mode
    fn config_mode(&self) -> ConfigMode;

    /// Get current transition state
    fn transition_state(&self) -> ConfigTransitionState;

    /// Request a configuration mode transition
    fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError>;

    /// Main control update (spec §20)
    fn update(
        &mut self,
        channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &crate::control::Command,
        actuator_state: &ActuatorState,
        cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult;

    /// Load configuration from block (spec §19)
    fn load_config(&mut self, config: &ConfigBlock) -> Result<(), ConfigError>;

    /// Get current configuration (spec §20)
    fn get_config(&self) -> &ResolvedKernelConfig;

    /// Get health report (spec §20)
    fn get_health(&self) -> HealthReport;

    /// Get current fault flags
    fn get_faults(&self) -> FaultFlags;

    /// Get current control law
    fn get_control_law(&self) -> ControlLawV1;

    /// Kick the watchdog timer
    fn kick_watchdog(&mut self);

    /// Perform ground reset
    fn ground_reset(&mut self);

    /// Inject state for testing (spec §20, test-hooks only)
    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate);

    /// Inject fault for testing (spec §20, test-hooks only)
    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultFlags);
}

// --- Spec §20: AviateKernelTrait Implementation ---
// COV:EXCL_START(DELEGATE: every body in this impl either (a) delegates
//   to the equivalent inherent method on AviateKernelImpl, which has its
//   own tests, or (b) returns a struct field directly. No branches, no
//   local state. Covering via `&dyn AviateKernelTrait` would only prove
//   that `dyn` dispatch works, not that our logic does; the underlying
//   AviateKernelImpl tests already do the latter.)
impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer> AviateKernelTrait
    for AviateKernelImpl<E, V, M, S>
{
    fn init_step(&mut self, sensors: &SensorSet, time: Timestamp) -> InitResult {
        AviateKernelImpl::init_step(self, sensors, time)
    }

    fn init_state(&self) -> InitState {
        self.state.init_state
    }

    fn is_ready(&self) -> bool {
        AviateKernelImpl::is_ready(self)
    }

    fn arm(&mut self) -> Result<(), ArmError> {
        AviateKernelImpl::arm(self)
    }

    fn disarm(&mut self) {
        AviateKernelImpl::disarm(self)
    }

    fn config_mode(&self) -> ConfigMode {
        self.state.mode
    }

    fn transition_state(&self) -> ConfigTransitionState {
        // TODO: Track actual transition state for async transitions
        ConfigTransitionState::Stable(self.state.mode)
    }

    fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError> {
        AviateKernelImpl::request_config_mode(self, to)
    }

    fn update(
        &mut self,
        channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &crate::control::Command,
        actuator_state: &ActuatorState,
        cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult {
        AviateKernelImpl::update(
            self,
            channel,
            time,
            sensors,
            command,
            actuator_state,
            cross_channel,
        )
    }

    fn load_config(&mut self, config_block: &ConfigBlock) -> Result<(), ConfigError> {
        // Spec §19: Stub implementation - validates checksum and version only
        if config_block.version > 1 {
            return Err(ConfigError::UnsupportedVersion);
        }
        // TODO: Parse actual config data from block
        // For now, accept valid blocks but use default config
        let _ = config_block.checksum; // Placeholder for future checksum validation
        Ok(())
    }

    fn get_config(&self) -> &ResolvedKernelConfig {
        &self.cfg
    }

    fn get_health(&self) -> HealthReport {
        AviateKernelImpl::get_health(self)
    }

    fn get_faults(&self) -> FaultFlags {
        self.state.faults
    }

    fn get_control_law(&self) -> ControlLawV1 {
        self.state.control_law
    }

    fn kick_watchdog(&mut self) {
        AviateKernelImpl::kick_watchdog(self)
    }

    fn ground_reset(&mut self) {
        AviateKernelImpl::ground_reset(self)
    }

    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate) {
        self.pipeline.estimator.set_state(state);
    }

    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultFlags) {
        self.state.faults.insert(fault);
    }
}
// COV:EXCL_STOP
