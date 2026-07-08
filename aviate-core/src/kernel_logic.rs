//! Kernel lifecycle and per-cycle methods on `AviateKernelImpl`.
//!
//! These are separated from `kernel.rs` so each file stays under the
//! 500-line per-.rs limit. The `update()` method lives in `kernel_update.rs`.

use crate::checks::{DegradationReason, PreArmFlags};
use crate::control::{ConfigMode, ControlLawV1, VehicleController};
use crate::ekf::Estimator;
use crate::fault::FaultFlags;
use crate::kernel::{AviateKernelImpl, InitState};
use crate::kernel_types::{
    ArmError, ChannelHealthV1, DegradationEvent, HealthReport, InitResult, TerminalCause,
    TransitionError, CRITICAL_FAULTS,
};
use crate::mixer::{ActuatorSanitizer, Mixer};
use crate::sensor::SensorSet;
use crate::time::Timestamp;

impl<E: Estimator, V: VehicleController, M: Mixer, S: ActuatorSanitizer>
    AviateKernelImpl<E, V, M, S>
{
    pub fn init_step(&mut self, sensors: &SensorSet, _time: Timestamp) -> InitResult {
        // 1. Update checks from sensor data (always, regardless of state)
        self.state.checks.pre_arm.update_from_sensors(sensors);
        self.state
            .checks
            .pre_arm
            .update_from_faults(self.state.faults);
        // EKF initialization gate: read it through the trait surface
        // (`Estimator::estimate(state).quality == Good` ⇔ initialized).
        // Routing through the trait keeps this code generic over
        // non-EKF estimators that don't have an `is_initialized` notion
        // but do produce a `StateEstimate.quality`.
        let est_initialized = matches!(
            self.pipeline
                .estimator
                .estimate(&self.state.estimator)
                .quality,
            crate::state::EstimateQuality::Good
        );
        self.state.checks.pre_arm.update_ekf(est_initialized);

        // Note: update_sensor_faults() is NOT called here.
        // Faults are runtime monitoring for armed operation.
        // Pre-arm checks are handled by the pre_arm flags (IMU_HEALTHY, etc.).

        // 3. State machine transitions
        match self.state.init_state {
            InitState::PowerOn => {
                self.state.init_state = InitState::ConfigLoading;
            }
            InitState::ConfigLoading => {
                // Config loaded (placeholder - would check actual config validity)
                self.state
                    .checks
                    .pre_arm
                    .current
                    .insert(PreArmFlags::CONFIG_VALID);
                self.state.init_state = InitState::SensorInit;
            }
            InitState::SensorInit => {
                // Wait for at least one valid sensor reading
                let has_sensors = self
                    .state
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_HEALTHY);
                if has_sensors {
                    self.state.init_state = InitState::EstimatorConverging;
                }
            }
            InitState::EstimatorConverging => {
                // Wait for sensor convergence and EKF initialization
                let converged = self
                    .state
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_CONVERGED)
                    && self
                        .state
                        .checks
                        .pre_arm
                        .current
                        .contains(PreArmFlags::EKF_CONVERGED);
                if converged {
                    self.state.init_state = InitState::PreArm;
                }
            }
            InitState::PreArm => {
                // Check all pre-arm requirements
                if self.state.checks.pre_arm.is_satisfied() {
                    self.state.init_state = InitState::Ready;
                }
            }
            InitState::Ready => {
                // Monitor for fault conditions
                if !self.state.checks.pre_arm.is_satisfied() {
                    self.state.init_state = InitState::PreArm;
                }
            }
            InitState::Armed => {} // COV:EXCL(EMPTY: monitoring only, disarm via disarm())
            InitState::Disarmed => {
                // Transition back to PreArm for potential re-arm
                // Reset sample counts for fresh convergence check
                self.state.checks.pre_arm.samples.reset();
                self.state.init_state = InitState::PreArm;
            }
            InitState::Fault => {
                // Require explicit reset to exit fault state
            }
        }

        InitResult {
            state: self.state.init_state,
            faults: self.state.faults,
            ready: self.state.init_state == InitState::Ready,
        }
    }

    /// Update fault flags based on sensor health
    pub(crate) fn update_sensor_faults(&mut self, sensors: &SensorSet) {
        use crate::sensor::SensorHealth;

        // IMU faults
        let imu_ok = sensors
            .imus
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !imu_ok {
            self.state.faults.insert(FaultFlags::ALL_IMU_FAILED);
        } else {
            self.state.faults.remove(FaultFlags::ALL_IMU_FAILED);
        }

        // Baro faults
        let baro_ok = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !baro_ok {
            self.state.faults.insert(FaultFlags::BARO_FAILED);
        } else {
            self.state.faults.remove(FaultFlags::BARO_FAILED);
        }

        // Mag faults
        let mag_ok = sensors
            .mags
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !mag_ok {
            self.state.faults.insert(FaultFlags::MAG_FAILED);
        } else {
            self.state.faults.remove(FaultFlags::MAG_FAILED);
        }

        // GNSS faults
        let gnss_ok = sensors
            .gnss
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !gnss_ok {
            self.state.faults.insert(FaultFlags::ALL_GNSS_LOST);
        } else {
            self.state.faults.remove(FaultFlags::ALL_GNSS_LOST);
        }
    }

    pub fn is_ready(&self) -> bool {
        self.state.init_state == InitState::Ready
    }

    pub fn arm(&mut self) -> Result<(), ArmError> {
        if self.state.init_state == InitState::Armed {
            return Err(ArmError::AlreadyArmed);
        }
        if self.state.init_state != InitState::Ready {
            return Err(ArmError::NotReady);
        }
        if !self.state.faults.is_empty() {
            return Err(ArmError::Faulted);
        }

        self.state.init_state = InitState::Armed;
        Ok(())
    }

    pub fn disarm(&mut self) {
        self.state.init_state = InitState::Disarmed;
        self.state.control_law = ControlLawV1::Backup; // Was Frozen, now Backup
        self.state.terminal_cause = TerminalCause::None;
        self.state.checks.in_flight.reset();
        // Reset controller persistent runtime state — disarm
        // invalidates accumulated integrators / anti-windup / mode
        // latches the same way ground_reset does (LLR-CTL-101).
        self.pipeline.controller.reset(&mut self.state.controller);
    }

    /// Check if the system can be reset from fault state
    ///
    /// Preconditions:
    /// - No critical faults active
    /// - IMU_HEALTHY (sensors recovered)
    /// - THROTTLE_LOW (safety)
    pub fn can_reset_from_fault(&self) -> bool {
        if self.state.init_state != InitState::Fault {
            return false;
        }

        // No critical faults remaining
        let no_critical = !self.state.faults.intersects(CRITICAL_FAULTS);

        // Sensors recovered
        let imu_healthy = self
            .state
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::IMU_HEALTHY);

        // Throttle low for safety
        let throttle_low = self
            .state
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::THROTTLE_LOW);

        no_critical && imu_healthy && throttle_low
    }

    /// Attempt to reset from fault state
    ///
    /// Returns Ok(()) if successfully reset to PreArm state.
    pub fn reset_from_fault(&mut self) -> Result<(), ArmError> {
        if self.state.init_state != InitState::Fault {
            return Err(ArmError::NotReady);
        }

        if !self.can_reset_from_fault() {
            return Err(ArmError::Faulted);
        }

        // Reset checks for fresh convergence
        self.state.checks.pre_arm.samples.reset();
        self.state.checks.in_flight.reset();

        // Transition to PreArm
        self.state.init_state = InitState::PreArm;
        Ok(())
    }

    /// Handle degradation based on in-flight check trigger
    ///
    /// Updates control law based on the degradation reason.
    /// Public for DO-178C MC/DC testing of all degradation paths.
    pub fn handle_degradation(
        &mut self,
        reason: DegradationReason,
        timestamp: Timestamp,
    ) -> Option<DegradationEvent> {
        let from = self.state.control_law;
        let to = match reason {
            // Total loss of attitude: no stable frame to descend in, so
            // motors go to the safe pattern (last-ditch, LLR-FLT-202).
            DegradationReason::AttitudeLost => ControlLawV1::Backup,
            // Terminal failsafes that keep a stabilizable frame ride the
            // vehicle down under the Descend/Land terminal (`Direct`)
            // rather than cutting thrust. Command/datalink loss is a
            // terminal failsafe by the same logic PX4 resolves it to
            // Land: no uplink is coming, so land now under control.
            DegradationReason::LandRequested => ControlLawV1::Direct,
            DegradationReason::CommandTimeout => ControlLawV1::Direct,
            DegradationReason::ImuDegraded => ControlLawV1::Alternate,
            DegradationReason::PositionLost => ControlLawV1::Alternate,
            DegradationReason::VelocityLost => ControlLawV1::Alternate,
            DegradationReason::EnvelopeViolation => ControlLawV1::Alternate,
            DegradationReason::BaroDegraded => ControlLawV1::Alternate,
            DegradationReason::RcLost => ControlLawV1::Alternate,
            DegradationReason::TimingViolation => ControlLawV1::Alternate,
        };

        // Only trigger if this is a degradation (worse state)
        if to.severity() > from.severity() {
            self.state.control_law = to;
            // Record why a Direct terminal engaged: command-loss
            // descents release when recency returns (LLR-FLT-209);
            // a commanded land is latched for the flight.
            self.state.terminal_cause = match (to, reason) {
                (ControlLawV1::Direct, DegradationReason::CommandTimeout) => {
                    TerminalCause::CommandLoss
                }
                (ControlLawV1::Direct, _) => TerminalCause::Commanded,
                _ => TerminalCause::None,
            };
            // Reset controller persistent runtime state when entering a
            // terminal law (Backup, or the Descend/Land terminal
            // `Direct`). The terminal law's authority envelope and
            // control objective differ from the prior law, so
            // accumulated integrators / anti-windup / mode latches
            // computed against the prior authority must not leak
            // (LLR-CTL-101).
            if to == ControlLawV1::Backup || to == ControlLawV1::Direct {
                self.pipeline.controller.reset(&mut self.state.controller);
            }
            Some(DegradationEvent {
                from,
                to,
                reason,
                timestamp,
            })
        } else {
            None
        }
    }

    /// Request a configuration mode transition
    ///
    /// Checks transition preconditions before starting the transition.
    pub fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError> {
        // Must be armed
        if self.state.init_state != InitState::Armed {
            return Err(TransitionError::NotArmed);
        }

        // Cannot be in fault state
        if self.state.faults.intersects(CRITICAL_FAULTS) {
            return Err(TransitionError::InFaultState);
        }

        // Check if already transitioning (Transition mode is the transition state)
        if self.state.mode == ConfigMode::Transition {
            return Err(TransitionError::AlreadyTransitioning);
        }

        // Check if already in requested mode
        if self.state.mode == to {
            return Err(TransitionError::AlreadyInMode);
        }

        // Update transition checks and verify
        let state = self.pipeline.estimator.estimate(&self.state.estimator);
        self.state.checks.transition.update_from_state(&state);
        self.state
            .checks
            .transition
            .update_from_actuators(&self.state.actuator_state, 0b1111); // Quad mask

        // Gate the transition
        self.state
            .checks
            .transition
            .can_transition()
            .map_err(TransitionError::ChecksFailed)?;

        // Start the transition (caller manages progress)
        // For now, just update the mode directly
        self.state.mode = to;
        Ok(())
    }

    pub fn get_health(&self) -> HealthReport {
        use crate::kernel_types::ConfigTransitionState;

        HealthReport {
            init_state: self.state.init_state,
            control_law: self.state.control_law,
            config_mode: self.state.mode,
            transition_state: ConfigTransitionState::Stable(self.state.mode),
            faults: self.state.faults,
            channel_health: ChannelHealthV1::Operative,
        }
    }

    /// Perform a ground reset, clearing transient states.
    ///
    /// Clears every field of `KernelState` that holds runtime/safety
    /// state — except cfg-derived defaults that should remain stable
    /// (mode_config etc. live in `cfg`, not `state`). This is the
    /// kernel's "back to factory un-init posture" entry point;
    /// post-conditions:
    ///
    /// - `init_state == InitState::ConfigLoading` (re-enter init)
    /// - `faults` empty
    /// - `control_law == Primary`
    /// - `mode == ConfigMode::Hover`
    /// - `checks` (pre_arm/in_flight/transition) all reset
    /// - `estimator` un-initialized (covariance back to factory `I*0.1`,
    ///   biases zero, quat IDENTITY, init/fault latches cleared)
    /// - `fallback` cleared (no inherited per-group last-good /
    ///   age / consecutive-fallback counters from before reset)
    /// - `actuator_state` cleared (no inherited commanded-actuator
    ///   snapshot)
    /// - `timing_stats` cleared (fresh per-cycle accumulators)
    ///
    /// Pre-Phase-4-followup, the fallback and actuator_state fields
    /// were silently inherited across resets — a hot-spare takeover
    /// or post-fault re-init would leak the sanitizer's last-good
    /// memory and the previous commanded-actuator snapshot,
    /// violating the spec §17 "init-from-clean-state" contract.
    /// LLR-STATE-107 documents the closure rule.
    #[inline(never)]
    pub fn ground_reset(&mut self) {
        // Only allowed if not armed
        if self.state.init_state == InitState::Armed {
            return;
        }

        self.state.faults = FaultFlags::empty();
        self.state.terminal_cause = TerminalCause::None;
        self.state.checks.pre_arm.reset();
        self.state.checks.in_flight.reset();
        self.state.checks.transition.reset();
        self.state.init_state = InitState::ConfigLoading; // Restart init sequence
                                                          // Reset estimator runtime state via the trait — works for
                                                          // any `E: Estimator`, not just EKF.
        self.pipeline.estimator.reset(&mut self.state.estimator);
        self.state.fallback = crate::mixer::ActuatorFallbackState::default();
        self.state.actuator_state = crate::mixer::ActuatorState::default();
        self.state.timing_stats = crate::kernel_types::TimingStats::default();
        self.state.mode = crate::control::ConfigMode::Hover;

        // Reset controller persistent runtime state (integrators,
        // anti-windup, filter memories, mode latches). Routes through
        // the controller so an impl that needs more than `runtime.reset()`
        // can override the trait's default `fn reset`.
        self.pipeline.controller.reset(&mut self.state.controller);

        self.state.control_law = ControlLawV1::Primary; // Reset law
    }

    pub fn kick_watchdog(&mut self) {
        use crate::kernel::Watchdog;
        self.kick();
    }

    /// Report a timing violation from external monitoring (spec §18)
    ///
    /// The caller should call this when the actual execution time of update()
    /// exceeds CONTROL_LOOP_DEADLINE_US (800us). After TIMING_VIOLATION_THRESHOLD
    /// consecutive violations, degradation to Alternate law is triggered.
    ///
    /// Call with `violation = true` when deadline exceeded, `false` when met.
    pub fn report_timing_violation(&mut self, violation: bool) {
        if violation {
            self.state.timing_stats.deadline_violations = self
                .state
                .timing_stats
                .deadline_violations
                .saturating_add(1);
            self.state.timing_stats.consecutive_violations = self
                .state
                .timing_stats
                .consecutive_violations
                .saturating_add(1);
        } else {
            self.state.timing_stats.consecutive_violations = 0;
        }
    }

    /// Check for critical faults and enter fault state if detected
    ///
    /// Returns true if fault state was entered.
    pub fn check_critical_faults(&mut self) -> bool {
        if self.state.faults.intersects(CRITICAL_FAULTS) {
            self.state.init_state = InitState::Fault;
            self.state.control_law = ControlLawV1::Backup; // Was Frozen
            self.state.terminal_cause = TerminalCause::None;
            // Reset controller persistent runtime state — entering
            // Backup on a critical fault means the prior law's
            // accumulated integrators / mode latches were computed
            // against state we now consider invalid (LLR-CTL-101).
            self.pipeline.controller.reset(&mut self.state.controller);
            true
        } else {
            false
        }
    }
}
