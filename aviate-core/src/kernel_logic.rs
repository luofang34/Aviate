//! Kernel lifecycle and per-cycle methods on `AviateKernelImpl`.
//!
//! These are separated from `kernel.rs` so each file stays under the
//! 500-line per-.rs limit. The `update()` method lives in `kernel_update.rs`.

use crate::checks::{DegradationReason, PreArmFlags};
use crate::control::{ConfigMode, ControlLawV1, VehicleController};
use crate::ekf::Ekf;
use crate::fault::FaultFlags;
use crate::kernel::{AviateKernelImpl, InitState};
use crate::kernel_types::{
    ArmError, ChannelHealthV1, DegradationEvent, HealthReport, InitResult, TransitionError,
    CRITICAL_FAULTS,
};
use crate::mixer::Mixer;
use crate::sensor::SensorSet;
use crate::time::Timestamp;

impl<V: VehicleController, M: Mixer> AviateKernelImpl<V, M> {
    pub fn init_step(&mut self, sensors: &SensorSet, _time: Timestamp) -> InitResult {
        // 1. Update checks from sensor data (always, regardless of state)
        self.checks.pre_arm.update_from_sensors(sensors);
        self.checks.pre_arm.update_from_faults(self.faults);
        self.checks.pre_arm.update_ekf(self.ekf.is_initialized());

        // Note: update_sensor_faults() is NOT called here.
        // Faults are runtime monitoring for armed operation.
        // Pre-arm checks are handled by the pre_arm flags (IMU_HEALTHY, etc.).

        // 3. State machine transitions
        match self.init_state {
            InitState::PowerOn => {
                self.init_state = InitState::ConfigLoading;
            }
            InitState::ConfigLoading => {
                // Config loaded (placeholder - would check actual config validity)
                self.checks
                    .pre_arm
                    .current
                    .insert(PreArmFlags::CONFIG_VALID);
                self.init_state = InitState::SensorInit;
            }
            InitState::SensorInit => {
                // Wait for at least one valid sensor reading
                let has_sensors = self
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_HEALTHY);
                if has_sensors {
                    self.init_state = InitState::EstimatorConverging;
                }
            }
            InitState::EstimatorConverging => {
                // Wait for sensor convergence and EKF initialization
                let converged = self
                    .checks
                    .pre_arm
                    .current
                    .contains(PreArmFlags::IMU_CONVERGED)
                    && self
                        .checks
                        .pre_arm
                        .current
                        .contains(PreArmFlags::EKF_CONVERGED);
                if converged {
                    self.init_state = InitState::PreArm;
                }
            }
            InitState::PreArm => {
                // Check all pre-arm requirements
                if self.checks.pre_arm.is_satisfied() {
                    self.init_state = InitState::Ready;
                }
            }
            InitState::Ready => {
                // Monitor for fault conditions
                if !self.checks.pre_arm.is_satisfied() {
                    self.init_state = InitState::PreArm;
                }
            }
            InitState::Armed => {} // COV:EXCL(EMPTY: monitoring only, disarm via disarm())
            InitState::Disarmed => {
                // Transition back to PreArm for potential re-arm
                // Reset sample counts for fresh convergence check
                self.checks.pre_arm.samples.reset();
                self.init_state = InitState::PreArm;
            }
            InitState::Fault => {
                // Require explicit reset to exit fault state
            }
        }

        InitResult {
            state: self.init_state,
            faults: self.faults,
            ready: self.init_state == InitState::Ready,
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
            self.faults.insert(FaultFlags::ALL_IMU_FAILED);
        } else {
            self.faults.remove(FaultFlags::ALL_IMU_FAILED);
        }

        // Baro faults
        let baro_ok = sensors
            .baros
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !baro_ok {
            self.faults.insert(FaultFlags::BARO_FAILED);
        } else {
            self.faults.remove(FaultFlags::BARO_FAILED);
        }

        // Mag faults
        let mag_ok = sensors
            .mags
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !mag_ok {
            self.faults.insert(FaultFlags::MAG_FAILED);
        } else {
            self.faults.remove(FaultFlags::MAG_FAILED);
        }

        // GNSS faults
        let gnss_ok = sensors
            .gnss
            .iter()
            .any(|s| s.valid && s.health == SensorHealth::Good);
        if !gnss_ok {
            self.faults.insert(FaultFlags::ALL_GNSS_LOST);
        } else {
            self.faults.remove(FaultFlags::ALL_GNSS_LOST);
        }
    }

    pub fn is_ready(&self) -> bool {
        self.init_state == InitState::Ready
    }

    pub fn arm(&mut self) -> Result<(), ArmError> {
        if self.init_state == InitState::Armed {
            return Err(ArmError::AlreadyArmed);
        }
        if self.init_state != InitState::Ready {
            return Err(ArmError::NotReady);
        }
        if !self.faults.is_empty() {
            return Err(ArmError::Faulted);
        }

        self.init_state = InitState::Armed;
        Ok(())
    }

    pub fn disarm(&mut self) {
        self.init_state = InitState::Disarmed;
        self.control_law = ControlLawV1::Backup; // Was Frozen, now Backup
        self.checks.in_flight.reset();
    }

    /// Check if the system can be reset from fault state
    ///
    /// Preconditions:
    /// - No critical faults active
    /// - IMU_HEALTHY (sensors recovered)
    /// - THROTTLE_LOW (safety)
    pub fn can_reset_from_fault(&self) -> bool {
        if self.init_state != InitState::Fault {
            return false;
        }

        // No critical faults remaining
        let no_critical = !self.faults.intersects(CRITICAL_FAULTS);

        // Sensors recovered
        let imu_healthy = self
            .checks
            .pre_arm
            .current
            .contains(PreArmFlags::IMU_HEALTHY);

        // Throttle low for safety
        let throttle_low = self
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
        if self.init_state != InitState::Fault {
            return Err(ArmError::NotReady);
        }

        if !self.can_reset_from_fault() {
            return Err(ArmError::Faulted);
        }

        // Reset checks for fresh convergence
        self.checks.pre_arm.samples.reset();
        self.checks.in_flight.reset();

        // Transition to PreArm
        self.init_state = InitState::PreArm;
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
        let from = self.control_law;
        let to = match reason {
            DegradationReason::AttitudeLost => ControlLawV1::Backup,
            DegradationReason::ImuDegraded => ControlLawV1::Alternate,
            DegradationReason::PositionLost => ControlLawV1::Alternate,
            DegradationReason::VelocityLost => ControlLawV1::Alternate,
            DegradationReason::CommandTimeout => ControlLawV1::Alternate,
            DegradationReason::EnvelopeViolation => ControlLawV1::Alternate,
            DegradationReason::BaroDegraded => ControlLawV1::Alternate,
            DegradationReason::RcLost => ControlLawV1::Alternate,
            DegradationReason::TimingViolation => ControlLawV1::Alternate,
        };

        // Only trigger if this is a degradation (worse state)
        if to.severity() > from.severity() {
            self.control_law = to;
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
        if self.init_state != InitState::Armed {
            return Err(TransitionError::NotArmed);
        }

        // Cannot be in fault state
        if self.faults.intersects(CRITICAL_FAULTS) {
            return Err(TransitionError::InFaultState);
        }

        // Check if already transitioning (Transition mode is the transition state)
        if self.mode == ConfigMode::Transition {
            return Err(TransitionError::AlreadyTransitioning);
        }

        // Check if already in requested mode
        if self.mode == to {
            return Err(TransitionError::AlreadyInMode);
        }

        // Update transition checks and verify
        let state = self.ekf.get_estimate();
        self.checks.transition.update_from_state(&state);
        self.checks
            .transition
            .update_from_actuators(&self.actuator_state, 0b1111); // Quad mask

        // Gate the transition
        self.checks
            .transition
            .can_transition()
            .map_err(TransitionError::ChecksFailed)?;

        // Start the transition (caller manages progress)
        // For now, just update the mode directly
        self.mode = to;
        Ok(())
    }

    pub fn get_health(&self) -> HealthReport {
        use crate::kernel_types::ConfigTransitionState;

        HealthReport {
            init_state: self.init_state,
            control_law: self.control_law,
            config_mode: self.mode,
            transition_state: ConfigTransitionState::Stable(self.mode),
            faults: self.faults,
            channel_health: ChannelHealthV1::Operative,
        }
    }

    /// Perform a ground reset, clearing transient states
    #[inline(never)]
    pub fn ground_reset(&mut self) {
        // Only allowed if not armed
        if self.init_state == InitState::Armed {
            return;
        }

        self.faults = FaultFlags::empty();
        self.checks.pre_arm.reset();
        self.checks.in_flight.reset();
        self.checks.transition.reset();
        self.init_state = InitState::ConfigLoading; // Restart init sequence
        self.ekf = Ekf::default(); // Reset estimator
        self.control_law = ControlLawV1::Primary; // Reset law
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
            self.timing_stats.deadline_violations =
                self.timing_stats.deadline_violations.saturating_add(1);
            self.timing_stats.consecutive_violations =
                self.timing_stats.consecutive_violations.saturating_add(1);
        } else {
            self.timing_stats.consecutive_violations = 0;
        }
    }

    /// Check for critical faults and enter fault state if detected
    ///
    /// Returns true if fault state was entered.
    pub fn check_critical_faults(&mut self) -> bool {
        if self.faults.intersects(CRITICAL_FAULTS) {
            self.init_state = InitState::Fault;
            self.control_law = ControlLawV1::Backup; // Was Frozen
            true
        } else {
            false
        }
    }
}
