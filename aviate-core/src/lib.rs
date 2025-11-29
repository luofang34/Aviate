#![no_std]
#![forbid(unsafe_code)]

pub mod types;
pub mod math;
pub mod time;
pub mod sensor;
pub mod state;
pub mod ekf;
pub mod control;
pub mod mixer;
pub mod fault;
pub mod hal;
pub mod checks;

use crate::ekf::Ekf;
use crate::control::{VehicleController, Command, ConfigMode, Limits, AuthorityProfile, ControlLaw, ControlMode};
use crate::control::envelope::{SimpleEnvelopeProtector, EnvelopeProtector, ProtectionStatus};
use crate::mixer::{Mixer, Sanitizer, ActuatorCmd, ActuatorSanitizer, ModeConfig, SanitizeReport, ActuatorState};
use crate::fault::{FaultFlags, FaultHandlingTable};
use crate::time::{Timestamp, TimeDelta};
use crate::sensor::SensorSet;
use crate::state::{StateEstimate, EstimateQuality};
use crate::types::{Normalized, Radians, RadiansPerSecond, Meters, MetersPerSecond};
use crate::checks::{KernelChecks, PreArmFlags, CheckInvariants};
pub use crate::checks::{DegradationReason, TransitionFailure, TransitionLimits, InFlightFlags, TransitionFlags};

/// Critical faults that trigger immediate fault state entry
///
/// These are faults that require the aircraft to enter a safe state immediately.
/// The kernel will transition to InitState::Fault when any of these are detected.
pub const CRITICAL_FAULTS: FaultFlags = FaultFlags::ALL_IMU_FAILED
    .union(FaultFlags::NUMERIC_ERROR)
    .union(FaultFlags::ESTIMATOR_DIVERGED);

/// Default command timeout in milliseconds
pub const DEFAULT_COMMAND_TIMEOUT_MS: u32 = 500;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InitState {
    PowerOn,
    ConfigLoading,
    SensorInit,
    EstimatorConverging,
    PreArm,
    Ready,
    Armed,
    Disarmed,
    Fault,
}

impl InitState {
    pub fn allows_active_control(&self) -> bool {
        matches!(self, InitState::Armed)
    }
    
    pub fn forced_control_law(&self) -> Option<ControlLaw> {
        if self.allows_active_control() { None } 
        else { Some(ControlLaw::Frozen) }
    }
}

#[derive(Clone, Debug)]
pub struct InitResult {
    pub state: InitState,
    pub faults: FaultFlags,
    pub ready: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ArmError {
    NotReady,
    Faulted,
    AlreadyArmed,
    ConfigInvalid,
    InFaultState,
}

/// Error returned when attempting a configuration mode transition
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransitionError {
    /// Not armed (transitions only allowed while armed)
    NotArmed,
    /// Already transitioning to another mode
    AlreadyTransitioning,
    /// Transition checks failed
    ChecksFailed(TransitionFailure),
    /// Target mode same as current mode
    AlreadyInMode,
    /// System in fault state
    InFaultState,
}

// --- Spec §16: Channel & Redundancy Model ---

/// Channel identifier for redundant flight controllers
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ChannelId(pub u8);

impl ChannelId {
    pub const PRIMARY: Self = Self(0);
    pub const SECONDARY: Self = Self(1);
    pub const TERTIARY: Self = Self(2);
    pub const MAX_CHANNELS: usize = 3;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChannelHealth { Operative, Degraded, Failed, Testing }

impl Default for ChannelHealth {
    fn default() -> Self { Self::Operative }
}

// --- Spec §18: Timing ---

/// Timing statistics for control loop
#[derive(Copy, Clone, Debug, Default)]
pub struct TimingStats {
    pub last_cycle_us: u32,
    pub max_cycle_us: u32,
    pub min_cycle_us: u32,
    pub deadline_violations: u32,
    pub consecutive_violations: u32,
    pub total_cycles: u64,
}

/// Per-cycle timing information
#[derive(Copy, Clone, Debug)]
pub struct CycleTiming {
    pub cycle_start_us: u32,
    pub cycle_end_us: u32,
    pub duration_us: u32,
    pub deadline_met: bool,
}

impl Default for CycleTiming {
    fn default() -> Self {
        Self {
            cycle_start_us: 0,
            cycle_end_us: 0,
            duration_us: 0,
            deadline_met: true,
        }
    }
}

// --- Spec §13: Envelope Margin ---

/// Remaining margin before limit breach (positive = margin remaining)
#[derive(Copy, Clone, Debug, Default)]
pub struct EnvelopeMargin {
    pub roll_rad: Radians,
    pub pitch_rad: Radians,
    pub yaw_rate_rad_s: RadiansPerSecond,
    pub altitude_m: Meters,
    pub airspeed_mps: MetersPerSecond,
    pub load_factor: f32,
}

// --- Spec §14: Degradation ---

#[derive(Copy, Clone, Debug)]
pub struct DegradationEvent {
    pub from: ControlLaw,
    pub to: ControlLaw,
    pub reason: DegradationReason,
    pub timestamp: Timestamp,
}

// --- Spec §4.4: Configuration Transition ---
// TransitionFailure is imported from checks.rs

/// Configuration transition state for morphing aircraft
#[derive(Clone, Debug)]
pub enum ConfigTransitionState {
    /// Stable in a configuration mode
    Stable(ConfigMode),
    /// Actively transitioning between modes
    Switching {
        from: ConfigMode,
        to: ConfigMode,
        progress: f32,
    },
    /// Transition failed
    Failed {
        intended: ConfigMode,
        actual: ConfigMode,
        reason: TransitionFailure,
    },
}

impl Default for ConfigTransitionState {
    fn default() -> Self { Self::Stable(ConfigMode::Hover) }
}

// --- Spec §16: Channel Status ---

/// Full per-cycle status from kernel
#[derive(Clone, Debug)]
pub struct ChannelStatus {
    pub mode: ControlMode,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub law: ControlLaw,
    pub health: ChannelHealth,
    pub faults: FaultFlags,
    pub confidence: EstimateQuality,
    pub envelope_margin: EnvelopeMargin,
    pub sequence: u32,
    pub protection: ProtectionStatus,
    pub sanitize_report: SanitizeReport,
}

impl Default for ChannelStatus {
    fn default() -> Self {
        Self {
            mode: ControlMode::Rate,
            config_mode: ConfigMode::Hover,
            transition_state: ConfigTransitionState::default(),
            law: ControlLaw::Normal,
            health: ChannelHealth::Operative,
            faults: FaultFlags::empty(),
            confidence: EstimateQuality::Good,
            envelope_margin: EnvelopeMargin::default(),
            sequence: 0,
            protection: ProtectionStatus::default(),
            sanitize_report: SanitizeReport::default(),
        }
    }
}

// --- Spec §20: Core Interface ---

/// Full result from kernel update() - spec §20
#[derive(Clone, Debug)]
pub struct UpdateResult {
    pub actuator: ActuatorCmd,
    pub status: ChannelStatus,
    pub estimate: StateEstimate,
    pub timing: CycleTiming,
    pub degradation: Option<DegradationEvent>,
}

/// Lightweight health snapshot - spec §20
#[derive(Clone, Debug)]
pub struct HealthReport {
    pub init_state: InitState,
    pub control_law: ControlLaw,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub faults: FaultFlags,
    pub channel_health: ChannelHealth,
}

pub struct AviateKernel<V: VehicleController, M: Mixer> {
    pub ekf: Ekf,
    pub controller: V,
    pub mixer: M,
    pub sanitizer: Sanitizer,
    pub protector: SimpleEnvelopeProtector,
    pub limits: Limits,
    pub mode: ConfigMode,
    pub mode_config: ModeConfig,

    // State Machine
    pub init_state: InitState,
    pub faults: FaultFlags,
    pub fault_table: FaultHandlingTable,
    pub control_law: ControlLaw,

    // Unified Check System (§17, §14, §4.5)
    pub checks: KernelChecks,

    // Actuator state tracking for transition checks
    pub actuator_state: ActuatorState,

    // Command timeout threshold (ms)
    pub command_timeout_ms: u32,

    // Safety
    pub safe_output: [Normalized; 16], // MAX_ACTUATORS = 16
}

pub trait Watchdog {
    fn kick(&mut self);
    fn check_deadline(&self) -> bool;
}

impl<V: VehicleController, M: Mixer> Watchdog for AviateKernel<V, M> {
    fn kick(&mut self) {
        // Minimal implementation: just a stub for now as we don't have full timing context
        // In a real system, this would update a timestamp
    }

    fn check_deadline(&self) -> bool {
        // Stub: always return true for now
        true
    }
}

impl<V: VehicleController, M: Mixer> AviateKernel<V, M> {
    pub fn new(controller: V, mixer: M, mode_config: ModeConfig) -> Self {
        Self {
            ekf: Ekf::default(),
            controller,
            mixer,
            sanitizer: Sanitizer::default(),
            protector: SimpleEnvelopeProtector,
            limits: Limits { 
                max_roll: crate::types::Radians(0.78), // ~45 deg
                max_pitch: crate::types::Radians(0.78),
                max_roll_rate: crate::types::RadiansPerSecond(3.0),
                max_pitch_rate: crate::types::RadiansPerSecond(3.0),
                max_yaw_rate: crate::types::RadiansPerSecond(3.0),
                max_horizontal_speed: crate::types::MetersPerSecond(10.0),
                max_climb_rate: crate::types::MetersPerSecond(2.0),
                max_descent_rate: crate::types::MetersPerSecond(2.0),
                max_altitude: crate::types::Meters(100.0),
                min_altitude: crate::types::Meters(0.0),
                min_airspeed: None,
                max_airspeed: None,
                max_load_factor: 2.0,
                min_load_factor: 0.0,
            },
            mode: ConfigMode::Hover,
            mode_config,
            
            init_state: InitState::PowerOn,
            faults: FaultFlags::empty(),
            fault_table: FaultHandlingTable::DEFAULT,
            control_law: ControlLaw::Normal,
            checks: KernelChecks::new(),
            actuator_state: ActuatorState::default(),
            command_timeout_ms: DEFAULT_COMMAND_TIMEOUT_MS,
            safe_output: [Normalized(0.0); 16],
        }
    }

    /// Create kernel with custom pre-arm requirements
    pub fn with_pre_arm_required(controller: V, mixer: M, mode_config: ModeConfig, required: PreArmFlags) -> Self {
        let mut kernel = Self::new(controller, mixer, mode_config);
        kernel.checks = KernelChecks::with_pre_arm_required(required);
        kernel
    }

    pub fn init_step(&mut self, sensors: &SensorSet, _time: Timestamp) -> InitResult {
        // 1. Update checks from sensor data (always, regardless of state)
        self.checks.pre_arm.update_from_sensors(sensors);
        self.checks.pre_arm.update_from_faults(self.faults);
        self.checks.pre_arm.update_ekf(self.ekf.is_initialized());

        // 2. Update faults from sensor health
        self.update_sensor_faults(sensors);

        // 3. State machine transitions
        match self.init_state {
            InitState::PowerOn => {
                self.init_state = InitState::ConfigLoading;
            }
            InitState::ConfigLoading => {
                // Config loaded (placeholder - would check actual config validity)
                self.checks.pre_arm.current.insert(PreArmFlags::CONFIG_VALID);
                self.init_state = InitState::SensorInit;
            }
            InitState::SensorInit => {
                // Wait for at least one valid sensor reading
                let has_sensors = self.checks.pre_arm.current.contains(PreArmFlags::IMU_HEALTHY);
                if has_sensors {
                    self.init_state = InitState::EstimatorConverging;
                }
            }
            InitState::EstimatorConverging => {
                // Wait for sensor convergence and EKF initialization
                let converged = self.checks.pre_arm.current.contains(PreArmFlags::IMU_CONVERGED)
                    && self.checks.pre_arm.current.contains(PreArmFlags::EKF_CONVERGED);
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
            InitState::Armed => {
                // Stay armed, monitor for critical faults
                // Disarm handled separately via disarm()
            }
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
    fn update_sensor_faults(&mut self, sensors: &SensorSet) {
        use crate::sensor::SensorHealth;

        // IMU faults
        let imu_ok = sensors.imus.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        if !imu_ok {
            self.faults.insert(FaultFlags::ALL_IMU_FAILED);
        } else {
            self.faults.remove(FaultFlags::ALL_IMU_FAILED);
        }

        // Baro faults
        let baro_ok = sensors.baros.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        if !baro_ok {
            self.faults.insert(FaultFlags::BARO_FAILED);
        } else {
            self.faults.remove(FaultFlags::BARO_FAILED);
        }

        // Mag faults
        let mag_ok = sensors.mags.iter().any(|s| s.valid && s.health == SensorHealth::Good);
        if !mag_ok {
            self.faults.insert(FaultFlags::MAG_FAILED);
        } else {
            self.faults.remove(FaultFlags::MAG_FAILED);
        }

        // GNSS faults
        let gnss_ok = sensors.gnss.iter().any(|s| s.valid && s.health == SensorHealth::Good);
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
        self.control_law = ControlLaw::Frozen;
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
        let imu_healthy = self.checks.pre_arm.current.contains(PreArmFlags::IMU_HEALTHY);

        // Throttle low for safety
        let throttle_low = self.checks.pre_arm.current.contains(PreArmFlags::THROTTLE_LOW);

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
    pub fn handle_degradation(&mut self, reason: DegradationReason, timestamp: Timestamp) -> Option<DegradationEvent> {
        let from = self.control_law;
        let to = match reason {
            DegradationReason::AttitudeLost => ControlLaw::Frozen,
            DegradationReason::ImuDegraded => ControlLaw::Degraded,
            DegradationReason::PositionLost => ControlLaw::Degraded,
            DegradationReason::VelocityLost => ControlLaw::Degraded,
            DegradationReason::CommandTimeout => ControlLaw::Failsafe,
            DegradationReason::EnvelopeViolation => ControlLaw::Degraded,
            DegradationReason::BaroDegraded => ControlLaw::Degraded,
            DegradationReason::RcLost => ControlLaw::Failsafe,
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
        self.checks.transition.update_from_actuators(&self.actuator_state, 0b1111); // Quad mask

        // Gate the transition
        self.checks.transition.can_transition()
            .map_err(TransitionError::ChecksFailed)?;

        // Start the transition (caller manages progress)
        // For now, just update the mode directly
        self.mode = to;
        Ok(())
    }

    pub fn get_health(&self) -> HealthReport {
        HealthReport {
            init_state: self.init_state,
            control_law: self.control_law,
            config_mode: self.mode,
            transition_state: ConfigTransitionState::Stable(self.mode),
            faults: self.faults,
            channel_health: ChannelHealth::Operative,
        }
    }
    
    /// Main control step with in-flight monitoring
    ///
    /// # Arguments
    /// * `cmd` - The command to execute
    /// * `sensors` - Current sensor readings for in-flight checks
    /// * `command_age_ms` - Age of the command in milliseconds
    pub fn step(&mut self, time_delta: TimeDelta, cmd: &Command, sensors: &SensorSet, command_age_ms: u32) -> ActuatorCmd {
        let timestamp = crate::time::Timestamp { ticks: 0, source: crate::time::TimeSource::Internal };

        // 1. Check InitState - return safe output if not armed
        if !self.init_state.allows_active_control() {
             return ActuatorCmd {
                 outputs: self.safe_output,
                 active_mask: 0,
                 sequence: cmd.sequence,
                 timestamp,
                 fallback_mask: 0,
                 sanitized: true,
             };
        }

        // 2. Update sensor faults and check for critical faults
        self.update_sensor_faults(sensors);
        if self.check_critical_faults() {
            return ActuatorCmd {
                outputs: self.safe_output,
                active_mask: 0,
                sequence: cmd.sequence,
                timestamp,
                fallback_mask: 0,
                sanitized: true,
            };
        }

        // 3. EKF Update (predict and update)
        let primary_imu = &sensors.imus[0];
        if primary_imu.valid && primary_imu.health == crate::sensor::SensorHealth::Good {
            self.ekf.predict(&primary_imu.value, time_delta.dt_sec.0);
        }

        // Apply sensor overrides from command
        if let Some(overrides) = &cmd.sensor_overrides {
            if let Some(gnss_force_state_u8) = overrides.gnss_force_state {
                let gnss_health = match gnss_force_state_u8 {
                    0 => crate::sensor::GnssHealth::Good,
                    1 => crate::sensor::GnssHealth::Suspect,
                    2 => crate::sensor::GnssHealth::Lost,
                    _ => crate::sensor::GnssHealth::Lost, // Default to Lost for unknown values
                };

                let mut primary_gnss_reading = sensors.gnss[0];
                primary_gnss_reading.health = match gnss_health {
                    crate::sensor::GnssHealth::Good => crate::sensor::SensorHealth::Good,
                    crate::sensor::GnssHealth::Suspect => crate::sensor::SensorHealth::Degraded,
                    crate::sensor::GnssHealth::Lost => crate::sensor::SensorHealth::Failed,
                };
                self.ekf.update_gnss(&primary_gnss_reading);
            }
        } else {
            // Normal sensor updates
            let primary_gnss = &sensors.gnss[0];
            if primary_gnss.valid && primary_gnss.health == crate::sensor::SensorHealth::Good {
                self.ekf.update_gnss(primary_gnss);
            }
        }
        
        let primary_baro = &sensors.baros[0];
        if primary_baro.valid && primary_baro.health == crate::sensor::SensorHealth::Good {
            self.ekf.update_baro(primary_baro);
        }

        let primary_mag = &sensors.mags[0];
        if primary_mag.valid && primary_mag.health == crate::sensor::SensorHealth::Good {
            self.ekf.update_mag(primary_mag);
        }

        // Get updated estimate
        let state = self.ekf.get_estimate();

        // 4. Update in-flight checks
        self.checks.in_flight.update_from_state(&state);
        self.checks.in_flight.update_from_sensors(sensors);
        self.checks.in_flight.update_command_status(command_age_ms, self.command_timeout_ms);

        // 5. Handle any degradation triggers
        if let Some(reason) = self.checks.in_flight.get_degradation_trigger() {
            let _event = self.handle_degradation(reason, timestamp);
            // Could log or emit the event here
        }

        // 6. Debug invariant verification
        #[cfg(debug_assertions)]
        {
            let _ok = CheckInvariants::verify_all(
                self.faults,
                &self.checks.pre_arm,
                &self.checks.in_flight,
            );
            // In debug builds, could assert!(_ok) or log violations
        }

        // 7. Envelope Protection
        let (constrained_sp, protection_status) = self.protector.constrain(
            &cmd.setpoint,
            &state,
            &self.limits,
            AuthorityProfile::HardEnvelope
        );

        // Update envelope status for in-flight checks
        self.checks.in_flight.update_from_envelope(&protection_status);

        let constrained_cmd = Command {
            setpoint: constrained_sp,
            mode: cmd.mode,
            config_mode_request: cmd.config_mode_request,
            sensor_overrides: cmd.sensor_overrides,
            sequence: cmd.sequence,
            source: cmd.source,
        };

        // 8. Control Step - use frozen output if control law is Frozen
        if self.control_law == ControlLaw::Frozen {
            return ActuatorCmd {
                outputs: self.safe_output,
                active_mask: 0b1111,
                sequence: cmd.sequence,
                timestamp,
                fallback_mask: 0xFF, // All fallback
                sanitized: true,
            };
        }

        let axis_cmd = self.controller.step(&state, &constrained_cmd, self.mode, &self.limits);

        // 9. Mixing
        let mut actuator_cmd = self.mixer.mix(&axis_cmd);

        // 10. Update actuator state for transition checks
        self.actuator_state.update_commanded(&actuator_cmd, timestamp);

        // 11. Sanitization
        self.sanitizer.sanitize(&mut actuator_cmd, &self.mode_config);

        actuator_cmd
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
    }

    pub fn kick_watchdog(&mut self) {
        self.kick();
    }

    /// Check for critical faults and enter fault state if detected
    ///
    /// Returns true if fault state was entered.
    pub fn check_critical_faults(&mut self) -> bool {
        if self.faults.intersects(CRITICAL_FAULTS) {
            self.init_state = InitState::Fault;
            self.control_law = ControlLaw::Frozen;
            true
        } else {
            false
        }
    }
}


/// Aviate core initialization
pub fn init_core() {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::mc::McController;
    use crate::mixer::{QuadXMixer, ModeConfig, ActuatorCmd};

    struct DummyMixer;
    impl Mixer for DummyMixer {
        fn mix(&self, _axis: &crate::control::AxisCommand) -> ActuatorCmd {
            let cmd = ActuatorCmd::default();
            cmd
        }
    }

    fn create_kernel() -> AviateKernel<McController, DummyMixer> {
        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };
        AviateKernel::new(McController::default(), DummyMixer, mode_config)
    }

    #[test]
    fn test_ground_reset_success_unit() {
        let mut kernel = create_kernel();
        kernel.init_state = InitState::Fault;
        kernel.faults = FaultFlags::ALL_IMU_FAILED;
        
        kernel.ground_reset();
        
        assert_eq!(kernel.init_state, InitState::ConfigLoading);
        assert!(kernel.faults.is_empty());

        // Cover DummyMixer
        kernel.mixer.mix(&crate::control::AxisCommand {
            roll: crate::types::NormalizedSigned(0.0),
            pitch: crate::types::NormalizedSigned(0.0),
            yaw: crate::types::NormalizedSigned(0.0),
            collective: crate::types::Normalized(0.0),
        });
    }
}