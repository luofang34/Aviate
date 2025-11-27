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

use crate::ekf::Ekf;
use crate::control::{VehicleController, Command, ConfigMode, Limits, AuthorityProfile, ControlLaw};
use crate::control::envelope::{SimpleEnvelopeProtector, EnvelopeProtector};
use crate::mixer::{Mixer, Sanitizer, ActuatorCmd, ActuatorSanitizer, ModeConfig};
use crate::fault::{FaultFlags, FaultHandlingTable};
use crate::time::{Timestamp, TimeDelta};
use crate::sensor::SensorSet;
use crate::types::Normalized;

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
    
    // Safety
    pub safe_output: [Normalized; 16], // MAX_ACTUATORS = 16
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
            safe_output: [Normalized(0.0); 16],
        }
    }
    
    pub fn init_step(&mut self, _sensors: &SensorSet, _time: Timestamp) -> InitResult {
        // Simple placeholder initialization logic
        match self.init_state {
            InitState::PowerOn => self.init_state = InitState::ConfigLoading,
            InitState::ConfigLoading => self.init_state = InitState::SensorInit,
            InitState::SensorInit => self.init_state = InitState::EstimatorConverging,
            InitState::EstimatorConverging => {
                if self.ekf.is_initialized() {
                    self.init_state = InitState::PreArm;
                } else {
                    // In real usage, we'd check convergence criteria
                    // For now, assume immediate
                     self.init_state = InitState::PreArm;
                }
            }
            InitState::PreArm => {
                if self.faults.is_empty() {
                    self.init_state = InitState::Ready;
                }
            }
            InitState::Ready => {
                if !self.faults.is_empty() {
                    self.init_state = InitState::PreArm; // Fallback if fault appears
                }
            }
            _ => {}
        }

        InitResult {
            state: self.init_state,
            faults: self.faults,
            ready: self.init_state == InitState::Ready,
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
        // In disarmed state, we might transition back to Ready or PreArm eventually,
        // but spec says Disarmed. Usually requires reset or check to go back to Ready.
        // For now, keep as Disarmed.
    }

    pub fn get_health(&self) -> () {
        // Placeholder for HealthReport
        ()
    }
    
    pub fn step(&mut self, cmd: &Command) -> ActuatorCmd {
        // 1. Check InitState
        if !self.init_state.allows_active_control() {
             return ActuatorCmd {
                 outputs: self.safe_output,
                 active_mask: 0, // Or appropriate mask
                 sequence: cmd.sequence,
                 timestamp: crate::time::Timestamp { ticks: 0, source: crate::time::TimeSource::Internal }, // Placeholder
                 fallback_mask: 0,
                 sanitized: true,
             };
        }

        // 2. EKF Update (usually happens before step in loop, but we get estimate here)
        let state = self.ekf.get_estimate();
        
        // 3. Envelope Protection
        let (constrained_sp, _protection_status) = self.protector.constrain(
            &cmd.setpoint, 
            &state, 
            &self.limits, 
            AuthorityProfile::HardEnvelope
        );
        
        let constrained_cmd = Command {
            setpoint: constrained_sp,
            mode: cmd.mode,
            config_mode_request: cmd.config_mode_request,
            sensor_overrides: cmd.sensor_overrides,
            sequence: cmd.sequence,
            source: cmd.source,
        };

        // 4. Control Step
        let axis_cmd = self.controller.step(&state, &constrained_cmd, self.mode, &self.limits);
        
        // 5. Mixing
        let mut actuator_cmd = self.mixer.mix(&axis_cmd);
        
        // 6. Sanitization
        self.sanitizer.sanitize(&mut actuator_cmd, &self.mode_config);
        
        actuator_cmd
    }
}


/// Aviate core initialization
pub fn init_core() {}