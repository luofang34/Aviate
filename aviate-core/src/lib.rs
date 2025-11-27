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

use crate::ekf::Ekf;
use crate::control::{VehicleController, Command, ConfigMode, Limits, AuthorityProfile};
use crate::control::envelope::{SimpleEnvelopeProtector, EnvelopeProtector};
use crate::mixer::{Mixer, Sanitizer, ActuatorCmd, ActuatorSanitizer, ModeConfig};

pub struct AviateKernel<V: VehicleController, M: Mixer> {
    pub ekf: Ekf,
    pub controller: V,
    pub mixer: M,
    pub sanitizer: Sanitizer,
    pub protector: SimpleEnvelopeProtector,
    pub limits: Limits,
    pub mode: ConfigMode,
    pub mode_config: ModeConfig, // Added to pass to sanitizer
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
        }
    }
    
    pub fn step(&mut self, cmd: &Command) -> ActuatorCmd {
        let state = self.ekf.get_estimate();
        
        // Envelope Protection
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

        let axis_cmd = self.controller.step(&state, &constrained_cmd, self.mode, &self.limits);
        let mut actuator_cmd = self.mixer.mix(&axis_cmd);
        self.sanitizer.sanitize(&mut actuator_cmd, &self.mode_config);
        actuator_cmd
    }
}

/// Aviate core initialization
pub fn init_core() {}