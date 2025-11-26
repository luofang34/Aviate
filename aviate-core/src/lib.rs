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
use crate::control::{VehicleController, Command, ConfigMode, Limits, AxisCommand};

pub struct AviateKernel<V: VehicleController> {
    pub ekf: Ekf,
    pub controller: V,
    // Spec mentions these, placeholder for now
    pub limits: Limits,
    pub mode: ConfigMode,
}

impl<V: VehicleController> AviateKernel<V> {
    pub fn new(controller: V) -> Self {
        Self {
            ekf: Ekf::default(),
            controller,
            limits: Limits { 
                max_roll: crate::types::Radians(0.78), // ~45 deg
                max_pitch: crate::types::Radians(0.78), 
            },
            mode: ConfigMode::Hover,
        }
    }
    
    pub fn step(&mut self, cmd: &Command) -> AxisCommand {
        let state = self.ekf.get_estimate();
        self.controller.step(&state, cmd, self.mode, &self.limits)
    }
}

/// Aviate core initialization
pub fn init_core() {}