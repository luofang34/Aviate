use crate::state::StateEstimate;
use crate::types::{Normalized, NormalizedSigned, Radians};
// We need these types, but Command/ConfigMode/Limits are likely in other modules I haven't created yet or need to define here.
// Checking the file list, I haven't created `command.rs`, `config.rs`.
// I'll assume I need to define stub types or imports for now.

// Re-exporting for submodules
pub use crate::types::Scalar;

// Placeholder structs for now since they aren't in my previous file list
#[derive(Clone, Debug)]
pub struct Command {
    // minimal stub
    pub collective_thrust: Normalized,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigMode {
    Hover,
    Cruise,
    Transition,
    Degraded,
}

#[derive(Clone, Debug)]
pub struct Limits {
    pub max_roll: Radians,
    pub max_pitch: Radians,
}

#[derive(Clone, Debug)]
pub struct AxisCommand {
    pub roll: NormalizedSigned,
    pub pitch: NormalizedSigned,
    pub yaw: NormalizedSigned,
    pub collective: Normalized,
}

pub trait VehicleController {
    fn step(
        &mut self,
        state: &StateEstimate,
        command: &Command,
        mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand;
}

#[cfg(feature = "mc")]
pub mod mc;

#[cfg(feature = "fw")]
pub mod fw;

#[cfg(feature = "vtol")]
pub mod vtol;
