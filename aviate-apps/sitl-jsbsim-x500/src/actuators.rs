//! The JSBSim X500's rotor commands, from the mixer's thrust command.
//!
//! JSBSim's X500 turns a command `c` in [0, 1] into rotor speed
//! `ω = 150 + 850·c` rad/s, and each rotor's thrust grows as `ω²`, so full
//! thrust is at 1000 rad/s. The mixer commands each rotor's thrust as a
//! fraction `f` of full, so the command that gives it is
//! `c = (1000·√f − 150) / 850`. Below the idle speed's thrust
//! (2.25 % of full) the rotor idles.
//!
//! Engine order is the mixer's: `QuadXMixerX500` already uses the PX4
//! quad-X order the JSBSim model does (0 FR CCW, 1 RL CCW, 2 FL CW,
//! 3 RR CW).

use aviate_core::mixer::ActuatorCmd;
use aviate_hal_xil::SimActuatorCmd;

/// Rotor speed at command 0, rad/s.
const OMEGA_IDLE: f32 = 150.0;
/// Rotor speed gained from command 0 to 1, rad/s.
const OMEGA_SPAN: f32 = 850.0;
/// Rotor speed at full thrust, rad/s.
pub(crate) const OMEGA_MAX: f32 = OMEGA_IDLE + OMEGA_SPAN;
/// The X500's rotors.
const ROTORS: usize = 4;

/// The JSBSim command for a rotor thrust fraction `thrust` of full.
/// A non-finite thrust idles the rotor.
pub(crate) fn command_for_thrust(thrust: f32) -> f32 {
    let thrust = if thrust.is_nan() {
        0.0
    } else {
        thrust.clamp(0.0, 1.0)
    };
    let omega = thrust.sqrt() * OMEGA_MAX;
    ((omega - OMEGA_IDLE) / OMEGA_SPAN).clamp(0.0, 1.0)
}

/// The board's actuator map for the JSBSim X500: rotor commands while
/// armed, all zero while disarmed.
pub(crate) fn x500_controls(command: &ActuatorCmd, armed: bool) -> SimActuatorCmd {
    let mut outputs = [0.0; 16];
    if armed {
        for (out, thrust) in outputs.iter_mut().zip(command.outputs.iter().take(ROTORS)) {
            *out = command_for_thrust(thrust.0);
        }
    }
    SimActuatorCmd {
        outputs,
        count: ROTORS as u8,
        armed,
        ..SimActuatorCmd::default()
    }
}

#[cfg(test)]
#[allow(clippy::panic)]
mod tests;
