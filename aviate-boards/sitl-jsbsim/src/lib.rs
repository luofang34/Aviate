//! JSBSim SITL board.
//!
//! The JSBSim sidecar owns the physics and the simulation clock. It
//! listens on TCP, sends one `HIL_STATE_QUATERNION` per physics step,
//! and waits for the `HIL_ACTUATOR_CONTROLS` that answers it. The
//! flight controller dials it. This board turns each truth state into a
//! sensor sample with the shared XIL synthesizer, runs one kernel cycle
//! on the simulation clock, and answers with the mixer's command through
//! the airframe's actuator map. Airframe selection stays with the
//! application: the board takes the kernel and the map by injection.
//!
//! A new TCP session, or simulation time that runs backwards, is a new
//! simulated world: the board states a new reset generation, and the
//! runner ground-resets the flight controller.

#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

mod backend;
mod board;
mod config;
mod truth;

pub use backend::JsbsimSimulatorBackend;
pub use board::{ActuatorMap, JsbsimBoard, JsbsimError, TruthStep};
pub use config::{JsbsimConfig, SIDECAR_BASE_PORT};
