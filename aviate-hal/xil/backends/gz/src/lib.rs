//! Gazebo backend for Aviate XIL.
//!
//! Connects the flight controller to the `AviateGzPlugin` running
//! inside gz-sim through the Rust-owned shared-memory contract
//! (`aviate-xil-contract`, #262).
//!
//! ## Architecture
//!
//! ```text
//! AviateGzPlugin (C++, loaded by Gazebo)
//!        ↓ /aviate_gz_bridge shared block (aviate-xil-contract)
//! plugin.rs — GzPluginBridge over aviate-xil-shm's FcSession
//!        ↓ ENU→NED conversion
//! SitlIO (simulator-neutral middleware)
//!        ↓
//! FakeSensors / Mixer
//! ```
//!
//! There is no C FFI on this path: the layout is Rust-owned, the
//! plugin consumes its cbindgen-generated C header, and every
//! shared-memory access goes through `aviate-xil-shm` — the one
//! crate in the SITL data plane that contains unsafe code.

mod backend;
mod bridge;
pub mod gazebo_bridge;
mod plugin;
mod sim_backend;

pub use backend::GazeboBackend;
pub use bridge::{GzBridge, GzBridgeConfig, GzBridgeError};
pub use gazebo_bridge::{GzMotorCmd, GzSensorData};
pub use plugin::{enu_to_ned, enu_to_ned_f32, enu_vel_to_ned, enu_vel_to_ned_f32};
pub use plugin::{AviateModelState, AviateMotorCommand, GzPluginBridge, GzPluginError};
pub use sim_backend::GazeboSimBackend;
