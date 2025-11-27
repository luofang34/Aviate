//! Gazebo Backend for Aviate XIL
//!
//! This crate implements `KinematicsBackend` for Gazebo Sim (gz-sim).
//! It provides integration via shared memory FFI to the AviateGzPlugin.
//!
//! ## Architecture
//!
//! ```text
//! aviate-backend-gz (this crate)
//!        ↓ FFI/IPC (shared memory)
//! aviate_gz_plugin (C++, loaded by Gazebo)
//!        ↓ gz-transport
//! gz-sim (physics)
//! ```
//!
//! ## Features
//!
//! - `gz-plugin`: Enable FFI bridge to AviateGzPlugin (requires libaviate_gz_bridge.so)

// Note: plugin.rs contains FFI to C++ shared memory, requires unsafe

mod plugin;
mod bridge;
mod backend;

pub use plugin::{GzPluginBridge, GzPluginError, AviateModelState, AviateMotorCommand};
pub use plugin::{enu_to_ned, enu_to_ned_f32, enu_vel_to_ned, enu_vel_to_ned_f32};
pub use bridge::{GzBridge, GzBridgeConfig, GzBridgeError};
pub use backend::GazeboBackend;
