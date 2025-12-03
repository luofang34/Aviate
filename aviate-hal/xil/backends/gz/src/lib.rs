//! Gazebo Backend for Aviate XIL
//!
//! This crate implements `KinematicsBackend` for Gazebo Sim (gz-sim).
//! It provides direct FFI integration with the AviateGzPlugin.
//!
//! ## Architecture
//!
//! ```text
//! AviateGzPlugin (C++, loaded by Gazebo)
//!        ↓ Direct FFI
//! gazebo_bridge.rs (ENU→NED conversion, Rust API)
//!        ↓
//! SitlIO (simulator-neutral middleware)
//!        ↓
//! FakeSensors / Mixer
//! ```
//!
//! ## FFI Functions
//!
//! The C++ plugin calls these functions directly:
//! - `aviate_gz_bridge_init(instance)` - Initialize for a vehicle instance
//! - `aviate_gz_bridge_feed_sensors(data)` - Feed sensor data from Gazebo
//! - `aviate_gz_bridge_get_motors(cmd)` - Get motor commands for Gazebo
//! - `aviate_gz_bridge_shutdown()` - Cleanup

// Note: FFI modules contain unsafe code for C interop

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
