//! Configuration data types
//!
//! Phase 1: Minimal stubs for compilation
//! Phase 2: Full TOML schema with all sections

use alloc::string::String;
use alloc::vec::Vec;
use serde::Deserialize;

/// Top-level application configuration
#[derive(Debug, Deserialize)]
pub struct AppConfig {
    pub app: AppInfo,
    #[serde(default)]
    pub telemetry: Option<TelemetryConfig>,
    #[serde(default)]
    pub security: Option<SecurityConfig>,
    #[serde(default)]
    pub transports: Vec<TransportConfig>,
    #[serde(default)]
    pub simulator: Option<SimulatorConfig>,
}

/// App metadata
#[derive(Debug, Deserialize)]
pub struct AppInfo {
    pub id: String,
    pub board: String,
    pub airframe: String,
    pub env: String,
}

/// Telemetry queue configuration
///
/// The `frame_size` and `queue_len` values are for validation only.
/// They must be ≤ the compile-time limits (`TELEMETRY_MAX_FRAME`, `TELEMETRY_MAX_QUEUE`).
#[derive(Debug, Deserialize)]
pub struct TelemetryConfig {
    pub frame_size: usize,
    pub queue_len: usize,
    #[serde(default = "default_heartbeat_hz")]
    pub heartbeat_hz: u8,
    #[serde(default = "default_attitude_hz")]
    pub attitude_hz: u8,
    #[serde(default = "default_position_hz")]
    pub position_hz: u8,
}

fn default_heartbeat_hz() -> u8 {
    1
}
fn default_attitude_hz() -> u8 {
    10
}
fn default_position_hz() -> u8 {
    4
}

impl Default for TelemetryConfig {
    fn default() -> Self {
        Self {
            frame_size: 280,
            queue_len: 32,
            heartbeat_hz: default_heartbeat_hz(),
            attitude_hz: default_attitude_hz(),
            position_hz: default_position_hz(),
        }
    }
}

/// Security profile configuration
#[derive(Debug, Deserialize)]
pub struct SecurityConfig {
    pub profile: String, // "none", "auth-only", "auth-and-encrypt"
}

impl Default for SecurityConfig {
    fn default() -> Self {
        Self {
            profile: "none".into(),
        }
    }
}

/// Transport configuration (port, protocol, roles)
#[derive(Debug, Deserialize)]
pub struct TransportConfig {
    pub port: String,
    pub protocol: String,
    pub roles: Vec<String>,
    #[serde(default)]
    pub baudrate: Option<u32>,
    #[serde(default)]
    pub port_sensor: Option<u16>,
    #[serde(default)]
    pub port_actuator: Option<u16>,
    /// UDP endpoint for telemetry/command (e.g., "127.0.0.1:14550")
    #[serde(default)]
    pub endpoint: Option<String>,
}

/// Simulator configuration (SITL only)
#[derive(Debug, Deserialize)]
pub struct SimulatorConfig {
    pub backend: String, // "gazebo", "jmavsim"
    #[serde(default)]
    pub headless: bool,
    #[serde(default)]
    pub lockstep: bool,
}

/// Configuration error type
#[derive(Debug)]
pub enum ConfigError {
    ParseError,
    ValidationError,
    IoError,
}
