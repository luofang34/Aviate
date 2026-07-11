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
    /// Application identity and target metadata.
    pub app: AppInfo,
    /// Telemetry queue/rate configuration; absent means defaults.
    #[serde(default)]
    pub telemetry: Option<TelemetryConfig>,
    /// Command-authentication profile; absent means no security.
    #[serde(default)]
    pub security: Option<SecurityConfig>,
    /// Transport endpoints (serial/UDP) and their roles.
    #[serde(default)]
    pub transports: Vec<TransportConfig>,
    /// Simulator backend settings; present only for SITL builds.
    #[serde(default)]
    pub simulator: Option<SimulatorConfig>,
}

/// App metadata
#[derive(Debug, Deserialize)]
pub struct AppInfo {
    /// Unique application identifier.
    pub id: String,
    /// Target board name.
    pub board: String,
    /// Airframe model this build flies.
    pub airframe: String,
    /// Runtime environment (`flight`, `sitl`, `hitl`).
    pub env: String,
}

/// Telemetry queue configuration
///
/// The `frame_size` and `queue_len` values are for validation only.
/// They must be ≤ the compile-time limits (`TELEMETRY_MAX_FRAME`, `TELEMETRY_MAX_QUEUE`).
#[derive(Debug, Deserialize)]
pub struct TelemetryConfig {
    /// Maximum telemetry frame size in bytes.
    pub frame_size: usize,
    /// Number of frames the outbound queue holds.
    pub queue_len: usize,
    /// Heartbeat message rate in Hz.
    #[serde(default = "default_heartbeat_hz")]
    pub heartbeat_hz: u8,
    /// Attitude message rate in Hz.
    #[serde(default = "default_attitude_hz")]
    pub attitude_hz: u8,
    /// Position message rate in Hz.
    #[serde(default = "default_position_hz")]
    pub position_hz: u8,
    /// Estimator-status message rate in Hz.
    #[serde(default = "default_estimator_status_hz")]
    pub estimator_status_hz: u8,
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
fn default_estimator_status_hz() -> u8 {
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
            estimator_status_hz: default_estimator_status_hz(),
        }
    }
}

/// Security profile configuration
#[derive(Debug, Deserialize)]
pub struct SecurityConfig {
    /// Security profile: `none`, `auth-only`, or `auth-and-encrypt`.
    pub profile: String,
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
    /// Serial device path or transport identifier.
    pub port: String,
    /// Wire protocol spoken on this transport (e.g. `mavlink`).
    pub protocol: String,
    /// Roles this transport serves (e.g. `telemetry`, `command`).
    pub roles: Vec<String>,
    /// Serial baud rate; absent for non-serial transports.
    #[serde(default)]
    pub baudrate: Option<u32>,
    /// UDP port for inbound sensor data (SITL).
    #[serde(default)]
    pub port_sensor: Option<u16>,
    /// UDP port for outbound actuator commands (SITL).
    #[serde(default)]
    pub port_actuator: Option<u16>,
    /// UDP endpoint for telemetry/command (e.g., "127.0.0.1:14550")
    #[serde(default)]
    pub endpoint: Option<String>,
}

/// Simulator configuration (SITL only)
#[derive(Debug, Deserialize)]
pub struct SimulatorConfig {
    /// Simulator backend: `gazebo` or `jmavsim`.
    pub backend: String,
    /// Run the simulator without a GUI.
    #[serde(default)]
    pub headless: bool,
    /// Advance the simulator in lockstep with the flight loop.
    #[serde(default)]
    pub lockstep: bool,
}

/// Configuration error type
#[derive(Debug)]
pub enum ConfigError {
    /// TOML could not be parsed into the config schema.
    ParseError,
    /// Parsed config violated a validation constraint.
    ValidationError,
    /// Underlying I/O failed while reading the config.
    IoError,
}
