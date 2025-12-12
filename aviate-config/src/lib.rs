//! Application configuration parsing for Aviate flight control apps
//!
//! # CRITICAL: DO-178C DAL Separation
//!
//! This crate is **LOW-DAL ONLY**:
//! - TOML parsing happens **once at startup** (init phase)
//! - High-DAL control loop **NEVER** calls TOML APIs
//! - Parse failure = safe abort / fail to arm
//!
//! ## Usage Pattern
//!
//! ```ignore
//! // Low-DAL init phase (main.rs, before starting tasks)
//! const APP_CONFIG_TOML: &str = include_str!("../AviateApp.toml");
//! let config = aviate_config::from_toml_str(APP_CONFIG_TOML)
//!     .expect("invalid AviateApp.toml");
//!
//! // After parsing: work with typed AppConfig (high-DAL safe)
//! let frame_size = config.telemetry.frame_size;
//! ```
//!
//! ## Features
//!
//! - **no_std + alloc**: Core API `from_toml_str(&str)` for embedded targets
//! - **std** (optional): `load_config_from_path()` for desktop/SITL tools

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

mod parser;
mod types;
mod validation;

pub use parser::from_toml_str;
pub use types::*;
pub use validation::validate;

// Convenience for desktop/SITL tools (std only)
#[cfg(feature = "std")]
pub fn load_config_from_path(path: &std::path::Path) -> Result<AppConfig, ConfigError> {
    let content = std::fs::read_to_string(path).map_err(|_| ConfigError::IoError)?;
    from_toml_str(&content)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_sitl_config() {
        let toml = r#"
[app]
id = "test-sitl"
board = "sitl-gazebo"
airframe = "x500"
env = "sitl"

[telemetry]
frame_size = 280
queue_len = 32

[security]
profile = "none"

[[transports]]
protocol = "mavlink"
port = "udp"
roles = ["telemetry", "command"]
port_sensor = 14560
port_actuator = 14561

[simulator]
backend = "gazebo"
headless = false
lockstep = false
"#;

        let config = from_toml_str(toml).expect("failed to parse config");
        assert_eq!(config.app.id, "test-sitl");
        assert_eq!(config.app.board, "sitl-gazebo");
        assert_eq!(config.app.env, "sitl");

        validate(&config).expect("validation failed");
    }

    #[test]
    fn test_validate_invalid_env() {
        let toml = r#"
[app]
id = "test"
board = "test-board"
airframe = "test"
env = "invalid"
"#;

        let config = from_toml_str(toml).expect("parse failed");
        assert!(validate(&config).is_err());
    }

    #[test]
    fn test_validate_invalid_transport_role() {
        let toml = r#"
[app]
id = "test"
board = "test-board"
airframe = "test"
env = "sitl"

[[transports]]
protocol = "mavlink"
port = "udp"
roles = ["invalid_role"]
"#;

        let config = from_toml_str(toml).expect("parse failed");
        let result = validate(&config);
        assert!(result.is_err(), "Expected validation to fail for invalid transport role, but it succeeded. Transports: {:?}", config.transports);
    }
}
