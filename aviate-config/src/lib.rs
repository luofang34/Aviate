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

mod types;
mod parser;
mod validation;

pub use types::*;
pub use parser::from_toml_str;
pub use validation::validate;

// Convenience for desktop/SITL tools (std only)
#[cfg(feature = "std")]
pub fn load_config_from_path(path: &std::path::Path) -> Result<AppConfig, ConfigError> {
    let content = std::fs::read_to_string(path)
        .map_err(|_| ConfigError::IoError)?;
    from_toml_str(&content)
}
