//! TOML parsing implementation
//!
//! Phase 1: Basic parsing
//! Phase 2: Add validation hooks

use crate::{AppConfig, ConfigError};

/// Parse TOML config from string
///
/// This function is **LOW-DAL ONLY** - call once during startup, never in control loop.
///
/// # Example
///
/// ```ignore
/// const CONFIG: &str = include_str!("../AviateApp.toml");
/// let config = from_toml_str(CONFIG)?;
/// ```
pub fn from_toml_str(content: &str) -> Result<AppConfig, ConfigError> {
    toml::from_str(content).map_err(|_| ConfigError::ParseError)
}
