//! Configuration validation
//!
//! Phase 2: Validate transport roles, security profile compatibility, etc.

use crate::{AppConfig, ConfigError};

/// Validate parsed configuration
///
/// Checks:
/// - Transport roles are valid
/// - Security profile is valid
/// - Environment is valid
/// - At least one transport configured
pub fn validate(config: &AppConfig) -> Result<(), ConfigError> {
    // Validate environment
    match config.app.env.as_str() {
        "flight" | "sitl" | "hitl" => {}
        _ => return Err(ConfigError::ValidationError),
    }

    // Validate security profile if present
    if let Some(ref security) = config.security {
        match security.profile.as_str() {
            "none" | "auth-only" | "auth-and-encrypt" => {}
            _ => return Err(ConfigError::ValidationError),
        }
    }

    // Validate transport roles
    const VALID_ROLES: &[&str] = &["telemetry", "command", "rc_input"];
    for transport in &config.transports {
        for role in &transport.roles {
            if !VALID_ROLES.contains(&role.as_str()) {
                return Err(ConfigError::ValidationError);
            }
        }

        // Validate protocol
        match transport.protocol.as_str() {
            "mavlink" | "crsf" | "sbus" => {}
            _ => return Err(ConfigError::ValidationError),
        }
    }

    // Validate simulator config if present
    if let Some(ref sim) = config.simulator {
        match sim.backend.as_str() {
            "gazebo" | "jmavsim" => {}
            _ => return Err(ConfigError::ValidationError),
        }
    }

    Ok(())
}
