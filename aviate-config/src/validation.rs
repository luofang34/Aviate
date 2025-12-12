//! Configuration validation
//!
//! Phase 1: Stub (validation deferred to Phase 2)
//! Phase 2: Validate transport roles, security profile compatibility, etc.

use crate::{AppConfig, ConfigError};

/// Validate parsed configuration
///
/// Phase 1: No-op (always succeeds)
/// Phase 2: Check transport roles, security profile, etc.
pub fn validate(_config: &AppConfig) -> Result<(), ConfigError> {
    // TODO Phase 2: Add validation logic
    // - Check transport roles are valid ("telemetry", "command", "rc_input")
    // - Check security profile is valid
    // - Check port names match board capabilities
    Ok(())
}
