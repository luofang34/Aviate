//! Multi-vehicle routing using mavrouter crate
//!
//! Provides in-process MAVLink routing for multi-vehicle SITL tests.
//! This avoids spawning an external mavrouter process.
//!
//! Architecture:
//! ```text
//! GCS (port 14550) <--> mavrouter <--> Vehicle 1 (port 14560)
//!                                  <--> Vehicle 2 (port 14561)
//!                                  <--> Vehicle N (port 14560+N)
//! ```

use crate::router_gen::{GCS_PORT, VEHICLE_BASE_PORT};
use crate::test_config::TestConfig;

// Re-export from mavrouter for convenience
pub use mavrouter::CancellationToken;

/// Handle to a running mavrouter instance
pub struct RouterHandle {
    router: mavrouter::Router,
}

impl RouterHandle {
    /// Stop the router gracefully
    pub async fn stop(self) {
        self.router.stop().await;
    }

    /// Get GCS port
    pub fn gcs_port(&self) -> u16 {
        GCS_PORT
    }

    /// Get vehicle port for instance
    pub fn vehicle_port(&self, instance: u8) -> u16 {
        VEHICLE_BASE_PORT + instance as u16
    }

    /// Check if router is still running
    pub fn is_running(&self) -> bool {
        self.router.is_running()
    }

    /// Get cancellation token for external shutdown control
    pub fn cancel_token(&self) -> CancellationToken {
        self.router.cancel_token()
    }
}

/// Router errors
#[derive(Debug)]
pub enum RouterError {
    Config(String),
    Start(String),
}

impl std::fmt::Display for RouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RouterError::Config(e) => write!(f, "Config error: {}", e),
            RouterError::Start(e) => write!(f, "Start error: {}", e),
        }
    }
}

impl std::error::Error for RouterError {}

impl From<mavrouter::error::RouterError> for RouterError {
    fn from(e: mavrouter::error::RouterError) -> Self {
        RouterError::Start(e.to_string())
    }
}

/// Start mavrouter with auto-generated config from test configuration
pub async fn start_router(config: &TestConfig) -> Result<RouterHandle, RouterError> {
    // Generate config content
    let router_toml = crate::router_gen::generate_router_config(
        config,
        &crate::router_gen::RouterParams::default(),
    );

    // Use the new high-level Router API from mavrouter 0.1.4
    let router = mavrouter::Router::from_str(&router_toml).await?;

    Ok(RouterHandle { router })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_router_ports() {
        assert_eq!(GCS_PORT, 14550);
        assert_eq!(VEHICLE_BASE_PORT, 14560);
        assert_eq!(VEHICLE_BASE_PORT + 1, 14561);
    }
}
