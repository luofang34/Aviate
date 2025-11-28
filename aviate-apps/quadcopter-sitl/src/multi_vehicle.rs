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

use std::sync::Arc;
use std::time::Duration;

use parking_lot::RwLock;
use tokio_util::sync::CancellationToken;

use crate::router_gen::{GCS_PORT, VEHICLE_BASE_PORT};
use crate::test_config::TestConfig;

/// Start mavrouter with auto-generated config from test configuration
pub async fn start_router(config: &TestConfig) -> Result<RouterHandle, RouterError> {
    // Generate config content
    let router_toml = crate::router_gen::generate_router_config(
        config,
        &crate::router_gen::RouterParams::default(),
    );

    // Parse into mavrouter config
    let mav_config = parse_router_config(&router_toml)?;

    // Create message bus
    let bus = mavrouter::router::create_bus(mav_config.general.bus_capacity);

    // Shared routing table and deduplication
    let routing_table = Arc::new(RwLock::new(mavrouter::routing::RoutingTable::new()));
    let dedup_period = mav_config.general.dedup_period_ms.unwrap_or(100);
    let dedup = mavrouter::dedup::ConcurrentDedup::new(Duration::from_millis(dedup_period));

    // Start endpoints
    let cancel_token = CancellationToken::new();
    let mut handles = Vec::new();

    for (id, endpoint_cfg) in mav_config.endpoint.iter().enumerate() {
        let handle = start_endpoint(
            id,
            endpoint_cfg,
            &bus,
            routing_table.clone(),
            dedup.clone(),
            cancel_token.clone(),
            mav_config.general.routing_table_ttl_secs,
        ).await?;
        handles.push(handle);
    }

    Ok(RouterHandle {
        cancel_token,
        handles,
    })
}

/// Handle to a running mavrouter instance
pub struct RouterHandle {
    cancel_token: tokio_util::sync::CancellationToken,
    handles: Vec<tokio::task::JoinHandle<()>>,
}

impl RouterHandle {
    /// Stop the router gracefully
    pub async fn stop(self) {
        self.cancel_token.cancel();
        for handle in self.handles {
            let _ = handle.await;
        }
    }

    /// Get GCS port
    pub fn gcs_port(&self) -> u16 {
        GCS_PORT
    }

    /// Get vehicle port for instance
    pub fn vehicle_port(&self, instance: u8) -> u16 {
        VEHICLE_BASE_PORT + instance as u16
    }
}

/// Router errors
#[derive(Debug)]
pub enum RouterError {
    Config(String),
    Endpoint(String),
    Io(std::io::Error),
}

impl std::fmt::Display for RouterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RouterError::Config(e) => write!(f, "Config error: {}", e),
            RouterError::Endpoint(e) => write!(f, "Endpoint error: {}", e),
            RouterError::Io(e) => write!(f, "IO error: {}", e),
        }
    }
}

impl std::error::Error for RouterError {}

impl From<std::io::Error> for RouterError {
    fn from(e: std::io::Error) -> Self {
        RouterError::Io(e)
    }
}

// Internal: Parse TOML string into mavrouter config
fn parse_router_config(toml_str: &str) -> Result<mavrouter::config::Config, RouterError> {
    // mavrouter::config::Config::from_str expects async file reading,
    // so we need to parse manually or use a temp file
    let temp_path = std::env::temp_dir().join("aviate_router_temp.toml");
    std::fs::write(&temp_path, toml_str)?;

    // Use blocking file read since we're in sync context
    let rt = tokio::runtime::Handle::current();
    let config = rt.block_on(async {
        mavrouter::config::Config::load(&temp_path).await
    }).map_err(|e| RouterError::Config(format!("{}", e)))?;

    // Clean up temp file
    let _ = std::fs::remove_file(&temp_path);

    Ok(config)
}

// Internal: Start a single endpoint based on its type
async fn start_endpoint(
    id: usize,
    cfg: &mavrouter::config::EndpointConfig,
    bus: &mavrouter::router::MessageBus,
    routing_table: Arc<RwLock<mavrouter::routing::RoutingTable>>,
    dedup: mavrouter::dedup::ConcurrentDedup,
    cancel: CancellationToken,
    routing_table_ttl_secs: u64,
) -> Result<tokio::task::JoinHandle<()>, RouterError> {
    // Get sender and subscribe to bus for this endpoint
    let bus_tx = bus.sender();
    let bus_rx = bus.subscribe();

    match cfg {
        mavrouter::config::EndpointConfig::Udp { address, mode, filters } => {
            let address = address.clone();
            let mode = mode.clone();
            let filters = filters.clone();

            Ok(tokio::spawn(async move {
                if let Err(e) = mavrouter::endpoints::udp::run(
                    id,
                    address,
                    mode,
                    bus_tx,
                    bus_rx,
                    routing_table,
                    dedup,
                    filters,
                    cancel,
                    routing_table_ttl_secs,
                ).await {
                    eprintln!("UDP endpoint {} error: {}", id, e);
                }
            }))
        }
        _ => {
            // For now, only UDP endpoints are supported for SITL
            Err(RouterError::Endpoint(
                "Unsupported endpoint type for SITL (only UDP supported)".to_string()
            ))
        }
    }
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
