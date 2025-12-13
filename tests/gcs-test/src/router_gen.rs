//! MAVRouter Configuration Generator
//!
//! Generates mavrouter TOML configuration from test configuration.
//! Each vehicle instance gets a unique UDP port for MAVLink communication:
//!   - Vehicle instance N connects on port from XilNetConfig (default base=20000, stride=16)
//!   - GCS connects to mavrouter on standard port 14550
//!
//! Port allocation uses XilNetConfig:
//!   - 14550: GCS endpoint (server, accepts GCS connections)
//!   - XilNetConfig.port(instance, SensorIn): Vehicle endpoints (client, connects to each SITL)

#![allow(dead_code)] // Only used with gazebo feature

use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use aviate_hal_xil::{PortSlot, TestConfig, XilNetConfig};

/// GCS MAVLink port (standard)
pub const GCS_PORT: u16 = 14550;

/// Router configuration parameters
#[derive(Debug, Clone)]
pub struct RouterParams {
    /// Bus capacity for message buffering
    pub bus_capacity: u32,
    /// Message deduplication period in milliseconds
    pub dedup_period_ms: u32,
    /// Routing table TTL in seconds
    pub routing_table_ttl_secs: u32,
    /// Routing table prune interval in seconds
    pub routing_table_prune_interval_secs: u32,
}

impl Default for RouterParams {
    fn default() -> Self {
        Self {
            bus_capacity: 1000,
            dedup_period_ms: 50,
            routing_table_ttl_secs: 60,
            routing_table_prune_interval_secs: 30,
        }
    }
}

/// Generate mavrouter TOML configuration from test configuration
pub fn generate_router_config(config: &TestConfig, params: &RouterParams) -> String {
    generate_router_config_with_net(config, params, XilNetConfig::default())
}

/// Generate mavrouter TOML configuration with custom network settings
pub fn generate_router_config_with_net(
    config: &TestConfig,
    params: &RouterParams,
    net: XilNetConfig,
) -> String {
    let mut toml = String::new();

    // Header comment
    writeln!(toml, "# Auto-generated mavrouter configuration").unwrap();
    writeln!(toml, "# Test: {}", config.name).unwrap();
    writeln!(toml, "# Vehicles: {}", config.vehicles.len()).unwrap();
    writeln!(toml, "#").unwrap();
    writeln!(toml, "# Architecture:").unwrap();
    writeln!(
        toml,
        "#   GCS <--> mavrouter (port {}) <--> Vehicle endpoints",
        GCS_PORT
    )
    .unwrap();
    for vehicle in &config.vehicles {
        let port = net.port(vehicle.instance as u16, PortSlot::SensorIn);
        writeln!(
            toml,
            "#     {} (instance {}) on port {}",
            vehicle.id, vehicle.instance, port
        )
        .unwrap();
    }
    writeln!(toml).unwrap();

    // General section
    writeln!(toml, "[general]").unwrap();
    writeln!(toml, "bus_capacity = {}", params.bus_capacity).unwrap();
    writeln!(toml, "dedup_period_ms = {}", params.dedup_period_ms).unwrap();
    writeln!(
        toml,
        "routing_table_ttl_secs = {}",
        params.routing_table_ttl_secs
    )
    .unwrap();
    writeln!(
        toml,
        "routing_table_prune_interval_secs = {}",
        params.routing_table_prune_interval_secs
    )
    .unwrap();
    writeln!(toml).unwrap();

    // GCS endpoint (server mode - accepts connections)
    writeln!(toml, "# GCS endpoint - Python/QGC connects here").unwrap();
    writeln!(toml, "[[endpoint]]").unwrap();
    writeln!(toml, "type = \"udp\"").unwrap();
    writeln!(toml, "address = \"0.0.0.0:{}\"", GCS_PORT).unwrap();
    writeln!(toml, "mode = \"server\"").unwrap();
    writeln!(toml).unwrap();

    // Vehicle endpoints (client mode - connects to SITL instances)
    for vehicle in &config.vehicles {
        let port = net.port(vehicle.instance as u16, PortSlot::SensorIn);
        writeln!(
            toml,
            "# {} (system_id = {})",
            vehicle.id,
            vehicle.instance + 1
        )
        .unwrap();
        writeln!(toml, "[[endpoint]]").unwrap();
        writeln!(toml, "type = \"udp\"").unwrap();
        writeln!(toml, "address = \"127.0.0.1:{}\"", port).unwrap();
        writeln!(toml, "mode = \"client\"").unwrap();
        writeln!(toml).unwrap();
    }

    toml
}

/// Generate router config and write to disk
pub fn generate_router_config_file(
    config: &TestConfig,
    params: &RouterParams,
    output_path: &Path,
) -> Result<(), String> {
    let toml = generate_router_config(config, params);

    // Ensure parent directory exists
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("Failed to create directory: {}", e))?;
    }

    fs::write(output_path, &toml).map_err(|e| format!("Failed to write router config: {}", e))?;

    Ok(())
}

/// Generate router config to a temporary location
pub fn generate_temp_router_config(
    config: &TestConfig,
    params: &RouterParams,
) -> Result<std::path::PathBuf, String> {
    let temp_dir = std::env::temp_dir();
    let filename = format!("aviate_router_{}.toml", config.name);
    let path = temp_dir.join(filename);

    generate_router_config_file(config, params, &path)?;
    Ok(path)
}

/// Get the UDP port for a vehicle instance using default XilNetConfig
pub fn vehicle_port(instance: u8) -> u16 {
    XilNetConfig::default().port(instance as u16, PortSlot::SensorIn)
}

#[cfg(test)]
mod tests {
    use super::*;
    use aviate_hal_xil::parse_test_config_str;

    #[test]
    fn test_generate_single_vehicle_router() {
        let config_str = r#"
[test]
name = "single_vehicle"
description = "Single vehicle test"
lockstep = true

[world]
file = "generated"

[[vehicles]]
id = "x500_0"
model = "x500"
instance = 0
spawn_position = [0.0, 0.0, 0.0]

[vehicles.mission]
name = "test"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = []
"#;
        let config = parse_test_config_str(config_str).unwrap();
        let params = RouterParams::default();
        let toml = generate_router_config(&config, &params);

        // Should have GCS endpoint
        assert!(toml.contains("address = \"0.0.0.0:14550\""));
        assert!(toml.contains("mode = \"server\""));

        // Should have vehicle endpoint (base port = 20000 for instance 0)
        assert!(toml.contains("address = \"127.0.0.1:20000\""));
        assert!(toml.contains("mode = \"client\""));

        // Should have general config
        assert!(toml.contains("bus_capacity = 1000"));
    }

    #[test]
    fn test_generate_two_vehicle_router() {
        let config_str = r#"
[test]
name = "two_vehicle"
description = "Two vehicle test"
lockstep = true

[world]
file = "generated"

[[vehicles]]
id = "x500_0"
model = "x500"
instance = 0
spawn_position = [0.0, 0.0, 0.0]

[vehicles.mission]
name = "leader"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = []

[[vehicles]]
id = "x500_1"
model = "x500"
instance = 1
spawn_position = [5.0, 0.0, 0.0]

[vehicles.mission]
name = "follower"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = []
"#;
        let config = parse_test_config_str(config_str).unwrap();
        let params = RouterParams::default();
        let toml = generate_router_config(&config, &params);

        // Should have GCS endpoint
        assert!(toml.contains("address = \"0.0.0.0:14550\""));

        // Should have two vehicle endpoints with correct ports
        // base=20000, stride=16: instance 0 = 20000, instance 1 = 20016
        assert!(toml.contains("address = \"127.0.0.1:20000\"")); // instance 0
        assert!(toml.contains("address = \"127.0.0.1:20016\"")); // instance 1

        // Should have comments identifying vehicles
        assert!(toml.contains("x500_0"));
        assert!(toml.contains("x500_1"));
    }

    #[test]
    fn test_vehicle_port() {
        // base=20000, stride=16
        assert_eq!(vehicle_port(0), 20000);
        assert_eq!(vehicle_port(1), 20016);
        assert_eq!(vehicle_port(2), 20032);
    }
}
