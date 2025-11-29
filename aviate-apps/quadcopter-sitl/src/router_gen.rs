//! MAVRouter Configuration Generator
//!
//! Generates mavrouter TOML configuration from test configuration.
//! Each vehicle instance gets a unique UDP port for MAVLink communication:
//!   - Vehicle instance N connects on port BASE_PORT + N
//!   - GCS connects to mavrouter on standard port 14550
//!
//! Port allocation:
//!   - 14550: GCS endpoint (server, accepts GCS connections)
//!   - 14560 + instance: Vehicle endpoints (client, connects to each SITL)

use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use crate::test_config::TestConfig;

/// Base port for vehicle MAVLink connections
pub const VEHICLE_BASE_PORT: u16 = 14560;

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
        let port = VEHICLE_BASE_PORT + vehicle.instance as u16;
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
        let port = VEHICLE_BASE_PORT + vehicle.instance as u16;
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

/// Get the UDP port for a vehicle instance
pub fn vehicle_port(instance: u8) -> u16 {
    VEHICLE_BASE_PORT + instance as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_config::parse_test_config_str;

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

        // Should have vehicle endpoint
        assert!(toml.contains("address = \"127.0.0.1:14560\""));
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
        assert!(toml.contains("address = \"127.0.0.1:14560\"")); // instance 0
        assert!(toml.contains("address = \"127.0.0.1:14561\"")); // instance 1

        // Should have comments identifying vehicles
        assert!(toml.contains("x500_0"));
        assert!(toml.contains("x500_1"));
    }

    #[test]
    fn test_vehicle_port() {
        assert_eq!(vehicle_port(0), 14560);
        assert_eq!(vehicle_port(1), 14561);
        assert_eq!(vehicle_port(2), 14562);
    }
}
