//! Test Configuration Parser
//!
//! This module parses TOML test configuration files into mission structures.

use std::fs;
use std::path::Path;
use std::time::Duration;

use crate::mission::{Action, Criterion, Mission, Phase, VehicleConfig};

/// Parsed test configuration
#[derive(Debug, Clone)]
pub struct TestConfig {
    pub name: String,
    pub description: String,
    pub lockstep: bool,
    pub world_file: String,
    pub vehicles: Vec<VehicleTestConfig>,
    pub global_verification: Option<GlobalVerification>,
}

/// Vehicle-specific test configuration
#[derive(Debug, Clone)]
pub struct VehicleTestConfig {
    pub id: String,
    pub model: String,
    pub instance: u8,
    pub spawn_position: [f32; 3],
    pub spawn_heading: f32,
    pub mission: Mission,
}

/// Global verification criteria (checked after all vehicles complete)
#[derive(Debug, Clone)]
pub struct GlobalVerification {
    pub min_separation: Option<f32>,
}

/// Parse a test configuration from a TOML file
pub fn parse_test_config(path: &Path) -> Result<TestConfig, String> {
    let content = fs::read_to_string(path)
        .map_err(|e| format!("Failed to read config file: {}", e))?;

    parse_test_config_str(&content)
}

/// Parse a test configuration from a TOML string
pub fn parse_test_config_str(content: &str) -> Result<TestConfig, String> {
    // Simple TOML parser (basic implementation without external dependency)
    // For production, consider using the `toml` crate

    let mut config = TestConfig {
        name: String::new(),
        description: String::new(),
        lockstep: true,
        world_file: String::new(),
        vehicles: Vec::new(),
        global_verification: None,
    };

    let mut current_section = String::new();
    let mut current_vehicle: Option<VehicleTestConfig> = None;
    let mut current_phases: Vec<Phase> = Vec::new();
    let mut current_phase: Option<PhaseBuilder> = None;

    for line in content.lines() {
        let line = line.trim();

        // Skip comments and empty lines
        if line.is_empty() || line.starts_with('#') {
            continue;
        }

        // Section headers
        if line.starts_with("[[") && line.ends_with("]]") {
            // Array table (e.g., [[vehicles]], [[vehicles.mission.phases]])
            let section = &line[2..line.len()-2];

            // Save any pending phase
            if let Some(phase) = current_phase.take() {
                current_phases.push(phase.build());
            }

            if section == "vehicles" {
                // Save previous vehicle
                if let Some(mut vehicle) = current_vehicle.take() {
                    vehicle.mission.phases = std::mem::take(&mut current_phases);
                    config.vehicles.push(vehicle);
                }

                // Start new vehicle
                current_vehicle = Some(VehicleTestConfig {
                    id: String::new(),
                    model: "x500".to_string(),
                    instance: 0,
                    spawn_position: [0.0, 0.0, 0.0],
                    spawn_heading: 0.0,
                    mission: Mission {
                        name: String::new(),
                        description: String::new(),
                        vehicle: VehicleConfig::default(),
                        lockstep: true,
                        phases: Vec::new(),
                        reset_between_runs: true,
                    },
                });
            } else if section == "vehicles.mission.phases" {
                current_phase = Some(PhaseBuilder::new());
            }

            current_section = section.to_string();
            continue;
        }

        if line.starts_with('[') && line.ends_with(']') {
            // Regular table
            let section = &line[1..line.len()-1];

            // Save any pending phase
            if let Some(phase) = current_phase.take() {
                current_phases.push(phase.build());
            }

            current_section = section.to_string();
            continue;
        }

        // Key-value pairs
        if let Some((key, value)) = parse_kv(line) {
            match current_section.as_str() {
                "test" => {
                    match key.as_str() {
                        "name" => config.name = value,
                        "description" => config.description = value,
                        "lockstep" => config.lockstep = value == "true",
                        _ => {}
                    }
                }
                "world" => {
                    match key.as_str() {
                        "file" => config.world_file = value,
                        _ => {}
                    }
                }
                "vehicles" => {
                    if let Some(ref mut vehicle) = current_vehicle {
                        match key.as_str() {
                            "id" => vehicle.id = value,
                            "model" => vehicle.model = value.clone(),
                            "instance" => vehicle.instance = value.parse().unwrap_or(0),
                            "spawn_position" => vehicle.spawn_position = parse_vec3(&value),
                            "spawn_heading" => vehicle.spawn_heading = value.parse().unwrap_or(0.0),
                            _ => {}
                        }
                    }
                }
                "vehicles.mission" => {
                    if let Some(ref mut vehicle) = current_vehicle {
                        match key.as_str() {
                            "name" => vehicle.mission.name = value,
                            _ => {}
                        }
                    }
                }
                "vehicles.mission.phases" => {
                    if let Some(ref mut phase) = current_phase {
                        match key.as_str() {
                            "name" => phase.name = value,
                            "duration_ms" => phase.duration_ms = value.parse().unwrap_or(1000),
                            "action" => phase.action_str = value,
                            "verify" => phase.verify_str = value,
                            _ => {}
                        }
                    }
                }
                "verification" => {
                    let verif = config.global_verification.get_or_insert(GlobalVerification {
                        min_separation: None,
                    });
                    match key.as_str() {
                        "min_separation" => verif.min_separation = value.parse().ok(),
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }

    // Save final phase
    if let Some(phase) = current_phase.take() {
        current_phases.push(phase.build());
    }

    // Save final vehicle
    if let Some(mut vehicle) = current_vehicle.take() {
        vehicle.mission.phases = current_phases;
        vehicle.mission.lockstep = config.lockstep;
        config.vehicles.push(vehicle);
    }

    Ok(config)
}

/// Parse a key-value pair from a line
fn parse_kv(line: &str) -> Option<(String, String)> {
    let parts: Vec<&str> = line.splitn(2, '=').collect();
    if parts.len() != 2 {
        return None;
    }

    let key = parts[0].trim().to_string();
    let mut value = parts[1].trim().to_string();

    // Remove quotes
    if value.starts_with('"') && value.ends_with('"') {
        value = value[1..value.len()-1].to_string();
    }

    Some((key, value))
}

/// Parse a [x, y, z] vector
fn parse_vec3(s: &str) -> [f32; 3] {
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');
    let parts: Vec<f32> = s.split(',')
        .map(|p| p.trim().parse().unwrap_or(0.0))
        .collect();

    [
        parts.get(0).copied().unwrap_or(0.0),
        parts.get(1).copied().unwrap_or(0.0),
        parts.get(2).copied().unwrap_or(0.0),
    ]
}

/// Builder for phases while parsing
struct PhaseBuilder {
    name: String,
    duration_ms: u64,
    action_str: String,
    verify_str: String,
}

impl PhaseBuilder {
    fn new() -> Self {
        Self {
            name: String::new(),
            duration_ms: 1000,
            action_str: String::new(),
            verify_str: String::new(),
        }
    }

    fn build(self) -> Phase {
        Phase {
            name: self.name,
            duration: Duration::from_millis(self.duration_ms),
            action: parse_action(&self.action_str),
            verify: parse_criteria(&self.verify_str),
        }
    }
}

/// Parse an action from TOML inline table format
fn parse_action(s: &str) -> Action {
    // Simple parsing of { type = "...", value = ..., ... }
    let s = s.trim().trim_start_matches('{').trim_end_matches('}');
    let mut type_str = String::new();
    let mut value: f32 = 0.0;
    let mut thrust: f32 = 0.0;
    let mut q: [f32; 4] = [1.0, 0.0, 0.0, 0.0]; // identity quaternion
    let mut position: [f32; 3] = [0.0, 0.0, 0.0];
    let mut heading: f32 = 0.0;

    for part in s.split(',') {
        if let Some((k, v)) = parse_kv(part) {
            match k.as_str() {
                "type" => type_str = v,
                "value" => value = v.parse().unwrap_or(0.0),
                "thrust" => thrust = v.parse().unwrap_or(0.0),
                "q" => q = parse_quat(&v),
                "position" => position = parse_vec3(&v),
                "heading" => heading = v.parse().unwrap_or(0.0),
                _ => {}
            }
        }
    }

    match type_str.as_str() {
        "arm" => Action::Arm,
        "disarm" => Action::Disarm,
        "thrust" => Action::Thrust(value),
        "wait" => Action::Wait,
        "attitude_target" => Action::AttitudeTarget { q, thrust },
        "goto" => Action::GoTo { position, heading },
        _ => Action::Wait,
    }
}

/// Parse a quaternion [w, x, y, z]
fn parse_quat(s: &str) -> [f32; 4] {
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');
    let parts: Vec<f32> = s.split(',')
        .map(|p| p.trim().parse().unwrap_or(0.0))
        .collect();

    [
        parts.first().copied().unwrap_or(1.0), // w defaults to 1 (identity)
        parts.get(1).copied().unwrap_or(0.0),
        parts.get(2).copied().unwrap_or(0.0),
        parts.get(3).copied().unwrap_or(0.0),
    ]
}

/// Parse criteria from TOML array format
fn parse_criteria(s: &str) -> Vec<Criterion> {
    if s.is_empty() || s == "[]" {
        return Vec::new();
    }

    let mut criteria = Vec::new();

    // Simple parsing of [{ type = "...", value = ... }, ...]
    let s = s.trim().trim_start_matches('[').trim_end_matches(']');

    for item in s.split("},") {
        let item = item.trim().trim_start_matches('{').trim_end_matches('}');
        let mut type_str = String::new();
        let mut value: f32 = 0.0;
        let mut bool_value = false;
        let mut target: f32 = 0.0;
        let mut tolerance: f32 = 1.0;
        let mut position: [f32; 3] = [0.0, 0.0, 0.0];

        for part in item.split(',') {
            if let Some((k, v)) = parse_kv(part) {
                match k.as_str() {
                    "type" => type_str = v,
                    "value" => {
                        if v == "true" {
                            bool_value = true;
                        } else if v == "false" {
                            bool_value = false;
                        } else {
                            value = v.parse().unwrap_or(0.0);
                        }
                    }
                    "target" => target = v.parse().unwrap_or(0.0),
                    "tolerance" => tolerance = v.parse().unwrap_or(1.0),
                    "position" => position = parse_vec3(&v),
                    _ => {}
                }
            }
        }

        let criterion = match type_str.as_str() {
            "armed" => Criterion::Armed(bool_value),
            "min_altitude" => Criterion::MinAltitude(value),
            "max_altitude" => Criterion::MaxAltitude(value),
            "max_drift" => Criterion::MaxDrift(value),
            "altitude_hold" => Criterion::AltitudeHold { target, tolerance },
            "position_hold" => Criterion::PositionHold { target: position, tolerance },
            "sensor_data" => Criterion::SensorDataReceived,
            _ => continue,
        };

        criteria.push(criterion);
    }

    criteria
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_basic_config() {
        let config_str = r#"
[test]
name = "basic_test"
description = "A test"
lockstep = true

[world]
file = "test.sdf"

[[vehicles]]
id = "x500_0"
model = "x500"
instance = 0
spawn_position = [1.0, 2.0, 3.0]

[vehicles.mission]
name = "test_mission"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = [{ type = "armed", value = true }]
"#;

        let config = parse_test_config_str(config_str).unwrap();
        assert_eq!(config.name, "basic_test");
        assert_eq!(config.vehicles.len(), 1);
        assert_eq!(config.vehicles[0].id, "x500_0");
        assert_eq!(config.vehicles[0].spawn_position, [1.0, 2.0, 3.0]);
        assert_eq!(config.vehicles[0].mission.phases.len(), 1);
    }

    #[test]
    fn test_parse_two_vehicle_formation() {
        let config_str = r#"
[test]
name = "two_vehicle_formation"
description = "Two vehicles takeoff in formation and verify separation"
lockstep = true

[world]
file = "worlds/x500_two_vehicle_lockstep.sdf"

[[vehicles]]
id = "x500_0"
model = "x500"
instance = 0
spawn_position = [0.0, 0.0, 0.0]
spawn_heading = 0.0

[vehicles.mission]
name = "leader_takeoff"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = [{ type = "armed", value = true }]

[[vehicles.mission.phases]]
name = "takeoff"
duration_ms = 5000
action = { type = "thrust", value = 0.8 }
verify = [{ type = "min_altitude", value = 5.0 }]

[[vehicles.mission.phases]]
name = "hover"
duration_ms = 5000
action = { type = "thrust", value = 0.65 }
verify = [{ type = "max_drift", value = 2.0 }]

[[vehicles.mission.phases]]
name = "land"
duration_ms = 5000
action = { type = "thrust", value = 0.0 }
verify = [{ type = "max_altitude", value = 0.5 }]

[[vehicles.mission.phases]]
name = "disarm"
duration_ms = 500
action = { type = "disarm" }
verify = [{ type = "armed", value = false }]

[[vehicles]]
id = "x500_1"
model = "x500"
instance = 1
spawn_position = [5.0, 0.0, 0.0]
spawn_heading = 0.0

[vehicles.mission]
name = "follower_takeoff"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = [{ type = "armed", value = true }]

[[vehicles.mission.phases]]
name = "takeoff"
duration_ms = 5000
action = { type = "thrust", value = 0.8 }
verify = [{ type = "min_altitude", value = 5.0 }]

[verification]
min_separation = 4.0
"#;

        let config = parse_test_config_str(config_str).unwrap();
        assert_eq!(config.name, "two_vehicle_formation");
        assert!(config.lockstep);
        assert_eq!(config.world_file, "worlds/x500_two_vehicle_lockstep.sdf");

        // Two vehicles
        assert_eq!(config.vehicles.len(), 2);

        // Vehicle 0 (leader)
        assert_eq!(config.vehicles[0].id, "x500_0");
        assert_eq!(config.vehicles[0].instance, 0);
        assert_eq!(config.vehicles[0].spawn_position, [0.0, 0.0, 0.0]);
        assert_eq!(config.vehicles[0].mission.name, "leader_takeoff");
        assert_eq!(config.vehicles[0].mission.phases.len(), 5);

        // Vehicle 1 (follower)
        assert_eq!(config.vehicles[1].id, "x500_1");
        assert_eq!(config.vehicles[1].instance, 1);
        assert_eq!(config.vehicles[1].spawn_position, [5.0, 0.0, 0.0]);
        assert_eq!(config.vehicles[1].mission.name, "follower_takeoff");
        assert_eq!(config.vehicles[1].mission.phases.len(), 2);

        // Global verification
        assert!(config.global_verification.is_some());
        assert_eq!(config.global_verification.unwrap().min_separation, Some(4.0));
    }
}
