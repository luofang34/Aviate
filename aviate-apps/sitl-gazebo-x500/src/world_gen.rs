//! World Generator for SITL Testing
//!
//! Generates Gazebo SDF world files from test configuration parameters.
//! This allows tests to define vehicles and world settings in TOML,
//! and have the world file generated automatically at runtime.

use std::fmt::Write as FmtWrite;
use std::fs;
use std::path::Path;

use aviate_hal_xil::{TestConfig, VehicleTestConfig};

/// World generation parameters
#[derive(Debug, Clone)]
pub struct WorldParams {
    /// World name
    pub name: String,
    /// Physics step size in seconds
    pub step_size: f64,
    /// Real-time factor (1.0 = real-time)
    pub real_time_factor: f64,
    /// Enable lockstep synchronization
    pub lockstep: bool,
    /// Lockstep timeout in microseconds
    pub lockstep_timeout_us: u32,
    /// GPS origin latitude
    pub latitude: f64,
    /// GPS origin longitude
    pub longitude: f64,
    /// GPS origin elevation
    pub elevation: f64,
}

impl Default for WorldParams {
    fn default() -> Self {
        Self {
            name: "aviate_sitl".to_string(),
            step_size: 0.001,
            real_time_factor: 1.0,
            lockstep: true,
            lockstep_timeout_us: 50000,
            latitude: 47.3977419, // Zurich
            longitude: 8.5455938,
            elevation: 488.0,
        }
    }
}

/// Generate a complete SDF world file from test configuration
pub fn generate_world(config: &TestConfig, params: &WorldParams) -> String {
    let mut sdf = String::new();

    // XML header
    writeln!(sdf, r#"<?xml version="1.0" ?>"#).unwrap();
    writeln!(sdf, r#"<!--"#).unwrap();
    writeln!(sdf, r#"  Auto-generated world for test: {}"#, config.name).unwrap();
    writeln!(sdf, r#"  Description: {}"#, config.description).unwrap();
    writeln!(sdf, r#"  Vehicles: {}"#, config.vehicles.len()).unwrap();
    writeln!(sdf, r#"  Lockstep: {}"#, params.lockstep).unwrap();
    writeln!(sdf, r#"-->"#).unwrap();

    writeln!(sdf, r#"<sdf version="1.9">"#).unwrap();
    writeln!(sdf, r#"  <world name="{}">"#, params.name).unwrap();

    // Physics
    write_physics(&mut sdf, params);

    // System plugins
    write_system_plugins(&mut sdf);

    // Aviate bridge plugins (one per vehicle instance)
    write_aviate_plugins(&mut sdf, config, params);

    // Spherical coordinates (GPS origin)
    write_spherical_coords(&mut sdf, params);

    // Environment
    write_environment(&mut sdf);

    // Ground plane
    write_ground_plane(&mut sdf);

    // Vehicles
    for vehicle in &config.vehicles {
        write_vehicle(&mut sdf, vehicle);
    }

    writeln!(sdf, r#"  </world>"#).unwrap();
    writeln!(sdf, r#"</sdf>"#).unwrap();

    sdf
}

fn write_physics(sdf: &mut String, params: &WorldParams) {
    writeln!(sdf, r#"    <physics name="1ms" type="ignored">"#).unwrap();
    writeln!(
        sdf,
        r#"      <max_step_size>{}</max_step_size>"#,
        params.step_size
    )
    .unwrap();
    writeln!(
        sdf,
        r#"      <real_time_factor>{}</real_time_factor>"#,
        params.real_time_factor
    )
    .unwrap();
    writeln!(sdf, r#"    </physics>"#).unwrap();
    writeln!(sdf).unwrap();
}

fn write_system_plugins(sdf: &mut String) {
    writeln!(sdf, r#"    <!-- Required system plugins -->"#).unwrap();

    let plugins = [
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        (
            "gz-sim-scene-broadcaster-system",
            "gz::sim::systems::SceneBroadcaster",
        ),
        (
            "gz-sim-user-commands-system",
            "gz::sim::systems::UserCommands",
        ),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        (
            "gz-sim-magnetometer-system",
            "gz::sim::systems::Magnetometer",
        ),
        (
            "gz-sim-air-pressure-system",
            "gz::sim::systems::AirPressure",
        ),
        ("gz-sim-navsat-system", "gz::sim::systems::NavSat"),
    ];

    for (filename, name) in plugins {
        writeln!(sdf, r#"    <plugin filename="{filename}""#).unwrap();
        writeln!(sdf, r#"            name="{name}">"#).unwrap();
        writeln!(sdf, r#"    </plugin>"#).unwrap();
    }

    // Sensors plugin with render engine
    writeln!(sdf, r#"    <plugin filename="gz-sim-sensors-system""#).unwrap();
    writeln!(sdf, r#"            name="gz::sim::systems::Sensors">"#).unwrap();
    writeln!(sdf, r#"      <render_engine>ogre2</render_engine>"#).unwrap();
    writeln!(sdf, r#"    </plugin>"#).unwrap();
    writeln!(sdf).unwrap();
}

fn write_aviate_plugins(sdf: &mut String, config: &TestConfig, params: &WorldParams) {
    writeln!(sdf, r#"    <!-- Aviate bridge plugins -->"#).unwrap();

    for vehicle in &config.vehicles {
        writeln!(sdf, r#"    <plugin filename="AviateGzPlugin""#).unwrap();
        writeln!(sdf, r#"            name="aviate::AviateGzPlugin">"#).unwrap();
        writeln!(sdf, r#"      <model_name>{}</model_name>"#, vehicle.id).unwrap();
        writeln!(sdf, r#"      <instance>{}</instance>"#, vehicle.instance).unwrap();
        writeln!(sdf, r#"      <lockstep>{}</lockstep>"#, params.lockstep).unwrap();
        if params.lockstep {
            writeln!(
                sdf,
                r#"      <lockstep_timeout_us>{}</lockstep_timeout_us>"#,
                params.lockstep_timeout_us
            )
            .unwrap();
        }
        writeln!(sdf, r#"    </plugin>"#).unwrap();
    }
    writeln!(sdf).unwrap();
}

fn write_spherical_coords(sdf: &mut String, params: &WorldParams) {
    writeln!(sdf, r#"    <!-- Spherical coordinates for GPS -->"#).unwrap();
    writeln!(sdf, r#"    <spherical_coordinates>"#).unwrap();
    writeln!(sdf, r#"      <surface_model>EARTH_WGS84</surface_model>"#).unwrap();
    writeln!(
        sdf,
        r#"      <world_frame_orientation>ENU</world_frame_orientation>"#
    )
    .unwrap();
    writeln!(
        sdf,
        r#"      <latitude_deg>{}</latitude_deg>"#,
        params.latitude
    )
    .unwrap();
    writeln!(
        sdf,
        r#"      <longitude_deg>{}</longitude_deg>"#,
        params.longitude
    )
    .unwrap();
    writeln!(sdf, r#"      <elevation>{}</elevation>"#, params.elevation).unwrap();
    writeln!(sdf, r#"    </spherical_coordinates>"#).unwrap();
    writeln!(sdf).unwrap();
}

fn write_environment(sdf: &mut String) {
    // Magnetic field (Zurich, Switzerland)
    writeln!(sdf, r#"    <!-- Magnetic field for magnetometer -->"#).unwrap();
    writeln!(
        sdf,
        r#"    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>"#
    )
    .unwrap();
    writeln!(sdf).unwrap();

    // Sunlight
    writeln!(sdf, r#"    <!-- Sunlight -->"#).unwrap();
    writeln!(sdf, r#"    <light type="directional" name="sun">"#).unwrap();
    writeln!(sdf, r#"      <cast_shadows>true</cast_shadows>"#).unwrap();
    writeln!(sdf, r#"      <pose>0 0 10 0 0 0</pose>"#).unwrap();
    writeln!(sdf, r#"      <diffuse>0.8 0.8 0.8 1</diffuse>"#).unwrap();
    writeln!(sdf, r#"      <specular>0.2 0.2 0.2 1</specular>"#).unwrap();
    writeln!(sdf, r#"      <attenuation>"#).unwrap();
    writeln!(sdf, r#"        <range>1000</range>"#).unwrap();
    writeln!(sdf, r#"        <constant>0.9</constant>"#).unwrap();
    writeln!(sdf, r#"        <linear>0.01</linear>"#).unwrap();
    writeln!(sdf, r#"        <quadratic>0.001</quadratic>"#).unwrap();
    writeln!(sdf, r#"      </attenuation>"#).unwrap();
    writeln!(sdf, r#"      <direction>-0.5 0.1 -0.9</direction>"#).unwrap();
    writeln!(sdf, r#"    </light>"#).unwrap();
    writeln!(sdf).unwrap();
}

fn write_ground_plane(sdf: &mut String) {
    writeln!(sdf, r#"    <!-- Ground plane -->"#).unwrap();
    writeln!(sdf, r#"    <model name="ground_plane">"#).unwrap();
    writeln!(sdf, r#"      <static>true</static>"#).unwrap();
    writeln!(sdf, r#"      <link name="link">"#).unwrap();
    writeln!(sdf, r#"        <collision name="collision">"#).unwrap();
    writeln!(sdf, r#"          <geometry>"#).unwrap();
    writeln!(sdf, r#"            <plane>"#).unwrap();
    writeln!(sdf, r#"              <normal>0 0 1</normal>"#).unwrap();
    writeln!(sdf, r#"              <size>100 100</size>"#).unwrap();
    writeln!(sdf, r#"            </plane>"#).unwrap();
    writeln!(sdf, r#"          </geometry>"#).unwrap();
    writeln!(sdf, r#"        </collision>"#).unwrap();
    writeln!(sdf, r#"        <visual name="visual">"#).unwrap();
    writeln!(sdf, r#"          <geometry>"#).unwrap();
    writeln!(sdf, r#"            <plane>"#).unwrap();
    writeln!(sdf, r#"              <normal>0 0 1</normal>"#).unwrap();
    writeln!(sdf, r#"              <size>100 100</size>"#).unwrap();
    writeln!(sdf, r#"            </plane>"#).unwrap();
    writeln!(sdf, r#"          </geometry>"#).unwrap();
    writeln!(sdf, r#"          <material>"#).unwrap();
    writeln!(sdf, r#"            <ambient>0.3 0.5 0.3 1</ambient>"#).unwrap();
    writeln!(sdf, r#"            <diffuse>0.3 0.5 0.3 1</diffuse>"#).unwrap();
    writeln!(sdf, r#"            <specular>0.1 0.1 0.1 1</specular>"#).unwrap();
    writeln!(sdf, r#"          </material>"#).unwrap();
    writeln!(sdf, r#"        </visual>"#).unwrap();
    writeln!(sdf, r#"      </link>"#).unwrap();
    writeln!(sdf, r#"    </model>"#).unwrap();
    writeln!(sdf).unwrap();
}

fn write_vehicle(sdf: &mut String, vehicle: &VehicleTestConfig) {
    let [x, y, z] = vehicle.spawn_position;
    let heading = vehicle.spawn_heading;

    writeln!(
        sdf,
        r#"    <!-- Vehicle: {} (instance {}) -->"#,
        vehicle.id, vehicle.instance
    )
    .unwrap();
    writeln!(sdf, r#"    <include>"#).unwrap();
    writeln!(sdf, r#"      <uri>model://{}</uri>"#, vehicle.model).unwrap();
    writeln!(sdf, r#"      <name>{}</name>"#, vehicle.id).unwrap();
    writeln!(
        sdf,
        r#"      <pose>{} {} {} 0 0 {}</pose>"#,
        x, y, z, heading
    )
    .unwrap();
    writeln!(sdf).unwrap();
    writeln!(sdf, r#"      <!-- Odometry publisher -->"#).unwrap();
    writeln!(
        sdf,
        r#"      <plugin filename="gz-sim-odometry-publisher-system""#
    )
    .unwrap();
    writeln!(
        sdf,
        r#"              name="gz::sim::systems::OdometryPublisher">"#
    )
    .unwrap();
    writeln!(sdf, r#"        <dimensions>3</dimensions>"#).unwrap();
    writeln!(
        sdf,
        r#"        <odom_publish_frequency>250</odom_publish_frequency>"#
    )
    .unwrap();
    writeln!(sdf, r#"      </plugin>"#).unwrap();
    writeln!(sdf, r#"    </include>"#).unwrap();
    writeln!(sdf).unwrap();
}

/// Generate world file and write to disk
pub fn generate_world_file(
    config: &TestConfig,
    params: &WorldParams,
    output_path: &Path,
) -> Result<(), String> {
    let sdf = generate_world(config, params);

    // Ensure parent directory exists
    if let Some(parent) = output_path.parent() {
        fs::create_dir_all(parent).map_err(|e| format!("Failed to create directory: {}", e))?;
    }

    fs::write(output_path, &sdf).map_err(|e| format!("Failed to write world file: {}", e))?;

    Ok(())
}

/// Generate world file to a temporary location
pub fn generate_temp_world(
    config: &TestConfig,
    params: &WorldParams,
) -> Result<std::path::PathBuf, String> {
    let temp_dir = std::env::temp_dir();
    let filename = format!("aviate_test_{}.sdf", config.name);
    let path = temp_dir.join(filename);

    generate_world_file(config, params, &path)?;
    Ok(path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use aviate_hal_xil::parse_test_config_str;

    #[test]
    fn test_generate_single_vehicle_world() {
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
spawn_heading = 0.0

[vehicles.mission]
name = "test"

[[vehicles.mission.phases]]
name = "arm"
duration_ms = 500
action = { type = "arm" }
verify = []
"#;
        let config = parse_test_config_str(config_str).unwrap();
        let params = WorldParams::default();
        let sdf = generate_world(&config, &params);

        assert!(sdf.contains("<world name=\"aviate_sitl\">"));
        assert!(sdf.contains("<model_name>x500_0</model_name>"));
        assert!(sdf.contains("<instance>0</instance>"));
        assert!(sdf.contains("<lockstep>true</lockstep>"));
        assert!(sdf.contains("<uri>model://x500</uri>"));
        assert!(sdf.contains("<name>x500_0</name>"));
        assert!(sdf.contains("<pose>0 0 0 0 0 0</pose>"));
    }

    #[test]
    fn test_generate_two_vehicle_world() {
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
        let params = WorldParams::default();
        let sdf = generate_world(&config, &params);

        // Should have two AviateGzPlugin entries
        assert!(sdf.contains("<model_name>x500_0</model_name>"));
        assert!(sdf.contains("<model_name>x500_1</model_name>"));
        assert!(sdf.contains("<instance>0</instance>"));
        assert!(sdf.contains("<instance>1</instance>"));

        // Should have two vehicle includes
        assert!(sdf.contains("<name>x500_0</name>"));
        assert!(sdf.contains("<name>x500_1</name>"));
        assert!(sdf.contains("<pose>0 0 0 0 0 0</pose>"));
        assert!(sdf.contains("<pose>5 0 0 0 0 0</pose>"));
    }
}
