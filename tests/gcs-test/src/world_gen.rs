//! World Generator for SITL Testing
//!
//! Generates Gazebo SDF world files from test configuration parameters.
//! This allows tests to define vehicles and world settings in TOML,
//! and have the world file generated automatically at runtime.
//!
//! Writes go through the [`emit!`] macro, which discards the
//! formally-fallible [`std::fmt::Result`] returned by `writeln!`
//! into a `String` via `.ok()` — `String`'s `fmt::Write` impl
//! never returns Err, so threading `?` through every SDF line
//! is unnecessary noise. The macro keeps the workspace-level
//! `unwrap_used = "forbid"` lint in effect for this file.

use std::fs;
use std::path::Path;

use aviate_hal_xil::{TestConfig, VehicleTestConfig};

/// Append a formatted line to a `String`. Equivalent to
/// `writeln!(s, ...)`, but discards the formally-fallible Result
/// because writing to a `String` cannot fail (no I/O underneath).
macro_rules! emit {
    ($s:expr, $($arg:tt)*) => {{
        use ::std::fmt::Write as _;
        writeln!($s, $($arg)*).ok();
    }};
}

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
    emit!(sdf, r#"<?xml version="1.0" ?>"#);
    emit!(sdf, r#"<!--"#);
    emit!(sdf, r#"  Auto-generated world for test: {}"#, config.name);
    emit!(sdf, r#"  Description: {}"#, config.description);
    emit!(sdf, r#"  Vehicles: {}"#, config.vehicles.len());
    emit!(sdf, r#"  Lockstep: {}"#, params.lockstep);
    emit!(sdf, r#"-->"#);

    emit!(sdf, r#"<sdf version="1.9">"#);
    emit!(sdf, r#"  <world name="{}">"#, params.name);

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

    emit!(sdf, r#"  </world>"#);
    emit!(sdf, r#"</sdf>"#);

    sdf
}

fn write_physics(sdf: &mut String, params: &WorldParams) {
    emit!(sdf, r#"    <physics name="1ms" type="ignored">"#);
    emit!(sdf, r#"      <max_step_size>{}</max_step_size>"#,
        params.step_size);
    emit!(sdf, r#"      <real_time_factor>{}</real_time_factor>"#,
        params.real_time_factor);
    emit!(sdf, r#"    </physics>"#);
    sdf.push('\n');
}

fn write_system_plugins(sdf: &mut String) {
    emit!(sdf, r#"    <!-- Required system plugins -->"#);

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
        emit!(sdf, r#"    <plugin filename="{filename}""#);
        emit!(sdf, r#"            name="{name}">"#);
        emit!(sdf, r#"    </plugin>"#);
    }

    // Sensors plugin with render engine
    emit!(sdf, r#"    <plugin filename="gz-sim-sensors-system""#);
    emit!(sdf, r#"            name="gz::sim::systems::Sensors">"#);
    emit!(sdf, r#"      <render_engine>ogre2</render_engine>"#);
    emit!(sdf, r#"    </plugin>"#);
    sdf.push('\n');
}

fn write_aviate_plugins(sdf: &mut String, config: &TestConfig, params: &WorldParams) {
    emit!(sdf, r#"    <!-- Aviate bridge plugins -->"#);

    for vehicle in &config.vehicles {
        emit!(sdf, r#"    <plugin filename="AviateGzPlugin""#);
        emit!(sdf, r#"            name="aviate::AviateGzPlugin">"#);
        emit!(sdf, r#"      <model_name>{}</model_name>"#, vehicle.id);
        emit!(sdf, r#"      <instance>{}</instance>"#, vehicle.instance);
        emit!(sdf, r#"      <lockstep>{}</lockstep>"#, params.lockstep);
        if params.lockstep {
            emit!(sdf, r#"      <lockstep_timeout_us>{}</lockstep_timeout_us>"#,
                params.lockstep_timeout_us);
        }
        emit!(sdf, r#"    </plugin>"#);
    }
    sdf.push('\n');
}

fn write_spherical_coords(sdf: &mut String, params: &WorldParams) {
    emit!(sdf, r#"    <!-- Spherical coordinates for GPS -->"#);
    emit!(sdf, r#"    <spherical_coordinates>"#);
    emit!(sdf, r#"      <surface_model>EARTH_WGS84</surface_model>"#);
    emit!(sdf, r#"      <world_frame_orientation>ENU</world_frame_orientation>"#);
    emit!(sdf, r#"      <latitude_deg>{}</latitude_deg>"#,
        params.latitude);
    emit!(sdf, r#"      <longitude_deg>{}</longitude_deg>"#,
        params.longitude);
    emit!(sdf, r#"      <elevation>{}</elevation>"#, params.elevation);
    emit!(sdf, r#"    </spherical_coordinates>"#);
    sdf.push('\n');
}

fn write_environment(sdf: &mut String) {
    // Magnetic field (Zurich, Switzerland)
    emit!(sdf, r#"    <!-- Magnetic field for magnetometer -->"#);
    emit!(sdf, r#"    <magnetic_field>6.0e-6 2.3e-5 -4.2e-5</magnetic_field>"#);
    sdf.push('\n');

    // Sunlight
    emit!(sdf, r#"    <!-- Sunlight -->"#);
    emit!(sdf, r#"    <light type="directional" name="sun">"#);
    emit!(sdf, r#"      <cast_shadows>true</cast_shadows>"#);
    emit!(sdf, r#"      <pose>0 0 10 0 0 0</pose>"#);
    emit!(sdf, r#"      <diffuse>0.8 0.8 0.8 1</diffuse>"#);
    emit!(sdf, r#"      <specular>0.2 0.2 0.2 1</specular>"#);
    emit!(sdf, r#"      <attenuation>"#);
    emit!(sdf, r#"        <range>1000</range>"#);
    emit!(sdf, r#"        <constant>0.9</constant>"#);
    emit!(sdf, r#"        <linear>0.01</linear>"#);
    emit!(sdf, r#"        <quadratic>0.001</quadratic>"#);
    emit!(sdf, r#"      </attenuation>"#);
    emit!(sdf, r#"      <direction>-0.5 0.1 -0.9</direction>"#);
    emit!(sdf, r#"    </light>"#);
    sdf.push('\n');
}

fn write_ground_plane(sdf: &mut String) {
    emit!(sdf, r#"    <!-- Ground plane -->"#);
    emit!(sdf, r#"    <model name="ground_plane">"#);
    emit!(sdf, r#"      <static>true</static>"#);
    emit!(sdf, r#"      <link name="link">"#);
    emit!(sdf, r#"        <collision name="collision">"#);
    emit!(sdf, r#"          <geometry>"#);
    emit!(sdf, r#"            <plane>"#);
    emit!(sdf, r#"              <normal>0 0 1</normal>"#);
    emit!(sdf, r#"              <size>100 100</size>"#);
    emit!(sdf, r#"            </plane>"#);
    emit!(sdf, r#"          </geometry>"#);
    emit!(sdf, r#"        </collision>"#);
    emit!(sdf, r#"        <visual name="visual">"#);
    emit!(sdf, r#"          <geometry>"#);
    emit!(sdf, r#"            <plane>"#);
    emit!(sdf, r#"              <normal>0 0 1</normal>"#);
    emit!(sdf, r#"              <size>100 100</size>"#);
    emit!(sdf, r#"            </plane>"#);
    emit!(sdf, r#"          </geometry>"#);
    emit!(sdf, r#"          <material>"#);
    emit!(sdf, r#"            <ambient>0.3 0.5 0.3 1</ambient>"#);
    emit!(sdf, r#"            <diffuse>0.3 0.5 0.3 1</diffuse>"#);
    emit!(sdf, r#"            <specular>0.1 0.1 0.1 1</specular>"#);
    emit!(sdf, r#"          </material>"#);
    emit!(sdf, r#"        </visual>"#);
    emit!(sdf, r#"      </link>"#);
    emit!(sdf, r#"    </model>"#);
    sdf.push('\n');
}

fn write_vehicle(sdf: &mut String, vehicle: &VehicleTestConfig) {
    let [x, y, z] = vehicle.spawn_position;
    let heading = vehicle.spawn_heading;

    emit!(
        sdf,
        r#"    <!-- Vehicle: {} (instance {}) -->"#,
        vehicle.id,
        vehicle.instance
    );
    emit!(sdf, r#"    <include>"#);
    emit!(sdf, r#"      <uri>model://{}</uri>"#, vehicle.model);
    emit!(sdf, r#"      <name>{}</name>"#, vehicle.id);
    emit!(sdf, r#"      <pose>{} {} {} 0 0 {}</pose>"#,
        x, y, z, heading);
    sdf.push('\n');
    emit!(sdf, r#"      <!-- Odometry publisher -->"#);
    emit!(sdf, r#"      <plugin filename="gz-sim-odometry-publisher-system""#);
    emit!(sdf, r#"              name="gz::sim::systems::OdometryPublisher">"#);
    emit!(sdf, r#"        <dimensions>3</dimensions>"#);
    emit!(sdf, r#"        <odom_publish_frequency>250</odom_publish_frequency>"#);
    emit!(sdf, r#"      </plugin>"#);
    emit!(sdf, r#"    </include>"#);
    sdf.push('\n');
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
#[allow(clippy::expect_used, clippy::panic)]
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
        let config = parse_test_config_str(config_str).expect("hand-written test TOML must parse");
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
        let config = parse_test_config_str(config_str).expect("hand-written test TOML must parse");
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
