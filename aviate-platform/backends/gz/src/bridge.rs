//! Gazebo Bridge for SITL
//!
//! This module bridges Gazebo Sim physics data to MAVLink HIL protocol.
//! It reads model state from Gazebo and converts to HIL_SENSOR/HIL_GPS MAVLink messages.
//!
//! ## Bridge Modes
//!
//! - **FFI Mode** (default, `gz-plugin` feature): Zero-copy shared memory via AviateGzPlugin
//!
//! FFI mode is recommended for production use - it provides ~1μs latency.

/// Gazebo bridge configuration
///
/// Supports multi-vehicle simulation via instance IDs.
/// Each instance uses separate shared memory and UDP ports.
///
/// Port allocation (base ports + instance * 10):
/// - Instance 0: aviate=14560, actuator=14561, test=14562
/// - Instance 1: aviate=14570, actuator=14571, test=14572
/// - Instance 2: aviate=14580, actuator=14581, test=14582
#[derive(Clone, Debug)]
pub struct GzBridgeConfig {
    /// Instance ID for multi-vehicle simulation (0 for single vehicle)
    pub instance: u8,
    /// Model name in Gazebo (for SDF plugin config)
    /// For multi-vehicle: x500_0, x500_1, etc.
    pub model_name: String,
    /// Motor command topic in Gazebo (used by plugin for gz-transport publish)
    pub motor_topic: String,
    /// UDP port to send HIL data to Aviate
    pub aviate_port: u16,
    /// UDP port to receive actuator commands from Aviate
    pub actuator_port: u16,
    /// UDP port to send position data to test client
    pub test_port: u16,
}

impl GzBridgeConfig {
    /// Create config for a specific instance ID
    ///
    /// Instance-based naming:
    /// - model_name: x500_<instance> (or just x500 for instance 0)
    /// - motor_topic: /<model_name>/command/motor_speed
    /// - ports: base + instance * 10
    pub fn for_instance(instance: u8) -> Self {
        let model_name = if instance == 0 {
            "x500".to_string()
        } else {
            format!("x500_{}", instance)
        };
        let motor_topic = format!("/{}/command/motor_speed", model_name);
        let base_port = 14560u16 + (instance as u16) * 10;

        Self {
            instance,
            model_name,
            motor_topic,
            aviate_port: base_port,
            actuator_port: base_port + 1,
            test_port: base_port + 2,
        }
    }
}

impl Default for GzBridgeConfig {
    fn default() -> Self {
        Self::for_instance(0)
    }
}

/// Error type for GzBridge operations
#[derive(Debug)]
pub enum GzBridgeError {
    /// IO error (socket binding, etc.)
    Io(std::io::Error),
    /// Plugin not running (shared memory not available)
    PluginNotRunning,
    /// Connection timeout
    Timeout,
}

impl std::fmt::Display for GzBridgeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Io(e) => write!(f, "IO error: {}", e),
            Self::PluginNotRunning => write!(f, "AviateGzPlugin not running in Gazebo"),
            Self::Timeout => write!(f, "Connection timeout"),
        }
    }
}

impl std::error::Error for GzBridgeError {}

impl From<std::io::Error> for GzBridgeError {
    fn from(e: std::io::Error) -> Self {
        Self::Io(e)
    }
}

// ============================================================================
// FFI Mode (gz-plugin feature) - Zero-copy shared memory
// ============================================================================

#[cfg(feature = "gz-plugin")]
mod ffi_bridge {
    use super::*;
    use std::net::UdpSocket;
    use std::time::Instant;
    use aviate_mavlink::{
        serialize_mavlink, parse_mavlink, HilSensor, HilGps, HilActuatorControls, LocalPositionNed,
        MavMessage,
    };
    use crate::plugin::{GzPluginBridge, GzPluginError, enu_to_ned_f32, enu_vel_to_ned_f32};

    /// Gazebo-MAVLink bridge using shared memory FFI
    pub struct GzBridge {
        config: GzBridgeConfig,
        plugin: Option<GzPluginBridge>,
        start_time: Instant,
        send_socket: UdpSocket,
        recv_socket: UdpSocket,
        seq: u8,
        // Statistics
        hil_sent: u64,
        motor_recv: u64,
        pos_sent: u64,
        // Cached state
        last_position: [f32; 3],
        last_velocity: [f32; 3],
    }

    impl GzBridge {
        /// Create a new Gazebo bridge
        pub fn new(config: GzBridgeConfig) -> Result<Self, GzBridgeError> {
            let send_socket = UdpSocket::bind("0.0.0.0:0")?;
            send_socket.set_nonblocking(true)?;

            let recv_socket = UdpSocket::bind(("0.0.0.0", config.actuator_port))?;
            recv_socket.set_nonblocking(true)?;

            Ok(Self {
                config,
                plugin: None,
                start_time: Instant::now(),
                send_socket,
                recv_socket,
                seq: 0,
                hil_sent: 0,
                motor_recv: 0,
                pos_sent: 0,
                last_position: [0.0; 3],
                last_velocity: [0.0; 3],
            })
        }

        /// Connect to the Gazebo plugin via shared memory
        ///
        /// Waits for the AviateGzPlugin to initialize shared memory.
        pub fn connect(&mut self, timeout_ms: u64) -> Result<(), GzBridgeError> {
            let max_attempts = (timeout_ms / 500).max(1) as u32;
            eprintln!("[GzBridge] Connecting to AviateGzPlugin ({}ms timeout)...", timeout_ms);

            match GzPluginBridge::connect_with_retry(max_attempts, 500) {
                Ok(plugin) => {
                    eprintln!("[GzBridge] Connected via shared memory FFI");
                    self.plugin = Some(plugin);
                    Ok(())
                }
                Err(GzPluginError::PluginNotRunning) => Err(GzBridgeError::PluginNotRunning),
                Err(_) => Err(GzBridgeError::Timeout),
            }
        }

        /// Check if connected to the plugin
        pub fn is_connected(&self) -> bool {
            self.plugin.as_ref().map(|p| p.is_connected()).unwrap_or(false)
        }

        /// Run one iteration of the bridge loop
        pub fn step(&mut self) {
            let now_us = self.start_time.elapsed().as_micros() as u64;
            let now_ms = (now_us / 1000) as u32;

            // Read physics state from plugin (zero-copy via shared memory)
            let (accel, gyro, position, velocity) = if let Some(ref plugin) = self.plugin {
                if let Some(state) = plugin.get_model_state() {
                    // Convert from ENU to NED for MAVLink
                    let ned_pos = enu_to_ned_f32(state.pos);
                    let ned_vel = enu_vel_to_ned_f32(state.vel);

                    // Use angular velocity as gyro (already in body frame from physics)
                    let gyro = [
                        state.ang_vel[0] as f32,
                        state.ang_vel[1] as f32,
                        state.ang_vel[2] as f32,
                    ];

                    // Simulated accelerometer (gravity + body acceleration)
                    let accel = [0.0f32, 0.0, -9.81];

                    self.last_position = ned_pos;
                    self.last_velocity = ned_vel;

                    (accel, gyro, ned_pos, ned_vel)
                } else {
                    // No valid data yet, use cached
                    ([0.0, 0.0, -9.81], [0.0; 3], self.last_position, self.last_velocity)
                }
            } else {
                // Plugin not connected
                ([0.0, 0.0, -9.81], [0.0; 3], [0.0; 3], [0.0; 3])
            };

            // Build and send HIL_SENSOR
            let hil_sensor = HilSensor {
                time_usec: now_us,
                xacc: accel[0],
                yacc: accel[1],
                zacc: accel[2],
                xgyro: gyro[0],
                ygyro: gyro[1],
                zgyro: gyro[2],
                xmag: 0.2,
                ymag: 0.0,
                zmag: 0.4,
                abs_pressure: 1013.25,
                diff_pressure: 0.0,
                pressure_alt: -position[2],  // NED z is down, altitude is positive up
                temperature: 25.0,
                fields_updated: 0x1FFF,
                id: 0,
            };

            self.send_mavlink(&MavMessage::HilSensor(hil_sensor));
            self.hil_sent += 1;

            // Print stats every second (250 iterations at 250 Hz)
            if self.hil_sent % 250 == 0 {
                eprintln!("[GzBridge] hil={}, motors={}, pos_sent={}, pos=[{:.2},{:.2},{:.2}]",
                    self.hil_sent, self.motor_recv, self.pos_sent,
                    position[0], position[1], position[2]);
            }

            // Send LOCAL_POSITION_NED to test client at 50Hz
            if self.seq % 5 == 0 {
                let local_pos = LocalPositionNed {
                    time_boot_ms: now_ms,
                    x: position[0],
                    y: position[1],
                    z: position[2],
                    vx: velocity[0],
                    vy: velocity[1],
                    vz: velocity[2],
                };
                self.send_to_test_client(&MavMessage::LocalPositionNed(local_pos));
                self.pos_sent += 1;
            }

            // Send HIL_GPS at 10Hz
            if self.seq % 25 == 0 {
                let lat = (position[0] / 111000.0 * 1e7) as i32;
                let lon = (position[1] / 111000.0 * 1e7) as i32;
                let alt = (-position[2] * 1000.0) as i32;

                let hil_gps = HilGps {
                    time_usec: now_us,
                    lat,
                    lon,
                    alt,
                    eph: 100,
                    epv: 100,
                    vel: 0,
                    vn: (velocity[0] * 100.0) as i16,
                    ve: (velocity[1] * 100.0) as i16,
                    vd: (velocity[2] * 100.0) as i16,
                    cog: 0,
                    fix_type: 3,
                    satellites_visible: 10,
                    id: 0,
                    yaw: 0,
                };

                self.send_mavlink(&MavMessage::HilGps(hil_gps));
            }

            // Receive actuator commands from Aviate
            self.receive_actuators();
        }

        /// Send a MAVLink message to Aviate
        fn send_mavlink(&mut self, msg: &MavMessage) {
            let mut buf = [0u8; 300];
            if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
                self.seq = self.seq.wrapping_add(1);
                let addr = ("127.0.0.1", self.config.aviate_port);
                let _ = self.send_socket.send_to(&buf[..len], addr);
            }
        }

        /// Send a MAVLink message to the test client
        fn send_to_test_client(&mut self, msg: &MavMessage) {
            let mut buf = [0u8; 300];
            if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
                let addr = ("127.0.0.1", self.config.test_port);
                let _ = self.send_socket.send_to(&buf[..len], addr);
            }
        }

        /// Receive and process actuator commands
        fn receive_actuators(&mut self) {
            let mut buf = [0u8; 512];

            loop {
                match self.recv_socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        if let Ok((msg, _)) = parse_mavlink(&buf[..len]) {
                            if let MavMessage::HilActuatorControls(ctrl) = msg {
                                self.motor_recv += 1;
                                self.send_motor_command(&ctrl);
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }

        /// Send motor command to Gazebo via shared memory (zero-copy)
        fn send_motor_command(&mut self, ctrl: &HilActuatorControls) {
            // Convert normalized thrust (0-1) to motor velocity (0-1000 rad/s)
            let velocities: [f64; 4] = [
                (ctrl.controls[0].max(0.0) * 1000.0) as f64,
                (ctrl.controls[1].max(0.0) * 1000.0) as f64,
                (ctrl.controls[2].max(0.0) * 1000.0) as f64,
                (ctrl.controls[3].max(0.0) * 1000.0) as f64,
            ];

            // Send via shared memory (zero-copy, plugin publishes to gz-transport)
            if let Some(ref plugin) = self.plugin {
                if plugin.set_motor_speeds(&velocities).is_ok() {
                    if self.motor_recv % 50 == 1 {
                        eprintln!("[GzBridge] Motor cmd: [{:.1},{:.1},{:.1},{:.1}] rad/s",
                            velocities[0], velocities[1], velocities[2], velocities[3]);
                    }
                } else if self.motor_recv % 250 == 1 {
                    eprintln!("[GzBridge] Warning: Failed to send motor command via shm");
                }
            }
        }

        /// Get timestamp in microseconds
        pub fn now_us(&self) -> u64 {
            self.start_time.elapsed().as_micros() as u64
        }
    }
}

#[cfg(feature = "gz-plugin")]
pub use ffi_bridge::GzBridge;

// ============================================================================
// Stub when neither feature is enabled
// ============================================================================

#[cfg(not(feature = "gz-plugin"))]
pub struct GzBridge;

#[cfg(not(feature = "gz-plugin"))]
impl GzBridge {
    pub fn new(_config: GzBridgeConfig) -> Result<Self, GzBridgeError> {
        Err(GzBridgeError::PluginNotRunning)
    }

    pub fn connect(&mut self, _timeout_ms: u64) -> Result<(), GzBridgeError> {
        Err(GzBridgeError::PluginNotRunning)
    }

    pub fn is_connected(&self) -> bool {
        false
    }

    pub fn step(&mut self) {}

    pub fn now_us(&self) -> u64 {
        0
    }
}
