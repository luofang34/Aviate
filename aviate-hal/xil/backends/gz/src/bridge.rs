//! Gazebo Bridge Configuration
//!
//! This module provides configuration for Gazebo simulation integration.
//! The actual bridge logic is in main.rs, which connects GzPluginBridge
//! to the flight controller's SitlIO.

#[cfg(feature = "gz-plugin")]
use log::info;

use aviate_hal_xil::{PortSlot, XilNetConfig};

/// Gazebo bridge configuration
///
/// Supports multi-vehicle simulation via instance IDs.
/// Each instance uses separate shared memory and UDP ports.
///
/// Port allocation uses XilNetConfig (base=20000, stride=16):
/// - Instance 0: sensor=20000, actuator=20001, test_telemetry=20004
/// - Instance 1: sensor=20016, actuator=20017, test_telemetry=20020
#[derive(Clone, Debug)]
pub struct GzBridgeConfig {
    /// Instance ID for multi-vehicle simulation (0 for single vehicle)
    pub instance: u8,
    /// Model name in Gazebo (for SDF plugin config)
    /// For multi-vehicle: x500_0, x500_1, etc.
    pub model_name: String,
    /// Motor command topic in Gazebo (used by plugin for gz-transport publish)
    pub motor_topic: String,
    /// Network configuration for port allocation
    pub net: XilNetConfig,
}

impl GzBridgeConfig {
    /// Create config for a specific instance ID
    ///
    /// Instance-based naming:
    /// - model_name: `x500_<instance>` (or just x500 for instance 0)
    /// - motor_topic: /<model_name>/command/motor_speed
    pub fn for_instance(instance: u8) -> Self {
        Self::for_instance_with_net(instance, XilNetConfig::default())
    }

    /// Create config with custom network settings
    pub fn for_instance_with_net(instance: u8, net: XilNetConfig) -> Self {
        let model_name = if instance == 0 {
            "x500".to_string()
        } else {
            format!("x500_{}", instance)
        };
        let motor_topic = format!("/{}/command/motor_speed", model_name);

        Self {
            instance,
            model_name,
            motor_topic,
            net,
        }
    }

    /// Get the sensor port for this instance
    #[inline]
    pub fn aviate_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::SensorIn)
    }

    /// Get the actuator port for this instance
    #[inline]
    pub fn actuator_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::ActuatorOut)
    }

    /// Get the test telemetry port for this instance
    #[inline]
    pub fn test_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::TestTelemetry)
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
// GzBridge - Connects GzPluginBridge to test infrastructure
// ============================================================================

#[cfg(feature = "gz-plugin")]
mod ffi_bridge {
    use super::*;
    use crate::plugin::{enu_to_ned_f32, enu_vel_to_ned_f32, GzPluginBridge, GzPluginError};
    use aviate_link::mavlink::protocol::LocalPositionNed;
    use aviate_link::mavlink::{serialize_mavlink, MavMessage};
    use std::net::UdpSocket;
    use std::time::Instant;

    /// Gazebo bridge for test infrastructure
    ///
    /// Sends position telemetry to test clients via MAVLink.
    /// Motor commands are now handled directly by main.rs.
    pub struct GzBridge {
        config: GzBridgeConfig,
        plugin: Option<GzPluginBridge>,
        start_time: Instant,
        send_socket: UdpSocket,
        seq: u8,
        // Statistics
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

            Ok(Self {
                config,
                plugin: None,
                start_time: Instant::now(),
                send_socket,
                seq: 0,
                pos_sent: 0,
                last_position: [0.0; 3],
                last_velocity: [0.0; 3],
            })
        }

        /// Connect to the Gazebo plugin via shared memory
        pub fn connect(&mut self, timeout_ms: u64) -> Result<(), GzBridgeError> {
            let max_attempts = (timeout_ms / 500).max(1) as u32;
            info!(
                "[GzBridge] Instance {} connecting to AviateGzPlugin ({}ms timeout)...",
                self.config.instance, timeout_ms
            );

            match GzPluginBridge::connect_instance_with_retry(
                self.config.instance,
                max_attempts,
                500,
            ) {
                Ok(plugin) => {
                    info!("[GzBridge] Connected via shared memory FFI");
                    self.plugin = Some(plugin);
                    Ok(())
                }
                Err(GzPluginError::PluginNotRunning) => Err(GzBridgeError::PluginNotRunning),
                Err(_) => Err(GzBridgeError::Timeout),
            }
        }

        /// Check if connected to the plugin
        pub fn is_connected(&self) -> bool {
            self.plugin.as_ref().is_some_and(|p| p.is_connected())
        }

        /// Run one iteration - sends position telemetry to test client
        pub fn step(&mut self) {
            let now_ms = (self.start_time.elapsed().as_micros() / 1000) as u32;

            // Read physics state from plugin
            let (position, velocity) = if let Some(ref plugin) = self.plugin {
                if let Some(state) = plugin.get_model_state() {
                    let ned_pos = enu_to_ned_f32(state.pos);
                    let ned_vel = enu_vel_to_ned_f32(state.vel);
                    self.last_position = ned_pos;
                    self.last_velocity = ned_vel;
                    (ned_pos, ned_vel)
                } else {
                    (self.last_position, self.last_velocity)
                }
            } else {
                ([0.0; 3], [0.0; 3])
            };

            // Send LOCAL_POSITION_NED to test client at 50Hz
            if self.seq.is_multiple_of(5) {
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

            self.seq = self.seq.wrapping_add(1);
        }

        /// Send a MAVLink message to the test client
        fn send_to_test_client(&mut self, msg: &MavMessage) {
            let mut buf = [0u8; 300];
            // System ID = instance + 1
            if let Some(len) = serialize_mavlink(msg, self.seq, self.config.instance + 1, 1, &mut buf) {
                let addr = ("127.0.0.1", self.config.test_port());
                let _ = self.send_socket.send_to(&buf[..len], addr);
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
// Stub when feature is not enabled
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
