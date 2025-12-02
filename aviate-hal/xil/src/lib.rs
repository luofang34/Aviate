//! X-In-Loop (XIL) Platform Core
//!
//! Backend-agnostic platform for SITL (Software-In-The-Loop) and HITL (Hardware-In-The-Loop)
//! simulation. This crate provides:
//!
//! - **Backend trait**: Interface for kinematics backends (Gazebo, Unity, Chrono, etc.)
//! - **World state**: Backend-agnostic representation of simulation world
//! - **Test infrastructure**: Mission framework, test runner, config parsing
//! - **SITL transport**: Network communication with simulators (MAVLink/UDP)
//!
//! ## Architecture
//!
//! ```text
//! aviate-hal-xil (this crate, no backend deps)
//!        ↑
//! aviate-backend-gz (implements KinematicsBackend)
//!        ↑ (FFI/IPC)
//! aviate_gz_plugin (C++, Gazebo)
//! ```
//!
//! The xil core does NOT depend on any specific backend. Backends implement
//! traits defined here and are selected at runtime via configuration.

#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod backend;
pub mod bridge;
pub mod config;
pub mod fault_ctrl;
pub mod fault_protocol;
pub mod flight_log;
pub mod mission;
pub mod mock;
pub mod sim_types;
pub mod sitl_io;
pub mod world;

// Core exports
pub use backend::{BackendConfig, BackendError, KinematicsBackend, LockstepMode, TimingMode};
pub use world::{
    AngularVelocity, Entity, EntityId, EntityState, Position, Quaternion, Velocity, World,
};

// Transport exports
pub use mock::SitlHal;
pub use sitl_io::{HilGpsData, HilSensorData, SitlIO};

// Simulator-neutral data types (for direct FFI integration)
pub use sim_types::{
    SimActuatorCmd, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
    SimTimestampUs,
};

// Re-export legacy name for compatibility
pub use SitlIO as SitlMavlink;

// Flight log exports
pub use flight_log::{FlightLog, FlightLogConfig, FlightSample, FlightStats};

// Test infrastructure exports
pub use config::{parse_test_config, parse_test_config_str, TestConfig, VehicleTestConfig};
pub use mission::{
    Action, Criterion, CriterionResult, FaultSpec, Mission, MissionResult, MultiVehicleCriterion,
    MultiVehicleMission, MultiVehiclePhase, Phase, PhaseResult, SensorTarget, VehicleConfig,
};

// Fault injection protocol exports
pub use fault_protocol::{
    AckStatus, FaultAck, FaultAction, FaultClient, FaultCommand, FAULT_ACK_MAGIC, FAULT_ACK_SIZE,
    FAULT_CMD_MAGIC, FAULT_CMD_SIZE,
};

// Fault controller exports
pub use fault_ctrl::{FaultController, FaultCtrlError};

/// XIL Network Configuration
///
/// Port allocation scheme: `base_port + instance * stride + slot`
///
/// Each instance occupies 16 ports (stride=16):
/// - +0: SensorIn (simulator → FC, sensor data)
/// - +1: ActuatorOut (FC → simulator, motor commands)
/// - +2: FaultCmd (test → FC, fault injection)
/// - +3: XilCtrl (pause/step/reset/time sync)
/// - +4: TestTelemetry (FC → test runner, EKF quality)
/// - +5: TraceProfile (profiling/trace)
/// - +6..+9: Payload slots (cameras, lidar, etc.)
/// - +10..+15: Reserved
///
/// Default: base=20000, stride=16 (~2845 instances per host)
/// Configurable via XIL_BASE_PORT and XIL_PORT_STRIDE env vars.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XilNetConfig {
    /// Base port for instance 0 (default 20000)
    pub base_port: u16,
    /// Port stride per instance (default 16, minimum 16)
    pub stride: u16,
}

impl Default for XilNetConfig {
    fn default() -> Self {
        Self {
            base_port: 20000,
            stride: 16,
        }
    }
}

impl XilNetConfig {
    /// Load from environment variables (XIL_BASE_PORT, XIL_PORT_STRIDE)
    pub fn from_env() -> Self {
        let base_port = std::env::var("XIL_BASE_PORT")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(20000);
        let stride = std::env::var("XIL_PORT_STRIDE")
            .ok()
            .and_then(|s| s.parse().ok())
            .filter(|&v| v >= 16)
            .unwrap_or(16);
        Self { base_port, stride }
    }

    /// Calculate port for a specific instance and slot (overflow-safe)
    #[inline]
    pub fn port(&self, instance: u16, slot: PortSlot) -> u16 {
        self.base_port
            .saturating_add(instance.saturating_mul(self.stride))
            .saturating_add(slot as u16)
    }

    /// Calculate base port for an instance (slot 0)
    #[inline]
    pub fn instance_base(&self, instance: u16) -> u16 {
        self.port(instance, PortSlot::SensorIn)
    }
}

/// Port slot within an instance's port range
///
/// Each instance occupies `stride` ports (default 16).
/// Slot values are offsets from the instance base port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortSlot {
    /// Sensor data from simulator
    SensorIn = 0,
    /// Actuator commands to simulator
    ActuatorOut = 1,
    /// Fault injection commands
    FaultCmd = 2,
    /// XIL control (pause, step, reset, time sync)
    XilCtrl = 3,
    /// Test telemetry (EKF quality, failsafe status)
    TestTelemetry = 4,
    /// Trace/profiling data
    TraceProfile = 5,
    /// Payload slot 0 (camera, video)
    Payload0 = 6,
    /// Payload slot 1 (lidar)
    Payload1 = 7,
    /// Payload slot 2
    Payload2 = 8,
    /// Payload slot 3
    Payload3 = 9,
}

/// XIL instance configuration
///
/// Supports multi-vehicle simulation via instance IDs.
/// Each instance uses separate UDP ports based on XilNetConfig.
#[derive(Clone, Debug)]
pub struct XilConfig {
    /// Instance ID for multi-vehicle simulation (0 for single vehicle)
    pub instance: u8,
    /// Network configuration (port allocation)
    pub net: XilNetConfig,
    /// Address to send telemetry to (GCS)
    pub gcs_addr: std::net::SocketAddr,
    /// Loop rate in Hz (default 1000)
    pub loop_rate_hz: u32,
}

impl XilConfig {
    /// Create config for a specific instance ID
    pub fn for_instance(instance: u8) -> Self {
        Self::for_instance_with_net(instance, XilNetConfig::default())
    }

    /// Create config for a specific instance with custom network settings
    pub fn for_instance_with_net(instance: u8, net: XilNetConfig) -> Self {
        Self {
            instance,
            net,
            gcs_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 14550)),
            loop_rate_hz: 1000,
        }
    }

    /// Get the sensor input port for this instance
    #[inline]
    pub fn sensor_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::SensorIn)
    }

    /// Get the actuator output port for this instance
    #[inline]
    pub fn actuator_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::ActuatorOut)
    }

    /// Get the fault command port for this instance
    #[inline]
    pub fn fault_cmd_port(&self) -> u16 {
        self.net.port(self.instance as u16, PortSlot::FaultCmd)
    }

    /// Get the simulator address (actuator port on localhost)
    pub fn simulator_addr(&self) -> std::net::SocketAddr {
        std::net::SocketAddr::from(([127, 0, 0, 1], self.actuator_port()))
    }
}

impl Default for XilConfig {
    fn default() -> Self {
        Self::for_instance(0)
    }
}

// Re-export legacy name for compatibility
pub type SitlConfig = XilConfig;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_xil_net_config_defaults() {
        let net = XilNetConfig::default();
        assert_eq!(net.base_port, 20000);
        assert_eq!(net.stride, 16);
    }

    #[test]
    fn test_xil_net_config_port_allocation() {
        let net = XilNetConfig::default();

        // Instance 0
        assert_eq!(net.port(0, PortSlot::SensorIn), 20000);
        assert_eq!(net.port(0, PortSlot::ActuatorOut), 20001);
        assert_eq!(net.port(0, PortSlot::FaultCmd), 20002);
        assert_eq!(net.port(0, PortSlot::XilCtrl), 20003);
        assert_eq!(net.port(0, PortSlot::TestTelemetry), 20004);

        // Instance 1
        assert_eq!(net.port(1, PortSlot::SensorIn), 20016);
        assert_eq!(net.port(1, PortSlot::ActuatorOut), 20017);
        assert_eq!(net.port(1, PortSlot::FaultCmd), 20018);

        // Instance 2
        assert_eq!(net.port(2, PortSlot::SensorIn), 20032);
    }

    #[test]
    fn test_xil_net_config_overflow_protection() {
        let net = XilNetConfig {
            base_port: 65000,
            stride: 16,
        };
        // Should saturate instead of overflow
        let port = net.port(100, PortSlot::SensorIn);
        // 65000 + 100*16 = 66600, which overflows u16
        // saturating_add should clamp to 65535
        assert_eq!(port, 65535);
    }

    #[test]
    fn test_xil_config_for_instance() {
        let config = XilConfig::for_instance(0);
        assert_eq!(config.instance, 0);
        assert_eq!(config.sensor_port(), 20000);
        assert_eq!(config.actuator_port(), 20001);
        assert_eq!(config.fault_cmd_port(), 20002);

        let config = XilConfig::for_instance(1);
        assert_eq!(config.instance, 1);
        assert_eq!(config.sensor_port(), 20016);
        assert_eq!(config.actuator_port(), 20017);
    }

    #[test]
    fn test_xil_config_simulator_addr() {
        let config = XilConfig::for_instance(0);
        let addr = config.simulator_addr();
        assert_eq!(addr.port(), 20001);
        assert_eq!(addr.ip().to_string(), "127.0.0.1");
    }

    #[test]
    fn test_xil_config_gcs_addr() {
        let config = XilConfig::for_instance(0);
        // GCS port 14550 for QGroundControl compatibility
        assert_eq!(config.gcs_addr.port(), 14550);
    }

    #[test]
    fn test_multi_vehicle_no_port_overlap() {
        let net = XilNetConfig::default();

        // Verify no port overlap for 100 vehicles (new scheme supports more)
        let mut ports = std::collections::HashSet::new();
        for instance in 0..100u16 {
            let sensor = net.port(instance, PortSlot::SensorIn);
            let actuator = net.port(instance, PortSlot::ActuatorOut);
            let fault = net.port(instance, PortSlot::FaultCmd);
            assert!(
                ports.insert(sensor),
                "Duplicate sensor port for instance {}",
                instance
            );
            assert!(
                ports.insert(actuator),
                "Duplicate actuator port for instance {}",
                instance
            );
            assert!(
                ports.insert(fault),
                "Duplicate fault port for instance {}",
                instance
            );
        }
    }
}
