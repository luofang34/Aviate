//! Network identity and port allocation for one XIL instance.

/// One port offset in an XIL instance range.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum PortSlot {
    /// Sensor input or bidirectional MAVLink traffic.
    SensorIn = 0,
    /// Actuator output.
    ActuatorOut = 1,
    /// Fault command input.
    FaultCmd = 2,
    /// XIL control input.
    XilCtrl = 3,
    /// Test telemetry output.
    TestTelemetry = 4,
    /// Trace data output.
    TraceProfile = 5,
    /// First payload channel.
    Payload0 = 6,
    /// Second payload channel.
    Payload1 = 7,
    /// Third payload channel.
    Payload2 = 8,
    /// Fourth payload channel.
    Payload3 = 9,
}

/// Port allocation for XIL instances.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct XilNetConfig {
    /// First port for instance zero.
    pub base_port: u16,
    /// Number of ports reserved for each instance.
    pub stride: u16,
}

impl Default for XilNetConfig {
    fn default() -> Self {
        Self {
            base_port: 20_000,
            stride: 16,
        }
    }
}

impl XilNetConfig {
    /// Load the network allocation from the process environment.
    #[must_use]
    pub fn from_env() -> Self {
        let base_port = std::env::var("XIL_BASE_PORT")
            .ok()
            .and_then(|value| value.parse().ok())
            .unwrap_or(20_000);
        let stride = std::env::var("XIL_PORT_STRIDE")
            .ok()
            .and_then(|value| value.parse().ok())
            .filter(|value| *value >= 16)
            .unwrap_or(16);
        Self { base_port, stride }
    }

    /// Calculate one port without integer overflow.
    #[must_use]
    pub fn port(&self, instance: u16, slot: PortSlot) -> u16 {
        self.base_port
            .saturating_add(instance.saturating_mul(self.stride))
            .saturating_add(slot as u16)
    }

    /// Calculate the first port for one instance.
    #[must_use]
    pub fn instance_base(&self, instance: u16) -> u16 {
        self.port(instance, PortSlot::SensorIn)
    }
}

/// Network and timing configuration for one XIL instance.
#[derive(Clone, Debug)]
pub struct XilConfig {
    /// Instance identifier.
    pub instance: u8,
    /// Port allocation.
    pub net: XilNetConfig,
    /// Telemetry destination.
    pub gcs_addr: std::net::SocketAddr,
    /// Control-loop rate in hertz.
    pub loop_rate_hz: u32,
}

impl XilConfig {
    /// Create a configuration from the process environment.
    #[must_use]
    pub fn for_instance(instance: u8) -> Self {
        Self::for_instance_with_net(instance, XilNetConfig::from_env())
    }

    /// Create a configuration with an explicit port allocation.
    #[must_use]
    pub fn for_instance_with_net(instance: u8, net: XilNetConfig) -> Self {
        Self {
            instance,
            net,
            gcs_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 14_550)),
            loop_rate_hz: 1_000,
        }
    }

    /// Get the sensor input port.
    #[must_use]
    pub fn sensor_port(&self) -> u16 {
        self.net.port(u16::from(self.instance), PortSlot::SensorIn)
    }

    /// Get the actuator output port.
    #[must_use]
    pub fn actuator_port(&self) -> u16 {
        self.net
            .port(u16::from(self.instance), PortSlot::ActuatorOut)
    }

    /// Get the fault command port.
    #[must_use]
    pub fn fault_cmd_port(&self) -> u16 {
        self.net.port(u16::from(self.instance), PortSlot::FaultCmd)
    }

    /// Get the simulator actuator endpoint.
    #[must_use]
    pub fn simulator_addr(&self) -> std::net::SocketAddr {
        std::net::SocketAddr::from(([127, 0, 0, 1], self.actuator_port()))
    }
}

impl Default for XilConfig {
    fn default() -> Self {
        Self::for_instance(0)
    }
}

/// Compatibility name for SITL users.
pub type SitlConfig = XilConfig;

#[cfg(test)]
mod tests;
