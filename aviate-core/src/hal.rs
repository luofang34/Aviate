//! Hardware Abstraction Layer traits
//!
//! Platform crates (SITL, H7, etc.) implement these traits to connect
//! aviate-core to hardware or simulation.

use crate::control::Command;
use crate::mixer::ActuatorCmd;
use crate::sensor::{AirspeedData, BaroData, GnssData, ImuData, MagData, SensorReading};
use crate::time::Timestamp;

/// Sensor input interface
///
/// Platform implements this to provide sensor data to the core.
/// Returns None if no new data available since last read.
pub trait SensorHal {
    /// Read IMU data (typically polled at ~1kHz)
    fn read_imu(&mut self) -> Option<SensorReading<ImuData>>;

    /// Read GNSS data (typically polled at ~10Hz)
    fn read_gnss(&mut self) -> Option<SensorReading<GnssData>>;

    /// Read barometer data (typically polled at ~50Hz)
    fn read_baro(&mut self) -> Option<SensorReading<BaroData>>;

    /// Read magnetometer data (typically polled at ~50Hz)
    fn read_mag(&mut self) -> Option<SensorReading<MagData>>;

    /// Read airspeed data (optional, for fixed-wing)
    // COV:EXCL_START(DEFAULT: optional sensor for fixed-wing only)
    fn read_airspeed(&mut self) -> Option<SensorReading<AirspeedData>> {
        None
    }
    // COV:EXCL_STOP
}

/// Actuator output interface
pub trait ActuatorHal {
    /// Write actuator commands to hardware (PWM, DShot, etc.)
    fn write(&mut self, cmd: &ActuatorCmd);

    /// Arm actuators (enable output)
    fn arm(&mut self);

    /// Disarm actuators (disable output, safe state)
    fn disarm(&mut self);

    /// Check if hardware arm switch is set
    fn is_armed(&self) -> bool;
}

/// System services interface
pub trait SystemHal {
    /// Get current timestamp
    fn now(&self) -> Timestamp;

    /// Get monotonic time in microseconds
    fn now_us(&self) -> u64;

    /// Delay for specified microseconds (blocking)
    fn delay_us(&self, us: u32);

    /// Kick hardware watchdog
    fn kick_watchdog(&mut self);

    /// System reboot (does not return)
    fn reboot(&mut self) -> !;

    /// Enter bootloader mode for firmware update
    fn enter_bootloader(&mut self) -> !;
}

/// System command from GCS/RC
#[derive(Clone, Debug)]
pub enum SystemCommand {
    FlightControl(Command),
    Arm,
    Disarm,
    // Future: Reboot, Shutdown, etc.
}

/// Command input interface (GCS/RC)
pub trait CommandHal {
    /// Receive the latest command from GCS/RC
    fn recv_command(&mut self) -> Option<SystemCommand>;
}

/// Communication interface for telemetry/commands
pub trait CommHal {
    /// Send telemetry data
    fn send(&mut self, data: &[u8]) -> Result<usize, CommError>;

    /// Receive command data (non-blocking)
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, CommError>;

    /// Check if data available to receive
    fn available(&self) -> usize;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommError {
    WouldBlock,
    BufferFull,
    Disconnected,
    Timeout,
    InvalidData,
}

/// Combined HAL trait for convenience
///
/// Platform can implement individual traits or this combined trait.
pub trait AviateHal: SensorHal + ActuatorHal + SystemHal + CommandHal {}
