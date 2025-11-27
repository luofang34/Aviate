//! MAVLink message definitions for HIL simulation

/// HIL_SENSOR (MAVLink #107)
/// IMU, barometer, and magnetometer data from simulator
#[derive(Copy, Clone, Debug, Default)]
pub struct HilSensor {
    /// Timestamp (microseconds since boot or Unix epoch)
    pub time_usec: u64,
    /// X acceleration (m/s^2)
    pub xacc: f32,
    /// Y acceleration (m/s^2)
    pub yacc: f32,
    /// Z acceleration (m/s^2)
    pub zacc: f32,
    /// X angular speed (rad/s)
    pub xgyro: f32,
    /// Y angular speed (rad/s)
    pub ygyro: f32,
    /// Z angular speed (rad/s)
    pub zgyro: f32,
    /// X magnetic field (gauss)
    pub xmag: f32,
    /// Y magnetic field (gauss)
    pub ymag: f32,
    /// Z magnetic field (gauss)
    pub zmag: f32,
    /// Absolute pressure (mbar)
    pub abs_pressure: f32,
    /// Differential pressure (mbar)
    pub diff_pressure: f32,
    /// Altitude (pressure altitude, meters)
    pub pressure_alt: f32,
    /// Temperature (degrees Celsius)
    pub temperature: f32,
    /// Bitmap of updated sensor fields
    pub fields_updated: u32,
    /// Sensor ID (0 for default)
    pub id: u8,
}

impl HilSensor {
    pub const MSG_ID: u32 = 107;
    pub const PAYLOAD_LEN: usize = 65;
}

/// HIL_GPS (MAVLink #113)
/// GNSS position and velocity from simulator
#[derive(Copy, Clone, Debug, Default)]
pub struct HilGps {
    /// Timestamp (microseconds since boot or Unix epoch)
    pub time_usec: u64,
    /// Latitude (degrees * 1e7)
    pub lat: i32,
    /// Longitude (degrees * 1e7)
    pub lon: i32,
    /// Altitude MSL (mm)
    pub alt: i32,
    /// GPS HDOP (cm)
    pub eph: u16,
    /// GPS VDOP (cm)
    pub epv: u16,
    /// Ground speed (cm/s)
    pub vel: u16,
    /// North velocity (cm/s)
    pub vn: i16,
    /// East velocity (cm/s)
    pub ve: i16,
    /// Down velocity (cm/s)
    pub vd: i16,
    /// Course over ground (degrees * 100)
    pub cog: u16,
    /// GPS fix type (0=No GPS, 1=No Fix, 2=2D, 3=3D, 4=DGPS, 5=RTK)
    pub fix_type: u8,
    /// Number of satellites visible
    pub satellites_visible: u8,
    /// GPS ID (0 for default)
    pub id: u8,
    /// Yaw of vehicle relative to Earth's North in degrees * 100
    pub yaw: u16,
}

impl HilGps {
    pub const MSG_ID: u32 = 113;
    pub const PAYLOAD_LEN: usize = 39;
}

/// HIL_ACTUATOR_CONTROLS (MAVLink #93)
/// Actuator outputs from autopilot to simulator
#[derive(Copy, Clone, Debug)]
pub struct HilActuatorControls {
    /// Timestamp (microseconds since boot or Unix epoch)
    pub time_usec: u64,
    /// Control outputs [-1..1] or [0..1]
    pub controls: [f32; 16],
    /// System mode (MAV_MODE_FLAG)
    pub mode: u8,
    /// Flags (reserved)
    pub flags: u64,
}

impl Default for HilActuatorControls {
    fn default() -> Self {
        Self {
            time_usec: 0,
            controls: [0.0; 16],
            mode: 0,
            flags: 0,
        }
    }
}

impl HilActuatorControls {
    pub const MSG_ID: u32 = 93;
    pub const PAYLOAD_LEN: usize = 81;
}

/// HEARTBEAT (MAVLink #0)
/// System alive signal
#[derive(Copy, Clone, Debug, Default)]
pub struct Heartbeat {
    /// Type of the system (quadrotor, fixed-wing, etc.)
    pub mav_type: u8,
    /// Autopilot type (PX4, ArduPilot, Aviate, etc.)
    pub autopilot: u8,
    /// System mode bitmap (MAV_MODE_FLAG)
    pub base_mode: u8,
    /// Custom mode (autopilot-specific)
    pub custom_mode: u32,
    /// System status (MAV_STATE)
    pub system_status: u8,
    /// MAVLink version (usually 3 for MAVLink 2.0)
    pub mavlink_version: u8,
}

impl Heartbeat {
    pub const MSG_ID: u32 = 0;
    pub const PAYLOAD_LEN: usize = 9;
}

/// SYSTEM_TIME (MAVLink #2)
/// System time synchronization
#[derive(Copy, Clone, Debug, Default)]
pub struct SystemTime {
    /// Unix timestamp (microseconds since Jan 1 1970)
    pub time_unix_usec: u64,
    /// Time since boot (milliseconds)
    pub time_boot_ms: u32,
}

impl SystemTime {
    pub const MSG_ID: u32 = 2;
    pub const PAYLOAD_LEN: usize = 12;
}

/// HIL_STATE_QUATERNION (MAVLink #115)
/// Ground truth state from simulator (for validation/logging)
#[derive(Copy, Clone, Debug, Default)]
pub struct HilStateQuaternion {
    /// Timestamp (microseconds since boot or Unix epoch)
    pub time_usec: u64,
    /// Attitude quaternion [w, x, y, z]
    pub attitude_quaternion: [f32; 4],
    /// Roll angular speed (rad/s)
    pub rollspeed: f32,
    /// Pitch angular speed (rad/s)
    pub pitchspeed: f32,
    /// Yaw angular speed (rad/s)
    pub yawspeed: f32,
    /// Latitude (degrees * 1e7)
    pub lat: i32,
    /// Longitude (degrees * 1e7)
    pub lon: i32,
    /// Altitude MSL (mm)
    pub alt: i32,
    /// Ground X speed (NED, cm/s)
    pub vx: i16,
    /// Ground Y speed (NED, cm/s)
    pub vy: i16,
    /// Ground Z speed (NED, cm/s)
    pub vz: i16,
    /// Indicated airspeed (cm/s)
    pub ind_airspeed: u16,
    /// True airspeed (cm/s)
    pub true_airspeed: u16,
    /// X acceleration (mG)
    pub xacc: i16,
    /// Y acceleration (mG)
    pub yacc: i16,
    /// Z acceleration (mG)
    pub zacc: i16,
}

impl HilStateQuaternion {
    pub const MSG_ID: u32 = 115;
    pub const PAYLOAD_LEN: usize = 64;
}

/// Enum of all supported MAVLink messages
#[derive(Copy, Clone, Debug)]
pub enum MavMessage {
    Heartbeat(Heartbeat),
    SystemTime(SystemTime),
    HilSensor(HilSensor),
    HilGps(HilGps),
    HilActuatorControls(HilActuatorControls),
    HilStateQuaternion(HilStateQuaternion),
    Unknown { msg_id: u32 },
}

impl MavMessage {
    /// Get the message ID
    pub fn msg_id(&self) -> u32 {
        match self {
            MavMessage::Heartbeat(_) => Heartbeat::MSG_ID,
            MavMessage::SystemTime(_) => SystemTime::MSG_ID,
            MavMessage::HilSensor(_) => HilSensor::MSG_ID,
            MavMessage::HilGps(_) => HilGps::MSG_ID,
            MavMessage::HilActuatorControls(_) => HilActuatorControls::MSG_ID,
            MavMessage::HilStateQuaternion(_) => HilStateQuaternion::MSG_ID,
            MavMessage::Unknown { msg_id } => *msg_id,
        }
    }
}
