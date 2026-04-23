//! MAVLink HIL Message Definitions
//!
//! Standard MAVLink v2 HIL (Hardware-In-The-Loop) messages for legacy simulator
//! compatibility. These messages are defined in the MAVLink common message set:
//! <https://mavlink.io/en/messages/common.html>
//!
//! Supported messages:
//! - HEARTBEAT (0): Connection heartbeat
//! - HIL_SENSOR (107): IMU/baro/mag sensor data from simulator
//! - HIL_GPS (113): GPS data from simulator
//! - HIL_STATE_QUATERNION (115): Full vehicle state from simulator
//! - HIL_ACTUATOR_CONTROLS (93): Motor/servo commands to simulator

/// HEARTBEAT message ID
pub const HEARTBEAT_ID: u8 = 0;

/// HEARTBEAT (0) - System heartbeat
///
/// Sent periodically to indicate system is alive and identify its type.
/// Required by jMAVSim to initialize HIL communication.
#[derive(Clone, Copy, Debug, Default)]
pub struct Heartbeat {
    /// Vehicle type (MAV_TYPE): 2 = quadrotor
    pub mav_type: u8,
    /// Autopilot type (MAV_AUTOPILOT): 12 = PX4-compatible
    pub autopilot: u8,
    /// System mode flags (MAV_MODE_FLAG)
    pub base_mode: u8,
    /// Custom mode (autopilot-specific)
    pub custom_mode: u32,
    /// System status (MAV_STATE)
    pub system_status: u8,
    /// MAVLink version
    pub mavlink_version: u8,
}

impl Heartbeat {
    /// Payload size in bytes
    pub const SIZE: usize = 9;

    /// MAV_TYPE: Quadrotor
    pub const MAV_TYPE_QUADROTOR: u8 = 2;
    /// MAV_AUTOPILOT: Generic (PX4-compatible)
    pub const MAV_AUTOPILOT_GENERIC: u8 = 0;
    /// MAV_MODE_FLAG: Safety armed
    pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;
    /// MAV_MODE_FLAG: HIL enabled
    pub const MAV_MODE_FLAG_HIL_ENABLED: u8 = 0x20;
    /// MAV_STATE: Standby
    pub const MAV_STATE_STANDBY: u8 = 3;
    /// MAV_STATE: Active
    pub const MAV_STATE_ACTIVE: u8 = 4;

    /// Create a default HIL heartbeat for a quadrotor
    pub fn new_quadrotor_hil(armed: bool) -> Self {
        let base_mode = Self::MAV_MODE_FLAG_HIL_ENABLED
            | if armed {
                Self::MAV_MODE_FLAG_SAFETY_ARMED
            } else {
                0
            };
        Self {
            mav_type: Self::MAV_TYPE_QUADROTOR,
            autopilot: Self::MAV_AUTOPILOT_GENERIC,
            base_mode,
            custom_mode: 0,
            system_status: if armed {
                Self::MAV_STATE_ACTIVE
            } else {
                Self::MAV_STATE_STANDBY
            },
            mavlink_version: 3,
        }
    }

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        // custom_mode is first (4 bytes) due to MAVLink field reordering
        buf[0..4].copy_from_slice(&self.custom_mode.to_le_bytes());
        buf[4] = self.mav_type;
        buf[5] = self.autopilot;
        buf[6] = self.base_mode;
        buf[7] = self.system_status;
        buf[8] = self.mavlink_version;
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            custom_mode: u32::from_le_bytes(data[0..4].try_into().ok()?),
            mav_type: data[4],
            autopilot: data[5],
            base_mode: data[6],
            system_status: data[7],
            mavlink_version: data[8],
        })
    }
}

/// HIL_SENSOR message ID
pub const HIL_SENSOR_ID: u8 = 107;

/// HIL_GPS message ID
pub const HIL_GPS_ID: u8 = 113;

/// HIL_STATE_QUATERNION message ID
pub const HIL_STATE_QUATERNION_ID: u8 = 115;

/// HIL_ACTUATOR_CONTROLS message ID
pub const HIL_ACTUATOR_CONTROLS_ID: u8 = 93;

/// HIL_SENSOR (107) - IMU readings in SI units in NED body frame
///
/// Sent from simulator to autopilot.
#[derive(Clone, Copy, Debug, Default)]
pub struct HilSensor {
    /// Timestamp (UNIX Epoch time or time since system boot) \[us\]
    pub time_usec: u64,
    /// X acceleration [m/s^2]
    pub xacc: f32,
    /// Y acceleration [m/s^2]
    pub yacc: f32,
    /// Z acceleration [m/s^2]
    pub zacc: f32,
    /// Angular speed around X axis in body frame [rad/s]
    pub xgyro: f32,
    /// Angular speed around Y axis in body frame [rad/s]
    pub ygyro: f32,
    /// Angular speed around Z axis in body frame [rad/s]
    pub zgyro: f32,
    /// X Magnetic field \[gauss\]
    pub xmag: f32,
    /// Y Magnetic field \[gauss\]
    pub ymag: f32,
    /// Z Magnetic field \[gauss\]
    pub zmag: f32,
    /// Absolute pressure \[hPa\]
    pub abs_pressure: f32,
    /// Differential pressure (airspeed) \[hPa\]
    pub diff_pressure: f32,
    /// Altitude calculated from pressure
    pub pressure_alt: f32,
    /// Temperature \[degC\]
    pub temperature: f32,
    /// Bitmap for fields that have updated since last message
    pub fields_updated: u32,
    /// Sensor ID (zero indexed). Used for multiple sensor inputs
    pub id: u8,
}

impl HilSensor {
    /// Payload size in bytes
    pub const SIZE: usize = 65;

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.time_usec.to_le_bytes());
        buf[8..12].copy_from_slice(&self.xacc.to_le_bytes());
        buf[12..16].copy_from_slice(&self.yacc.to_le_bytes());
        buf[16..20].copy_from_slice(&self.zacc.to_le_bytes());
        buf[20..24].copy_from_slice(&self.xgyro.to_le_bytes());
        buf[24..28].copy_from_slice(&self.ygyro.to_le_bytes());
        buf[28..32].copy_from_slice(&self.zgyro.to_le_bytes());
        buf[32..36].copy_from_slice(&self.xmag.to_le_bytes());
        buf[36..40].copy_from_slice(&self.ymag.to_le_bytes());
        buf[40..44].copy_from_slice(&self.zmag.to_le_bytes());
        buf[44..48].copy_from_slice(&self.abs_pressure.to_le_bytes());
        buf[48..52].copy_from_slice(&self.diff_pressure.to_le_bytes());
        buf[52..56].copy_from_slice(&self.pressure_alt.to_le_bytes());
        buf[56..60].copy_from_slice(&self.temperature.to_le_bytes());
        buf[60..64].copy_from_slice(&self.fields_updated.to_le_bytes());
        buf[64] = self.id;
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            time_usec: u64::from_le_bytes(data[0..8].try_into().ok()?),
            xacc: f32::from_le_bytes(data[8..12].try_into().ok()?),
            yacc: f32::from_le_bytes(data[12..16].try_into().ok()?),
            zacc: f32::from_le_bytes(data[16..20].try_into().ok()?),
            xgyro: f32::from_le_bytes(data[20..24].try_into().ok()?),
            ygyro: f32::from_le_bytes(data[24..28].try_into().ok()?),
            zgyro: f32::from_le_bytes(data[28..32].try_into().ok()?),
            xmag: f32::from_le_bytes(data[32..36].try_into().ok()?),
            ymag: f32::from_le_bytes(data[36..40].try_into().ok()?),
            zmag: f32::from_le_bytes(data[40..44].try_into().ok()?),
            abs_pressure: f32::from_le_bytes(data[44..48].try_into().ok()?),
            diff_pressure: f32::from_le_bytes(data[48..52].try_into().ok()?),
            pressure_alt: f32::from_le_bytes(data[52..56].try_into().ok()?),
            temperature: f32::from_le_bytes(data[56..60].try_into().ok()?),
            fields_updated: u32::from_le_bytes(data[60..64].try_into().ok()?),
            id: data[64],
        })
    }
}

/// HIL_GPS (113) - GPS sensor data from simulator
///
/// Sent from simulator to autopilot. Values in GPS frame (right-handed, Z-up).
#[derive(Clone, Copy, Debug, Default)]
pub struct HilGps {
    /// Timestamp (UNIX Epoch time or time since system boot) \[us\]
    pub time_usec: u64,
    /// Latitude \[degE7\]
    pub lat: i32,
    /// Longitude \[degE7\]
    pub lon: i32,
    /// Altitude MSL \[mm\]
    pub alt: i32,
    /// GPS HDOP horizontal dilution of position [unitless * 100]
    pub eph: u16,
    /// GPS VDOP vertical dilution of position [unitless * 100]
    pub epv: u16,
    /// GPS ground speed [cm/s]
    pub vel: u16,
    /// GPS velocity in north direction (NED) [cm/s]
    pub vn: i16,
    /// GPS velocity in east direction (NED) [cm/s]
    pub ve: i16,
    /// GPS velocity in down direction (NED) [cm/s]
    pub vd: i16,
    /// Course over ground \[cdeg\], 0..35999, 65535 if unknown
    pub cog: u16,
    /// GPS fix type: 0-1=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK
    pub fix_type: u8,
    /// Number of satellites visible (255 if unknown)
    pub satellites_visible: u8,
    /// GPS ID (zero indexed, extension field)
    pub id: u8,
    /// Yaw of vehicle relative to Earth's North \[cdeg\], 0=not available, 36000=north
    pub yaw: u16,
}

impl HilGps {
    /// Payload size in bytes (including extensions)
    pub const SIZE: usize = 39;
    /// Minimum payload size (v1 compatibility)
    pub const MIN_SIZE: usize = 36;

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.time_usec.to_le_bytes());
        buf[8..12].copy_from_slice(&self.lat.to_le_bytes());
        buf[12..16].copy_from_slice(&self.lon.to_le_bytes());
        buf[16..20].copy_from_slice(&self.alt.to_le_bytes());
        buf[20..22].copy_from_slice(&self.eph.to_le_bytes());
        buf[22..24].copy_from_slice(&self.epv.to_le_bytes());
        buf[24..26].copy_from_slice(&self.vel.to_le_bytes());
        buf[26..28].copy_from_slice(&self.vn.to_le_bytes());
        buf[28..30].copy_from_slice(&self.ve.to_le_bytes());
        buf[30..32].copy_from_slice(&self.vd.to_le_bytes());
        buf[32..34].copy_from_slice(&self.cog.to_le_bytes());
        buf[34] = self.fix_type;
        buf[35] = self.satellites_visible;
        buf[36] = self.id;
        buf[37..39].copy_from_slice(&self.yaw.to_le_bytes());
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::MIN_SIZE {
            return None;
        }
        let mut msg = Self {
            time_usec: u64::from_le_bytes(data[0..8].try_into().ok()?),
            lat: i32::from_le_bytes(data[8..12].try_into().ok()?),
            lon: i32::from_le_bytes(data[12..16].try_into().ok()?),
            alt: i32::from_le_bytes(data[16..20].try_into().ok()?),
            eph: u16::from_le_bytes(data[20..22].try_into().ok()?),
            epv: u16::from_le_bytes(data[22..24].try_into().ok()?),
            vel: u16::from_le_bytes(data[24..26].try_into().ok()?),
            vn: i16::from_le_bytes(data[26..28].try_into().ok()?),
            ve: i16::from_le_bytes(data[28..30].try_into().ok()?),
            vd: i16::from_le_bytes(data[30..32].try_into().ok()?),
            cog: u16::from_le_bytes(data[32..34].try_into().ok()?),
            fix_type: data[34],
            satellites_visible: data[35],
            id: 0,
            yaw: 0,
        };
        // Parse extension fields if present
        if data.len() >= Self::SIZE {
            msg.id = data[36];
            msg.yaw = u16::from_le_bytes(data[37..39].try_into().ok()?);
        }
        Some(msg)
    }
}

/// HIL_STATE_QUATERNION (115) - Full vehicle state from simulator
///
/// Sent from simulator to autopilot. Contains attitude, position, velocity,
/// and acceleration for ground truth or sensor fusion.
#[derive(Clone, Copy, Debug, Default)]
pub struct HilStateQuaternion {
    /// Timestamp (UNIX Epoch time or time since system boot) \[us\]
    pub time_usec: u64,
    /// Vehicle attitude quaternion [w, x, y, z] (normalized, Hamilton convention)
    pub attitude_quaternion: [f32; 4],
    /// Body frame roll rate [rad/s]
    pub rollspeed: f32,
    /// Body frame pitch rate [rad/s]
    pub pitchspeed: f32,
    /// Body frame yaw rate [rad/s]
    pub yawspeed: f32,
    /// Latitude \[degE7\]
    pub lat: i32,
    /// Longitude \[degE7\]
    pub lon: i32,
    /// Altitude MSL \[mm\]
    pub alt: i32,
    /// Ground X speed (latitude direction) [cm/s]
    pub vx: i16,
    /// Ground Y speed (longitude direction) [cm/s]
    pub vy: i16,
    /// Ground Z speed (altitude direction, positive down) [cm/s]
    pub vz: i16,
    /// Indicated airspeed [cm/s]
    pub ind_airspeed: u16,
    /// True airspeed [cm/s]
    pub true_airspeed: u16,
    /// X-axis acceleration \[mG\] (milliGs)
    pub xacc: i16,
    /// Y-axis acceleration \[mG\] (milliGs)
    pub yacc: i16,
    /// Z-axis acceleration \[mG\] (milliGs)
    pub zacc: i16,
}

impl HilStateQuaternion {
    /// Payload size in bytes
    pub const SIZE: usize = 64;

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.time_usec.to_le_bytes());
        for (i, &q) in self.attitude_quaternion.iter().enumerate() {
            let offset = 8 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&q.to_le_bytes());
        }
        buf[24..28].copy_from_slice(&self.rollspeed.to_le_bytes());
        buf[28..32].copy_from_slice(&self.pitchspeed.to_le_bytes());
        buf[32..36].copy_from_slice(&self.yawspeed.to_le_bytes());
        buf[36..40].copy_from_slice(&self.lat.to_le_bytes());
        buf[40..44].copy_from_slice(&self.lon.to_le_bytes());
        buf[44..48].copy_from_slice(&self.alt.to_le_bytes());
        buf[48..50].copy_from_slice(&self.vx.to_le_bytes());
        buf[50..52].copy_from_slice(&self.vy.to_le_bytes());
        buf[52..54].copy_from_slice(&self.vz.to_le_bytes());
        buf[54..56].copy_from_slice(&self.ind_airspeed.to_le_bytes());
        buf[56..58].copy_from_slice(&self.true_airspeed.to_le_bytes());
        buf[58..60].copy_from_slice(&self.xacc.to_le_bytes());
        buf[60..62].copy_from_slice(&self.yacc.to_le_bytes());
        buf[62..64].copy_from_slice(&self.zacc.to_le_bytes());
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        let mut attitude_quaternion = [0.0f32; 4];
        for (i, q) in attitude_quaternion.iter_mut().enumerate() {
            let offset = 8 + i * 4;
            *q = f32::from_le_bytes(data[offset..offset + 4].try_into().ok()?);
        }
        Some(Self {
            time_usec: u64::from_le_bytes(data[0..8].try_into().ok()?),
            attitude_quaternion,
            rollspeed: f32::from_le_bytes(data[24..28].try_into().ok()?),
            pitchspeed: f32::from_le_bytes(data[28..32].try_into().ok()?),
            yawspeed: f32::from_le_bytes(data[32..36].try_into().ok()?),
            lat: i32::from_le_bytes(data[36..40].try_into().ok()?),
            lon: i32::from_le_bytes(data[40..44].try_into().ok()?),
            alt: i32::from_le_bytes(data[44..48].try_into().ok()?),
            vx: i16::from_le_bytes(data[48..50].try_into().ok()?),
            vy: i16::from_le_bytes(data[50..52].try_into().ok()?),
            vz: i16::from_le_bytes(data[52..54].try_into().ok()?),
            ind_airspeed: u16::from_le_bytes(data[54..56].try_into().ok()?),
            true_airspeed: u16::from_le_bytes(data[56..58].try_into().ok()?),
            xacc: i16::from_le_bytes(data[58..60].try_into().ok()?),
            yacc: i16::from_le_bytes(data[60..62].try_into().ok()?),
            zacc: i16::from_le_bytes(data[62..64].try_into().ok()?),
        })
    }
}

/// HIL_ACTUATOR_CONTROLS (93) - Hardware in the loop control outputs
///
/// Sent from autopilot to simulator.
#[derive(Clone, Copy, Debug)]
pub struct HilActuatorControls {
    /// Timestamp (UNIX Epoch time or time since system boot) \[us\]
    pub time_usec: u64,
    /// Control outputs -1..1, channel assignment depends on simulated hardware
    pub controls: [f32; 16],
    /// Flags bitmask
    pub flags: u64,
    /// System mode (includes arming state)
    pub mode: u8,
}

impl Default for HilActuatorControls {
    fn default() -> Self {
        Self {
            time_usec: 0,
            controls: [0.0; 16],
            flags: 0,
            mode: 0,
        }
    }
}

impl HilActuatorControls {
    /// Payload size in bytes
    pub const SIZE: usize = 81;

    /// Mode flag: system is armed
    pub const MODE_FLAG_ARMED: u8 = 0x80;

    /// Serialize to bytes (little-endian)
    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..8].copy_from_slice(&self.time_usec.to_le_bytes());
        for (i, &ctrl) in self.controls.iter().enumerate() {
            let offset = 8 + i * 4;
            buf[offset..offset + 4].copy_from_slice(&ctrl.to_le_bytes());
        }
        buf[72..80].copy_from_slice(&self.flags.to_le_bytes());
        buf[80] = self.mode;
        buf
    }

    /// Deserialize from bytes (little-endian)
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < Self::SIZE {
            return None;
        }
        let mut controls = [0.0f32; 16];
        for (i, ctrl) in controls.iter_mut().enumerate() {
            let offset = 8 + i * 4;
            *ctrl = f32::from_le_bytes(data[offset..offset + 4].try_into().ok()?);
        }
        Some(Self {
            time_usec: u64::from_le_bytes(data[0..8].try_into().ok()?),
            controls,
            flags: u64::from_le_bytes(data[72..80].try_into().ok()?),
            mode: data[80],
        })
    }

    /// Check if armed
    pub fn is_armed(&self) -> bool {
        self.mode & Self::MODE_FLAG_ARMED != 0
    }
}

/// HIL message types
#[derive(Clone, Debug)]
pub enum HilMessage {
    Heartbeat(Heartbeat),
    Sensor(HilSensor),
    Gps(HilGps),
    StateQuaternion(HilStateQuaternion),
    ActuatorControls(HilActuatorControls),
}

impl HilMessage {
    /// Get message ID
    pub fn msg_id(&self) -> u8 {
        match self {
            Self::Heartbeat(_) => HEARTBEAT_ID,
            Self::Sensor(_) => HIL_SENSOR_ID,
            Self::Gps(_) => HIL_GPS_ID,
            Self::StateQuaternion(_) => HIL_STATE_QUATERNION_ID,
            Self::ActuatorControls(_) => HIL_ACTUATOR_CONTROLS_ID,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hil_sensor_roundtrip() {
        let sensor = HilSensor {
            time_usec: 1234567890,
            xacc: 0.1,
            yacc: 0.2,
            zacc: -9.81,
            xgyro: 0.01,
            ygyro: 0.02,
            zgyro: 0.03,
            xmag: 0.2,
            ymag: 0.0,
            zmag: 0.4,
            abs_pressure: 1013.25,
            diff_pressure: 0.0,
            pressure_alt: 100.0,
            temperature: 25.0,
            fields_updated: 0xFFFF_FFFF,
            id: 0,
        };
        let bytes = sensor.to_bytes();
        let parsed = HilSensor::from_bytes(&bytes).expect("parse failed");
        assert_eq!(sensor.time_usec, parsed.time_usec);
        assert!((sensor.xacc - parsed.xacc).abs() < 1e-6);
        assert!((sensor.zacc - parsed.zacc).abs() < 1e-6);
        assert_eq!(sensor.fields_updated, parsed.fields_updated);
    }

    #[test]
    fn test_hil_gps_roundtrip() {
        let gps = HilGps {
            time_usec: 1234567890,
            lat: 473977420, // 47.3977420 deg
            lon: 85455940,  // 8.5455940 deg
            alt: 488000,    // 488m
            eph: 100,
            epv: 150,
            vel: 500, // 5 m/s
            vn: 100,
            ve: 200,
            vd: -50,
            cog: 9000, // 90 deg
            fix_type: 3,
            satellites_visible: 12,
            id: 0,
            yaw: 0,
        };
        let bytes = gps.to_bytes();
        let parsed = HilGps::from_bytes(&bytes).expect("parse failed");
        assert_eq!(gps.time_usec, parsed.time_usec);
        assert_eq!(gps.lat, parsed.lat);
        assert_eq!(gps.lon, parsed.lon);
        assert_eq!(gps.fix_type, parsed.fix_type);
    }

    #[test]
    fn test_hil_actuator_controls_roundtrip() {
        let mut controls = HilActuatorControls::default();
        controls.time_usec = 1234567890;
        controls.controls[0] = 0.5;
        controls.controls[1] = 0.6;
        controls.controls[2] = 0.7;
        controls.controls[3] = 0.8;
        controls.mode = HilActuatorControls::MODE_FLAG_ARMED;
        controls.flags = 0x1234;

        let bytes = controls.to_bytes();
        let parsed = HilActuatorControls::from_bytes(&bytes).expect("parse failed");
        assert_eq!(controls.time_usec, parsed.time_usec);
        assert!((controls.controls[0] - parsed.controls[0]).abs() < 1e-6);
        assert!(parsed.is_armed());
    }

    #[test]
    fn test_hil_sensor_size() {
        assert_eq!(HilSensor::SIZE, 65);
    }

    #[test]
    fn test_hil_gps_size() {
        assert_eq!(HilGps::SIZE, 39);
        assert_eq!(HilGps::MIN_SIZE, 36);
    }

    #[test]
    fn test_hil_actuator_controls_size() {
        assert_eq!(HilActuatorControls::SIZE, 81);
    }

    #[test]
    fn test_hil_state_quaternion_roundtrip() {
        let state = HilStateQuaternion {
            time_usec: 1234567890,
            attitude_quaternion: [1.0, 0.0, 0.0, 0.0], // Identity quaternion
            rollspeed: 0.01,
            pitchspeed: 0.02,
            yawspeed: 0.03,
            lat: 473977420, // 47.3977420 deg
            lon: 85455940,  // 8.5455940 deg
            alt: 488000,    // 488m in mm
            vx: 100,        // 1 m/s
            vy: 200,        // 2 m/s
            vz: -50,        // -0.5 m/s
            ind_airspeed: 1500,
            true_airspeed: 1550,
            xacc: 0,
            yacc: 0,
            zacc: -1000, // ~1g down in mG
        };
        let bytes = state.to_bytes();
        let parsed = HilStateQuaternion::from_bytes(&bytes).expect("parse failed");
        assert_eq!(state.time_usec, parsed.time_usec);
        assert!((state.attitude_quaternion[0] - parsed.attitude_quaternion[0]).abs() < 1e-6);
        assert_eq!(state.lat, parsed.lat);
        assert_eq!(state.lon, parsed.lon);
        assert_eq!(state.alt, parsed.alt);
        assert_eq!(state.vx, parsed.vx);
        assert_eq!(state.zacc, parsed.zacc);
    }

    #[test]
    fn test_hil_state_quaternion_size() {
        assert_eq!(HilStateQuaternion::SIZE, 64);
    }

    #[test]
    fn test_heartbeat_roundtrip() {
        let heartbeat = Heartbeat::new_quadrotor_hil(true);
        let bytes = heartbeat.to_bytes();
        let parsed = Heartbeat::from_bytes(&bytes).expect("parse failed");
        assert_eq!(heartbeat.mav_type, parsed.mav_type);
        assert_eq!(heartbeat.autopilot, parsed.autopilot);
        assert_eq!(heartbeat.base_mode, parsed.base_mode);
        assert_eq!(heartbeat.custom_mode, parsed.custom_mode);
        assert_eq!(heartbeat.system_status, parsed.system_status);
        assert_eq!(heartbeat.mavlink_version, parsed.mavlink_version);
    }

    #[test]
    fn test_heartbeat_size() {
        assert_eq!(Heartbeat::SIZE, 9);
    }

    #[test]
    fn test_heartbeat_armed_flags() {
        let armed = Heartbeat::new_quadrotor_hil(true);
        assert_eq!(
            armed.base_mode & Heartbeat::MAV_MODE_FLAG_SAFETY_ARMED,
            Heartbeat::MAV_MODE_FLAG_SAFETY_ARMED
        );
        assert_eq!(armed.system_status, Heartbeat::MAV_STATE_ACTIVE);

        let disarmed = Heartbeat::new_quadrotor_hil(false);
        assert_eq!(
            disarmed.base_mode & Heartbeat::MAV_MODE_FLAG_SAFETY_ARMED,
            0
        );
        assert_eq!(disarmed.system_status, Heartbeat::MAV_STATE_STANDBY);
    }
}
