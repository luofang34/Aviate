//! Simulator-neutral data types for XIL (X-In-Loop) simulation
//!
//! These types are used for direct communication between simulator backends
//! (like Gazebo) and SitlIO, bypassing MAVLink for sensor/actuator data.
//!
//! World position and velocity use NED (North-East-Down). Inertial and
//! magnetic vectors use the flight-controller body-FRD frame. A backend
//! converts its native world and body frames before it creates these values.

/// Timestamp in microseconds since simulation start
pub type SimTimestampUs = u64;

/// Complete sensor-lane presence bits for one simulator sample.
pub mod sensor_presence {
    /// Accelerometer X, Y, and Z lanes.
    pub const ACCELEROMETER: u16 = 0b111;
    /// Gyroscope X, Y, and Z lanes.
    pub const GYROSCOPE: u16 = 0b111 << 3;
    /// Magnetometer X, Y, and Z lanes.
    pub const MAGNETOMETER: u16 = 0b111 << 6;
    /// Absolute pressure lane.
    pub const ABSOLUTE_PRESSURE: u16 = 1 << 9;
    /// Differential pressure lane.
    pub const DIFFERENTIAL_PRESSURE: u16 = 1 << 10;
    /// Pressure altitude lane.
    pub const PRESSURE_ALTITUDE: u16 = 1 << 11;
}

/// IMU sensor data (accelerometer + gyroscope)
#[derive(Debug, Clone, Copy, Default)]
pub struct SimImuData {
    /// Accelerometer X, Y, Z in m/s² (body-FRD frame)
    pub accel: [f32; 3],
    /// Gyroscope X, Y, Z in rad/s (body-FRD frame)
    pub gyro: [f32; 3],
    /// Optional temperature in Celsius
    pub temperature: Option<f32>,
}

/// Barometer sensor data
#[derive(Debug, Clone, Copy, Default)]
pub struct SimBaroData {
    /// Static pressure in Pascals
    pub pressure_pa: f32,
    /// Differential pressure in Pascals when the sample contains it.
    pub differential_pressure_pa: Option<f32>,
    /// Explicit pressure altitude in meters when the sample contains it.
    pub pressure_altitude_m: Option<f32>,
    /// Temperature in Celsius
    pub temperature_c: f32,
}

/// Magnetometer sensor data
#[derive(Debug, Clone, Copy, Default)]
pub struct SimMagData {
    /// Magnetic field X, Y, Z in microtesla (body-FRD frame)
    pub field_ut: [f32; 3],
}

/// GNSS fix type
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum SimGnssFix {
    #[default]
    None = 0,
    TwoD = 2,
    ThreeD = 3,
    RtkFloat = 5,
    RtkFixed = 6,
}

/// GNSS sensor data
#[derive(Debug, Clone, Copy, Default)]
pub struct SimGnssData {
    /// Latitude in degrees
    pub lat_deg: f64,
    /// Longitude in degrees
    pub lon_deg: f64,
    /// Altitude above MSL in meters
    pub alt_m: f32,
    /// Local NED position in meters (N, E, D). Aviate's kernel consumes
    /// `position_ned` directly; the lat/lon/alt fields are kept for
    /// telemetry/diagnostics and for real GNSS receivers that have not
    /// done the projection upstream.
    pub position_ned: [f32; 3],
    /// Velocity NED in m/s
    pub vel_ned: [f32; 3],
    /// Fix type
    pub fix: SimGnssFix,
    /// Horizontal accuracy estimate in meters
    pub h_acc: f32,
    /// Vertical accuracy estimate in meters
    pub v_acc: f32,
    /// Number of satellites visible
    pub satellites: u8,
}

/// Combined sensor data packet from simulator
///
/// Each field is optional - simulators may not provide all sensors
/// at every update. Backends fill in only the sensors they have data for.
#[derive(Debug, Clone, Copy, Default)]
pub struct SimSensorPacket {
    /// Timestamp in microseconds since simulation start
    pub timestamp_us: SimTimestampUs,
    /// Raw lane presence bitmap after backend protocol decoding.
    pub presence_mask: u16,
    /// IMU data (accel + gyro)
    pub imu: Option<SimImuData>,
    /// Barometer data
    pub baro: Option<SimBaroData>,
    /// Magnetometer data
    pub mag: Option<SimMagData>,
    /// GNSS data (typically at lower rate)
    pub gnss: Option<SimGnssData>,
}

impl SimSensorPacket {
    /// Create an empty packet with timestamp
    pub const fn new(timestamp_us: SimTimestampUs) -> Self {
        Self {
            timestamp_us,
            presence_mask: 0,
            imu: None,
            baro: None,
            mag: None,
            gnss: None,
        }
    }

    /// Builder: set IMU data
    pub const fn with_imu(mut self, imu: SimImuData) -> Self {
        self.imu = Some(imu);
        self.presence_mask |= sensor_presence::ACCELEROMETER | sensor_presence::GYROSCOPE;
        self
    }

    /// Builder: set barometer data
    pub const fn with_baro(mut self, baro: SimBaroData) -> Self {
        self.presence_mask |= sensor_presence::ABSOLUTE_PRESSURE;
        if baro.differential_pressure_pa.is_some() {
            self.presence_mask |= sensor_presence::DIFFERENTIAL_PRESSURE;
        }
        if baro.pressure_altitude_m.is_some() {
            self.presence_mask |= sensor_presence::PRESSURE_ALTITUDE;
        }
        self.baro = Some(baro);
        self
    }

    /// Builder: set magnetometer data
    pub const fn with_mag(mut self, mag: SimMagData) -> Self {
        self.mag = Some(mag);
        self.presence_mask |= sensor_presence::MAGNETOMETER;
        self
    }

    /// Builder: set GNSS data
    pub const fn with_gnss(mut self, gnss: SimGnssData) -> Self {
        self.gnss = Some(gnss);
        self
    }
}

/// Actuator command from flight controller to simulator
#[derive(Debug, Clone, Copy)]
pub struct SimActuatorCmd {
    /// Timestamp in microseconds
    pub timestamp_us: SimTimestampUs,
    /// Motor/actuator outputs (normalized 0.0-1.0 or rad/s depending on config)
    pub outputs: [f32; 16],
    /// Number of active outputs
    pub count: u8,
    /// Armed state
    pub armed: bool,
}

impl Default for SimActuatorCmd {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            outputs: [0.0; 16],
            count: 4, // Default to quadcopter
            armed: false,
        }
    }
}

impl SimActuatorCmd {
    /// Create a new actuator command
    pub const fn new(timestamp_us: SimTimestampUs, count: u8, armed: bool) -> Self {
        Self {
            timestamp_us,
            outputs: [0.0; 16],
            count,
            armed,
        }
    }

    /// Set motor outputs (copies from slice)
    pub fn set_outputs(&mut self, outputs: &[f32]) {
        let len = outputs.len().min(16);
        self.outputs[..len].copy_from_slice(&outputs[..len]);
        self.count = len as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sim_sensor_packet_builder() {
        let imu = SimImuData {
            accel: [0.0, 0.0, -9.81],
            gyro: [0.0, 0.0, 0.0],
            temperature: Some(25.0),
        };

        let packet = SimSensorPacket::new(1000).with_imu(imu);

        assert_eq!(packet.timestamp_us, 1000);
        assert!(packet.imu.is_some());
        assert!(packet.baro.is_none());
        assert!(packet.mag.is_none());
        assert!(packet.gnss.is_none());
    }

    #[test]
    fn test_sim_actuator_cmd() {
        let mut cmd = SimActuatorCmd::new(5000, 4, true);
        cmd.set_outputs(&[0.5, 0.5, 0.5, 0.5]);

        assert_eq!(cmd.timestamp_us, 5000);
        assert_eq!(cmd.count, 4);
        assert!(cmd.armed);
        assert!((cmd.outputs[0] - 0.5).abs() < f32::EPSILON);
    }

    #[test]
    fn test_sim_gnss_fix_default() {
        let fix = SimGnssFix::default();
        assert_eq!(fix, SimGnssFix::None);
    }
}
