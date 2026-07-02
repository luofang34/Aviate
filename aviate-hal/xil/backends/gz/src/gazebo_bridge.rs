//! Gazebo Bridge - Direct FFI for C++ plugin integration
//!
//! This module provides C FFI functions that the AviateGzPlugin (C++) calls directly
//! to feed sensor data and retrieve motor commands. This is the direct path between
//! Gazebo and the flight controller, bypassing MAVLink entirely.
//!
//! ## Architecture
//!
//! ```text
//! AviateGzPlugin (C++, in gz-sim)
//!        ↓ Direct FFI
//! gazebo_bridge.rs (this module)
//!        ↓ ENU→NED conversion
//!        ↓ Rust API
//! SitlIO (simulator-neutral middleware)
//!        ↓
//! FakeSensors / Mixer
//! ```
//!
//! ## Thread Safety
//!
//! The bridge uses a global singleton protected by a Mutex. All FFI functions
//! acquire the lock before accessing state. The C++ plugin must not call
//! these functions from multiple threads simultaneously.
//!
//! ## Coordinate Frame Conversion
//!
//! Gazebo uses ENU (East-North-Up), avionics uses NED (North-East-Down):
//! - Position: ENU \[x,y,z\] → NED \[y,x,-z\]
//! - Velocity: same conversion
//! - Angular velocity: ENU \[wx,wy,wz\] → NED \[wx,-wy,-wz\] (body frame)

use std::ffi::c_int;
use std::sync::Mutex;

use log::info;
use once_cell::sync::Lazy;

use aviate_hal_xil::{
    SimActuatorCmd, SimBaroData, SimGnssData, SimGnssFix, SimImuData, SimMagData, SimSensorPacket,
};

/// Global singleton for the bridge state
static BRIDGE_STATE: Lazy<Mutex<Option<GazeboBridgeState>>> = Lazy::new(|| Mutex::new(None));

/// Bridge state holding buffered data
struct GazeboBridgeState {
    /// Instance ID for multi-vehicle support
    instance: i32,

    /// Buffered sensor packet (set by C++ plugin via feed_sensors)
    sensor_packet: Option<SimSensorPacket>,

    /// Buffered actuator command (set by FC via set_motors, read by C++ plugin)
    actuator_cmd: Option<SimActuatorCmd>,
}

impl GazeboBridgeState {
    fn new(instance: i32) -> Self {
        Self {
            instance,
            sensor_packet: None,
            actuator_cmd: None,
        }
    }
}

// ============================================================================
// FFI Data Structures (C-compatible)
// ============================================================================

/// Sensor data from Gazebo (all values in SI units, ENU frame)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GzSensorData {
    /// Simulation time (microseconds)
    pub time_us: u64,

    /// IMU accelerometer [x, y, z] (m/s², body frame)
    pub accel: [f32; 3],
    /// IMU gyroscope [x, y, z] (rad/s, body frame)
    pub gyro: [f32; 3],
    /// IMU temperature (Celsius)
    pub imu_temp: f32,

    /// Barometer pressure (Pa)
    pub pressure_pa: f32,
    /// Barometer temperature (Celsius)
    pub baro_temp: f32,

    /// Magnetometer field [x, y, z] (µT, body frame)
    pub mag_field: [f32; 3],

    /// GNSS latitude (degrees)
    pub lat_deg: f64,
    /// GNSS longitude (degrees)
    pub lon_deg: f64,
    /// GNSS altitude (meters MSL)
    pub alt_m: f32,
    /// GNSS velocity [vn, ve, vd] (m/s, NED frame - already converted by plugin)
    pub vel_ned: [f32; 3],
    /// GNSS fix type (0=none, 2=2D, 3=3D, 5=RTK float, 6=RTK fixed)
    pub fix_type: u8,
    /// GNSS horizontal accuracy (meters)
    pub h_acc: f32,
    /// GNSS vertical accuracy (meters)
    pub v_acc: f32,
    /// Number of satellites
    pub satellites: u8,

    /// Valid flags (bitmask): bit0=imu, bit1=baro, bit2=mag, bit3=gnss
    pub valid_mask: u8,
}

/// Motor command for Gazebo (values in rad/s)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct GzMotorCmd {
    /// Motor velocities (rad/s) - up to 16 motors
    pub velocities: [f32; 16],
    /// Number of motors
    pub count: u8,
    /// Armed state
    pub armed: u8,
    /// Timestamp (microseconds)
    pub time_us: u64,
}

impl Default for GzMotorCmd {
    fn default() -> Self {
        Self {
            velocities: [0.0; 16],
            count: 4,
            armed: 0,
            time_us: 0,
        }
    }
}

// Valid mask bits
const VALID_IMU: u8 = 1 << 0;
const VALID_BARO: u8 = 1 << 1;
const VALID_MAG: u8 = 1 << 2;
const VALID_GNSS: u8 = 1 << 3;

// ============================================================================
// Coordinate Frame Conversion (ENU → NED)
// ============================================================================

/// Convert ENU accelerometer to NED body frame
/// Body frame: X forward, Y right, Z down
/// For accelerometer in body frame, just negate Z
#[inline]
fn enu_accel_to_ned(enu: [f32; 3]) -> [f32; 3] {
    // Gazebo reports accel in body frame
    // ENU body: x forward, y left, z up
    // NED body: x forward, y right, z down
    [enu[0], -enu[1], -enu[2]]
}

/// Convert ENU gyroscope to NED body frame
#[inline]
fn enu_gyro_to_ned(enu: [f32; 3]) -> [f32; 3] {
    // Same as accel - body frame conversion
    [enu[0], -enu[1], -enu[2]]
}

/// Convert ENU magnetometer to NED body frame
#[inline]
fn enu_mag_to_ned(enu: [f32; 3]) -> [f32; 3] {
    // Same as accel - body frame conversion
    [enu[0], -enu[1], -enu[2]]
}

// ============================================================================
// FFI Functions (called by C++ plugin)
// ============================================================================

/// Initialize the Gazebo bridge for a specific instance
///
/// # Arguments
/// * `instance` - Instance ID (0 for single vehicle, 0..N for multi-vehicle)
///
/// # Returns
/// * 0 on success
/// * -1 if already initialized
/// * -2 if lock failed
#[no_mangle]
pub extern "C" fn aviate_gz_bridge_init(instance: c_int) -> c_int {
    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return -2; // Lock failed
    };

    if guard.is_some() {
        return -1; // Already initialized
    }

    *guard = Some(GazeboBridgeState::new(instance));
    info!("[GazeboBridge] Initialized for instance {}", instance);
    0
}

/// Shutdown the Gazebo bridge
#[no_mangle]
pub extern "C" fn aviate_gz_bridge_shutdown() {
    if let Ok(mut guard) = BRIDGE_STATE.lock() {
        if let Some(state) = guard.take() {
            info!("[GazeboBridge] Shutdown instance {}", state.instance);
        }
    }
}

/// Feed sensor data from Gazebo to the flight controller
///
/// Called by the C++ plugin in PostUpdate() to provide sensor data.
/// The data is converted from ENU to NED and buffered for the FC to read.
///
/// # Arguments
/// * `data` - Pointer to GzSensorData struct with current sensor values
///
/// # Returns
/// * 0 on success
/// * -1 if not initialized
/// * -2 if lock failed
/// * -3 if null pointer
///
/// # Safety
/// The caller must ensure `data` points to a valid GzSensorData struct if not null.
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn aviate_gz_bridge_feed_sensors(data: *const GzSensorData) -> c_int {
    if data.is_null() {
        return -3;
    }

    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return -2;
    };

    let Some(state) = guard.as_mut() else {
        return -1;
    };

    // SAFETY: We checked for null above
    let data = unsafe { &*data };

    // Build sensor packet with ENU→NED conversion
    let mut packet = SimSensorPacket {
        timestamp_us: data.time_us,
        imu: None,
        baro: None,
        mag: None,
        gnss: None,
    };

    // IMU (convert ENU body frame to NED body frame)
    if data.valid_mask & VALID_IMU != 0 {
        packet.imu = Some(SimImuData {
            accel: enu_accel_to_ned(data.accel),
            gyro: enu_gyro_to_ned(data.gyro),
            temperature: Some(data.imu_temp),
        });
    }

    // Barometer (no conversion needed)
    if data.valid_mask & VALID_BARO != 0 {
        packet.baro = Some(SimBaroData {
            pressure_pa: data.pressure_pa,
            temperature_c: data.baro_temp,
        });
    }

    // Magnetometer (convert ENU body frame to NED body frame)
    if data.valid_mask & VALID_MAG != 0 {
        packet.mag = Some(SimMagData {
            field_ut: enu_mag_to_ned(data.mag_field),
        });
    }

    // GNSS (velocity already in NED from plugin)
    if data.valid_mask & VALID_GNSS != 0 {
        let fix = match data.fix_type {
            0 | 1 => SimGnssFix::None,
            2 => SimGnssFix::TwoD,
            3 | 4 => SimGnssFix::ThreeD,
            5 => SimGnssFix::RtkFloat,
            6 => SimGnssFix::RtkFixed,
            _ => SimGnssFix::None,
        };

        packet.gnss = Some(SimGnssData {
            lat_deg: data.lat_deg,
            lon_deg: data.lon_deg,
            alt_m: data.alt_m,
            // This backend hands over plain GNSS-shaped fields; the
            // SITL FC binary populates `position_ned` itself when it
            // synthesizes a packet from ground truth.
            position_ned: [0.0; 3],
            vel_ned: data.vel_ned,
            fix,
            h_acc: data.h_acc,
            v_acc: data.v_acc,
            satellites: data.satellites,
        });
    }

    state.sensor_packet = Some(packet);
    0
}

/// Get motor commands for Gazebo
///
/// Called by the C++ plugin in PreUpdate() to get motor commands from the FC.
///
/// # Arguments
/// * `cmd` - Pointer to GzMotorCmd struct to fill with motor commands
///
/// # Returns
/// * 0 on success (command available)
/// * 1 if no new command available (use last command)
/// * -1 if not initialized
/// * -2 if lock failed
/// * -3 if null pointer
///
/// # Safety
/// The caller must ensure `cmd` points to a valid GzMotorCmd struct if not null.
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn aviate_gz_bridge_get_motors(cmd: *mut GzMotorCmd) -> c_int {
    if cmd.is_null() {
        return -3;
    }

    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return -2;
    };

    let Some(state) = guard.as_mut() else {
        return -1;
    };

    // Check if we have a command
    let Some(actuator_cmd) = state.actuator_cmd.take() else {
        return 1; // No new command
    };

    // SAFETY: We checked for null above
    let cmd = unsafe { &mut *cmd };

    // Convert normalized [0,1] to motor velocity (rad/s)
    // Assume max motor speed of 1000 rad/s (~9550 RPM)
    const MAX_MOTOR_RADS: f32 = 1000.0;

    for (vel, &output) in cmd.velocities.iter_mut().zip(actuator_cmd.outputs.iter()) {
        *vel = output * MAX_MOTOR_RADS;
    }
    cmd.count = actuator_cmd.count;
    cmd.armed = if actuator_cmd.armed { 1 } else { 0 };
    cmd.time_us = actuator_cmd.timestamp_us;

    0
}

/// Set motor command from the flight controller
///
/// Called by the FC/board to set motor commands that the C++ plugin will read.
/// This is the Rust API counterpart to feed_sensors.
///
/// # Returns
/// * 0 on success
/// * -1 if not initialized
/// * -2 if lock failed
/// * -3 if null pointer
///
/// # Safety
/// The caller must ensure `cmd` points to a valid GzMotorCmd struct if not null.
#[no_mangle]
#[allow(clippy::not_unsafe_ptr_arg_deref)]
pub extern "C" fn aviate_gz_bridge_set_motors(cmd: *const GzMotorCmd) -> c_int {
    if cmd.is_null() {
        return -3;
    }

    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return -2;
    };

    let Some(state) = guard.as_mut() else {
        return -1;
    };

    // SAFETY: We checked for null above
    let cmd = unsafe { &*cmd };

    // Convert motor velocity (rad/s) back to normalized [0,1]
    const MAX_MOTOR_RADS: f32 = 1000.0;

    let mut outputs = [0.0f32; 16];
    for (output, &vel) in outputs.iter_mut().zip(cmd.velocities.iter()) {
        *output = (vel / MAX_MOTOR_RADS).clamp(0.0, 1.0);
    }

    state.actuator_cmd = Some(SimActuatorCmd {
        timestamp_us: cmd.time_us,
        outputs,
        count: cmd.count,
        armed: cmd.armed != 0,
    });

    0
}

/// Take sensor packet (Rust API)
///
/// Called by SitlIO to get the latest sensor data from Gazebo.
/// Returns None if no new data available.
pub fn take_sensor_packet() -> Option<SimSensorPacket> {
    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return None;
    };

    guard.as_mut().and_then(|s| s.sensor_packet.take())
}

/// Set actuator command (Rust API)
///
/// Called by the board to set actuator commands for Gazebo to read.
pub fn set_actuator_cmd(cmd: SimActuatorCmd) -> bool {
    let Ok(mut guard) = BRIDGE_STATE.lock() else {
        return false;
    };

    if let Some(state) = guard.as_mut() {
        state.actuator_cmd = Some(cmd);
        true
    } else {
        false
    }
}

/// Check if bridge is initialized
pub fn is_initialized() -> bool {
    BRIDGE_STATE.lock().map(|g| g.is_some()).unwrap_or(false)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_enu_accel_to_ned() {
        // ENU body: x forward, y left, z up
        // NED body: x forward, y right, z down
        let enu = [1.0, 2.0, 3.0];
        let ned = enu_accel_to_ned(enu);
        assert_eq!(ned, [1.0, -2.0, -3.0]);
    }

    #[test]
    fn test_valid_mask() {
        assert_eq!(VALID_IMU, 0b0001);
        assert_eq!(VALID_BARO, 0b0010);
        assert_eq!(VALID_MAG, 0b0100);
        assert_eq!(VALID_GNSS, 0b1000);
    }

    #[test]
    fn test_gz_motor_cmd_default() {
        let cmd = GzMotorCmd::default();
        assert_eq!(cmd.count, 4);
        assert_eq!(cmd.armed, 0);
    }
}
