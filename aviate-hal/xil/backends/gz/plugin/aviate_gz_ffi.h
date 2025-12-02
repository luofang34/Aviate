// Aviate Gazebo Bridge - FFI Header
//
// This header defines the C interface for calling Rust functions from
// the AviateGzPlugin (C++). The Rust library (libaviate_backend_gz.so)
// provides these functions for direct sensor/actuator communication.
//
// Usage:
//   1. Link against libaviate_backend_gz.so
//   2. Include this header
//   3. Call aviate_gz_bridge_init() in Configure()
//   4. Call aviate_gz_bridge_feed_sensors() in PostUpdate()
//   5. Call aviate_gz_bridge_get_motors() in PreUpdate()
//   6. Call aviate_gz_bridge_shutdown() on plugin unload

#ifndef AVIATE_GZ_FFI_H
#define AVIATE_GZ_FFI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Valid mask bits for GzSensorData
#define AVIATE_VALID_IMU   (1 << 0)
#define AVIATE_VALID_BARO  (1 << 1)
#define AVIATE_VALID_MAG   (1 << 2)
#define AVIATE_VALID_GNSS  (1 << 3)

/// Sensor data from Gazebo (all values in SI units)
/// IMU/Mag are in body frame (ENU convention - Rust will convert to NED)
/// GNSS velocity should be in NED frame
typedef struct GzSensorData {
    /// Simulation time (microseconds)
    uint64_t time_us;

    /// IMU accelerometer [x, y, z] (m/s², body frame ENU)
    float accel[3];
    /// IMU gyroscope [x, y, z] (rad/s, body frame ENU)
    float gyro[3];
    /// IMU temperature (Celsius)
    float imu_temp;

    /// Barometer pressure (Pa)
    float pressure_pa;
    /// Barometer temperature (Celsius)
    float baro_temp;

    /// Magnetometer field [x, y, z] (µT, body frame ENU)
    float mag_field[3];

    /// GNSS latitude (degrees)
    double lat_deg;
    /// GNSS longitude (degrees)
    double lon_deg;
    /// GNSS altitude (meters MSL)
    float alt_m;
    /// GNSS velocity [vn, ve, vd] (m/s, NED frame)
    float vel_ned[3];
    /// GNSS fix type (0=none, 2=2D, 3=3D, 5=RTK float, 6=RTK fixed)
    uint8_t fix_type;
    /// GNSS horizontal accuracy (meters)
    float h_acc;
    /// GNSS vertical accuracy (meters)
    float v_acc;
    /// Number of satellites
    uint8_t satellites;

    /// Valid flags (bitmask): bit0=imu, bit1=baro, bit2=mag, bit3=gnss
    uint8_t valid_mask;
} GzSensorData;

/// Motor command for Gazebo (values in rad/s)
typedef struct GzMotorCmd {
    /// Motor velocities (rad/s) - up to 16 motors
    float velocities[16];
    /// Number of motors
    uint8_t count;
    /// Armed state (0=disarmed, 1=armed)
    uint8_t armed;
    /// Timestamp (microseconds)
    uint64_t time_us;
} GzMotorCmd;

/// Initialize the Gazebo bridge for a specific instance
///
/// @param instance Instance ID (0 for single vehicle, 0..N for multi-vehicle)
/// @return 0 on success, -1 if already initialized, -2 if lock failed
int32_t aviate_gz_bridge_init(int32_t instance);

/// Shutdown the Gazebo bridge
void aviate_gz_bridge_shutdown(void);

/// Feed sensor data from Gazebo to the flight controller
///
/// Called by the C++ plugin in PostUpdate() to provide sensor data.
/// The data is converted from ENU to NED internally.
///
/// @param data Pointer to GzSensorData struct with current sensor values
/// @return 0 on success, -1 if not initialized, -2 if lock failed, -3 if null
int32_t aviate_gz_bridge_feed_sensors(const GzSensorData* data);

/// Get motor commands for Gazebo
///
/// Called by the C++ plugin in PreUpdate() to get motor commands.
///
/// @param cmd Pointer to GzMotorCmd struct to fill with motor commands
/// @return 0 on success (command available), 1 if no new command,
///         -1 if not initialized, -2 if lock failed, -3 if null
int32_t aviate_gz_bridge_get_motors(GzMotorCmd* cmd);

/// Set motor commands from flight controller (alternative API)
///
/// @param cmd Pointer to GzMotorCmd struct with motor commands
/// @return 0 on success, negative on error
int32_t aviate_gz_bridge_set_motors(const GzMotorCmd* cmd);

#ifdef __cplusplus
}
#endif

#endif // AVIATE_GZ_FFI_H
