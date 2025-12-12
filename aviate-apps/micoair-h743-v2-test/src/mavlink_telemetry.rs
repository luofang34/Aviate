//! MAVLink telemetry output via USB CDC
//!
//! Sends IMU data and state estimates to ground station for monitoring

use aviate_core::sensor::ImuData;
use aviate_link::mavlink::protocol::{
    serialize_mavlink, AttitudeQuaternion, Heartbeat,
    AVIATE_SYSTEM_ID, AVIATE_COMPONENT_ID,
};
use usbd_serial::SerialPort;
use usb_device::class_prelude::UsbBus;

/// Send HEARTBEAT message (#0)
pub fn send_heartbeat<B: UsbBus>(
    serial: &mut SerialPort<B>,
    system_status: u8,
    base_mode: u8,
) -> bool {
    let msg = Heartbeat {
        mav_type: 2,      // MAV_TYPE_QUADROTOR
        autopilot: 18,    // MAV_AUTOPILOT_AVIATE (custom)
        base_mode,        // Armed/disarmed status
        custom_mode: 0,
        system_status,    // MAV_STATE
        mavlink_version: 3,
    };

    let mut buf = [0u8; 256];
    if let Ok(len) = serialize_mavlink(&msg, &mut buf, AVIATE_SYSTEM_ID, AVIATE_COMPONENT_ID, 0) {
        serial.write(&buf[..len]).is_ok()
    } else {
        false
    }
}

/// Send ATTITUDE_QUATERNION message (#31) from raw IMU data
/// For now, sends identity quaternion with gyro rates (no integration yet)
pub fn send_imu_attitude<B: UsbBus>(
    serial: &mut SerialPort<B>,
    imu: &ImuData,
    time_boot_ms: u32,
) -> bool {
    // For initial testing: send identity quaternion (no rotation)
    // Gyro rates are in body frame
    let msg = AttitudeQuaternion {
        time_boot_ms,
        q1: 1.0,  // w component (identity = no rotation)
        q2: 0.0,  // x component
        q3: 0.0,  // y component
        q4: 0.0,  // z component
        rollspeed: imu.gyro[0].0,   // Body frame X [rad/s]
        pitchspeed: imu.gyro[1].0,  // Body frame Y [rad/s]
        yawspeed: imu.gyro[2].0,    // Body frame Z [rad/s]
        repr_offset_q: [0.0; 4],
    };

    let mut buf = [0u8; 256];
    if let Ok(len) = serialize_mavlink(&msg, &mut buf, AVIATE_SYSTEM_ID, AVIATE_COMPONENT_ID, 0) {
        serial.write(&buf[..len]).is_ok()
    } else {
        false
    }
}
