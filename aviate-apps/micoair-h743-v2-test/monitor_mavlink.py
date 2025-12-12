#!/usr/bin/env python3
"""
MAVLink Monitor for MicoAir H743-V2 Test App

Receives and displays MAVLink telemetry from the flight controller via USB CDC.
Shows real-time IMU data (gyro rates, accelerations) and attitude information.

Usage:
    python3 monitor_mavlink.py [--port /dev/ttyACM0] [--baud 115200]

Requirements:
    pip3 install pymavlink pyserial
"""

import argparse
import sys
import time
from pymavlink import mavutil

def find_serial_port():
    """Auto-detect USB CDC serial port"""
    import glob

    # Try common USB CDC device names
    patterns = [
        '/dev/ttyACM*',    # Linux
        '/dev/cu.usbmodem*',  # macOS
        'COM*',            # Windows
    ]

    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return ports[0]

    return None

def format_attitude(msg):
    """Format ATTITUDE_QUATERNION message for display"""
    import math

    # Extract quaternion components
    w, x, y, z = msg.q1, msg.q2, msg.q3, msg.q4

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convert to degrees
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    # Gyro rates (already in rad/s)
    rollspeed = msg.rollspeed
    pitchspeed = msg.pitchspeed
    yawspeed = msg.yawspeed

    return {
        'roll': roll_deg,
        'pitch': pitch_deg,
        'yaw': yaw_deg,
        'rollspeed': rollspeed,
        'pitchspeed': pitchspeed,
        'yawspeed': yawspeed,
        'time_ms': msg.time_boot_ms,
    }

def format_heartbeat(msg):
    """Format HEARTBEAT message for display"""
    mav_types = {
        0: 'GENERIC',
        1: 'FIXED_WING',
        2: 'QUADROTOR',
        3: 'COAXIAL',
        4: 'HELICOPTER',
        13: 'HEXAROTOR',
        14: 'OCTOROTOR',
    }

    mav_states = {
        0: 'UNINIT',
        1: 'BOOT',
        2: 'CALIBRATING',
        3: 'STANDBY',
        4: 'ACTIVE',
        5: 'CRITICAL',
        6: 'EMERGENCY',
        7: 'POWEROFF',
        8: 'FLIGHT_TERMINATION',
    }

    vehicle_type = mav_types.get(msg.type, f'UNKNOWN({msg.type})')
    state = mav_states.get(msg.system_status, f'UNKNOWN({msg.system_status})')
    armed = 'ARMED' if (msg.base_mode & 128) else 'DISARMED'

    return {
        'type': vehicle_type,
        'state': state,
        'armed': armed,
        'custom_mode': msg.custom_mode,
    }

def main():
    parser = argparse.ArgumentParser(description='MAVLink monitor for MicoAir H743-V2')
    parser.add_argument('--port', help='Serial port (auto-detect if not specified)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--verbose', '-v', action='store_true', help='Show all messages')
    args = parser.parse_args()

    # Find serial port
    port = args.port
    if not port:
        port = find_serial_port()
        if not port:
            print("ERROR: No USB CDC device found!")
            print("Please specify port with --port /dev/ttyACM0")
            sys.exit(1)
        print(f"Auto-detected port: {port}")

    print(f"Connecting to {port} at {args.baud} baud...")

    try:
        # Create MAVLink connection
        mav = mavutil.mavlink_connection(port, baud=args.baud)

        print("Waiting for heartbeat...")
        mav.wait_heartbeat()
        print(f"✓ Heartbeat received from system {mav.target_system}, component {mav.target_component}")
        print()
        print("=== MAVLink Monitor Active ===")
        print("Tip: Rotate/move the board to see IMU data change!")
        print()

        last_heartbeat_time = 0
        last_attitude_time = 0
        message_count = {'HEARTBEAT': 0, 'ATTITUDE_QUATERNION': 0, 'OTHER': 0}

        while True:
            msg = mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                print(".", end='', flush=True)
                continue

            msg_type = msg.get_type()
            current_time = time.time()

            if msg_type == 'HEARTBEAT':
                message_count['HEARTBEAT'] += 1
                # Display heartbeat at most once per second
                if current_time - last_heartbeat_time >= 1.0:
                    hb = format_heartbeat(msg)
                    print(f"\n[{time.strftime('%H:%M:%S')}] HEARTBEAT: "
                          f"{hb['type']} | {hb['state']} | {hb['armed']}")
                    last_heartbeat_time = current_time

            elif msg_type == 'ATTITUDE_QUATERNION':
                message_count['ATTITUDE_QUATERNION'] += 1
                # Display attitude at ~5 Hz (every 200ms)
                if current_time - last_attitude_time >= 0.2:
                    att = format_attitude(msg)
                    print(f"  ATT: Roll={att['roll']:7.2f}° Pitch={att['pitch']:7.2f}° Yaw={att['yaw']:7.2f}° "
                          f"| Gyro: X={att['rollspeed']:7.3f} Y={att['pitchspeed']:7.3f} Z={att['yawspeed']:7.3f} rad/s "
                          f"[{att['time_ms']}ms]", end='\r')
                    last_attitude_time = current_time

            elif msg_type == 'LOCAL_POSITION_NED':
                print(f"\n  POS: N={msg.x:7.2f}m E={msg.y:7.2f}m D={msg.z:7.2f}m "
                      f"| Vel: {msg.vx:6.2f} {msg.vy:6.2f} {msg.vz:6.2f} m/s")

            else:
                message_count['OTHER'] += 1
                if args.verbose:
                    print(f"\n  {msg_type}: {msg}")

    except KeyboardInterrupt:
        print("\n\n=== Statistics ===")
        print(f"HEARTBEAT messages: {message_count['HEARTBEAT']}")
        print(f"ATTITUDE_QUATERNION messages: {message_count['ATTITUDE_QUATERNION']}")
        print(f"Other messages: {message_count['OTHER']}")
        print("\nMonitor stopped by user")

    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
