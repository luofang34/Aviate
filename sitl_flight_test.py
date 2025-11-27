#!/usr/bin/env python3
import time
import sys
from pymavlink import mavutil

AVIATE_CMD_IP = "127.0.0.1"
AVIATE_CMD_PORT = 14560
AVIATE_TELEM_PORT = 14550

MAV_MODE_FLAG_SAFETY_ARMED = 128

def wait_for_heartbeat(conn):
    print("Waiting for heartbeat...")
    msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if not msg:
        print("No heartbeat received!")
        sys.exit(1)
    print("Heartbeat received.")

def arm_vehicle(cmd_conn, telem_conn):
    print("Arming...")
    for i in range(10):
        # Send Arm Command
        cmd_conn.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0, 0, 0, 0, 0, 0, 0
        )
        
        # Check status
        msg = telem_conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg and (msg.base_mode & MAV_MODE_FLAG_SAFETY_ARMED):
            print("Vehicle Armed!")
            return True
        print(f"Retry arming ({i+1}/10)...")
    
    print("Failed to arm vehicle.")
    return False

def main():
    print(f"Connecting to Aviate Command at {AVIATE_CMD_IP}:{AVIATE_CMD_PORT}")
    cmd_conn = mavutil.mavlink_connection(f"udpout:{AVIATE_CMD_IP}:{AVIATE_CMD_PORT}", source_system=255, source_component=190)
    
    print(f"Listening for Telemetry on port {AVIATE_TELEM_PORT}")
    telem_conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{AVIATE_TELEM_PORT}")

    BOOT_TIME = time.time()

    # Send some GCS heartbeats to wake up link if needed
    for _ in range(3):
        cmd_conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(0.1)

    wait_for_heartbeat(telem_conn)

    if not arm_vehicle(cmd_conn, telem_conn):
        sys.exit(1)
    
    time.sleep(1)
    
    print("Taking off (80% Thrust)...")
    start_time = time.time()
    max_alt = -999.0
    
    # Run for 10s or until target altitude
    while time.time() - start_time < 10:
        current_time_ms = int((time.time() - BOOT_TIME) * 1000)
        
        # Send setpoint
        cmd_conn.mav.set_attitude_target_send(
            current_time_ms,
            1, 1,
            0,
            [1.0, 0.0, 0.0, 0.0], # Level
            0.0, 0.0, 0.0,
            0.8 # High Thrust
        )
        
        # Check telemetry
        msg = telem_conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            alt = -msg.z
            max_alt = max(max_alt, alt)
            if alt > 2.0:
                print(f"Target altitude reached: {alt:.2f}m")
                break
        
        time.sleep(0.02)

    print(f"Max altitude reached: {max_alt:.2f}m")
    if max_alt < 0.5:
        print("Failed to takeoff! (Alt < 0.5m)")
        sys.exit(1)

    print("Hovering (60% Thrust)...")
    hover_start = time.time()
    while time.time() - hover_start < 3:
        current_time_ms = int((time.time() - BOOT_TIME) * 1000)
        cmd_conn.mav.set_attitude_target_send(
            current_time_ms, 1, 1, 0, [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0.6
        )
        time.sleep(0.02)

    print("Landing (0% Thrust)...")
    land_start = time.time()
    landed = False
    
    while time.time() - land_start < 10:
        current_time_ms = int((time.time() - BOOT_TIME) * 1000)
        
        cmd_conn.mav.set_attitude_target_send(
            current_time_ms,
            1, 1, 0,
            [1.0, 0.0, 0.0, 0.0],
            0.0, 0.0, 0.0,
            0.0 # Low thrust
        )
        
        msg = telem_conn.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            alt = -msg.z
            if alt < 0.5:
                print(f"Landed (alt: {alt:.2f}m)")
                landed = True
                break
        
        time.sleep(0.02)
        
    if not landed:
        print("Failed to land quickly or no telemetry.")

    print("Disarming...")
    cmd_conn.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0, 0, 0, 0, 0, 0, 0
    )

    print("Test Complete")

if __name__ == "__main__":
    main()
