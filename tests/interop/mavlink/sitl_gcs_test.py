#!/usr/bin/env python3
import time
import sys
from pymavlink import mavutil

# Configuration - XilNetConfig (base=20000, stride=16)
XIL_BASE_PORT = 20000
XIL_STRIDE = 16
AVIATE_IP = "127.0.0.1"
AVIATE_PORT = XIL_BASE_PORT + 0  # Aviate listens here (SensorIn slot)

def main():
    print(f"Connecting to Aviate at {AVIATE_IP}:{AVIATE_PORT}")
    # Source system 255 (GCS), component 190
    master = mavutil.mavlink_connection(f"udpout:{AVIATE_IP}:{AVIATE_PORT}", source_system=255, source_component=190)
    
    BOOT_TIME = time.time()

    print("Sending Heartbeats...")
    for _ in range(5):
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(0.1)

    print("Arming...")
    # MAV_CMD_COMPONENT_ARM_DISARM = 400
    master.mav.command_long_send(
        1, 1, # Target system, component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, # Confirmation
        1.0, # Param1: 1.0 = Arm
        0, 0, 0, 0, 0, 0 # Param2-7
    )
    
    time.sleep(1)
    
    print("Taking off (50% Thrust)...")
    start_time = time.time()
    while time.time() - start_time < 5:
        # Send SET_ATTITUDE_TARGET
        # Type mask: Ignore rates (1|2|4), map q to attitude
        # But for now we just set thrust
        current_time_ms = int((time.time() - BOOT_TIME) * 1000)
        master.mav.set_attitude_target_send(
            current_time_ms,
            1, 1,
            0, # Type mask (0 = enable all fields? No, bits ignore)
            # We want to control thrust and attitude
            # q = [1, 0, 0, 0] (Level)
            [1.0, 0.0, 0.0, 0.0],
            0.0, 0.0, 0.0, # Body rates
            0.6 # Thrust (0.6 > hover typically)
        )
        time.sleep(0.02) # 50Hz

    print("Landing (0% Thrust)...")
    current_time_ms = int((time.time() - BOOT_TIME) * 1000)
    master.mav.set_attitude_target_send(
        current_time_ms,
        1, 1, 0,
        [1.0, 0.0, 0.0, 0.0],
        0.0, 0.0, 0.0,
        0.0 # Thrust
    )
    
    time.sleep(1)

    print("Disarming...")
    master.mav.command_long_send(
        1, 1,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0, # Param1: 0.0 = Disarm
        0, 0, 0, 0, 0, 0
    )

    print("Test Complete")

if __name__ == "__main__":
    main()
