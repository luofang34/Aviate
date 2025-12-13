#!/usr/bin/env python3
import time
import sys
from pymavlink import mavutil

def wait_heartbeat(connection, timeout=10):
    print("Waiting for heartbeat...")
    start = time.time()
    while time.time() - start < timeout:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        if msg:
            print(f"Heartbeat from System {msg.get_srcSystem()}, Component {msg.get_srcComponent()}")
            return True
    return False

def main():
    # Connect to the vehicle
    # In SITL, mavrouter usually exposes 14550 for GCS
    print("Connecting to 127.0.0.1:14550")
    connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')

    if not wait_heartbeat(connection):
        print("Error: No heartbeat received!")
        sys.exit(1)

    print("Checking telemetry...")
    # Wait for some attitude data (ATTITUDE_QUATERNION is msg 31)
    msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True, timeout=5.0)
    if msg:
        print(f"Attitude: q=[{msg.q1:.3f}, {msg.q2:.3f}, {msg.q3:.3f}, {msg.q4:.3f}]")
    else:
        print("Error: No ATTITUDE_QUATERNION message received")
        sys.exit(1)

    print("Python Heterogeneous Test PASSED")
    sys.exit(0)

if __name__ == "__main__":
    main()
