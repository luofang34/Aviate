#!/usr/bin/env python3
"""
SITL UDP Communication Test

Tests live MAVLink communication between pymavlink and aviate-mavlink.
This script acts as a simple simulator, sending HIL_SENSOR data and
receiving HIL_ACTUATOR_CONTROLS.

Usage:
    # Terminal 1: Start Aviate SITL
    cargo run -p aviate-app-quadcopter-sitl

    # Terminal 2: Run this test
    python3 tests/sitl_udp_test.py
"""

import socket
import struct
import time
import threading
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# Configuration matching aviate-platform-sitl
AVIATE_LISTEN_PORT = 14560   # Aviate listens here for sensor data
AVIATE_SEND_PORT = 14561     # Aviate sends actuator data here
LOCALHOST = "127.0.0.1"

class SimpleSimulator:
    """Minimal simulator for testing MAVLink communication"""

    def __init__(self):
        # Socket to send sensor data to Aviate
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Socket to receive actuator commands from Aviate
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind((LOCALHOST, AVIATE_SEND_PORT))
        self.recv_sock.setblocking(False)

        # MAVLink encoder
        self.mav = mavlink2.MAVLink(None, srcSystem=142, srcComponent=1)
        self.seq = 0

        # State
        self.running = False
        self.received_actuators = []
        self.received_heartbeats = []

    def send_heartbeat(self):
        """Send simulator heartbeat"""
        msg = self.mav.heartbeat_encode(
            type=mavlink2.MAV_TYPE_GCS,
            autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavlink2.MAV_STATE_ACTIVE
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))

    def send_hil_sensor(self, time_usec):
        """Send HIL_SENSOR with simulated IMU data"""
        msg = self.mav.hil_sensor_encode(
            time_usec=time_usec,
            xacc=0.01,      # Small noise
            yacc=-0.02,
            zacc=-9.81,     # Gravity
            xgyro=0.001,
            ygyro=-0.001,
            zgyro=0.0,
            xmag=0.2,
            ymag=0.0,
            zmag=0.4,
            abs_pressure=1013.25,
            diff_pressure=0.0,
            pressure_alt=0.0,
            temperature=25.0,
            fields_updated=0x1FFF  # All sensor fields
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))

    def send_hil_gps(self, time_usec):
        """Send HIL_GPS with simulated position"""
        msg = self.mav.hil_gps_encode(
            time_usec=time_usec,
            fix_type=3,  # 3D fix
            lat=int(47.397742 * 1e7),
            lon=int(8.545594 * 1e7),
            alt=500000,  # 500m in mm
            eph=100,
            epv=100,
            vel=0,
            vn=0,
            ve=0,
            vd=0,
            cog=0,
            satellites_visible=12
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))

    def receive_messages(self):
        """Try to receive messages from Aviate"""
        try:
            data, addr = self.recv_sock.recvfrom(1024)
            if len(data) > 0:
                # Parse MAVLink message
                if data[0] == 0xFD:  # MAVLink 2.0
                    msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
                    return (msg_id, data)
        except BlockingIOError:
            pass
        return None

    def run_test(self, duration_sec=5):
        """Run the test for specified duration"""
        print(f"\n{'='*60}")
        print("SITL UDP Communication Test")
        print(f"{'='*60}")
        print(f"Sending to: {LOCALHOST}:{AVIATE_LISTEN_PORT}")
        print(f"Receiving on: {LOCALHOST}:{AVIATE_SEND_PORT}")
        print(f"Duration: {duration_sec} seconds")
        print()

        start_time = time.time()
        time_usec = 0
        sensor_count = 0
        gps_count = 0
        heartbeat_count = 0

        print("Sending sensor data...")

        while (time.time() - start_time) < duration_sec:
            # Send heartbeat at 1 Hz
            if int(time_usec / 1000000) > heartbeat_count:
                self.send_heartbeat()
                heartbeat_count += 1
                print(f"  [TX] HEARTBEAT #{heartbeat_count}")

            # Send HIL_SENSOR at ~100 Hz
            self.send_hil_sensor(time_usec)
            sensor_count += 1

            # Send HIL_GPS at ~10 Hz
            if sensor_count % 10 == 0:
                self.send_hil_gps(time_usec)
                gps_count += 1

            # Check for responses
            result = self.receive_messages()
            if result:
                msg_id, data = result
                if msg_id == 93:  # HIL_ACTUATOR_CONTROLS
                    self.received_actuators.append(data)
                    if len(self.received_actuators) <= 5:
                        print(f"  [RX] HIL_ACTUATOR_CONTROLS (len={len(data)})")
                elif msg_id == 0:  # HEARTBEAT
                    self.received_heartbeats.append(data)
                    print(f"  [RX] HEARTBEAT from Aviate")
                else:
                    print(f"  [RX] Message ID {msg_id}")

            # Simulate ~100 Hz loop
            time.sleep(0.01)
            time_usec += 10000  # 10ms in microseconds

        # Summary
        print(f"\n{'='*60}")
        print("Test Summary")
        print(f"{'='*60}")
        print(f"Sent:")
        print(f"  - HIL_SENSOR: {sensor_count}")
        print(f"  - HIL_GPS: {gps_count}")
        print(f"  - HEARTBEAT: {heartbeat_count}")
        print(f"Received:")
        print(f"  - HIL_ACTUATOR_CONTROLS: {len(self.received_actuators)}")
        print(f"  - HEARTBEAT: {len(self.received_heartbeats)}")

        if len(self.received_actuators) > 0:
            print(f"\n✓ Communication successful!")
            return True
        else:
            print(f"\n✗ No actuator commands received")
            print("  Make sure aviate-app-quadcopter-sitl is running:")
            print("  cargo run -p aviate-app-quadcopter-sitl")
            return False

    def close(self):
        self.send_sock.close()
        self.recv_sock.close()


def send_arm_command():
    """Send ARM command via COMMAND_LONG"""
    print("\n--- Sending ARM command ---")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)

    msg = mav.command_long_encode(
        target_system=1,
        target_component=1,
        command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
        confirmation=0,
        param1=1.0,  # ARM
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0
    )
    raw = msg.pack(mav)
    sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))
    print(f"  [TX] COMMAND_LONG (ARM) -> {LOCALHOST}:{AVIATE_LISTEN_PORT}")
    sock.close()


def main():
    # First try just sending an ARM command
    send_arm_command()
    time.sleep(0.5)

    # Then run the full sensor simulation
    sim = SimpleSimulator()
    try:
        success = sim.run_test(duration_sec=5)
    finally:
        sim.close()

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
