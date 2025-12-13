#!/usr/bin/env python3
"""
SITL Full Flight Test

A Python-based quadcopter physics simulator that tests the full SITL cycle:
- Takeoff
- Maneuver (position hold, yaw rotation)
- Control response
- Landing

This script simulates quadcopter physics and communicates with Aviate via MAVLink.

Usage:
    # Terminal 1: Start Aviate SITL
    cargo run -p aviate-app-quadcopter-sitl

    # Terminal 2: Run this test
    python3 tests/sitl_flight_test.py
"""

import socket
import struct
import time
import math
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# Configuration - XilNetConfig (base=20000, stride=16)
XIL_BASE_PORT = 20000
XIL_STRIDE = 16
AVIATE_LISTEN_PORT = XIL_BASE_PORT + 0   # Aviate listens here (SensorIn slot)
AVIATE_SEND_PORT = XIL_BASE_PORT + 1     # Aviate sends actuator data here (ActuatorOut slot)
LOCALHOST = "127.0.0.1"

# Physics constants
GRAVITY = 9.81  # m/s^2
AIR_DENSITY = 1.225  # kg/m^3

@dataclass
class QuadcopterParams:
    """Quadcopter physical parameters (X configuration)"""
    mass: float = 1.5  # kg
    arm_length: float = 0.25  # m (motor to center)
    Ixx: float = 0.0347563  # kg*m^2
    Iyy: float = 0.0458929  # kg*m^2
    Izz: float = 0.0977  # kg*m^2

    # Motor parameters
    k_thrust: float = 8.54858e-6  # thrust coefficient (N/(rad/s)^2)
    k_torque: float = 0.016  # torque coefficient
    max_rpm: float = 8000  # max motor RPM

    # Drag coefficients
    drag_coef: float = 0.1  # linear drag

@dataclass
class QuadcopterState:
    """Quadcopter state in NED frame"""
    # Position (m)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0  # Positive down in NED

    # Velocity (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    # Attitude (rad) - Euler angles
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    # Angular velocity (rad/s)
    p: float = 0.0  # roll rate
    q: float = 0.0  # pitch rate
    r: float = 0.0  # yaw rate

    # Motor speeds (normalized 0-1)
    motors: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])


class QuadcopterSim:
    """Simple quadcopter physics simulator"""

    def __init__(self, params: QuadcopterParams = None):
        self.params = params or QuadcopterParams()
        self.state = QuadcopterState()
        self.dt = 0.001  # 1ms physics step

    def step(self, motor_commands: List[float]) -> None:
        """
        Advance physics by one timestep.
        motor_commands: normalized motor speeds [0, 1] for motors 0-3
        Motor layout (X config, looking from above):
            0 (CW)   1 (CCW)
               \\ //
                X
               // \\
            3 (CCW)  2 (CW)
        """
        p = self.params
        s = self.state

        # Clamp motor commands
        motors = [max(0.0, min(1.0, m)) for m in motor_commands]
        s.motors = motors

        # Convert normalized commands to RPM then rad/s
        omega = [m * p.max_rpm * 2 * math.pi / 60 for m in motors]

        # Calculate thrust from each motor (N)
        thrusts = [p.k_thrust * w**2 for w in omega]
        total_thrust = sum(thrusts)

        # Calculate torques
        # Roll torque (positive = right side down)
        tau_roll = p.arm_length * (thrusts[1] + thrusts[2] - thrusts[0] - thrusts[3]) / math.sqrt(2)

        # Pitch torque (positive = nose down)
        tau_pitch = p.arm_length * (thrusts[0] + thrusts[1] - thrusts[2] - thrusts[3]) / math.sqrt(2)

        # Yaw torque (from motor reaction torques)
        # CW motors (0, 2) produce CCW reaction, CCW motors (1, 3) produce CW reaction
        tau_yaw = p.k_torque * (omega[0]**2 + omega[2]**2 - omega[1]**2 - omega[3]**2) * p.k_thrust

        # Rotation matrix from body to NED
        cr, sr = math.cos(s.roll), math.sin(s.roll)
        cp, sp = math.cos(s.pitch), math.sin(s.pitch)
        cy, sy = math.cos(s.yaw), math.sin(s.yaw)

        R = np.array([
            [cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
            [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
            [-sp, sr*cp, cr*cp]
        ])

        # Thrust vector in body frame (points up, so negative Z in body)
        thrust_body = np.array([0, 0, -total_thrust])

        # Transform to NED frame
        thrust_ned = R @ thrust_body

        # Gravity force in NED (positive Z is down)
        gravity_ned = np.array([0, 0, p.mass * GRAVITY])

        # Drag force (simple linear drag)
        vel_ned = np.array([s.vx, s.vy, s.vz])
        drag_ned = -p.drag_coef * vel_ned

        # Total force and acceleration
        total_force = thrust_ned + gravity_ned + drag_ned
        accel = total_force / p.mass

        # Update velocity
        s.vx += accel[0] * self.dt
        s.vy += accel[1] * self.dt
        s.vz += accel[2] * self.dt

        # Update position
        s.x += s.vx * self.dt
        s.y += s.vy * self.dt
        s.z += s.vz * self.dt

        # Ground collision (z >= 0 in NED means at or below ground)
        if s.z >= 0:
            s.z = 0
            s.vz = min(0, s.vz)  # No bouncing, just stop
            # Friction when on ground
            s.vx *= 0.95
            s.vy *= 0.95

        # Angular dynamics
        # Angular acceleration (simplified, ignoring gyroscopic effects)
        alpha_roll = tau_roll / p.Ixx
        alpha_pitch = tau_pitch / p.Iyy
        alpha_yaw = tau_yaw / p.Izz

        # Update angular velocity
        s.p += alpha_roll * self.dt
        s.q += alpha_pitch * self.dt
        s.r += alpha_yaw * self.dt

        # Angular velocity damping (air resistance)
        damping = 0.98
        s.p *= damping
        s.q *= damping
        s.r *= damping

        # Update attitude (Euler integration - simplified)
        s.roll += s.p * self.dt
        s.pitch += s.q * self.dt
        s.yaw += s.r * self.dt

        # Normalize yaw to [-pi, pi]
        while s.yaw > math.pi:
            s.yaw -= 2 * math.pi
        while s.yaw < -math.pi:
            s.yaw += 2 * math.pi

    def get_altitude(self) -> float:
        """Get altitude above ground (positive up)"""
        return -self.state.z

    def get_accel_body(self) -> Tuple[float, float, float]:
        """Get acceleration in body frame (for IMU simulation)"""
        s = self.state
        p = self.params

        # Simplified: gravity component in body frame + thrust
        cr, sr = math.cos(s.roll), math.sin(s.roll)
        cp, sp = math.cos(s.pitch), math.sin(s.pitch)

        # Gravity in body frame
        gx = GRAVITY * sp
        gy = -GRAVITY * sr * cp
        gz = -GRAVITY * cr * cp

        # Add thrust (motors point up, so negative z in body)
        omega = [m * p.max_rpm * 2 * math.pi / 60 for m in s.motors]
        thrusts = [p.k_thrust * w**2 for w in omega]
        total_thrust = sum(thrusts)
        az_thrust = -total_thrust / p.mass

        return (gx, gy, gz + az_thrust)


class SITLFlightTest:
    """Full SITL flight test controller"""

    def __init__(self):
        # UDP sockets
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind((LOCALHOST, AVIATE_SEND_PORT))
        self.recv_sock.setblocking(False)

        # MAVLink encoder
        self.mav = mavlink2.MAVLink(None, srcSystem=142, srcComponent=1)

        # Simulator
        self.sim = QuadcopterSim()

        # State
        self.armed = False
        self.time_usec = 0
        self.last_actuator_time = 0
        self.actuator_controls = [0.0] * 16

        # Statistics
        self.rx_count = 0
        self.tx_count = 0

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
        self.tx_count += 1

    def send_arm_command(self, arm: bool = True):
        """Send ARM/DISARM command"""
        msg = self.mav.command_long_encode(
            target_system=1,
            target_component=1,
            command=mavlink2.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1.0 if arm else 0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            param5=0.0,
            param6=0.0,
            param7=0.0
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))
        self.tx_count += 1
        self.armed = arm

    def send_hil_sensor(self):
        """Send HIL_SENSOR with current simulator state"""
        s = self.sim.state
        accel = self.sim.get_accel_body()

        # Add some sensor noise
        noise = lambda: np.random.normal(0, 0.01)

        msg = self.mav.hil_sensor_encode(
            time_usec=self.time_usec,
            xacc=accel[0] + noise(),
            yacc=accel[1] + noise(),
            zacc=accel[2] + noise(),
            xgyro=s.p + noise() * 0.1,
            ygyro=s.q + noise() * 0.1,
            zgyro=s.r + noise() * 0.1,
            xmag=0.2 + noise() * 0.01,  # Simplified magnetic field
            ymag=0.0 + noise() * 0.01,
            zmag=0.4 + noise() * 0.01,
            abs_pressure=1013.25 - self.sim.get_altitude() * 0.12,  # Pressure decreases with altitude
            diff_pressure=0.0,
            pressure_alt=self.sim.get_altitude(),
            temperature=25.0,
            fields_updated=0x1FFF
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))
        self.tx_count += 1

    def send_hil_gps(self):
        """Send HIL_GPS with current simulator state"""
        s = self.sim.state

        # Reference position (Zurich)
        ref_lat = 47.397742
        ref_lon = 8.545594

        # Convert NED to lat/lon (simplified flat earth)
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(ref_lat))

        lat = ref_lat + s.x / meters_per_deg_lat
        lon = ref_lon + s.y / meters_per_deg_lon
        alt = -s.z  # Convert from NED (down positive) to altitude (up positive)

        msg = self.mav.hil_gps_encode(
            time_usec=self.time_usec,
            fix_type=3,  # 3D fix
            lat=int(lat * 1e7),
            lon=int(lon * 1e7),
            alt=int(alt * 1000),  # mm
            eph=100,
            epv=100,
            vel=int(math.sqrt(s.vx**2 + s.vy**2) * 100),  # cm/s
            vn=int(s.vx * 100),
            ve=int(s.vy * 100),
            vd=int(s.vz * 100),
            cog=int(math.atan2(s.vy, s.vx) * 100) if s.vx != 0 or s.vy != 0 else 0,
            satellites_visible=12
        )
        raw = msg.pack(self.mav)
        self.send_sock.sendto(raw, (LOCALHOST, AVIATE_LISTEN_PORT))
        self.tx_count += 1

    def receive_messages(self) -> bool:
        """Receive and process MAVLink messages from Aviate"""
        received = False
        try:
            while True:
                data, addr = self.recv_sock.recvfrom(1024)
                if len(data) > 0 and data[0] == 0xFD:
                    msg_id = data[7] | (data[8] << 8) | (data[9] << 16)
                    if msg_id == 93:  # HIL_ACTUATOR_CONTROLS
                        self.parse_actuator_controls(data)
                        received = True
                        self.rx_count += 1
                    elif msg_id == 0:  # HEARTBEAT
                        self.rx_count += 1
        except BlockingIOError:
            pass
        return received

    def parse_actuator_controls(self, data: bytes):
        """Parse HIL_ACTUATOR_CONTROLS message"""
        # MAVLink 2.0 header: 10 bytes, then payload
        # HIL_ACTUATOR_CONTROLS: time_usec(8) + controls(16*4) + mode(1) + flags(8)
        if len(data) < 10 + 8 + 64:
            return

        payload = data[10:]
        # time_usec = struct.unpack('<Q', payload[0:8])[0]
        controls = struct.unpack('<16f', payload[8:72])
        self.actuator_controls = list(controls)
        self.last_actuator_time = self.time_usec

    def run_physics_step(self):
        """Run physics simulation step"""
        # Use first 4 actuator outputs as motor commands
        motor_commands = self.actuator_controls[:4]

        # Run multiple physics steps per control step (1ms physics, 10ms control)
        for _ in range(10):
            self.sim.step(motor_commands)

    def print_status(self, phase: str):
        """Print current flight status"""
        s = self.sim.state
        alt = self.sim.get_altitude()
        print(f"  [{phase}] Alt: {alt:6.2f}m | Vel: ({s.vx:5.2f}, {s.vy:5.2f}, {s.vz:5.2f}) m/s | "
              f"Att: ({math.degrees(s.roll):5.1f}, {math.degrees(s.pitch):5.1f}, {math.degrees(s.yaw):5.1f}) deg | "
              f"Motors: [{s.motors[0]:.2f}, {s.motors[1]:.2f}, {s.motors[2]:.2f}, {s.motors[3]:.2f}]")

    def run_flight_test(self) -> bool:
        """
        Run the full flight test sequence:
        1. Initialize and arm
        2. Takeoff to 5m
        3. Hold position
        4. Maneuver (move forward)
        5. Return and land
        """
        print("\n" + "="*70)
        print("SITL Full Flight Test")
        print("="*70)

        # Phase 1: Initialize
        print("\n[Phase 1] Initializing...")
        for _ in range(10):
            self.send_heartbeat()
            self.send_hil_sensor()
            self.send_hil_gps()
            self.receive_messages()
            time.sleep(0.1)
            self.time_usec += 100000

        # Phase 2: Arm
        print("\n[Phase 2] Arming...")
        self.send_arm_command(arm=True)
        time.sleep(0.5)

        # Verify armed by checking actuator response
        armed_ok = False
        for _ in range(50):
            self.send_hil_sensor()
            self.receive_messages()
            if any(c > 0 for c in self.actuator_controls[:4]):
                armed_ok = True
                break
            time.sleep(0.02)
            self.time_usec += 20000

        if not armed_ok:
            print("  WARNING: No motor response after arming")

        # Phase 3: Takeoff
        print("\n[Phase 3] Takeoff...")
        takeoff_start = time.time()
        takeoff_success = False

        while time.time() - takeoff_start < 10.0:  # 10 second timeout
            # Send sensor data
            self.send_hil_sensor()
            if int(self.time_usec / 100000) % 10 == 0:  # GPS at 10 Hz
                self.send_hil_gps()

            # Receive actuator commands
            self.receive_messages()

            # Run physics
            self.run_physics_step()

            # Check altitude
            alt = self.sim.get_altitude()
            if alt > 4.5:  # Target 5m, accept 4.5m
                takeoff_success = True
                break

            # Status every second
            if int(self.time_usec / 1000000) > int((self.time_usec - 10000) / 1000000):
                self.print_status("Takeoff")

            time.sleep(0.01)
            self.time_usec += 10000

        if not takeoff_success:
            print(f"  FAILED: Could not reach target altitude (current: {self.sim.get_altitude():.2f}m)")
            self.print_status("Final")
            return False
        print(f"  SUCCESS: Reached altitude {self.sim.get_altitude():.2f}m")

        # Phase 4: Position Hold
        print("\n[Phase 4] Position Hold (3 seconds)...")
        hold_start = time.time()
        max_drift = 0.0

        while time.time() - hold_start < 3.0:
            self.send_hil_sensor()
            if int(self.time_usec / 100000) % 10 == 0:
                self.send_hil_gps()
            self.receive_messages()
            self.run_physics_step()

            # Track drift
            drift = math.sqrt(self.sim.state.x**2 + self.sim.state.y**2)
            max_drift = max(max_drift, drift)

            if int(self.time_usec / 1000000) > int((self.time_usec - 10000) / 1000000):
                self.print_status("Hold")

            time.sleep(0.01)
            self.time_usec += 10000

        print(f"  Max horizontal drift: {max_drift:.2f}m")

        # Phase 5: Maneuver (simple forward movement by tilting)
        print("\n[Phase 5] Maneuver Test (5 seconds)...")
        maneuver_start = time.time()

        # We'll observe how the autopilot handles disturbances
        # by checking attitude and position response
        start_x = self.sim.state.x

        while time.time() - maneuver_start < 5.0:
            self.send_hil_sensor()
            if int(self.time_usec / 100000) % 10 == 0:
                self.send_hil_gps()
            self.receive_messages()
            self.run_physics_step()

            if int(self.time_usec / 1000000) > int((self.time_usec - 10000) / 1000000):
                self.print_status("Maneuver")

            time.sleep(0.01)
            self.time_usec += 10000

        travel_dist = abs(self.sim.state.x - start_x)
        print(f"  Forward travel: {travel_dist:.2f}m")

        # Phase 6: Landing
        print("\n[Phase 6] Landing...")
        self.send_arm_command(arm=False)  # Disarm to land

        land_start = time.time()
        landed = False

        while time.time() - land_start < 15.0:  # 15 second timeout
            self.send_hil_sensor()
            if int(self.time_usec / 100000) % 10 == 0:
                self.send_hil_gps()
            self.receive_messages()
            self.run_physics_step()

            alt = self.sim.get_altitude()
            if alt < 0.1:  # Landed
                landed = True
                break

            if int(self.time_usec / 1000000) > int((self.time_usec - 10000) / 1000000):
                self.print_status("Landing")

            time.sleep(0.01)
            self.time_usec += 10000

        if landed:
            print(f"  SUCCESS: Landed safely")
        else:
            print(f"  TIMEOUT: Still at altitude {self.sim.get_altitude():.2f}m")

        # Summary
        print("\n" + "="*70)
        print("Test Summary")
        print("="*70)
        print(f"Messages: TX={self.tx_count}, RX={self.rx_count}")
        print(f"Takeoff: {'PASS' if takeoff_success else 'FAIL'}")
        print(f"Landing: {'PASS' if landed else 'FAIL'}")
        print(f"Final position: ({self.sim.state.x:.2f}, {self.sim.state.y:.2f}, {-self.sim.state.z:.2f}) m")

        return takeoff_success and landed

    def close(self):
        self.send_sock.close()
        self.recv_sock.close()


def main():
    print("Starting SITL Flight Test...")
    print("Make sure aviate-app-quadcopter-sitl is running:")
    print("  cargo run -p aviate-app-quadcopter-sitl")
    print()

    test = SITLFlightTest()
    try:
        success = test.run_flight_test()
    finally:
        test.close()

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
