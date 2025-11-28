"""
Pytest fixtures for Aviate GCS tests.

Provides:
- MAVLink connection management
- Multi-vehicle routing via mavrouter (auto-generated config)
- HIL sensor simulation for pre-arm checks
"""
import os
import subprocess
import tempfile
import time
import pytest
from pathlib import Path
from pymavlink import mavutil

try:
    import tomllib
except ImportError:
    import tomli as tomllib  # Python < 3.11


# Default ports (matching router_gen.rs)
DEFAULT_GCS_PORT = 14550      # GCS connects to mavrouter here
VEHICLE_BASE_PORT = 14560     # Vehicle N on port BASE + N


def vehicle_port(instance: int) -> int:
    """Get UDP port for vehicle instance (matching Rust router_gen.rs)."""
    return VEHICLE_BASE_PORT + instance


class MavConnection:
    """Wrapper for pymavlink connection with helper methods."""

    def __init__(self, connection_string: str, source_system: int = 255, source_component: int = 190):
        self.conn = mavutil.mavlink_connection(
            connection_string,
            source_system=source_system,
            source_component=source_component
        )
        self.boot_time = time.time()

    def time_boot_ms(self) -> int:
        """Get time since boot in milliseconds."""
        return int((time.time() - self.boot_time) * 1000)

    def send_heartbeat(self):
        """Send GCS heartbeat."""
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

    def send_hil_sensor(self, accel=(0, 0, -9.81), gyro=(0, 0, 0), mag=(0.2, 0, 0.4),
                        pressure=1013.25, temperature=25.0, sensor_id=0):
        """Send HIL_SENSOR message to simulate sensor data for pre-arm checks."""
        self.conn.mav.hil_sensor_send(
            int(time.time() * 1e6),  # time_usec
            accel[0], accel[1], accel[2],  # xacc, yacc, zacc (m/s^2)
            gyro[0], gyro[1], gyro[2],  # xgyro, ygyro, zgyro (rad/s)
            mag[0], mag[1], mag[2],  # xmag, ymag, zmag (Gauss)
            pressure,  # abs_pressure (mbar)
            0,  # diff_pressure
            0,  # pressure_alt
            temperature,  # temperature (C)
            0x1FF,  # fields_updated (all fields)
            sensor_id  # id
        )

    def send_arm_command(self, target_system: int = 1, target_component: int = 1, arm: bool = True) -> bool:
        """Send arm/disarm command and wait for ACK."""
        self.conn.mav.command_long_send(
            target_system, target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1.0 if arm else 0.0,  # param1: arm=1, disarm=0
            0, 0, 0, 0, 0, 0
        )
        return self._wait_for_command_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)

    def send_attitude_target(self, target_system: int, target_component: int,
                             q=(1, 0, 0, 0), rates=(0, 0, 0), thrust: float = 0.0):
        """Send SET_ATTITUDE_TARGET message."""
        self.conn.mav.set_attitude_target_send(
            self.time_boot_ms(),
            target_system, target_component,
            0,  # type_mask (0 = use all fields)
            q,  # quaternion [w, x, y, z]
            rates[0], rates[1], rates[2],  # body rates
            thrust
        )

    def _wait_for_command_ack(self, command: int, timeout: float = 2.0) -> bool:
        """Wait for COMMAND_ACK message."""
        start = time.time()
        while time.time() - start < timeout:
            msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.1)
            if msg and msg.command == command:
                return msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        return False

    def recv_heartbeat(self, timeout: float = 5.0):
        """Wait for heartbeat from vehicle."""
        return self.conn.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)

    def send_position_target(self, x: float, y: float, z: float,
                              yaw: float = 0.0, type_mask: int = 0x0F8,
                              target_system: int = 1, target_component: int = 1):
        """
        Send SET_POSITION_TARGET_LOCAL_NED message.

        Args:
            x, y, z: Position in NED frame (meters)
            yaw: Heading (radians)
            type_mask: Which fields to ignore (default: ignore velocity/accel)
        """
        self.conn.mav.set_position_target_local_ned_send(
            self.time_boot_ms(),
            target_system, target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x, y, z,  # position
            0, 0, 0,  # velocity (ignored by type_mask)
            0, 0, 0,  # acceleration (ignored)
            yaw, 0    # yaw, yaw_rate
        )

    def send_velocity_target(self, vx: float, vy: float, vz: float,
                              yaw_rate: float = 0.0,
                              target_system: int = 1, target_component: int = 1):
        """
        Send SET_POSITION_TARGET_LOCAL_NED for velocity control.

        Args:
            vx, vy, vz: Velocity in NED frame (m/s)
            yaw_rate: Yaw rate (rad/s)
        """
        # Ignore position and acceleration, use velocity
        type_mask = 0b0000_0111_1100_0111  # 0x07C7
        self.conn.mav.set_position_target_local_ned_send(
            self.time_boot_ms(),
            target_system, target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,      # position (ignored)
            vx, vy, vz,   # velocity
            0, 0, 0,      # acceleration (ignored)
            0, yaw_rate   # yaw (ignored), yaw_rate
        )

    def send_command_long(self, command: int, param1: float = 0, param2: float = 0,
                          param3: float = 0, param4: float = 0, param5: float = 0,
                          param6: float = 0, param7: float = 0,
                          target_system: int = 1, target_component: int = 1) -> bool:
        """Send generic COMMAND_LONG and wait for ACK."""
        self.conn.mav.command_long_send(
            target_system, target_component,
            command, 0,  # confirmation
            param1, param2, param3, param4, param5, param6, param7
        )
        return self._wait_for_command_ack(command)

    def recv_actuator_controls(self, timeout: float = 1.0):
        """Receive HIL_ACTUATOR_CONTROLS message."""
        return self.conn.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=True, timeout=timeout)

    def recv_any(self, timeout: float = 1.0):
        """Receive any MAVLink message."""
        return self.conn.recv_match(blocking=True, timeout=timeout)

    def drain_messages(self, timeout: float = 0.1):
        """Drain all pending messages from the receive buffer."""
        while True:
            msg = self.conn.recv_match(blocking=True, timeout=timeout)
            if msg is None:
                break

    def close(self):
        """Close connection."""
        self.conn.close()


def generate_router_config(test_config: dict) -> str:
    """
    Generate mavrouter TOML config from test configuration.

    Mirrors the Rust router_gen.rs logic so Python tests use the same
    port allocation as the Rust SITL test runner.
    """
    vehicles = test_config.get('vehicles', [])
    name = test_config.get('test', {}).get('name', 'unknown')

    lines = [
        "# Auto-generated mavrouter configuration",
        f"# Test: {name}",
        f"# Vehicles: {len(vehicles)}",
        "",
        "[general]",
        "bus_capacity = 1000",
        "dedup_period_ms = 50",
        "routing_table_ttl_secs = 60",
        "routing_table_prune_interval_secs = 30",
        "",
        "# GCS endpoint",
        "[[endpoint]]",
        'type = "udp"',
        f'address = "0.0.0.0:{DEFAULT_GCS_PORT}"',
        'mode = "server"',
        "",
    ]

    for v in vehicles:
        instance = v.get('instance', 0)
        vid = v.get('id', f'vehicle_{instance}')
        port = vehicle_port(instance)
        lines.extend([
            f"# {vid} (system_id = {instance + 1})",
            "[[endpoint]]",
            'type = "udp"',
            f'address = "127.0.0.1:{port}"',
            'mode = "client"',
            "",
        ])

    return "\n".join(lines)


def parse_test_config(config_path: str) -> dict:
    """Parse a test configuration TOML file."""
    with open(config_path, 'rb') as f:
        return tomllib.load(f)


class MavRouter:
    """Manages mavrouter subprocess for multi-vehicle routing."""

    def __init__(self, config_content: str = None, config_path: str = None):
        """
        Initialize router with either config content (string) or path.

        If config_content is provided, a temp file is created.
        If config_path is provided, it's used directly.
        """
        self.config_content = config_content
        self.config_path = config_path
        self.temp_file = None
        self.process = None

    @classmethod
    def from_test_config(cls, test_config_path: str) -> 'MavRouter':
        """Create router from test configuration TOML file."""
        config = parse_test_config(test_config_path)
        router_toml = generate_router_config(config)
        return cls(config_content=router_toml)

    def start(self):
        """Start mavrouter subprocess."""
        # Create temp file if using config_content
        if self.config_content:
            self.temp_file = tempfile.NamedTemporaryFile(
                mode='w', suffix='.toml', delete=False
            )
            self.temp_file.write(self.config_content)
            self.temp_file.close()
            config_file = self.temp_file.name
        else:
            config_file = self.config_path

        self.process = subprocess.Popen(
            ['mavrouter', '-c', config_file],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(0.5)  # Allow router to initialize
        if self.process.poll() is not None:
            stdout, stderr = self.process.communicate()
            self._cleanup_temp()
            raise RuntimeError(f"mavrouter failed to start: {stderr.decode()}")

    def stop(self):
        """Stop mavrouter subprocess."""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
        self._cleanup_temp()

    def _cleanup_temp(self):
        """Remove temporary config file."""
        if self.temp_file and os.path.exists(self.temp_file.name):
            os.unlink(self.temp_file.name)
            self.temp_file = None

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()


@pytest.fixture
def mav_connection():
    """Single vehicle MAVLink connection (direct to SITL on port 14560)."""
    conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
    yield conn
    conn.close()


@pytest.fixture
def mav_connection_via_router():
    """MAVLink connection via mavrouter (GCS port 14550)."""
    conn = MavConnection(f"udpin:127.0.0.1:{DEFAULT_GCS_PORT}")
    yield conn
    conn.close()


@pytest.fixture
def mavrouter_two_vehicle():
    """Start mavrouter configured for two-vehicle formation test."""
    test_config_path = Path(__file__).parent.parent / 'quadcopter' / 'two_vehicle_formation.toml'
    if not test_config_path.exists():
        pytest.skip(f"Test config not found: {test_config_path}")

    router = MavRouter.from_test_config(str(test_config_path))
    router.start()
    yield router
    router.stop()


@pytest.fixture
def mavrouter_single_vehicle():
    """Start mavrouter configured for single vehicle test."""
    test_config_path = Path(__file__).parent.parent / 'quadcopter' / 'basic_flight.toml'
    if not test_config_path.exists():
        pytest.skip(f"Test config not found: {test_config_path}")

    router = MavRouter.from_test_config(str(test_config_path))
    router.start()
    yield router
    router.stop()


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "multi_vehicle: marks tests requiring mavrouter")
    config.addinivalue_line("markers", "sitl: marks tests requiring SITL running")
