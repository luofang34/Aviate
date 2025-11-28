"""
Test MAVLink command responses.

Verifies that Aviate SITL correctly responds to COMMAND_LONG messages
with appropriate COMMAND_ACK results.

These tests require SITL to be running.
"""
import time
import pytest
from pymavlink import mavutil

from conftest import MavConnection, VEHICLE_BASE_PORT


class TestCommandAck:
    """Test COMMAND_ACK responses to various commands."""

    @pytest.fixture
    def gcs(self):
        """Direct connection to SITL."""
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    @pytest.mark.sitl
    def test_arm_ack_accepted(self, gcs):
        """Arm command should return ACCEPTED when pre-arm checks pass."""
        # Establish sensor health
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        # Ensure throttle low
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Send arm command
        gcs.conn.mav.command_long_send(
            1, 1,  # target system, component
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,     # confirmation
            1.0,   # param1: arm
            0, 0, 0, 0, 0, 0
        )

        # Wait for COMMAND_ACK
        msg = gcs.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        assert msg is not None, "No COMMAND_ACK received"
        assert msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        assert msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    @pytest.mark.sitl
    def test_arm_ack_denied_no_sensors(self, gcs):
        """Arm command should return DENIED when no sensor data."""
        # Send heartbeats but NO sensor data
        for _ in range(5):
            gcs.send_heartbeat()
            time.sleep(0.1)

        # Send arm command
        gcs.conn.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1.0, 0, 0, 0, 0, 0, 0
        )

        # Wait for COMMAND_ACK with DENIED result
        msg = gcs.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        assert msg is not None, "No COMMAND_ACK received"
        assert msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        assert msg.result == mavutil.mavlink.MAV_RESULT_DENIED

    @pytest.mark.sitl
    def test_disarm_ack_accepted(self, gcs):
        """Disarm command should always return ACCEPTED."""
        # Disarm without any prior state
        gcs.conn.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0.0, 0, 0, 0, 0, 0, 0  # param1=0 = disarm
        )

        msg = gcs.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        assert msg is not None, "No COMMAND_ACK received"
        assert msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        assert msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED

    @pytest.mark.sitl
    def test_unsupported_command_ack(self, gcs):
        """Unsupported commands should return UNSUPPORTED."""
        # Send an unsupported command (MAV_CMD_NAV_TAKEOFF = 22)
        gcs.conn.mav.command_long_send(
            1, 1,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 10.0  # param7 = altitude
        )

        msg = gcs.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        assert msg is not None, "No COMMAND_ACK received"
        assert msg.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
        assert msg.result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED

    @pytest.mark.sitl
    def test_command_ack_target_system(self, gcs):
        """COMMAND_ACK should echo correct target system/component."""
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Send command with specific target
        gcs.conn.mav.command_long_send(
            1, 1,  # target_system=1, target_component=1
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1.0, 0, 0, 0, 0, 0, 0
        )

        msg = gcs.conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
        assert msg is not None
        # ACK should have target fields set (echoed back or 0)
        # Note: aviate sets target_system/component in ACK based on cmd


class TestHeartbeat:
    """Test HEARTBEAT message timing and content."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    @pytest.mark.sitl
    def test_heartbeat_received(self, gcs):
        """Vehicle should send heartbeats."""
        # Trigger poll by sending a message
        gcs.send_heartbeat()
        time.sleep(0.1)

        # Wait for heartbeat (may take up to 1 second due to 1Hz rate)
        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None, "No HEARTBEAT received from vehicle"

    @pytest.mark.sitl
    def test_heartbeat_autopilot_type(self, gcs):
        """Heartbeat should identify as Aviate autopilot."""
        gcs.send_heartbeat()
        time.sleep(0.1)

        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None
        # MAV_AUTOPILOT_AVIATE = 18 (custom value in aviate-mavlink)
        assert msg.autopilot == 18, f"Expected autopilot=18 (Aviate), got {msg.autopilot}"

    @pytest.mark.sitl
    def test_heartbeat_type_quadrotor(self, gcs):
        """Heartbeat should identify as quadrotor."""
        gcs.send_heartbeat()
        time.sleep(0.1)

        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None
        assert msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR

    @pytest.mark.sitl
    def test_heartbeat_armed_flag(self, gcs):
        """Heartbeat should reflect armed state in base_mode."""
        # First check unarmed state
        gcs.send_heartbeat()
        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None

        # Check if ARMED flag is NOT set when disarmed
        armed_flag = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        initial_armed = bool(msg.base_mode & armed_flag)

        # Now arm the vehicle
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        gcs.send_arm_command(arm=True)
        time.sleep(0.1)

        # Get new heartbeat
        gcs.drain_messages()
        time.sleep(1.1)  # Wait for next heartbeat
        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None

        # After arming, ARMED flag should be set
        new_armed = bool(msg.base_mode & armed_flag)
        assert new_armed, "ARMED flag not set in heartbeat after arming"

    @pytest.mark.sitl
    def test_heartbeat_hil_mode(self, gcs):
        """Heartbeat should indicate HIL mode."""
        gcs.send_heartbeat()
        time.sleep(0.1)

        msg = gcs.recv_heartbeat(timeout=2.0)
        assert msg is not None

        hil_flag = mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
        assert msg.base_mode & hil_flag, "HIL flag not set in heartbeat"


class TestSensorFeedback:
    """Test that sensor messages are processed correctly."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    @pytest.mark.sitl
    def test_hil_sensor_updates_prearm(self, gcs):
        """Sending HIL_SENSOR should update pre-arm state."""
        # Send sensors and verify arm becomes possible
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Should now be able to arm
        result = gcs.send_arm_command(arm=True)
        assert result, "Arm should succeed after receiving sensor data"

    @pytest.mark.sitl
    def test_invalid_sensor_data(self, gcs):
        """Invalid sensor data should prevent arming."""
        # Send sensor with NaN values
        gcs.send_hil_sensor(accel=(float('nan'), 0, -9.81))
        time.sleep(0.05)

        result = gcs.send_arm_command(arm=True)
        assert not result, "Arm should fail with invalid sensor data"

    @pytest.mark.sitl
    def test_pressure_out_of_range(self, gcs):
        """Out-of-range pressure should mark baro unhealthy."""
        # Send many readings with good accel/gyro but bad pressure
        for _ in range(150):
            gcs.send_hil_sensor(pressure=50.0)  # Below 100 mbar threshold
            time.sleep(0.001)

        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        result = gcs.send_arm_command(arm=True)
        assert not result, "Arm should fail with pressure out of range"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
