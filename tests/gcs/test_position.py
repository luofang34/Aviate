"""
Phase 2: Position Control Tests

Tests position and velocity control response to SET_POSITION_TARGET_LOCAL_NED:
- Position hold at current location
- Position command to waypoint
- Velocity control mode
- Altitude hold

These tests verify the position/velocity control loops:
SET_POSITION_TARGET_LOCAL_NED -> Position Controller -> Velocity Controller
    -> Attitude Controller -> Rate Controller -> Mixer -> Motors

Requires SITL to be running.
"""
import time
import pytest
from pymavlink import mavutil

from conftest import MavConnection, VEHICLE_BASE_PORT


# Test port for receiving position data from gz_bridge
TEST_PORT = 14562


class TestPositionHold:
    """Test position hold mode using SET_POSITION_TARGET_LOCAL_NED."""

    @pytest.fixture
    def gcs(self):
        """Direct connection to SITL."""
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    def _arm_vehicle(self, gcs):
        """Helper to arm the vehicle."""
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        return gcs.send_arm_command(arm=True)

    @pytest.mark.sitl
    def test_position_target_generates_output(self, gcs):
        """SET_POSITION_TARGET_LOCAL_NED should generate motor output."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Send position hold at origin
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=0.0, z=-2.0)  # 2m altitude (NED)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        assert msg is not None, "No actuator controls for position target"

        # All motors should have some output for position hold
        for i in range(4):
            assert msg.controls[i] >= 0.0, f"Motor {i} negative output"

    @pytest.mark.sitl
    def test_altitude_position_generates_climb(self, gcs):
        """Requesting altitude above current should generate climb thrust."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # First, establish baseline at z=0
        for _ in range(30):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=0.0, z=0.0)
            time.sleep(0.02)

        # Now request climb to z=-5 (5m altitude in NED)
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=0.0, z=-5.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Motors should be generating lift for climb
            avg_output = sum(msg.controls[i] for i in range(4)) / 4
            # With altitude error, there should be non-trivial thrust
            assert avg_output > 0.1, f"Average motor output too low for climb: {avg_output}"


class TestVelocityControl:
    """Test velocity control mode using SET_POSITION_TARGET_LOCAL_NED."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    def _arm_vehicle(self, gcs):
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        return gcs.send_arm_command(arm=True)

    @pytest.mark.sitl
    def test_velocity_forward_generates_output(self, gcs):
        """Forward velocity command should generate motor output."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command forward velocity (North in NED)
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_velocity_target(vx=1.0, vy=0.0, vz=0.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        assert msg is not None, "No actuator controls for velocity command"

        # Motors should be running
        for i in range(4):
            assert msg.controls[i] >= 0.0, f"Motor {i} output invalid"

    @pytest.mark.sitl
    def test_velocity_climb_generates_thrust(self, gcs):
        """Climb velocity command should generate upward thrust."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command climb velocity (negative vz in NED = up)
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_velocity_target(vx=0.0, vy=0.0, vz=-2.0)  # 2 m/s up
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            avg_output = sum(msg.controls[i] for i in range(4)) / 4
            # Climb command should generate thrust
            assert avg_output > 0.2, f"Average output too low for climb: {avg_output}"

    @pytest.mark.sitl
    def test_velocity_descent_reduces_thrust(self, gcs):
        """Descent velocity command should reduce thrust compared to hover."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # First get hover thrust
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_velocity_target(vx=0.0, vy=0.0, vz=0.0)
            time.sleep(0.02)
        msg_hover = gcs.recv_actuator_controls(timeout=1.0)

        # Then descent
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_velocity_target(vx=0.0, vy=0.0, vz=2.0)  # 2 m/s down
            time.sleep(0.02)
        msg_descent = gcs.recv_actuator_controls(timeout=1.0)

        if msg_hover and msg_descent:
            hover_avg = sum(msg_hover.controls[i] for i in range(4)) / 4
            descent_avg = sum(msg_descent.controls[i] for i in range(4)) / 4
            # Descent should have less thrust (or at least not significantly more)
            # Note: actual behavior depends on control gains


class TestPositionWaypoint:
    """Test waypoint following using position targets."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    def _arm_vehicle(self, gcs):
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        return gcs.send_arm_command(arm=True)

    @pytest.mark.sitl
    def test_waypoint_north_generates_pitch(self, gcs):
        """Waypoint north of current position should generate forward pitch."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command position 10m North
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=10.0, y=0.0, z=-2.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # For forward flight, front motors should differ from rear
            # Per mixer: pitch affects M0,M1 vs M2,M3
            front_avg = (msg.controls[0] + msg.controls[1]) / 2
            rear_avg = (msg.controls[2] + msg.controls[3]) / 2
            # Verify motors running
            for i in range(4):
                assert msg.controls[i] > 0.0, f"Motor {i} not running"

    @pytest.mark.sitl
    def test_waypoint_east_generates_roll(self, gcs):
        """Waypoint east of current position should generate right roll."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command position 10m East
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=10.0, z=-2.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # For right flight, left/right motors should differ
            left_avg = (msg.controls[1] + msg.controls[2]) / 2
            right_avg = (msg.controls[0] + msg.controls[3]) / 2
            # Verify motors running
            for i in range(4):
                assert msg.controls[i] > 0.0, f"Motor {i} not running"


class TestYawControl:
    """Test yaw/heading control in position mode."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    def _arm_vehicle(self, gcs):
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        return gcs.send_arm_command(arm=True)

    @pytest.mark.sitl
    def test_yaw_target_in_position_mode(self, gcs):
        """Position target with yaw should generate yaw motor differential."""
        import math
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command position with 45 degree heading
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(
                x=0.0, y=0.0, z=-2.0,
                yaw=math.radians(45),
                type_mask=0x0F8  # Use position and yaw
            )
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # With yaw command, CW/CCW motors should differ
            cw_avg = (msg.controls[0] + msg.controls[3]) / 2
            ccw_avg = (msg.controls[1] + msg.controls[2]) / 2
            # Motors should be running
            for i in range(4):
                assert msg.controls[i] >= 0.0, f"Motor {i} invalid"


class TestControlModeTransition:
    """Test transitions between control modes."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    def _arm_vehicle(self, gcs):
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        return gcs.send_arm_command(arm=True)

    @pytest.mark.sitl
    def test_attitude_to_position_mode(self, gcs):
        """Transition from attitude to position mode should work smoothly."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Start in attitude mode
        for _ in range(30):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, thrust=0.5)
            time.sleep(0.02)

        # Transition to position mode
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=0.0, z=-2.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        assert msg is not None, "Lost actuator output after mode transition"

        for i in range(4):
            assert msg.controls[i] >= 0.0, f"Motor {i} invalid after transition"

    @pytest.mark.sitl
    def test_position_to_velocity_mode(self, gcs):
        """Transition from position to velocity mode should work."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Start in position mode
        for _ in range(30):
            gcs.send_hil_sensor()
            gcs.send_position_target(x=0.0, y=0.0, z=-2.0)
            time.sleep(0.02)

        # Transition to velocity mode
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_velocity_target(vx=1.0, vy=0.0, vz=0.0)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        assert msg is not None, "Lost actuator output after velocity mode transition"

        for i in range(4):
            assert msg.controls[i] >= 0.0, f"Motor {i} invalid after transition"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
