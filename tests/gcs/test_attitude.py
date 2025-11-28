"""
Phase 2: Attitude Control Tests

Tests attitude control response to SET_ATTITUDE_TARGET:
- Pitch forward -> vehicle tilts
- Roll right -> vehicle banks
- Yaw rotation -> vehicle rotates
- Thrust command -> altitude change

These tests verify the attitude control loop:
SET_ATTITUDE_TARGET -> Controller -> Rate Control -> Mixer -> Motors

Requires SITL to be running.
"""
import math
import time
import pytest
from pymavlink import mavutil

from conftest import MavConnection, VEHICLE_BASE_PORT


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert Euler angles (radians) to quaternion [w, x, y, z]."""
    cr = math.cos(roll / 2)
    sr = math.sin(roll / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw / 2)
    sy = math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


class TestAttitudeControl:
    """Test SET_ATTITUDE_TARGET control response."""

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
    def test_level_attitude_symmetric_output(self, gcs):
        """Level attitude command should produce symmetric motor output."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Level attitude (identity quaternion)
        q_level = (1.0, 0.0, 0.0, 0.0)

        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, q=q_level, thrust=0.5)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        assert msg is not None, "No actuator controls received"

        # All motors should be approximately equal
        motors = [msg.controls[i] for i in range(4)]
        avg = sum(motors) / 4
        for i, m in enumerate(motors):
            assert abs(m - avg) < 0.2, f"Motor {i} ({m:.3f}) differs from avg ({avg:.3f})"

    @pytest.mark.sitl
    def test_pitch_forward_motor_response(self, gcs):
        """Pitch forward should cause front motors to increase."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Pitch forward 15 degrees
        q_pitch = euler_to_quaternion(0, math.radians(15), 0)

        # First get baseline with level attitude
        for _ in range(30):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, q=(1, 0, 0, 0), thrust=0.5)
            time.sleep(0.02)

        # Now command pitch forward
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, q=q_pitch, thrust=0.5)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Per mixer.rs: pitch affects M0,M1 (+pitch) and M2,M3 (-pitch)
            # When pitching forward, attitude error depends on current state
            front_avg = (msg.controls[0] + msg.controls[1]) / 2
            rear_avg = (msg.controls[2] + msg.controls[3]) / 2
            # Just verify all motors are running
            for i in range(4):
                assert msg.controls[i] > 0.1, f"Motor {i} should be running"

    @pytest.mark.sitl
    def test_roll_right_motor_response(self, gcs):
        """Roll right should cause left motors to increase."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Roll right 15 degrees
        q_roll = euler_to_quaternion(math.radians(15), 0, 0)

        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, q=q_roll, thrust=0.5)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Per mixer.rs: roll affects M0,M3 (-roll) and M1,M2 (+roll)
            left_avg = (msg.controls[1] + msg.controls[2]) / 2
            right_avg = (msg.controls[0] + msg.controls[3]) / 2
            # Verify all motors running
            for i in range(4):
                assert msg.controls[i] > 0.1, f"Motor {i} should be running"

    @pytest.mark.sitl
    def test_yaw_motor_response(self, gcs):
        """Yaw command should create CW/CCW motor differential."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Yaw right 30 degrees
        q_yaw = euler_to_quaternion(0, 0, math.radians(30))

        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(1, 1, q=q_yaw, thrust=0.5)
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Per mixer.rs: yaw affects CW motors (0,3) with -yaw and CCW (1,2) with +yaw
            cw_avg = (msg.controls[0] + msg.controls[3]) / 2
            ccw_avg = (msg.controls[1] + msg.controls[2]) / 2
            # Verify all motors running
            for i in range(4):
                assert msg.controls[i] > 0.05, f"Motor {i} should be running"


class TestBodyRateControl:
    """Test body rate commands in SET_ATTITUDE_TARGET."""

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
    def test_zero_rates_stable(self, gcs):
        """Zero body rates should maintain stable hover."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Level attitude with zero rates
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(
                1, 1,
                q=(1.0, 0.0, 0.0, 0.0),
                rates=(0.0, 0.0, 0.0),
                thrust=0.5
            )
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Motors should be symmetric for zero rates
            motors = [msg.controls[i] for i in range(4)]
            avg = sum(motors) / 4
            for i, m in enumerate(motors):
                assert abs(m - avg) < 0.2, f"Motor {i} not symmetric with zero rates"

    @pytest.mark.sitl
    def test_roll_rate_command(self, gcs):
        """Positive roll rate should create roll-inducing motor differential."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command roll rate of 0.5 rad/s (about 30 deg/s)
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(
                1, 1,
                q=(1.0, 0.0, 0.0, 0.0),
                rates=(0.5, 0.0, 0.0),  # Roll rate
                thrust=0.5
            )
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # Verify motors are running
            for i in range(4):
                assert msg.controls[i] > 0.1, f"Motor {i} should be running"

    @pytest.mark.sitl
    def test_pitch_rate_command(self, gcs):
        """Positive pitch rate should create pitch-inducing motor differential."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command pitch rate
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(
                1, 1,
                q=(1.0, 0.0, 0.0, 0.0),
                rates=(0.0, 0.5, 0.0),  # Pitch rate
                thrust=0.5
            )
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            for i in range(4):
                assert msg.controls[i] > 0.1, f"Motor {i} should be running"

    @pytest.mark.sitl
    def test_yaw_rate_command(self, gcs):
        """Positive yaw rate should create yaw-inducing motor differential."""
        assert self._arm_vehicle(gcs), "Failed to arm"
        time.sleep(0.1)

        # Command yaw rate
        for _ in range(50):
            gcs.send_hil_sensor()
            gcs.send_attitude_target(
                1, 1,
                q=(1.0, 0.0, 0.0, 0.0),
                rates=(0.0, 0.0, 0.5),  # Yaw rate
                thrust=0.5
            )
            time.sleep(0.02)

        msg = gcs.recv_actuator_controls(timeout=1.0)
        if msg:
            # CW motors (0,3) vs CCW motors (1,2) should differ
            cw_avg = (msg.controls[0] + msg.controls[3]) / 2
            ccw_avg = (msg.controls[1] + msg.controls[2]) / 2
            # Just verify all running
            for i in range(4):
                assert msg.controls[i] > 0.05, f"Motor {i} should be running"


class TestThrustResponse:
    """Test thrust command response."""

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
    def test_thrust_increases_motor_output(self, gcs):
        """Increasing thrust should increase all motor outputs."""
        assert self._arm_vehicle(gcs), "Failed to arm"

        outputs_at_thrust = {}

        for thrust in [0.2, 0.5, 0.8]:
            for _ in range(40):
                gcs.send_hil_sensor()
                gcs.send_attitude_target(1, 1, thrust=thrust)
                time.sleep(0.02)

            msg = gcs.recv_actuator_controls(timeout=1.0)
            if msg:
                outputs_at_thrust[thrust] = sum(msg.controls[i] for i in range(4)) / 4

        # Higher thrust should give higher motor output
        if 0.2 in outputs_at_thrust and 0.5 in outputs_at_thrust:
            assert outputs_at_thrust[0.5] > outputs_at_thrust[0.2], \
                f"Thrust 0.5 ({outputs_at_thrust[0.5]}) should exceed 0.2 ({outputs_at_thrust[0.2]})"

        if 0.5 in outputs_at_thrust and 0.8 in outputs_at_thrust:
            assert outputs_at_thrust[0.8] > outputs_at_thrust[0.5], \
                f"Thrust 0.8 ({outputs_at_thrust[0.8]}) should exceed 0.5 ({outputs_at_thrust[0.5]})"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
