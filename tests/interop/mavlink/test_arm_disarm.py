"""
Test arm/disarm with pre-arm checks.

Tests the full pre-arm check sequence:
1. Arm denied when sensors not healthy
2. Arm denied when EKF not converged (insufficient sensor data)
3. Arm denied when throttle not low
4. Arm accepted when all conditions met
5. Disarm always accepted
"""
import time
import pytest

from conftest import MavConnection, VEHICLE_BASE_PORT


# Pre-arm check requirements (from aviate-platform/sitl/src/udp.rs):
# - imu_healthy: valid accel/gyro in HIL_SENSOR
# - baro_healthy: pressure 100-2000 mbar
# - mag_healthy: valid magnetometer data
# - sensor_count >= 100 (EKF convergence)
# - last_thrust below THROTTLE_LOW_MAX_COLLECTIVE (0.01, force-domain
#   NormalizedThrust fraction of max total thrust — see
#   aviate-core/src/kernel_types.rs)


class TestPreArmChecks:
    """Test pre-arm check enforcement."""

    @pytest.fixture
    def gcs(self):
        """Direct connection to SITL (no mavrouter needed for single vehicle)."""
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    @pytest.mark.sitl
    def test_arm_denied_no_sensors(self, gcs):
        """Arm should be denied when no sensor data received."""
        # Send heartbeats but no HIL_SENSOR
        for _ in range(5):
            gcs.send_heartbeat()
            time.sleep(0.1)

        # Attempt to arm - should be denied
        result = gcs.send_arm_command(arm=True)
        assert not result, "Arm should be denied without sensor data"

    @pytest.mark.sitl
    def test_arm_denied_insufficient_sensor_count(self, gcs):
        """Arm should be denied when EKF not converged (< 100 sensor readings)."""
        # Send only a few sensor readings (less than MIN_SENSOR_COUNT=100)
        for _ in range(50):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        # Attempt to arm - should be denied (sensor_count < 100)
        result = gcs.send_arm_command(arm=True)
        assert not result, "Arm should be denied with insufficient sensor data"

    @pytest.mark.sitl
    def test_arm_denied_throttle_not_low(self, gcs):
        """Arm should be denied when throttle is not low."""
        # Send enough sensor readings for EKF convergence
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        # Send attitude target with non-zero thrust
        gcs.send_attitude_target(1, 1, thrust=0.5)
        time.sleep(0.05)

        # Attempt to arm - should be denied (thrust 0.5 is far above
        # THROTTLE_LOW_MAX_COLLECTIVE = 0.01, force domain)
        result = gcs.send_arm_command(arm=True)
        assert not result, "Arm should be denied with throttle not low"

    @pytest.mark.sitl
    def test_arm_success_all_checks_pass(self, gcs):
        """Arm should succeed when all pre-arm checks pass."""
        # Send enough sensor readings for EKF convergence
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        # Ensure throttle is low (send zero thrust command)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Attempt to arm - should succeed
        result = gcs.send_arm_command(arm=True)
        assert result, "Arm should succeed with all pre-arm checks passed"

    @pytest.mark.sitl
    def test_disarm_always_accepted(self, gcs):
        """Disarm should always be accepted."""
        # First arm the vehicle (with proper pre-arm sequence)
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)
        gcs.send_arm_command(arm=True)
        time.sleep(0.1)

        # Disarm should always succeed
        result = gcs.send_arm_command(arm=False)
        assert result, "Disarm should always be accepted"


class TestArmDisarmSequence:
    """Test complete arm/disarm sequences."""

    @pytest.fixture
    def gcs(self):
        conn = MavConnection(f"udpout:127.0.0.1:{VEHICLE_BASE_PORT}")
        yield conn
        conn.close()

    @pytest.mark.sitl
    def test_arm_disarm_cycle(self, gcs):
        """Test a complete arm -> fly -> disarm cycle."""
        # Pre-arm: establish sensor health
        for _ in range(150):
            gcs.send_hil_sensor()
            time.sleep(0.001)

        # Ensure throttle low
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Arm
        assert gcs.send_arm_command(arm=True), "Arm failed"

        # Simulate flight with throttle
        for _ in range(50):
            gcs.send_attitude_target(1, 1, thrust=0.6)
            gcs.send_hil_sensor()
            time.sleep(0.02)

        # Reduce throttle
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.1)

        # Disarm
        assert gcs.send_arm_command(arm=False), "Disarm failed"


class TestMultiVehicleArm:
    """Test arm/disarm with multiple vehicles via mavrouter."""

    @pytest.mark.multi_vehicle
    @pytest.mark.sitl
    def test_arm_vehicle_1(self, mavrouter_two_vehicle, mav_connection_via_router):
        """Arm vehicle 1 (system_id=1) via mavrouter."""
        gcs = mav_connection_via_router

        # Pre-arm sequence for vehicle 1
        for _ in range(150):
            gcs.send_hil_sensor(sensor_id=0)  # instance 0
            time.sleep(0.001)
        gcs.send_attitude_target(1, 1, thrust=0.0)
        time.sleep(0.05)

        # Arm vehicle 1
        result = gcs.send_arm_command(target_system=1, arm=True)
        assert result, "Vehicle 1 arm failed"

    @pytest.mark.multi_vehicle
    @pytest.mark.sitl
    def test_arm_vehicle_2(self, mavrouter_two_vehicle, mav_connection_via_router):
        """Arm vehicle 2 (system_id=2) via mavrouter."""
        gcs = mav_connection_via_router

        # Pre-arm sequence for vehicle 2
        for _ in range(150):
            gcs.send_hil_sensor(sensor_id=1)  # instance 1
            time.sleep(0.001)
        gcs.send_attitude_target(2, 1, thrust=0.0)
        time.sleep(0.05)

        # Arm vehicle 2
        result = gcs.send_arm_command(target_system=2, arm=True)
        assert result, "Vehicle 2 arm failed"

    @pytest.mark.multi_vehicle
    @pytest.mark.sitl
    def test_arm_both_vehicles(self, mavrouter_two_vehicle, mav_connection_via_router):
        """Arm both vehicles sequentially via mavrouter."""
        gcs = mav_connection_via_router

        # Pre-arm for both vehicles (interleaved sensor data)
        for _ in range(150):
            gcs.send_hil_sensor(sensor_id=0)
            gcs.send_hil_sensor(sensor_id=1)
            time.sleep(0.001)

        # Set throttle low for both
        gcs.send_attitude_target(1, 1, thrust=0.0)
        gcs.send_attitude_target(2, 1, thrust=0.0)
        time.sleep(0.05)

        # Arm vehicle 1
        result1 = gcs.send_arm_command(target_system=1, arm=True)
        assert result1, "Vehicle 1 arm failed"

        # Arm vehicle 2
        result2 = gcs.send_arm_command(target_system=2, arm=True)
        assert result2, "Vehicle 2 arm failed"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
