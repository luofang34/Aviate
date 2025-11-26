#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::ekf::Ekf;
    use aviate_core::math::{Vector3, Quaternion};
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond, Meters, MetersPerSecond, Normalized};
    use aviate_core::sensor::{ImuData, GnssData, GnssFix, SensorReading, SensorHealth, GnssHealth};
    use aviate_core::time::{Timestamp, TimeSource};
    use aviate_core::AviateKernel;
    use aviate_core::control::Command;

    #[test]
    fn test_ekf_init_predict() {
        let mut ekf = Ekf::default();
        assert!(!ekf.is_initialized());

        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );
        assert!(ekf.is_initialized());

        // Test prediction with zero input (stationary)
        let imu_zero = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        // dt = 0.01s
        ekf.predict(&imu_zero, 0.01);

        let est = ekf.get_estimate();
        // Should barely move (biases are zero initially)
        assert!(est.position_ned[0].0.abs() < 1e-5);
    }

    #[test]
    fn test_ekf_accel_integration() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        // Accel x = 1.0 m/s^2
        let imu_accel = ImuData {
            accel: [MetersPerSecondSquared(1.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0)],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        let dt = 0.1;
        // Run 10 steps
        for _ in 0..10 {
            ekf.predict(&imu_accel, dt);
        }

        let est = ekf.get_estimate();

        // Expected vel = a * t = 1.0 * 1.0 = 1.0
        let vel_x = est.velocity_ned[0].0;
        assert!((vel_x - 1.0).abs() < 0.1, "Velocity X should be ~1.0, got {}", vel_x);

        // Expected pos = 0.5 * a * t^2 = 0.5 * 1.0 * 1.0 = 0.5
        let pos_x = est.position_ned[0].0;
        assert!((pos_x - 0.5).abs() < 0.1, "Position X should be ~0.5, got {}", pos_x);
    }

    #[test]
    fn test_ekf_gnss_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        // Predict once to grow uncertainty
        let imu_zero = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };
        ekf.predict(&imu_zero, 1.0);

        let gnss = GnssData {
            position_ned: [Meters(1.0), Meters(0.0), Meters(0.0)], // Measure 1.0m (was 10.0m, rejected by gate)
            velocity_ned: [MetersPerSecond(0.0); 3],
            fix: GnssFix::ThreeD,
            health: GnssHealth::Good,
        };

        let gnss_reading = SensorReading {
            value: gnss,
            valid: true,
            source_id: 0,
            timestamp: Timestamp { ticks: 0, source: TimeSource::Internal },
            health: SensorHealth::Good,
        };

        ekf.update_gnss(&gnss_reading);

        let est = ekf.get_estimate();
        // Estimate should move significantly towards 1.0
        // P ~ 0.1, R ~ 0.5. K ~ 0.16.
        // Pos ~ 0 + 0.16 * 1.0 = 0.16.
        assert!(est.position_ned[0].0 > 0.1, "Position should move towards measurement");
        assert!(est.position_ned[0].0 < 1.0, "Position should not overshoot measurement");
    }

    #[test]
    fn test_kernel_mc() {
        use aviate_core::control::mc::McController;
        let mut kernel = AviateKernel::new(McController);
        let cmd = Command { collective_thrust: Normalized(0.5) };
        let axis_cmd = kernel.step(&cmd);
        assert_eq!(axis_cmd.collective.0, 0.5);
    }

    #[test]
    fn test_kernel_fw() {
        use aviate_core::control::fw::FwController;
        let mut kernel = AviateKernel::new(FwController);
        let cmd = Command { collective_thrust: Normalized(0.5) };
        let axis_cmd = kernel.step(&cmd);
        assert_eq!(axis_cmd.collective.0, 0.5);
    }
}
