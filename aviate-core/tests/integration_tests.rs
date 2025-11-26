#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::ekf::Ekf;
    use aviate_core::math::{Vector3, Quaternion};
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond, Meters, MetersPerSecond, Normalized, Scalar, Pascals, Microtesla};
    use aviate_core::sensor::{ImuData, GnssData, GnssFix, SensorReading, SensorHealth, GnssHealth, BaroData, AirData, MagData};
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

        let imu_zero = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        ekf.predict(&imu_zero, 0.01);

        let est = ekf.get_estimate();
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

        let imu_accel = ImuData {
            accel: [MetersPerSecondSquared(1.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0)],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        let dt = 0.1;
        for _ in 0..10 {
            ekf.predict(&imu_accel, dt);
        }

        let est = ekf.get_estimate();

        let vel_x = est.velocity_ned[0].0;
        assert!((vel_x - 1.0).abs() < 0.1, "Velocity X should be ~1.0, got {}", vel_x);

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

        let imu_stationary = ImuData {
            accel: [MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(-9.81)],
            gyro: [RadiansPerSecond(0.0); 3],
        };
        ekf.predict(&imu_stationary, 1.0);

        let gnss = GnssData {
            position_ned: [Meters(1.0), Meters(0.0), Meters(0.0)],
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

    #[test]
    fn test_attitude_integration() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let imu_rot = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [RadiansPerSecond(0.0), RadiansPerSecond(0.0), RadiansPerSecond(1.0)],
        };

        let dt = 0.1;
        for _ in 0..10 {
            ekf.predict(&imu_rot, dt);
        }

        let est = ekf.get_estimate();
        assert!(est.attitude.z > 0.4, "Should have rotated around Z axis");
    }

    #[test]
    fn test_gnss_health_rejection() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let gnss = GnssData {
            position_ned: [Meters(100.0), Meters(0.0), Meters(0.0)],
            velocity_ned: [MetersPerSecond(0.0); 3],
            fix: GnssFix::ThreeD,
            health: GnssHealth::Suspect,
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
        assert_eq!(est.position_ned[0].0, 0.0, "Suspect GNSS should be ignored");
    }

    #[test]
    fn test_innovation_gating() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let gnss = GnssData {
            position_ned: [Meters(1000.0), Meters(0.0), Meters(0.0)],
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
        assert_eq!(est.position_ned[0].0, 0.0, "Outlier GNSS should be gated");
    }

    #[test]
    fn test_sensor_health_rejection() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let baro_reading = SensorReading {
            value: BaroData {
                altitude: Some(Meters(100.0)),
                air: AirData {
                    static_pressure: Some(Pascals(90000.0)),
                    dynamic_pressure: None,
                    total_pressure: None,
                    temperature: None,
                    indicated_airspeed: None,
                    true_airspeed: None,
                }
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp { ticks: 0, source: TimeSource::Internal },
            health: SensorHealth::Failed,
        };

        ekf.update_baro(&baro_reading);
        let est = ekf.get_estimate();
        assert_eq!(est.position_ned[2].0, 0.0, "Failed sensor should be ignored");
    }

    #[test]
    fn test_gnss_fix_none_rejection() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let gnss = GnssData {
            position_ned: [Meters(10.0), Meters(0.0), Meters(0.0)],
            velocity_ned: [MetersPerSecond(0.0); 3],
            fix: GnssFix::None,
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
        assert_eq!(est.position_ned[0].0, 0.0, "GnssFix::None should be ignored");
    }

    #[test]
    fn test_nan_input_handling() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let imu_nan = ImuData {
            accel: [MetersPerSecondSquared(Scalar::NAN); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        ekf.predict(&imu_nan, 0.01);
        let est = ekf.get_estimate();
        assert!(!est.position_ned[0].0.is_nan(), "State should not be NaN after bad input");
    }

    #[test]
    fn test_baro_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let imu_stationary = ImuData {
            accel: [MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(-9.81)],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        let baro_reading = SensorReading {
            value: BaroData {
                altitude: None,
                air: AirData {
                    static_pressure: Some(Pascals(101313.0)),
                    dynamic_pressure: None,
                    total_pressure: None,
                    temperature: None,
                    indicated_airspeed: None,
                    true_airspeed: None,
                }
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp { ticks: 0, source: TimeSource::Internal },
            health: SensorHealth::Good,
        };

        for _ in 0..50 {
            ekf.predict(&imu_stationary, 0.1);
            ekf.update_baro(&baro_reading);
        }

        let est = ekf.get_estimate();
        assert!(est.position_ned[2].0 < -0.5, "Baro update should move Z position (NED)");
    }

    #[test]
    fn test_mag_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let mag_reading = SensorReading {
            value: MagData {
                field_ut: [Microtesla(0.0); 3],
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp { ticks: 0, source: TimeSource::Internal },
            health: SensorHealth::Good,
        };

        ekf.update_mag(&mag_reading);
    }

    #[test]
    fn test_long_run_stability() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let imu = ImuData {
            accel: [MetersPerSecondSquared(0.0), MetersPerSecondSquared(0.0), MetersPerSecondSquared(-9.81)],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        for _ in 0..1000 {
            ekf.predict(&imu, 0.01);
        }

        let est = ekf.get_estimate();
        assert!(!est.position_ned[0].0.is_nan());
        assert!(est.position_ned[0].0.abs() < 1.0);
    }
}
