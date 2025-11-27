#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::ekf::Ekf;
    use aviate_core::math::{Vector3, Quaternion};
    use aviate_core::types::{MetersPerSecondSquared, RadiansPerSecond, Meters, MetersPerSecond, Normalized, Scalar, Pascals, Microtesla, FloatExt};
    use aviate_core::sensor::{ImuData, GnssData, GnssFix, SensorReading, SensorHealth, GnssHealth, BaroData, AirData, MagData};
    use aviate_core::time::{Timestamp, TimeSource};
    use aviate_core::AviateKernel;
    use aviate_core::control::Command;
    use aviate_core::control::rate::RateController;
    use aviate_core::control::attitude::AttitudeController;
    use aviate_core::control::position::PositionController; // Added
    use aviate_core::control::velocity::VelocityController; // Added


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
        use aviate_core::mixer::{QuadXMixer, ModeConfig};
        use aviate_core::control::{ConfigMode, Setpoint, CommandSource, ControlMode};
        use aviate_core::sensor::SensorSet;
        
        fn dummy_time() -> Timestamp { Timestamp { ticks: 0, source: TimeSource::Internal } }
        let mixer = QuadXMixer { timestamp_source: dummy_time };
        
        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };
        
        let mut kernel = AviateKernel::new(McController::default(), mixer, mode_config);
        
        // Initialize EKF to ensure it's ready for transition (though current placeholder init_step is simple)
        kernel.ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let cmd = Command { 
            mode: ControlMode::Attitude,
            setpoint: Setpoint {
                collective_thrust: Normalized(0.5),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 0,
            source: CommandSource::Pilot,
        };
        
        // Before arming: Expect safe output (0.0)
        let act_cmd_safe = kernel.step(&cmd);
        for i in 0..4 {
            assert!((act_cmd_safe.outputs[i].0).abs() < 1e-5, "Should be zero when disarmed");
        }

        // Cycle init state
        // PowerOn -> ConfigLoading -> SensorInit -> EstimatorConverging -> PreArm -> Ready
        // My implementation moves one state per call
        
        let empty_sensors = SensorSet {
            imus: [SensorReading::default(), SensorReading::default(), SensorReading::default()],
            gnss: [SensorReading::default(), SensorReading::default()],
            mags: [SensorReading::default(), SensorReading::default()],
            baros: [SensorReading::default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        };
        
        // 5 transitions needed?
        for _ in 0..10 {
            kernel.init_step(&empty_sensors, dummy_time());
            if kernel.is_ready() { break; }
        }
        assert!(kernel.is_ready(), "Kernel failed to become ready");
        
        // Arm
        kernel.arm().expect("Failed to arm");
        
        // After arming: Expect control output
        let act_cmd = kernel.step(&cmd);
        
        // QuadXMixer with 0 R/P/Y should output collective on all 4 motors
        for i in 0..4 {
            assert!((act_cmd.outputs[i].0 - 0.5).abs() < 1e-5, "Should be 0.5 when armed");
        }
    }

    #[test]
    fn test_kernel_fw() {
        use aviate_core::control::fw::FwController;
        use aviate_core::mixer::{QuadXMixer, ModeConfig};
        use aviate_core::control::{ConfigMode, Setpoint, CommandSource, ControlMode};
        use aviate_core::sensor::SensorSet;
        
        fn dummy_time() -> Timestamp { Timestamp { ticks: 0, source: TimeSource::Internal } }
        let mixer = QuadXMixer { timestamp_source: dummy_time };
        
        let mode_config = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &[],
        };
        
        let mut kernel = AviateKernel::new(FwController, mixer, mode_config);
        
        // Init EKF
        kernel.ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(MetersPerSecond(0.0), MetersPerSecond(0.0), MetersPerSecond(0.0)),
            Quaternion::IDENTITY
        );

        let cmd = Command { 
            mode: ControlMode::Attitude,
            setpoint: Setpoint {
                collective_thrust: Normalized(0.5),
                ..Default::default()
            },
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 0,
            source: CommandSource::Pilot,
        };
        
        // Cycle init state
        let empty_sensors = SensorSet {
            imus: [SensorReading::default(), SensorReading::default(), SensorReading::default()],
            gnss: [SensorReading::default(), SensorReading::default()],
            mags: [SensorReading::default(), SensorReading::default()],
            baros: [SensorReading::default(), SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        };

        for _ in 0..10 {
            kernel.init_step(&empty_sensors, dummy_time());
            if kernel.is_ready() { break; }
        }
        assert!(kernel.is_ready(), "Kernel failed to become ready");
        
        kernel.arm().expect("Failed to arm");
        
        let act_cmd = kernel.step(&cmd);
        
        // FwController currently outputs 0 R/P/Y and passes collective.
        // So QuadXMixer should still produce 0.5 on motors.
        for i in 0..4 {
            assert!((act_cmd.outputs[i].0 - 0.5).abs() < 1e-5);
        }
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

    // --- Rate Controller Tests ---
    #[test]
    fn test_rate_controller_zero_error() {
        let ctrl = RateController::new([1.0, 1.0, 1.0]);
        let sp = [RadiansPerSecond(1.0), RadiansPerSecond(0.5), RadiansPerSecond(-0.5)];
        let cur = [RadiansPerSecond(1.0), RadiansPerSecond(0.5), RadiansPerSecond(-0.5)];
        let out = ctrl.step(sp, cur);
        assert!((out[0].0).abs() < 1e-5);
        assert!((out[1].0).abs() < 1e-5);
        assert!((out[2].0).abs() < 1e-5);
    }

    #[test]
    fn test_rate_controller_positive_error() {
        let ctrl = RateController::new([1.0, 1.0, 1.0]);
        let sp = [RadiansPerSecond(1.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)];
        let cur = [RadiansPerSecond(0.0); 3];
        let out = ctrl.step(sp, cur);
        assert!(out[0].0 > 0.0); // Positive error → positive output
        assert!((out[0].0 - 1.0).abs() < 1e-5);
        assert!((out[1].0).abs() < 1e-5);
        assert!((out[2].0).abs() < 1e-5);
    }

    #[test]
    fn test_rate_controller_negative_error() {
        let ctrl = RateController::new([1.0, 1.0, 1.0]);
        let sp = [RadiansPerSecond(0.0); 3];
        let cur = [RadiansPerSecond(1.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)];
        let out = ctrl.step(sp, cur);
        assert!(out[0].0 < 0.0); // Negative error → negative output
        assert!((out[0].0 - (-1.0)).abs() < 1e-5);
        assert!((out[1].0).abs() < 1e-5);
        assert!((out[2].0).abs() < 1e-5);
    }

    #[test]
    fn test_rate_controller_saturation() {
        let ctrl = RateController::new([0.5, 0.5, 0.5]); // Smaller gain to test saturation
        let sp = [RadiansPerSecond(3.0), RadiansPerSecond(0.0), RadiansPerSecond(0.0)]; // Large error
        let cur = [RadiansPerSecond(0.0); 3];
        let out = ctrl.step(sp, cur);
        assert!((out[0].0 - 1.0).abs() < 1e-5); // Should clamp to 1.0
    }

    // --- Attitude Controller Tests ---
    #[test]
    fn test_attitude_controller_level_correction() {
        let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
        let setpoint = Quaternion::IDENTITY; // Level flight
        // Tilted current: 10 deg pitch (approx)
        let current = Quaternion::from_axis_angle(Vector3::new(0.0, 1.0, 0.0), 0.1745); // 0.1745 rad = 10 deg pitch
        
        let rate_sp = ctrl.step(&setpoint, &current);
        
        // Expect negative pitch rate to correct to level
        assert!(rate_sp[1].0 < 0.0, "Expected negative pitch rate to correct pitch error");
        assert!((rate_sp[0].0).abs() < 1e-5, "Expected no roll rate");
        assert!((rate_sp[2].0).abs() < 1e-5, "Expected no yaw rate");
        
        // Check magnitude: 2 * y_err * gain[1] = 2 * sin(theta/2) * gain
        // q_pitch = [cos(angle/2), 0, sin(angle/2), 0]
        // sin(0.1745/2) = sin(0.08725) ~ 0.087
        // pitch_err = 2 * 0.087 = 0.174
        // rate_sp[1].0 = 0.174 * 6.0 = 1.044
        assert!((rate_sp[1].0 - (-1.04)).abs() < 0.01, "Expected specific pitch rate");
    }
    
    #[test]
    fn test_attitude_controller_inverted() {
        let ctrl = AttitudeController::new([6.0, 6.0, 2.0]);
        let setpoint = Quaternion::IDENTITY;
        // Inverted current (180 deg roll around X-axis)
        let current = Quaternion::new(0.0, 1.0, 0.0, 0.0); // Exactly 180 deg roll
        
        let rate_sp = ctrl.step(&setpoint, &current);
        
        // For 180 deg roll error, the shortest path is 180 deg roll.
        // The quaternion error from identity to current is [0, -1, 0, 0].
        // roll_err = 2 * x = 2 * (-1) = -2.0. (because q_err = [0, -1, 0, 0])
        // rate_sp[0] = roll_err * gain[0] = -2.0 * 6.0 = -12.0.
        assert!((rate_sp[0].0 - (-12.0)).abs() < 1e-5, "Expected -12 rad/s roll rate setpoint");
        assert!((rate_sp[1].0).abs() < 1e-5, "Expected no pitch rate setpoint");
        assert!((rate_sp[2].0).abs() < 1e-5, "Expected no yaw rate setpoint");
    }}
