#![forbid(unsafe_code)]

#[cfg(test)]
mod tests {
    use aviate_core::control::attitude::AttitudeController;
    use aviate_core::control::rate::RateController;
    use aviate_core::control::{Command, VehicleController};
    use aviate_core::ekf::Ekf;
    use aviate_core::math::{Quaternion, Vector3};
    use aviate_core::mixer::{ActuatorCmd, Mixer, Sanitizer};
    use aviate_core::sensor::{
        AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth,
        SensorReading, SensorSet,
    };
    use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
    use aviate_core::types::Seconds;
    use aviate_core::types::{
        Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Normalized, Pascals,
        RadiansPerSecond, Scalar,
    };
    use aviate_core::{AviateKernel, ChannelId};

    trait KernelTestExt {
        fn step_test(
            &mut self,
            time_delta: TimeDelta,
            cmd: &Command,
            sensors: &SensorSet,
            command_age_ms: u32,
        ) -> ActuatorCmd;
    }

    impl<
            E: aviate_core::ekf::Estimator,
            V: VehicleController,
            M: Mixer,
            S: aviate_core::mixer::ActuatorSanitizer,
        > KernelTestExt for AviateKernel<E, V, M, S>
    {
        fn step_test(
            &mut self,
            time_delta: TimeDelta,
            cmd: &Command,
            sensors: &SensorSet,
            _command_age_ms: u32,
        ) -> ActuatorCmd {
            let actuator_state = self.actuator_state.clone();
            let res = self.update(
                ChannelId::PRIMARY,
                time_delta,
                sensors,
                cmd,
                &actuator_state,
                None,
            );
            res.actuator
        }
    }

    fn dummy_time_delta() -> TimeDelta {
        TimeDelta {
            dt_sec: Seconds(0.01),
            tick_delta: 10000,
        }
    }

    #[test]
    fn test_ekf_init_predict() {
        let mut ekf = Ekf::default();
        assert!(!ekf.is_initialized());

        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu_accel = ImuData {
            accel: [
                MetersPerSecondSquared(1.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
            ],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        let dt = 0.1;
        for _ in 0..10 {
            ekf.predict(&imu_accel, dt);
        }

        let est = ekf.get_estimate();

        let vel_x = est.velocity_ned[0].0;
        assert!(
            (vel_x - 1.0).abs() < 0.1,
            "Velocity X should be ~1.0, got {}",
            vel_x
        );

        let pos_x = est.position_ned[0].0;
        assert!(
            (pos_x - 0.5).abs() < 0.1,
            "Position X should be ~0.5, got {}",
            pos_x
        );
    }

    #[test]
    fn test_ekf_gnss_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu_stationary = ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
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
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Good,
        };

        ekf.update_gnss(&gnss_reading);

        let est = ekf.get_estimate();
        assert!(
            est.position_ned[0].0 > 0.1,
            "Position should move towards measurement"
        );
        assert!(
            est.position_ned[0].0 < 1.0,
            "Position should not overshoot measurement"
        );
    }

    /// Create valid sensor data for testing
    /// Provides healthy IMU, baro, and mag readings
    fn valid_test_sensors() -> aviate_core::sensor::SensorSet {
        use aviate_core::sensor::SensorSet;
        use aviate_core::types::Celsius;

        let ts = Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        };

        // Valid IMU with gravity on Z axis
        let valid_imu = SensorReading {
            value: ImuData {
                accel: [
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(0.0),
                    MetersPerSecondSquared(-9.81),
                ],
                gyro: [
                    RadiansPerSecond(0.0),
                    RadiansPerSecond(0.0),
                    RadiansPerSecond(0.0),
                ],
            },
            valid: true,
            source_id: 0,
            timestamp: ts,
            health: SensorHealth::Good,
        };

        // Valid baro at sea level
        let valid_baro = SensorReading {
            value: BaroData {
                altitude: Some(Meters(0.0)),
                air: AirData {
                    static_pressure: Some(Pascals(101325.0)),
                    dynamic_pressure: None,
                    total_pressure: None,
                    temperature: Some(Celsius(20.0)),
                    indicated_airspeed: None,
                    true_airspeed: None,
                },
            },
            valid: true,
            source_id: 0,
            timestamp: ts,
            health: SensorHealth::Good,
        };

        // Valid mag
        let valid_mag = SensorReading {
            value: MagData {
                field_ut: [Microtesla(20.0), Microtesla(0.0), Microtesla(40.0)],
            },
            valid: true,
            source_id: 0,
            timestamp: ts,
            health: SensorHealth::Good,
        };

        // Valid GNSS (to avoid ALL_GNSS_LOST fault)
        let valid_gnss = SensorReading {
            value: GnssData {
                position_ned: [Meters(0.0), Meters(0.0), Meters(0.0)],
                velocity_ned: [
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                    MetersPerSecond(0.0),
                ],
                fix: GnssFix::ThreeD,
                health: GnssHealth::Good,
            },
            valid: true,
            source_id: 0,
            timestamp: ts,
            health: SensorHealth::Good,
        };

        SensorSet {
            imus: [
                valid_imu,
                SensorReading::default(),
                SensorReading::default(),
            ],
            gnss: [valid_gnss, SensorReading::default()],
            mags: [valid_mag, SensorReading::default()],
            baros: [valid_baro, SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
        }
    }

    #[test]
    fn test_kernel_mc() {
        use aviate_core::checks::PreArmFlags;
        use aviate_core::control::multirotor::MultirotorController;
        use aviate_core::control::{CommandSource, ConfigMode, ControlMode, Setpoint};
        use aviate_core::mixer::{ModeConfig, QuadXMixer};

        fn dummy_time() -> Timestamp {
            Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            }
        }
        let mixer = QuadXMixer {
            timestamp_source: dummy_time,
        };

        let mode_config = ModeConfig {
            mode: ConfigMode::Hover,
            groups: &[],
        };

        // Use minimal pre-arm requirements for testing
        // Note: Don't require NO_FAULTS since we're not providing GPS (ALL_GNSS_LOST expected)
        let test_required = PreArmFlags::IMU_HEALTHY
            | PreArmFlags::IMU_CONVERGED
            | PreArmFlags::EKF_CONVERGED
            | PreArmFlags::THROTTLE_LOW
            | PreArmFlags::CONFIG_VALID;

        let mut kernel = AviateKernel::with_pre_arm_required(
            Ekf::default(),
            MultirotorController::default(),
            mixer,
            Sanitizer::default(),
            mode_config,
            test_required,
        );

        // Initialize EKF
        kernel.pipeline.estimator.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
        let safe_sensors = valid_test_sensors();
        let act_cmd_safe = kernel.step_test(dummy_time_delta(), &cmd, &safe_sensors, 0);
        for i in 0..4 {
            assert!(
                (act_cmd_safe.outputs[i].0).abs() < 1e-5,
                "Should be zero when disarmed"
            );
        }

        // Provide valid sensor data and set throttle low
        let valid_sensors = valid_test_sensors();
        kernel.checks.pre_arm.update_throttle(true); // Throttle low

        // Cycle through init states with valid sensor data
        // Need 100+ iterations for sensor convergence
        for _ in 0..150 {
            kernel.init_step(&valid_sensors, dummy_time());
            if kernel.is_ready() {
                break;
            }
        }
        assert!(
            kernel.is_ready(),
            "Kernel failed to become ready. Missing: {:?}",
            kernel.checks.pre_arm.missing()
        );

        // Arm
        kernel.arm().expect("Failed to arm");

        let act_cmd = kernel.step_test(dummy_time_delta(), &cmd, &valid_sensors, 0);

        // QuadXMixer with 0 R/P/Y should output collective on all 4 motors
        for i in 0..4 {
            assert!(
                (act_cmd.outputs[i].0 - 0.5).abs() < 1e-5,
                "Should be 0.5 when armed"
            );
        }
    }

    #[test]
    fn test_kernel_fw() {
        use aviate_core::checks::PreArmFlags;
        use aviate_core::control::fixed_wing::FixedWingController;
        use aviate_core::control::{CommandSource, ConfigMode, ControlMode, Setpoint};
        use aviate_core::mixer::{ModeConfig, QuadXMixer};

        fn dummy_time() -> Timestamp {
            Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            }
        }
        let mixer = QuadXMixer {
            timestamp_source: dummy_time,
        };

        let mode_config = ModeConfig {
            mode: ConfigMode::Cruise,
            groups: &[],
        };

        // Use minimal pre-arm requirements for testing
        // Note: Don't require NO_FAULTS since we're not providing GPS (ALL_GNSS_LOST expected)
        let test_required = PreArmFlags::IMU_HEALTHY
            | PreArmFlags::IMU_CONVERGED
            | PreArmFlags::EKF_CONVERGED
            | PreArmFlags::THROTTLE_LOW
            | PreArmFlags::CONFIG_VALID;

        let mut kernel = AviateKernel::with_pre_arm_required(
            Ekf::default(),
            FixedWingController,
            mixer,
            Sanitizer::default(),
            mode_config,
            test_required,
        );

        // Init EKF
        kernel.pipeline.estimator.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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

        // Provide valid sensor data and set throttle low
        let valid_sensors = valid_test_sensors();
        kernel.checks.pre_arm.update_throttle(true);

        // Cycle through init states with valid sensor data
        for _ in 0..150 {
            kernel.init_step(&valid_sensors, dummy_time());
            if kernel.is_ready() {
                break;
            }
        }
        assert!(
            kernel.is_ready(),
            "Kernel failed to become ready. Missing: {:?}",
            kernel.checks.pre_arm.missing()
        );

        kernel.arm().expect("Failed to arm");

        let act_cmd = kernel.step_test(dummy_time_delta(), &cmd, &valid_sensors, 0);

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
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu_rot = ImuData {
            accel: [MetersPerSecondSquared(0.0); 3],
            gyro: [
                RadiansPerSecond(0.0),
                RadiansPerSecond(0.0),
                RadiansPerSecond(1.0),
            ],
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
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
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
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
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
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
                },
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Failed,
        };

        ekf.update_baro(&baro_reading);
        let est = ekf.get_estimate();
        assert_eq!(
            est.position_ned[2].0, 0.0,
            "Failed sensor should be ignored"
        );
    }

    #[test]
    fn test_gnss_fix_none_rejection() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
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
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Good,
        };

        ekf.update_gnss(&gnss_reading);
        let est = ekf.get_estimate();
        assert_eq!(
            est.position_ned[0].0, 0.0,
            "GnssFix::None should be ignored"
        );
    }

    #[test]
    fn test_nan_input_handling() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu_nan = ImuData {
            accel: [MetersPerSecondSquared(Scalar::NAN); 3],
            gyro: [RadiansPerSecond(0.0); 3],
        };

        ekf.predict(&imu_nan, 0.01);
        let est = ekf.get_estimate();
        assert!(
            !est.position_ned[0].0.is_nan(),
            "State should not be NaN after bad input"
        );
    }

    #[test]
    fn test_baro_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu_stationary = ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
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
                },
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Good,
        };

        for _ in 0..50 {
            ekf.predict(&imu_stationary, 0.1);
            ekf.update_baro(&baro_reading);
        }

        let est = ekf.get_estimate();
        assert!(
            est.position_ned[2].0 < -0.5,
            "Baro update should move Z position (NED)"
        );
    }

    #[test]
    fn test_mag_update() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let mag_reading = SensorReading {
            value: MagData {
                field_ut: [Microtesla(0.0); 3],
            },
            valid: true,
            source_id: 0,
            timestamp: Timestamp {
                ticks: 0,
                source: TimeSource::Internal,
            },
            health: SensorHealth::Good,
        };

        ekf.update_mag(&mag_reading);
    }

    #[test]
    fn test_long_run_stability() {
        let mut ekf = Ekf::default();
        ekf.init(
            Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
            Vector3::new(
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
                MetersPerSecond(0.0),
            ),
            Quaternion::IDENTITY,
        );

        let imu = ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
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
        let sp = [
            RadiansPerSecond(1.0),
            RadiansPerSecond(0.5),
            RadiansPerSecond(-0.5),
        ];
        let cur = [
            RadiansPerSecond(1.0),
            RadiansPerSecond(0.5),
            RadiansPerSecond(-0.5),
        ];
        let out = ctrl.step(sp, cur);
        assert!((out[0].0).abs() < 1e-5);
        assert!((out[1].0).abs() < 1e-5);
        assert!((out[2].0).abs() < 1e-5);
    }

    #[test]
    fn test_rate_controller_positive_error() {
        let ctrl = RateController::new([1.0, 1.0, 1.0]);
        let sp = [
            RadiansPerSecond(1.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
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
        let cur = [
            RadiansPerSecond(1.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ];
        let out = ctrl.step(sp, cur);
        assert!(out[0].0 < 0.0); // Negative error → negative output
        assert!((out[0].0 - (-1.0)).abs() < 1e-5);
        assert!((out[1].0).abs() < 1e-5);
        assert!((out[2].0).abs() < 1e-5);
    }

    #[test]
    fn test_rate_controller_saturation() {
        let ctrl = RateController::new([0.5, 0.5, 0.5]); // Smaller gain to test saturation
        let sp = [
            RadiansPerSecond(3.0),
            RadiansPerSecond(0.0),
            RadiansPerSecond(0.0),
        ]; // Large error
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
        assert!(
            rate_sp[1].0 < 0.0,
            "Expected negative pitch rate to correct pitch error"
        );
        assert!((rate_sp[0].0).abs() < 1e-5, "Expected no roll rate");
        assert!((rate_sp[2].0).abs() < 1e-5, "Expected no yaw rate");

        // Check magnitude: 2 * y_err * gain[1] = 2 * sin(theta/2) * gain
        // q_pitch = [cos(angle/2), 0, sin(angle/2), 0]
        // sin(0.1745/2) = sin(0.08725) ~ 0.087
        // pitch_err = 2 * 0.087 = 0.174
        // rate_sp[1].0 = 0.174 * 6.0 = 1.044
        assert!(
            (rate_sp[1].0 - (-1.04)).abs() < 0.01,
            "Expected specific pitch rate"
        );
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
        assert!(
            (rate_sp[0].0 - (-12.0)).abs() < 1e-5,
            "Expected -12 rad/s roll rate setpoint"
        );
        assert!(
            (rate_sp[1].0).abs() < 1e-5,
            "Expected no pitch rate setpoint"
        );
        assert!((rate_sp[2].0).abs() < 1e-5, "Expected no yaw rate setpoint");
    }
}
