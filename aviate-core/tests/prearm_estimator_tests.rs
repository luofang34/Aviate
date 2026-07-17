//! The estimator observes from boot (#277): a DISARMED kernel fed
//! healthy sensors must fuse GNSS/baro and authorize
//! POSITION/VELOCITY at `Good` quality — a pose-gated GCS needs that
//! authorization to send the very first ARM — while every actuator
//! output stays the forced-safe pattern the disarmed gate promises
//! (LLR-FLT-201/202 are untouched: only state estimation runs, never
//! actuator authority).

#![allow(clippy::expect_used, clippy::panic)]

use aviate_core::checks::PreArmFlags;
use aviate_core::control::multirotor::MultirotorController;
use aviate_core::control::{Command, CommandSource, ConfigMode, ControlMode, Setpoint};
use aviate_core::ekf::Ekf;
use aviate_core::fault::FaultFlags;
use aviate_core::math::{Quaternion, Vector3};
use aviate_core::mixer::{ActuatorState, ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::sensor::{
    AirData, BaroData, GnssData, GnssFix, GnssHealth, ImuData, MagData, SensorHealth,
    SensorReading, SensorSet,
};
use aviate_core::state::{EstimateQuality, StateValidFlags};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Celsius, Meters, MetersPerSecond, MetersPerSecondSquared, Microtesla, Pascals,
    RadiansPerSecond, Seconds,
};
use aviate_core::{ChannelId, InitState};

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn dt_100hz() -> TimeDelta {
    TimeDelta {
        dt_sec: Seconds(0.01),
        tick_delta: 10_000,
    }
}

fn make_kernel() -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
    let mixer = QuadXMixer {
        timestamp_source: timestamp,
    };
    let mode_config = ModeConfig {
        mode: ConfigMode::Hover,
        groups: &[],
    };
    let required = PreArmFlags::IMU_HEALTHY
        | PreArmFlags::IMU_CONVERGED
        | PreArmFlags::EKF_CONVERGED
        | PreArmFlags::THROTTLE_LOW
        | PreArmFlags::CONFIG_VALID;
    let mut kernel = aviate_core::kernel::builder::AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(mixer)
        .sanitizer(Sanitizer)
        .pre_arm_required(required)
        .config(aviate_core::kernel::config::ResolvedKernelConfig {
            mode_config,
            ..Default::default()
        })
        .build()
        .expect("checked construction must accept the default binding");
    kernel.state.checks.pre_arm.update_throttle(true);
    kernel
}

/// A stationary vehicle at the origin: consistent IMU, baro, mag,
/// and ThreeD-fix GNSS — the healthy bench scenario.
fn still_sensors() -> SensorSet {
    let ts = timestamp();
    SensorSet {
        imus: [
            SensorReading {
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
            },
            SensorReading::default(),
            SensorReading::default(),
        ],
        baros: [
            SensorReading {
                value: BaroData {
                    altitude: Some(Meters(0.0)),
                    air: AirData {
                        static_pressure: Some(Pascals(101_325.0)),
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
            },
            SensorReading::default(),
        ],
        mags: [
            SensorReading {
                value: MagData {
                    field_ut: [Microtesla(20.0), Microtesla(0.0), Microtesla(40.0)],
                },
                valid: true,
                source_id: 0,
                timestamp: ts,
                health: SensorHealth::Good,
            },
            SensorReading::default(),
        ],
        gnss: [
            SensorReading {
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
            },
            SensorReading::default(),
        ],
        airspeeds: [SensorReading::default(), SensorReading::default()],
        geometry: None,
    }
}

fn zero_cmd() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Pilot,
    }
}

/// Seed the estimator (the runner's TRIAD init in production) and walk
/// the init state machine to Ready WITHOUT arming.
fn ready_disarmed_kernel() -> aviate_core::DefaultAviateKernel<MultirotorController, QuadXMixer> {
    let mut kernel = make_kernel();
    kernel.state.estimator.init(
        Vector3::new(Meters(0.0), Meters(0.0), Meters(0.0)),
        Vector3::new(
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
            MetersPerSecond(0.0),
        ),
        Quaternion::IDENTITY,
    );
    let sensors = still_sensors();
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
    assert_eq!(kernel.state.init_state, InitState::Ready);
    kernel
}

#[test]
fn disarmed_kernel_authorizes_position_and_velocity() {
    let mut kernel = ready_disarmed_kernel();
    let sensors = still_sensors();
    let cmd = zero_cmd();

    let mut last_estimate = None;
    for _ in 0..300 {
        let result = kernel.update(
            ChannelId::PRIMARY,
            dt_100hz(),
            &sensors,
            &cmd,
            0,
            &ActuatorState::default(),
            None,
        );
        // The disarmed gate's promise is untouched: forced-safe
        // output on every cycle while the estimator converges.
        assert!(
            result.actuator.outputs.iter().all(|o| o.0.abs() < 1e-6),
            "disarmed output must stay the safe pattern"
        );
        last_estimate = Some(result.estimate);
    }

    // Never armed — authorization must not have required it.
    assert_eq!(kernel.state.init_state, InitState::Ready);

    let estimate = last_estimate.expect("update ran");
    assert_eq!(
        estimate.quality,
        EstimateQuality::Good,
        "healthy bench aiding must reach Good before arming"
    );
    assert!(estimate.valid_flags.contains(StateValidFlags::ATTITUDE));
    assert!(
        estimate.valid_flags.contains(StateValidFlags::POSITION),
        "POSITION must authorize disarmed — the first ARM of a \
         pose-gated GCS depends on it (#277)"
    );
    assert!(estimate.valid_flags.contains(StateValidFlags::VELOCITY));
}

#[test]
fn disarmed_kernel_skips_observation_under_critical_faults() {
    let mut kernel = ready_disarmed_kernel();
    let sensors = still_sensors();
    let cmd = zero_cmd();

    // A latched critical fault must keep the disarmed path from
    // feeding a compromised estimator, exactly like the armed path.
    kernel.state.faults.insert(FaultFlags::ESTIMATOR_DIVERGED);

    for _ in 0..300 {
        let result = kernel.update(
            ChannelId::PRIMARY,
            dt_100hz(),
            &sensors,
            &cmd,
            0,
            &ActuatorState::default(),
            None,
        );
        assert!(result.actuator.outputs.iter().all(|o| o.0.abs() < 1e-6));
    }

    // No observation ran: aiding never fused, so the estimate stays
    // at the seed's Degraded ceiling instead of reaching Good.
    let estimate = kernel.state.estimator.get_estimate();
    assert_eq!(estimate.quality, EstimateQuality::Degraded);
    assert!(!estimate.valid_flags.contains(StateValidFlags::POSITION));
}

// --- boot-time fusion gates --------------------------------------------------
//
// With observation running from boot, GNSS/baro readings can now
// arrive BEFORE the attitude seed. The per-update `initialized`
// gates (mirroring the mag update's) must drop them: fusing into an
// unseeded filter would correct states that carry no meaning yet.

#[test]
fn gnss_update_on_unseeded_filter_is_dropped() {
    use aviate_core::ekf::EkfState;

    let ekf = Ekf::default();
    let mut state = EkfState::default();
    assert!(!state.is_initialized());

    let sensors = still_sensors();
    let mut gnss = sensors.gnss[0];
    gnss.value.position_ned = [Meters(10.0), Meters(5.0), Meters(-50.0)];
    ekf.update_gnss_state(&mut state, &gnss);

    let estimate = state.get_estimate();
    assert_eq!(estimate.quality, EstimateQuality::Unusable);
    assert!(
        estimate.position_ned.iter().all(|p| p.0 == 0.0),
        "an unseeded filter must not absorb GNSS position"
    );
}

#[test]
fn baro_update_on_unseeded_filter_is_dropped() {
    use aviate_core::ekf::EkfState;

    let ekf = Ekf::default();
    let mut state = EkfState::default();
    assert!(!state.is_initialized());

    // ~1000 m pressure altitude: a large innovation if it fused.
    let mut baro = still_sensors().baros[0];
    baro.value.air.static_pressure = Some(Pascals(90_000.0));
    ekf.update_baro_state(&mut state, &baro);

    let estimate = state.get_estimate();
    assert_eq!(estimate.quality, EstimateQuality::Unusable);
    assert!(
        estimate.position_ned.iter().all(|p| p.0 == 0.0),
        "an unseeded filter must not absorb or datum-latch baro height"
    );
}
