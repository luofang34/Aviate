//! Command freshness tracking and decimated sensor caching for [`super::DirectXilBoard`].

use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};
use aviate_core::sensor::{
    BaroData, GeomagneticReference, GnssData, GnssHealth, ImuData, MagData, SensorReading,
    SensorSet,
};
use aviate_core::types::{Microtesla, NormalizedThrust};
use aviate_core::InitState;
use aviate_hal_io::SystemCommand;
use aviate_link::mavlink::MavState;

/// Command ingress state machine enforcing `#133` setpoint freshness semantics.
#[derive(Debug, Clone, Default)]
pub(super) struct DirectCommandIngress {
    setpoint: Option<Command>,
    setpoint_rx_us: Option<u64>,
}

impl DirectCommandIngress {
    /// Record `cmd` at `now_us`; discrete lifecycle commands never refresh setpoint age.
    pub(super) fn receive(&mut self, cmd: &SystemCommand, now_us: u64) {
        if let SystemCommand::FlightControl(flight) = cmd {
            self.setpoint = Some(flight.clone());
            self.setpoint_rx_us = Some(now_us);
        }
    }

    /// Return the latest retained flight setpoint, or [`default_failsafe_command`] before the first.
    pub(super) fn active_command(&self) -> Command {
        self.setpoint
            .clone()
            .unwrap_or_else(default_failsafe_command)
    }

    /// Return setpoint age in milliseconds (`u32::MAX` before the first `FlightControl` arrives).
    pub(super) fn setpoint_age_ms(&self, now_us: u64) -> u32 {
        match self.setpoint_rx_us {
            Some(rx_us) => u32::try_from(now_us.saturating_sub(rx_us) / 1_000).unwrap_or(u32::MAX),
            None => u32::MAX,
        }
    }
}

/// Cached sensor readings preserving health across decimated aiding-sensor intervals.
///
/// On intermediate high-rate IMU ticks where barometer, magnetometer, or GNSS did not emit a new
/// measurement, the cached reading keeps `valid = true` and `health = SensorHealth::Good` so
/// pre-arm and runtime health monitors see live sensors, while its payload value signals no new
/// scalar fusion to `Ekf::observe`.
#[derive(Debug, Clone, Default)]
pub(super) struct DirectSensorCache {
    pub(super) imu: Option<SensorReading<ImuData>>,
    pub(super) baro: Option<SensorReading<BaroData>>,
    pub(super) mag: Option<SensorReading<MagData>>,
    pub(super) gnss: Option<SensorReading<GnssData>>,
    pub(super) baro_fresh: bool,
    pub(super) mag_fresh: bool,
    pub(super) gnss_fresh: bool,
    pub(super) baro_pressure_pa: Option<f32>,
    pub(super) baro_temperature_c: f32,
}

impl DirectSensorCache {
    /// Mark aiding channels non-fresh at the start of a new step before reading `BoardHal`.
    pub(super) fn begin_step(&mut self) {
        self.baro_fresh = false;
        self.mag_fresh = false;
        self.gnss_fresh = false;
    }

    /// Store a newly read barometer sample.
    pub(super) fn record_baro(&mut self, reading: SensorReading<BaroData>) {
        if let Some(p) = reading.value.air.static_pressure {
            self.baro_pressure_pa = Some(p.0);
        }
        if let Some(t) = reading.value.air.temperature {
            self.baro_temperature_c = t.0;
        }
        self.baro = Some(reading);
        self.baro_fresh = true;
    }

    /// Store a newly read magnetometer sample.
    pub(super) fn record_mag(&mut self, reading: SensorReading<MagData>) {
        self.mag = Some(reading);
        self.mag_fresh = true;
    }

    /// Store a newly read GNSS sample.
    pub(super) fn record_gnss(&mut self, reading: SensorReading<GnssData>) {
        self.gnss = Some(reading);
        self.gnss_fresh = true;
    }

    /// Build the [`SensorSet`] for the current kernel step.
    pub(super) fn to_sensor_set(&self, geomagnetic: Option<GeomagneticReference>) -> SensorSet {
        let baro0 = match self.baro {
            Some(mut baro) => {
                if !self.baro_fresh {
                    baro.value.air.static_pressure = None;
                }
                baro
            }
            None => SensorReading::default(),
        };
        let mag0 = match self.mag {
            Some(mut mag) => {
                if !self.mag_fresh {
                    mag.value.field_ut = [Microtesla(0.0); 3];
                }
                mag
            }
            None => SensorReading::default(),
        };
        let gnss0 = match self.gnss {
            Some(mut gnss) => {
                if !self.gnss_fresh && gnss.value.health == GnssHealth::Good {
                    gnss.value.health = GnssHealth::Suspect;
                }
                gnss
            }
            None => SensorReading::default(),
        };
        SensorSet {
            imus: [
                self.imu.unwrap_or_default(),
                SensorReading::default(),
                SensorReading::default(),
            ],
            gnss: [gnss0, SensorReading::default()],
            mags: [mag0, SensorReading::default()],
            baros: [baro0, SensorReading::default()],
            airspeeds: [SensorReading::default(), SensorReading::default()],
            geometry: None,
            geomagnetic,
        }
    }
}

/// Construct a zero-collective failsafe attitude [`Command`].
pub(super) fn default_failsafe_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(0.0),
            ..Setpoint::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
        source: CommandSource::Failsafe,
    }
}

/// Map [`InitState`] to MAVLink `MAV_STATE`.
pub(super) const fn init_state_to_mav_state(state: InitState) -> MavState {
    match state {
        InitState::PowerOn | InitState::ConfigLoading => MavState::Boot,
        InitState::SensorInit | InitState::EstimatorConverging | InitState::PreArm => {
            MavState::Calibrating
        }
        InitState::Ready | InitState::Disarmed => MavState::Standby,
        InitState::Armed => MavState::Active,
        InitState::Fault => MavState::Critical,
    }
}
