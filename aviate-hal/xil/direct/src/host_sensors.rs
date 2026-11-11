//! Sensor samples that a host builds for one [`crate::DirectXilBoard::step_sensors`] call.
//!
//! The host owns the sensor models. This module holds only the hand-over
//! types: HAL readings, the time of each measurement, and the optional
//! geomagnetic reference.

use aviate_core::sensor::GeomagneticReference;
use aviate_hal_io::{
    RawAirspeedReading, RawBaroReading, RawGnssReading, RawImuReading, RawMagReading,
};

use crate::synthesis::DirectStepSample;

/// One HAL reading and the time at which the sensor measured it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DirectSensorSample<T> {
    /// Measurement time in microseconds on the simulation clock.
    pub timestamp_us: u64,
    /// Sensor output in the HAL reading type.
    pub reading: T,
}

impl<T> DirectSensorSample<T> {
    /// Pair `reading` with its measurement time.
    #[must_use]
    pub const fn new(timestamp_us: u64, reading: T) -> Self {
        Self {
            timestamp_us,
            reading,
        }
    }
}

/// Host-built sensor samples for one kernel step.
///
/// Each lane is `None` on a step in which that sensor supplied no new
/// sample. The board then keeps the last value of the lane and does not
/// fuse it again, as a flight board does between samples.
///
/// Frames and units are those of the HAL readings: body axes are
/// forward-right-down, the local frame is north-east-down with its origin
/// at [`crate::DirectBoardConfig::origin`], and all quantities are SI.
#[derive(Debug, Clone, Copy, Default)]
pub struct DirectHostSensors {
    /// Simulation time of this step in microseconds.
    ///
    /// Time must increase within one `reset_generation`. The first call of
    /// a generation sets the time reference and runs no kernel cycle.
    pub time_us: u64,
    /// Simulator world epoch; a change signals a world reset.
    pub reset_generation: u32,
    /// Accelerometer specific force and gyroscope rate in body axes.
    pub imu: Option<DirectSensorSample<RawImuReading>>,
    /// Static pressure and temperature.
    pub baro: Option<DirectSensorSample<RawBaroReading>>,
    /// Magnetic field in body axes.
    pub mag: Option<DirectSensorSample<RawMagReading>>,
    /// Position and velocity fix. A receiver without a fix supplies either
    /// no sample or a sample whose `fix` is `GnssFix::None`.
    pub gnss: Option<DirectSensorSample<RawGnssReading>>,
    /// Pitot-static differential pressure.
    pub airspeed: Option<DirectSensorSample<RawAirspeedReading>>,
    /// Magnetic declination that the estimator adds to the magnetic heading.
    ///
    /// `None` selects the value from the geomagnetic model of the board.
    /// The board evaluates that model at the last GNSS fix, or at the
    /// origin before the first fix.
    pub geomagnetic: Option<GeomagneticReference>,
}

impl DirectStepSample {
    /// Return this synthesized sample as host sensor input.
    ///
    /// Each reading carries the sample timestamp. `geomagnetic` passes
    /// through unchanged.
    #[must_use]
    pub fn host_sensors(&self, geomagnetic: Option<GeomagneticReference>) -> DirectHostSensors {
        let at = self.timestamp_us;
        DirectHostSensors {
            time_us: at,
            reset_generation: self.reset_generation,
            imu: Some(DirectSensorSample::new(at, self.imu)),
            baro: self
                .baro
                .map(|reading| DirectSensorSample::new(at, reading)),
            mag: self.mag.map(|reading| DirectSensorSample::new(at, reading)),
            gnss: self
                .gnss
                .map(|reading| DirectSensorSample::new(at, reading)),
            airspeed: self
                .airspeed
                .map(|reading| DirectSensorSample::new(at, reading)),
            geomagnetic,
        }
    }
}
