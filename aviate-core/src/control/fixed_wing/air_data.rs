//! Airspeed for the fixed-wing law: the pitot-static latch and the
//! fallback when no measurement is usable.

use crate::kernel::config::FixedWingParams;
use crate::sensor::AirData;
use crate::state::{StateEstimate, StateValidFlags};
#[allow(unused_imports)] // FloatExt supplies the no_std math methods
use crate::types::FloatExt;
use crate::types::Scalar;

use super::runtime_state::AirSample;

/// Air density at sea level in the standard atmosphere \[kg/m³\].
/// Indicated airspeed is defined against this density, so it converts
/// a dynamic pressure to an indicated airspeed for every airframe.
pub const SEA_LEVEL_AIR_DENSITY_KGPM3: Scalar = 1.225;

/// Specific gas constant of dry air \[J/(kg·K)\]. Gives the air density
/// at the probe from its static pressure and temperature.
pub const DRY_AIR_GAS_CONSTANT_JPKGK: Scalar = 287.05;

/// Offset from degrees Celsius to kelvin.
pub const CELSIUS_TO_KELVIN_OFFSET: Scalar = 273.15;

/// Age \[s\] at which the law stops flying on the last airspeed sample.
/// A pitot-static sensor supplies a sample at least each 100 ms, so ten
/// missed samples are a failed sensor and not a late one. An aged-out
/// airspeed moves the law to its fallback; a longer hold would keep the
/// stall protection on a number that no longer describes the airflow.
pub const AIRSPEED_STALE_AFTER_S: Scalar = 1.0;

const _: () = assert!(SEA_LEVEL_AIR_DENSITY_KGPM3 > 0.0);
const _: () = assert!(DRY_AIR_GAS_CONSTANT_JPKGK > 0.0);
const _: () = assert!(AIRSPEED_STALE_AFTER_S > 0.0);

/// Last airspeed measurement and its age.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AirDataLatch {
    /// Filtered indicated airspeed \[m/s\].
    pub indicated_mps: Scalar,
    /// Filtered true airspeed \[m/s\].
    pub true_mps: Scalar,
    /// Rate of the filtered true airspeed \[m/s²\].
    pub true_rate_mps2: Scalar,
    /// Time since the last sample \[s\].
    pub age_s: Scalar,
    /// Whether a sample has arrived since the last reset.
    pub primed: bool,
}

/// Indicated and true airspeed from one pitot-static sample, or `None`
/// when the sample carries no usable dynamic pressure.
pub(super) fn airspeeds_of(air: &AirData) -> Option<AirSample> {
    let dynamic_pa = air.dynamic_pressure.map(|pressure| pressure.0)?;
    if !dynamic_pa.is_finite() {
        return None;
    }
    // A negative reading is sensor noise about zero airspeed.
    let indicated = (2.0 * dynamic_pa.max(0.0) / SEA_LEVEL_AIR_DENSITY_KGPM3).sqrt();
    let density = air
        .static_pressure
        .zip(air.temperature)
        .map(|(pressure, temperature)| {
            pressure.0 / (DRY_AIR_GAS_CONSTANT_JPKGK * (temperature.0 + CELSIUS_TO_KELVIN_OFFSET))
        })
        .filter(|density| density.is_finite() && *density > 0.0)
        // A probe without static pressure and temperature gives no
        // density: true airspeed is then taken as indicated airspeed.
        .unwrap_or(SEA_LEVEL_AIR_DENSITY_KGPM3);
    Some(AirSample {
        indicated_mps: indicated,
        true_mps: indicated * (SEA_LEVEL_AIR_DENSITY_KGPM3 / density).sqrt(),
    })
}

impl AirDataLatch {
    /// Whether the last sample is recent enough to fly on.
    pub fn is_fresh(&self) -> bool {
        self.primed && self.age_s <= AIRSPEED_STALE_AFTER_S
    }

    /// Age the latch by one cycle and take the sample of this cycle.
    pub(super) fn observe(&mut self, sample: Option<AirSample>, dt: Scalar, filter_tau: Scalar) {
        let was_fresh = self.is_fresh();
        // The age stops at twice the limit: a latch that stays silent
        // for hours must not lose resolution in its own counter.
        self.age_s = (self.age_s + dt).min(2.0 * AIRSPEED_STALE_AFTER_S);
        let Some(AirSample {
            indicated_mps: indicated,
            true_mps: true_speed,
        }) = sample
        else {
            return;
        };
        let since_last = self.age_s;
        if was_fresh && since_last > 0.0 {
            let blend = since_last / (filter_tau + since_last);
            let previous_true = self.true_mps;
            self.indicated_mps += blend * (indicated - self.indicated_mps);
            self.true_mps += blend * (true_speed - self.true_mps);
            self.true_rate_mps2 = (self.true_mps - previous_true) / since_last;
        } else {
            // The first sample, or the first after an outage, has no
            // history to filter against.
            self.indicated_mps = indicated;
            self.true_mps = true_speed;
            self.true_rate_mps2 = 0.0;
        }
        self.age_s = 0.0;
        self.primed = true;
    }
}

/// Where the airspeed of one cycle came from.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AirspeedSource {
    /// A fresh pitot-static sample.
    Measured,
    /// The estimated velocity over the ground: correct in still air.
    GroundSpeed,
    /// The cruise airspeed of the airframe description.
    #[default]
    Assumed,
}

/// Airspeed that the law uses in one cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Airspeed {
    /// Indicated airspeed \[m/s\].
    pub indicated: Scalar,
    /// True airspeed \[m/s\].
    pub true_speed: Scalar,
    /// Rate of the true airspeed \[m/s²\].
    pub rate: Scalar,
    /// Origin of the values.
    pub source: AirspeedSource,
}

impl Airspeed {
    /// Select the best available airspeed for this cycle.
    pub(super) fn resolve(
        latch: &AirDataLatch,
        state: &StateEstimate,
        params: &FixedWingParams,
    ) -> Self {
        if latch.is_fresh() {
            return Self {
                indicated: latch.indicated_mps,
                true_speed: latch.true_mps,
                rate: latch.true_rate_mps2,
                source: AirspeedSource::Measured,
            };
        }
        if state.valid_flags.contains(StateValidFlags::VELOCITY) {
            let [north, east, down] = state.velocity_ned;
            let speed = (north.0 * north.0 + east.0 * east.0 + down.0 * down.0).sqrt();
            return Self {
                indicated: speed,
                true_speed: speed,
                rate: 0.0,
                source: AirspeedSource::GroundSpeed,
            };
        }
        Self {
            indicated: params.cruise_airspeed.0,
            true_speed: params.cruise_airspeed.0,
            rate: 0.0,
            source: AirspeedSource::Assumed,
        }
    }

    /// Whether the value is a measurement of the airflow.
    pub(super) fn is_measured(&self) -> bool {
        self.source == AirspeedSource::Measured
    }

    /// True airspeed for use as a divisor: never below the stall speed,
    /// so a slow or grounded vehicle cannot produce an unbounded rate.
    pub(super) fn divisor(&self, params: &FixedWingParams) -> Scalar {
        self.true_speed.max(params.stall_airspeed.0)
    }
}
