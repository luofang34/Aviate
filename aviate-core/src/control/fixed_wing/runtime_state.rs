//! Persistent state of the fixed-wing law, owned by `KernelState.controller`.

use crate::control::runtime::ControllerRuntimeState;
use crate::replicable::{copy_into, Replicable};
use crate::sensor::AirData;
use crate::types::{Scalar, Seconds};

use super::air_data::{airspeeds_of, AirDataLatch, AirspeedSource, StaticAir};
use super::attitude::RateLoopState;
use super::energy::EnergyLoopState;
use super::lateral::LateralLoopState;

/// One pitot-static sample: airspeeds \[m/s\] and the static port.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AirSample {
    /// Indicated airspeed.
    pub indicated_mps: Scalar,
    /// True airspeed.
    pub true_mps: Scalar,
    /// Static pressure and its height scale, when the probe has the port.
    pub static_air: Option<StaticAir>,
}

/// Targets that the outer loops handed to the inner loops in the last
/// cycle. The law writes them for an observer and never reads them.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FlightTargets {
    /// Bank target \[rad\].
    pub bank: Scalar,
    /// Pitch target \[rad\].
    pub pitch: Scalar,
    /// Body-rate demand in roll, pitch, yaw order \[rad/s\].
    pub rates: [Scalar; 3],
    /// Origin of the airspeed that the cycle used.
    pub airspeed_source: AirspeedSource,
}

/// Persistent runtime state of the fixed-wing law.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FixedWingRuntimeState {
    /// Interval of the next controller cycle \[s\]; zero until the
    /// kernel supplies a valid interval.
    pub dt_sec: Scalar,
    /// Airspeed sample that the kernel offered for the next cycle.
    pub air_sample: Option<AirSample>,
    /// Last airspeed measurement and its age.
    pub air: AirDataLatch,
    /// Static pressure \[Pa\] of the level that the throttle holds while
    /// the airspeed is on pitch.
    pub held_static_pa: Option<Scalar>,
    /// Memory of the lateral guidance loop.
    pub lateral: LateralLoopState,
    /// Memory of the energy loops.
    pub energy: EnergyLoopState,
    /// Memory of the body-rate loops.
    pub rates: RateLoopState,
    /// Targets of the last cycle.
    pub targets: FlightTargets,
}

impl ControllerRuntimeState for FixedWingRuntimeState {
    fn set_cycle_interval(&mut self, interval: Seconds) {
        self.dt_sec = if interval.0.is_finite() && interval.0 > 0.0 {
            interval.0
        } else {
            0.0
        };
    }

    fn set_air_data(&mut self, air: Option<&AirData>) {
        self.air_sample = air.and_then(airspeeds_of);
    }

    fn reset(&mut self) {
        *self = Self::default();
    }
}

const fn source_tag(source: AirspeedSource) -> u8 {
    match source {
        AirspeedSource::Assumed => 0,
        AirspeedSource::GroundSpeed => 1,
        AirspeedSource::Measured => 2,
    }
}

impl Replicable for FixedWingRuntimeState {
    // 24 f32 lanes and 7 one-byte tags. Every persistent field must
    // appear here: an omitted field lets two lockstep channels diverge
    // in hidden state while they compare byte-equal.
    const ENCODED_LEN: usize = 103;

    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let sample = self.air_sample.unwrap_or_default();
        let sample_port = sample.static_air.unwrap_or_default();
        let [roll_integrator, pitch_integrator, yaw_integrator] = self.rates.integrators;
        let [roll_demand, pitch_demand, yaw_demand] = self.targets.rates;
        let lanes: [Scalar; 24] = [
            self.dt_sec,
            sample.indicated_mps,
            sample.true_mps,
            sample_port.pressure_pa,
            sample_port.scale_height_m,
            self.air.indicated_mps,
            self.air.true_mps,
            self.air.true_rate_mps2,
            self.air.static_pa,
            self.air.static_rate_paps,
            self.air.scale_height_m,
            self.air.age_s,
            self.held_static_pa.unwrap_or_default(),
            self.lateral.turn_rate_integrator,
            self.energy.throttle_integrator,
            self.energy.pitch_integrator,
            roll_integrator,
            pitch_integrator,
            yaw_integrator,
            self.targets.bank,
            self.targets.pitch,
            roll_demand,
            pitch_demand,
            yaw_demand,
        ];
        let tags = [
            u8::from(self.air_sample.is_some()),
            u8::from(sample.static_air.is_some()),
            u8::from(self.air.primed),
            u8::from(self.air.static_primed),
            u8::from(self.held_static_pa.is_some()),
            u8::from(self.energy.underspeed),
            source_tag(self.targets.airspeed_source),
        ];
        let mut written = 0_usize;
        for lane in lanes {
            written += copy_into(buf, written, &lane.to_le_bytes());
        }
        written += copy_into(buf, written, &tags);
        written
    }
}
