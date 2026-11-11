//! Fixed-wing control law: total-energy control of airspeed and height,
//! course guidance into bank, coordinated-turn attitude loops and
//! airspeed-scaled body-rate loops.
//!
//! Every airframe number comes from
//! [`FixedWingParams`](crate::kernel::config::FixedWingParams) in the
//! resolved configuration; persistent state lives in
//! `KernelState.controller` as a [`FixedWingRuntimeState`].

pub mod air_data;
mod attitude;
mod energy;
mod lateral;
mod runtime_state;
mod targets;
#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests;

pub use air_data::{AirDataLatch, AirspeedSource};
pub use attitude::RateLoopState;
pub use energy::EnergyLoopState;
pub use runtime_state::{AirSample, FixedWingRuntimeState, FlightTargets};

use crate::control::{
    AxisCommand, Command, ConfigMode, ControlMode, ControllerConfigMismatch, Limits,
    VehicleControlMode, VehicleController,
};
use crate::kernel::config::{FixedWingParams, FixedWingParamsError, ResolvedKernelConfig};
use crate::state::StateEstimate;
use crate::types::{NormalizedSigned, NormalizedThrust};

use air_data::Airspeed;
use energy::EnergyInputs;
use targets::{Envelope, Targets};

/// Fixed-wing controller for one airframe description.
pub struct FixedWingController {
    params: FixedWingParams,
    /// Canonical identity over `params`; the builder compares it against
    /// the resolved configuration before a kernel exists.
    tuning_identity: u64,
}

impl FixedWingController {
    /// Construct a controller for `params`.
    ///
    /// # Errors
    ///
    /// Returns [`FixedWingParamsError`] when `params` does not describe a
    /// vehicle that the law can fly.
    pub fn new(params: FixedWingParams) -> Result<Self, FixedWingParamsError> {
        params.validate()?;
        Ok(Self {
            params,
            tuning_identity: crate::kernel::config::canonical_fixed_wing_tuning_identity(&params),
        })
    }

    /// Airframe description that this controller flies.
    #[must_use]
    pub const fn params(&self) -> &FixedWingParams {
        &self.params
    }
}

impl VehicleController for FixedWingController {
    type RuntimeState = FixedWingRuntimeState;

    // Registered in cert/algorithm_id_registry.toml as
    // "controller.fixed_wing.v2".
    const ALGORITHM_ID: u64 = 0x4354_4C46_5747_5632; // "CTLFWGV2"

    fn verify_config_binding(
        &self,
        cfg: &ResolvedKernelConfig,
    ) -> Result<(), ControllerConfigMismatch> {
        // A configuration without a fixed-wing block vouches for no
        // fixed-wing tuning: its identity is reported as zero.
        let config_identity = cfg.fixed_wing_tuning_identity().unwrap_or(0);
        if config_identity == self.tuning_identity {
            Ok(())
        } else {
            Err(ControllerConfigMismatch {
                controller_identity: self.tuning_identity,
                config_identity,
            })
        }
    }

    /// A fixed wing cannot hold a position: every other mode is flown.
    fn supports_mode(&self, mode: ControlMode) -> bool {
        mode != ControlMode::PositionHold
    }

    fn step(
        &self,
        runtime: &mut FixedWingRuntimeState,
        state: &StateEstimate,
        command: &Command,
        flags: &VehicleControlMode,
        _mode: ConfigMode,
        limits: &Limits,
    ) -> AxisCommand {
        let params = &self.params;
        let dt = runtime.dt_sec;
        let sample = runtime.air_sample.take();
        runtime
            .air
            .observe(sample, dt, params.airspeed_filter_time_constant.0);
        let air = Airspeed::resolve(&runtime.air, state, params);
        let envelope = Envelope::resolve(params, limits);

        let (demand, throttle) = match Targets::resolve(command, flags, state, params) {
            Targets::Rates { rates, throttle } => {
                runtime.energy = EnergyLoopState::default();
                (rates, throttle.clamp(0.0, 1.0))
            }
            Targets::Flight {
                lateral,
                vertical,
                airspeed,
                throttle,
            } => {
                let bank = lateral::bank_command(lateral, state, &air, params, &envelope);
                let energy = energy::step(
                    &mut runtime.energy,
                    &EnergyInputs {
                        vertical,
                        airspeed_target: airspeed,
                        throttle,
                        state,
                        air: &air,
                        params,
                        envelope: &envelope,
                        dt,
                    },
                );
                runtime.targets.bank = bank;
                runtime.targets.pitch = energy.pitch;
                (
                    attitude::rate_demand(bank, energy.pitch, state, &air, params, &envelope),
                    energy.throttle,
                )
            }
        };
        let demand = attitude::limit_rates(demand, &envelope);
        runtime.targets.rates = demand;
        runtime.targets.airspeed_source = air.source;

        let [roll, pitch, yaw] =
            attitude::rate_loop(&mut runtime.rates, demand, state, &air, params, dt);
        AxisCommand {
            roll: NormalizedSigned(roll),
            pitch: NormalizedSigned(pitch),
            yaw: NormalizedSigned(yaw),
            collective: NormalizedThrust(throttle),
        }
    }
}
