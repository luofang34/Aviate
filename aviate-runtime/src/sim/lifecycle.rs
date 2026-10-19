//! Reset support for one simulator generation.

use aviate_core::hal::ActuatorHal;
use aviate_hal_io::SystemCommand;

use super::{default_command, SitlRunner};
use crate::command_ingress::CommandIngress;
use crate::sensor_cache::SensorCache;

impl<C, M> SitlRunner<C, M>
where
    C: aviate_core::control::VehicleController,
    M: aviate_core::mixer::Mixer,
{
    /// Clear runtime data that must not cross a simulator reset.
    pub fn reset_for_simulator_generation(&mut self) {
        self.transport.clear_generation_state();
        self.reset_runner_for_generation();
    }

    /// Adopt a new world epoch that an incoming sample stated.
    ///
    /// The transport has already dropped what it buffered from the old
    /// world and holds the new world's first sample, so only the runner
    /// and the kernel are reset here. The kernel's ground reset is what
    /// clears faults latched in the old world: a simulator respawning
    /// the vehicle is that vehicle's next power cycle.
    pub(crate) fn adopt_generation_change(&mut self) {
        if self.transport.take_generation_change() {
            log::info!("simulator world reset; ground-resetting the flight controller");
            self.reset_runner_for_generation();
        }
    }

    fn reset_runner_for_generation(&mut self) {
        if self.is_armed() {
            self.kernel.terminate();
        }
        self.kernel.ground_reset();
        self.kernel.state.checks.pre_arm.update_throttle(true);
        self.board_hal.disarm();
        self.ingress = CommandIngress::<SystemCommand>::default();
        self.last_effective_command = default_command();
        self.last_controller_observation = Default::default();
        self.last_command_provenance = None;
        self.last_imu_time = None;
        self.sensor_cache = SensorCache::new();
        self.ekf_initialized = false;
        self.iteration = 0;
        self.geomagnetic.forget_model();
        self.geodetic.last_fix = None;

        let (imu, baro, mag, gnss) = self.board_hal.sensors_mut();
        imu.clear();
        imu.clear_faults();
        baro.clear();
        baro.clear_faults();
        mag.clear();
        mag.clear_faults();
        gnss.clear();
        gnss.clear_faults();
    }
}

#[cfg(test)]
mod tests;
