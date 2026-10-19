//! Where the sidecar listens and how the board reads its world.

use std::net::{Ipv4Addr, SocketAddr};

use aviate_hal_xil::synthesis::{GeoOrigin, NoiseTier};
use aviate_hal_xil::SitlConfig;
use world_core::DecimalYear;

/// The sidecar for instance `n` listens on this port plus `n`.
pub const SIDECAR_BASE_PORT: u16 = 4_560;

/// One JSBSim board's links and world.
#[derive(Debug, Clone)]
pub struct JsbsimConfig {
    /// The sidecar's TCP endpoint.
    pub sidecar_addr: SocketAddr,
    /// MAVLink system id the board sends with.
    pub sys_id: u8,
    /// MAVLink component id the board sends with.
    pub comp_id: u8,
    /// The XIL ports the runner's GCS link binds.
    pub xil: SitlConfig,
    /// The world origin: the sidecar's spawn point, as the launcher
    /// states it through `AVIATE_GNSS_ORIGIN`.
    pub origin: GeoOrigin,
    /// The date the simulated Earth field is evaluated on.
    pub epoch: DecimalYear,
    /// Sensor noise on the synthesized samples.
    pub noise: NoiseTier,
    /// Seed of the sensor noise.
    pub noise_seed: u64,
}

impl JsbsimConfig {
    /// The configuration of SITL instance `instance`: the sidecar on
    /// localhost at [`SIDECAR_BASE_PORT`] plus the instance, MAVLink
    /// system id 1 plus the instance, and the instance's XIL ports.
    #[must_use]
    pub fn for_instance(instance: u8, origin: GeoOrigin, epoch: DecimalYear) -> Self {
        Self {
            sidecar_addr: SocketAddr::from((
                Ipv4Addr::LOCALHOST,
                SIDECAR_BASE_PORT.saturating_add(u16::from(instance)),
            )),
            sys_id: instance.saturating_add(1),
            comp_id: 1,
            xil: SitlConfig::for_instance(instance),
            origin,
            epoch,
            noise: NoiseTier::Off,
            noise_seed: u64::from(instance),
        }
    }
}
