//! Exact raw MAVLink command provenance for simulator ingress.

use std::io;
use std::net::SocketAddr;

use sha2::{Digest as _, Sha256};

/// A MAVLink setpoint family accepted by simulator command ingress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MavlinkCommandFamily {
    /// MAVLink `SET_ATTITUDE_TARGET`.
    AttitudeTarget,
    /// MAVLink `SET_POSITION_TARGET_LOCAL_NED`.
    PositionTargetLocalNed,
}

/// Exact producer-side identity of one received MAVLink setpoint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MavlinkCommandProvenance {
    /// UDP source endpoint observed by Aviate.
    pub source_endpoint: SocketAddr,
    /// Nonzero source epoch for this process and sender incarnation.
    pub source_epoch: u64,
    /// MAVLink header system identifier.
    pub mavlink_system_id: u8,
    /// MAVLink header component identifier.
    pub mavlink_component_id: u8,
    /// Full MAVLink header sequence field.
    pub mavlink_frame_sequence: u8,
    /// MAVLink setpoint boot time.
    pub time_boot_ms: u32,
    /// MAVLink setpoint family.
    pub command_family: MavlinkCommandFamily,
    /// SHA-256 digest of the exact received frame.
    pub frame_digest: [u8; 32],
}

#[derive(Debug)]
pub(crate) struct SourceEpochTracker {
    epoch: u64,
    identity: Option<(SocketAddr, u8, u8)>,
    last_time_boot_ms: Option<u32>,
}

impl SourceEpochTracker {
    pub(crate) fn new() -> io::Result<Self> {
        let mut bytes = [0_u8; 8];
        getrandom::fill(&mut bytes).map_err(io::Error::other)?;
        let epoch = u64::from_le_bytes(bytes);
        if epoch == 0 {
            return Err(io::Error::other("the random MAVLink source epoch is zero"));
        }
        Ok(Self {
            epoch,
            identity: None,
            last_time_boot_ms: None,
        })
    }

    pub(crate) fn observe(
        &mut self,
        source_endpoint: SocketAddr,
        system_id: u8,
        component_id: u8,
        time_boot_ms: u32,
    ) -> u64 {
        let identity = (source_endpoint, system_id, component_id);
        let changed = self.identity.is_some_and(|current| current != identity)
            || self
                .last_time_boot_ms
                .is_some_and(|previous| time_boot_ms < previous);
        if changed {
            self.epoch = next_nonzero(self.epoch);
        }
        self.identity = Some(identity);
        self.last_time_boot_ms = Some(time_boot_ms);
        self.epoch
    }
}

impl MavlinkCommandProvenance {
    pub(crate) fn new(
        tracker: &mut SourceEpochTracker,
        source_endpoint: SocketAddr,
        header: aviate_link::mavlink::protocol::MavHeader,
        time_boot_ms: u32,
        command_family: MavlinkCommandFamily,
        frame: &[u8],
    ) -> Self {
        Self {
            source_endpoint,
            source_epoch: tracker.observe(
                source_endpoint,
                header.sysid,
                header.compid,
                time_boot_ms,
            ),
            mavlink_system_id: header.sysid,
            mavlink_component_id: header.compid,
            mavlink_frame_sequence: header.seq,
            time_boot_ms,
            command_family,
            frame_digest: Sha256::digest(frame).into(),
        }
    }
}

const fn next_nonzero(value: u64) -> u64 {
    let next = value.wrapping_add(1);
    if next == 0 {
        1
    } else {
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tracker(epoch: u64) -> SourceEpochTracker {
        SourceEpochTracker {
            epoch,
            identity: None,
            last_time_boot_ms: None,
        }
    }

    #[test]
    fn sequence_wrap_keeps_epoch_but_sender_and_boot_restart_advance_it() {
        let source = SocketAddr::from(([127, 0, 0, 1], 30_000));
        let mut tracker = tracker(10);
        assert_eq!(tracker.observe(source, 255, 190, 5_000), 10);
        assert_eq!(tracker.observe(source, 255, 190, 5_001), 10);
        assert_eq!(tracker.observe(source, 255, 190, 1), 11);
        assert_eq!(tracker.observe(source, 255, 190, 2), 11);
        let competitor = SocketAddr::from(([127, 0, 0, 1], 30_001));
        assert_eq!(tracker.observe(competitor, 255, 190, 2), 12);
    }

    #[test]
    fn epoch_increment_never_emits_zero() {
        assert_eq!(next_nonzero(u64::MAX), 1);
    }

    #[test]
    fn process_restart_seed_changes_the_source_epoch() {
        let source = SocketAddr::from(([127, 0, 0, 1], 30_000));
        let mut first_process = tracker(10);
        let mut second_process = tracker(20);
        assert_ne!(
            first_process.observe(source, 255, 190, 1),
            second_process.observe(source, 255, 190, 1)
        );
    }
}
