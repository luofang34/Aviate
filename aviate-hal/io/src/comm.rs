//! Command and communication HAL traits.
//!
//! Lives outside `aviate-core` because the kernel itself does not consume
//! GCS/RC commands or telemetry framing — those are link-layer concerns
//! that belong to the platform / runtime crate. Spec §2.2: the kernel's
//! external surface is sensor input, actuator output, and system services.

use aviate_core::control::Command;

/// System command from GCS/RC.
#[derive(Clone, Debug)]
pub enum SystemCommand {
    /// Flight-control setpoint (position/velocity/attitude/rate per `Command`).
    FlightControl(Command),
    /// Request to arm actuators.
    Arm,
    /// Request to disarm actuators.
    Disarm,
}

/// Command input interface (GCS/RC).
pub trait CommandHal {
    /// Receive the latest command from GCS/RC, or `None` if no new command.
    fn recv_command(&mut self) -> Option<SystemCommand>;
}

/// Bidirectional byte-stream interface for telemetry and command framing.
pub trait CommHal {
    /// Send `data`. Returns the number of bytes accepted by the link.
    fn send(&mut self, data: &[u8]) -> Result<usize, CommError>;

    /// Non-blocking receive into `buf`. Returns the number of bytes written.
    fn recv(&mut self, buf: &mut [u8]) -> Result<usize, CommError>;

    /// Bytes available for `recv()` without blocking.
    fn available(&self) -> usize;
}

/// Errors surfaced by `CommHal` operations.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommError {
    /// Operation would block (no data / no space).
    WouldBlock,
    /// Internal buffer is full; producer must retry.
    BufferFull,
    /// Underlying transport is disconnected.
    Disconnected,
    /// Operation timed out.
    Timeout,
    /// Received data failed framing or sanity checks.
    InvalidData,
}

#[cfg(test)]
mod tests {
    // TST-MOD-001: structural witness for LLR-MOD-101.
    use crate::{CommError, CommHal, CommandHal, SystemCommand};
    use aviate_core::control::{Command, CommandSource, ControlMode, Setpoint};

    struct StubCmd {
        next: Option<SystemCommand>,
    }

    impl CommandHal for StubCmd {
        fn recv_command(&mut self) -> Option<SystemCommand> {
            self.next.take()
        }
    }

    struct StubComm;

    impl CommHal for StubComm {
        fn send(&mut self, data: &[u8]) -> Result<usize, CommError> {
            Ok(data.len())
        }
        fn recv(&mut self, _buf: &mut [u8]) -> Result<usize, CommError> {
            Err(CommError::WouldBlock)
        }
        fn available(&self) -> usize {
            0
        }
    }

    fn placeholder_command() -> Command {
        Command {
            source: CommandSource::Pilot,
            mode: ControlMode::Rate,
            setpoint: Setpoint::default(),
            config_mode_request: None,
            sensor_overrides: None,
            sequence: 0,
        }
    }

    #[test]
    fn comm_surface_lives_in_aviate_hal_io() {
        // Variants are constructible from the crate root re-export.
        let _ = SystemCommand::Arm;
        let _ = SystemCommand::Disarm;
        let _ = SystemCommand::FlightControl(placeholder_command());

        // CommError is exhaustively matchable.
        for err in [
            CommError::WouldBlock,
            CommError::BufferFull,
            CommError::Disconnected,
            CommError::Timeout,
            CommError::InvalidData,
        ] {
            match err {
                CommError::WouldBlock
                | CommError::BufferFull
                | CommError::Disconnected
                | CommError::Timeout
                | CommError::InvalidData => {}
            }
        }

        // CommandHal / CommHal are implementable on user types.
        let mut cmd = StubCmd {
            next: Some(SystemCommand::Arm),
        };
        assert!(matches!(cmd.recv_command(), Some(SystemCommand::Arm)));
        assert!(cmd.recv_command().is_none());

        let mut comm = StubComm;
        assert_eq!(comm.send(&[1, 2, 3]), Ok(3));
        assert_eq!(comm.recv(&mut [0u8; 4]), Err(CommError::WouldBlock));
        assert_eq!(comm.available(), 0);
    }
}
