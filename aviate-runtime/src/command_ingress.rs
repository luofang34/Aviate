//! Shared command-ingress state machine (#133): one freshness
//! implementation for hardware and SITL runners.
//!
//! Two command classes with different lifecycles:
//!
//! - **Setpoint** commands (flight control) are *retained*: the
//!   cascade consumes the latest setpoint every tick. The retained
//!   value carries its own receive timestamp, and ONLY a new
//!   setpoint-class receive refreshes it.
//! - **Discrete** commands (arm/disarm) are *one-shot events*:
//!   consumed exactly once on the tick they arrive, never retained,
//!   and — the #133 defect — never allowed to refresh the setpoint
//!   age. A redundant `Arm` on a link that has stopped carrying
//!   setpoints must not un-stale the retained setpoint and release
//!   the kernel's CommandLoss terminal.

use aviate_hal_io::SystemCommand;

/// Command lifecycle class. See the module docs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandClass {
    /// Retained flight setpoint; its receive time is tracked.
    Setpoint,
    /// One-shot event; consumed once, never stamps setpoint age.
    Discrete,
}

/// Classification hook the ingress uses to route a command. Implement
/// for the runner's command type; the generic runner stays reusable
/// in tests with simple types.
pub trait ClassifyCommand {
    /// Which lifecycle this command value follows.
    fn class(&self) -> CommandClass;
}

impl ClassifyCommand for SystemCommand {
    fn class(&self) -> CommandClass {
        match self {
            SystemCommand::FlightControl(_) => CommandClass::Setpoint,
            SystemCommand::Arm | SystemCommand::Disarm => CommandClass::Discrete,
        }
    }
}

/// Per-runner ingress state: the retained setpoint and its OWN
/// receive timestamp. Both runners (hardware `FlightRunner`, SITL
/// `SitlRunner`) route received commands through this so freshness
/// semantics cannot drift between environments.
#[derive(Debug, Clone)]
pub struct CommandIngress<C> {
    setpoint: Option<C>,
    setpoint_rx_us: Option<u64>,
}

impl<C> Default for CommandIngress<C> {
    fn default() -> Self {
        Self {
            setpoint: None,
            setpoint_rx_us: None,
        }
    }
}

impl<C: ClassifyCommand> CommandIngress<C> {
    /// Route one received command. Setpoint-class values are retained
    /// and stamped; the command is returned either way so the caller
    /// can act on discrete events exactly once.
    pub fn receive(&mut self, cmd: C, now_us: u64) -> CommandClass {
        match cmd.class() {
            CommandClass::Setpoint => {
                self.setpoint = Some(cmd);
                self.setpoint_rx_us = Some(now_us);
                CommandClass::Setpoint
            }
            CommandClass::Discrete => CommandClass::Discrete,
        }
    }

    /// The retained setpoint, if any has ever arrived.
    pub fn setpoint(&self) -> Option<&C> {
        self.setpoint.as_ref()
    }

    /// Age of the last setpoint-class receive \[ms\]; `u32::MAX`
    /// before the first. Discrete receives never refresh this — that
    /// is the #133 contract.
    pub fn setpoint_age_ms(&self, now_us: u64) -> u32 {
        match self.setpoint_rx_us {
            Some(rx) => u32::try_from(now_us.wrapping_sub(rx) / 1_000).unwrap_or(u32::MAX),
            None => u32::MAX,
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[derive(Clone)]
    enum Fake {
        Set(u32),
        Event,
    }
    impl ClassifyCommand for Fake {
        fn class(&self) -> CommandClass {
            match self {
                Fake::Set(_) => CommandClass::Setpoint,
                Fake::Event => CommandClass::Discrete,
            }
        }
    }

    #[test]
    fn discrete_never_refreshes_setpoint_age() {
        let mut ing = CommandIngress::default();
        ing.receive(Fake::Set(1), 0);
        assert_eq!(ing.setpoint_age_ms(500_000), 500);
        // A redundant discrete event a second later must not touch it.
        ing.receive(Fake::Event, 1_000_000);
        assert_eq!(ing.setpoint_age_ms(1_200_000), 1_200);
        assert!(matches!(ing.setpoint(), Some(Fake::Set(1))));
    }

    #[test]
    fn setpoint_receive_refreshes_and_retains() {
        let mut ing = CommandIngress::default();
        assert_eq!(ing.setpoint_age_ms(9_000_000), u32::MAX);
        ing.receive(Fake::Set(7), 1_000_000);
        assert_eq!(ing.setpoint_age_ms(1_000_000), 0);
        ing.receive(Fake::Set(8), 2_000_000);
        assert_eq!(ing.setpoint_age_ms(2_100_000), 100);
        assert!(matches!(ing.setpoint(), Some(Fake::Set(8))));
    }
}
