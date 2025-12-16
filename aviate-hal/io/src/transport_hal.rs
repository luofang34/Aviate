//! Transport abstraction for Aviate flight controller
//!
//! Provides a high-level transport interface for the runner, abstracting
//! over different physical transports (USB CDC, UART, CAN, UDP).
//!
//! ## Design Principles
//!
//! 1. **Non-blocking**: All operations return immediately
//! 2. **Infallible**: Errors reported via status counters, not Result
//! 3. **Protocol-agnostic**: Works with MAVLink, custom protocols, etc.
//!
//! ## DO-178C Compliance
//!
//! - `try_recv_command()` MUST return in bounded time (no blocking)
//! - `try_send_telemetry()` drops frames on buffer full (best-effort)
//! - `poll()` MUST be fast and bounded (services DMA, USB device, etc.)
//! - Failsafe uses command timeout (`link_ok`), NOT `connected` flag

/// Transport status for health monitoring
///
/// These counters are informational - the runner uses command timeout
/// (not `connected`) to determine failsafe state.
///
/// ## `connected` Semantics by Transport Type
///
/// | Transport | `connected = true` means |
/// |-----------|--------------------------|
/// | USB CDC   | Enumerated and configured |
/// | UART      | Always true (no handshake) |
/// | CAN       | Bus-off = false, seen recent traffic |
/// | UDP       | Socket bound successfully |
///
/// **NOTE**: Do NOT use `connected` for failsafe decisions!
/// Use command timeout (`link_ok` in `RunnerHealth`) instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TransportStatus {
    /// RX error count (CRC errors, framing errors, etc.)
    pub rx_errors: u32,
    /// TX error count (buffer full, send failures, etc.)
    pub tx_errors: u32,
    /// Whether transport link is physically connected (informational only)
    pub connected: bool,
}

/// System state for transport to include in heartbeat
///
/// This is a simplified version of MAVLink MAV_STATE.
/// Transport implementations map this to protocol-specific values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SystemState {
    /// System uninitialized
    #[default]
    Uninit = 0,
    /// System booting
    Boot = 1,
    /// Calibrating sensors
    Calibrating = 2,
    /// Standby (disarmed, ready to arm)
    Standby = 3,
    /// Active (armed, motors running)
    Active = 4,
    /// Critical (failsafe active)
    Critical = 5,
    /// Emergency (severe failure)
    Emergency = 6,
}

/// Transport trait for flight controller communication
///
/// High-level abstraction over physical transports (USB, UART, CAN, UDP).
/// All methods are non-blocking and infallible (errors via status counters).
///
/// ## Contract
///
/// 1. **`try_recv_command()`**: Returns immediately, `None` if no command
/// 2. **`try_send_telemetry()`**: Returns immediately, `false` if buffer full
/// 3. **`poll()`**: Fast and bounded, services hardware state machines
///
/// ## Watchdog Note
///
/// Watchdog kicking is handled separately via `WatchdogHal` trait.
/// This keeps transport concerns separate from system liveness concerns.
///
/// ## Generic Parameter `Cmd`
///
/// The command type is generic to allow different implementations:
/// - SITL: Uses `aviate_link::Command` directly
/// - Hardware: May use a simplified command type
pub trait TransportHal<Cmd> {
    /// Attempt to receive a command (non-blocking)
    ///
    /// # Returns
    ///
    /// - `Some(cmd)` if a complete command is available
    /// - `None` if no command available (NOT an error!)
    ///
    /// # Timing Guarantee
    ///
    /// WCET: O(1) for buffer check, O(frame_len) if parsing needed.
    /// Typically < 10 microseconds.
    fn try_recv_command(&mut self) -> Option<Cmd>;

    /// Attempt to send telemetry frame (non-blocking, best-effort)
    ///
    /// # Arguments
    ///
    /// * `frame` - Complete protocol frame (e.g., MAVLink message)
    ///
    /// # Returns
    ///
    /// - `true` if frame was queued successfully
    /// - `false` if buffer full (frame dropped, increment `tx_errors`)
    ///
    /// # Timing Guarantee
    ///
    /// WCET: O(frame.len()) for memcpy. Typically < 1 microsecond.
    fn try_send_telemetry(&mut self, frame: &[u8]) -> bool;

    /// Set system state (included in heartbeat/status messages)
    ///
    /// Transport implementations emit periodic heartbeats containing this state.
    fn set_system_state(&mut self, state: SystemState);

    /// Set armed state (included in heartbeat/status messages)
    ///
    /// Transport implementations include this in system status messages.
    fn set_armed(&mut self, armed: bool);

    /// Poll transport state machine (service DMA, USB, interrupts, etc.)
    ///
    /// # Timing Guarantee
    ///
    /// MUST be fast and bounded. Typical operations:
    /// - USB: Service USB device state machine, process setup packets
    /// - UART: Drain RX ring buffer, refill TX ring
    /// - CAN: Process RX FIFO, check bus-off status
    ///
    /// This is called frequently (before each tick check) so it MUST NOT block.
    fn poll(&mut self);

    /// Get transport status (error counters, connected state)
    ///
    /// # Note
    ///
    /// Do NOT use `status().connected` for failsafe decisions!
    /// Use command timeout instead.
    fn status(&self) -> TransportStatus;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transport_status_default() {
        let status = TransportStatus::default();
        assert_eq!(status.rx_errors, 0);
        assert_eq!(status.tx_errors, 0);
        assert!(!status.connected);
    }

    #[test]
    fn test_system_state_default() {
        let state = SystemState::default();
        assert_eq!(state, SystemState::Uninit);
        assert_eq!(state as u8, 0);
    }
}
