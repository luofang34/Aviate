//! Telemetry task for GCS communication
//!
//! This module provides a protocol-agnostic telemetry system with clean DAL separation:
//!
//! - **High-DAL**: `update_state()` - trivial field copies from control loop
//! - **Low-DAL**: `tick_and_flush()` - formatting (via backend), queue management, I/O
//!
//! ## DO-178C Design
//!
//! Telemetry is classified as low-DAL because:
//! - Failure = GCS HUD stops updating (annoying, not hazardous)
//! - Aircraft can still fly and land without GCS telemetry
//! - The high-DAL control loop only performs trivial copies via `update_state()`
//!
//! ## Protocol Agnosticism
//!
//! This module does NOT know about MAVLink, CCSDS, or any specific protocol.
//! Protocol-specific formatting is handled by `TelemetryCycleFormatter` implementations
//! in aviate-link (e.g., `MavlinkCycleFormatter`).
//!
//! ## Usage
//!
//! ```ignore
//! use aviate_link::mavlink::MavlinkCycleFormatter;
//! use aviate_runtime::telemetry::{FrameTx, TelemetryTask};
//!
//! // Create formatter (protocol-specific, from aviate-link)
//! let formatter = MavlinkCycleFormatter::new(&telem_cfg, 1000)?;
//!
//! // Create task (protocol-agnostic, from aviate-runtime)
//! let task = TelemetryTask::new(udp_tx, formatter);
//!
//! // High-DAL: trivial field copy
//! task.update_state(snapshot);
//!
//! // Low-DAL: format + queue + send
//! task.tick_and_flush();
//! ```

// Re-export types from aviate-link for convenience
pub use aviate_link::{
    DefaultTelemetryQueue, TelemetryCycleFormatter, TelemetrySnapshot, TELEMETRY_MAX_FRAME,
};

/// Frame transmission trait (transport-agnostic)
///
/// Implemented by UDP, serial, or other transports.
///
/// Returns `()` on success, or `()` on failure. We use unit type for error
/// because telemetry failure is low-DAL (not critical) - we just drop frames.
#[allow(clippy::result_unit_err)]
pub trait FrameTx {
    /// Attempt to enqueue one frame; `Err(())` drops it (low-DAL, non-critical).
    fn try_send(&mut self, frame: &[u8]) -> Result<(), ()>;
}

/// Telemetry task with protocol-agnostic backend
///
/// - **High-DAL**: `update_state()` - trivial copy
/// - **Low-DAL**: `tick_and_flush()` - formatting (via backend) + queue + send
///
/// ## Type Parameters
///
/// - `Tx`: Transport implementing `FrameTx` (e.g., UDP, serial)
/// - `F`: Protocol formatter implementing `TelemetryCycleFormatter` (e.g., `MavlinkCycleFormatter`)
pub struct TelemetryTask<Tx, F>
where
    Tx: FrameTx,
    F: TelemetryCycleFormatter,
{
    tx: Tx,
    queue: DefaultTelemetryQueue,
    formatter: F,
    last_state: TelemetrySnapshot,
}

impl<Tx, F> TelemetryTask<Tx, F>
where
    Tx: FrameTx,
    F: TelemetryCycleFormatter,
{
    /// Create a new telemetry task
    ///
    /// # Parameters
    /// - `tx`: Transport implementing FrameTx
    /// - `formatter`: Protocol formatter implementing TelemetryCycleFormatter
    pub fn new(tx: Tx, formatter: F) -> Self {
        Self {
            tx,
            queue: DefaultTelemetryQueue::new(),
            formatter,
            last_state: TelemetrySnapshot::default(),
        }
    }

    /// High-DAL: trivial copy of current state into snapshot
    ///
    /// Called from control loop - just field copies, easy to audit.
    /// This is the ONLY high-DAL operation in the telemetry system.
    pub fn update_state(&mut self, snapshot: TelemetrySnapshot) {
        self.last_state = snapshot;
    }

    /// Low-DAL: formatting (via backend) + queue + send
    ///
    /// Protocol-specific formatting happens in the formatter (from aviate-link).
    /// Called at end of control loop or in lower-priority task.
    pub fn tick_and_flush(&mut self) {
        // Format messages using protocol-specific formatter
        self.formatter
            .format_cycle(&self.last_state, &mut self.queue);

        // Send all queued frames (low-DAL I/O)
        while self.queue.pop_with(|frame| {
            let _ = self.tx.try_send(frame);
        }) {}
    }

    /// Access mutable reference to transport (e.g. to update target address)
    pub fn frame_tx_mut(&mut self) -> &mut Tx {
        &mut self.tx
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use aviate_core::state::StateEstimate;
    use aviate_core::ChannelStatus;

    struct MockTx {
        sent: Vec<Vec<u8>>,
    }

    impl MockTx {
        fn new() -> Self {
            Self { sent: Vec::new() }
        }
    }

    impl FrameTx for MockTx {
        fn try_send(&mut self, frame: &[u8]) -> Result<(), ()> {
            self.sent.push(frame.to_vec());
            Ok(())
        }
    }

    /// Mock formatter that pushes a fixed frame on every cycle
    struct MockFormatter {
        frame: Vec<u8>,
    }

    impl MockFormatter {
        fn new(frame: &[u8]) -> Self {
            Self {
                frame: frame.to_vec(),
            }
        }
    }

    impl TelemetryCycleFormatter for MockFormatter {
        fn format_cycle(
            &mut self,
            _snapshot: &TelemetrySnapshot,
            queue: &mut DefaultTelemetryQueue,
        ) {
            let _ = queue.push(&self.frame);
        }
    }

    #[test]
    fn test_telemetry_task_basic() {
        let tx = MockTx::new();
        let formatter = MockFormatter::new(b"test_frame");
        let mut task = TelemetryTask::new(tx, formatter);

        // Update state (high-DAL)
        let snapshot = TelemetrySnapshot {
            time_ms: 1000,
            iteration: 1,
            status: ChannelStatus::default(),
            state: StateEstimate::default(),
        };
        task.update_state(snapshot);

        // Tick and flush (low-DAL)
        task.tick_and_flush();

        // Should have sent one frame
        assert_eq!(task.tx.sent.len(), 1);
        assert_eq!(task.tx.sent[0], b"test_frame");
    }

    #[test]
    fn test_telemetry_task_multiple_cycles() {
        let tx = MockTx::new();
        let formatter = MockFormatter::new(b"frame");
        let mut task = TelemetryTask::new(tx, formatter);

        // Multiple cycles
        for i in 0..3 {
            let snapshot = TelemetrySnapshot {
                time_ms: i * 100,
                iteration: i,
                status: ChannelStatus::default(),
                state: StateEstimate::default(),
            };
            task.update_state(snapshot);
            task.tick_and_flush();
        }

        // Should have sent 3 frames
        assert_eq!(task.tx.sent.len(), 3);
    }
}
