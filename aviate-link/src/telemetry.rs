//! High-level telemetry interface for low-DAL tasks
//!
//! This module provides two complementary telemetry abstractions:
//!
//! ## 1. `TelemetryBackend` trait (Low-DAL I/O sender)
//!
//! For direct message transmission. Implementations perform I/O.
//!
//! ## 2. `TelemetryCycleFormatter` trait (Cycle-based formatting)
//!
//! For use with `TelemetryTask` in aviate-runtime. This trait allows
//! protocol-agnostic telemetry formatting in the control loop:
//!
//! - **High-DAL control code**: Calls `update_state()` (trivial field copies)
//! - **Low-DAL telemetry code**: Calls `tick_and_flush()` which uses
//!   `TelemetryCycleFormatter::format_cycle()` to format messages and push to queue
//!
//! ## DO-178C Usage Pattern
//!
//! ```ignore
//! // High-DAL control_task (provable WCET, no I/O)
//! pub fn control_task(ctx: &mut AppContext) {
//!     let state = ctx.kernel.estimate();
//!
//!     // Trivial field copy - HIGH-DAL safe
//!     let snapshot = TelemetrySnapshot { ... };
//!     ctx.telemetry_task.update_state(snapshot);
//! }
//!
//! // Low-DAL telemetry_task (can fail, doesn't affect control)
//! pub fn telemetry_task(ctx: &mut AppContext) {
//!     ctx.telemetry_task.tick_and_flush();  // Formats + sends
//! }
//! ```

use aviate_core::mixer::ActuatorCmd;
use aviate_core::state::StateEstimate;
use aviate_core::ChannelStatus;

use crate::errors::TelemetryResult;
use crate::queue::DefaultTelemetryQueue;

// ============================================================================
// TelemetrySnapshot (protocol-agnostic state for formatting)
// ============================================================================

/// State snapshot for telemetry (POD, trivially copyable)
///
/// This structure is copied from the high-DAL control loop to the low-DAL
/// telemetry task. The copy is the only high-DAL operation - all formatting
/// and I/O happens in low-DAL `tick_and_flush()`.
///
/// Defined here in aviate-link so both aviate-runtime and protocol implementations
/// can use it without circular dependencies.
#[derive(Clone, Default)]
pub struct TelemetrySnapshot {
    /// System time in milliseconds since boot
    pub time_ms: u32,
    /// Control loop iteration counter (for rate dividers)
    pub iteration: u32,
    /// Channel status (for HEARTBEAT)
    pub status: ChannelStatus,
    /// State estimate (for ATTITUDE, POSITION)
    pub state: StateEstimate,
}

// ============================================================================
// TelemetryCycleFormatter trait (protocol-agnostic cycle formatting)
// ============================================================================

/// Protocol-agnostic telemetry cycle formatter
///
/// Implementations handle protocol-specific formatting (MAVLink, CCSDS, etc.)
/// and push formatted frames to the telemetry queue.
///
/// This trait is used by `TelemetryTask` in aviate-runtime for protocol-agnostic
/// telemetry output. The runtime code never imports MAVLink types directly.
///
/// ## Implementing
///
/// Protocol implementations (e.g., `MavlinkCycleFormatter`) should:
/// 1. Check rate dividers based on `snapshot.iteration`
/// 2. Format messages using protocol-specific helpers
/// 3. Push formatted frames to the queue
///
/// ## Example Implementation
///
/// ```ignore
/// impl TelemetryCycleFormatter for MavlinkCycleFormatter {
///     fn format_cycle(&mut self, snapshot: &TelemetrySnapshot, queue: &mut DefaultTelemetryQueue) {
///         if snapshot.iteration % self.heartbeat_div == 0 {
///             if let Ok(len) = format_heartbeat(&snapshot.status, ..., &mut buf) {
///                 let _ = queue.push(&buf[..len]);
///             }
///         }
///         // ... other messages
///     }
/// }
/// ```
pub trait TelemetryCycleFormatter {
    /// Format telemetry messages for current cycle and push to queue
    ///
    /// Called by `TelemetryTask::tick_and_flush()` in low-DAL context.
    /// Implementations should check rate dividers and format appropriate messages.
    fn format_cycle(&mut self, snapshot: &TelemetrySnapshot, queue: &mut DefaultTelemetryQueue);
}

// ============================================================================
// TelemetryBackend trait (Low-DAL I/O sender)
// ============================================================================

/// High-level telemetry interface for low-DAL tasks.
///
/// **CRITICAL**: This trait is for LOW-DAL code only!
/// High-DAL control code MUST use pure format helpers + TelemetryQueue instead.
///
/// Implementations perform I/O and may fail (e.g., USB disconnected, buffer full).
pub trait TelemetryBackend {
    /// Send channel status (heartbeat)
    ///
    /// Maps to MAVLink HEARTBEAT or similar messages in other protocols.
    fn send_status(&mut self, status: &ChannelStatus) -> TelemetryResult<()>;

    /// Send state estimate (attitude, rates)
    ///
    /// Maps to MAVLink ATTITUDE_QUATERNION or similar messages.
    ///
    /// Emits a bare numeric frame with no estimator-status frame. The
    /// same-timestamp status pairing contract in
    /// `docs/ESTIMATOR_TELEMETRY.md` holds only on the cycle-formatter
    /// path; a consumer applying its fail-closed rules discards frames
    /// sent this way. Prefer the cycle formatter for GCS links.
    fn send_state(&mut self, state: &StateEstimate, time_ms: u32) -> TelemetryResult<()>;

    /// Send actuator commands (motor outputs)
    ///
    /// Maps to MAVLink ACTUATOR_OUTPUT_STATUS or similar messages.
    fn send_actuators(&mut self, cmd: &ActuatorCmd) -> TelemetryResult<()>;

    // Future: Add sensor-specific variants as needed
    // fn send_imu(&mut self, imu: &ImuData) -> TelemetryResult<()>;
    // fn send_gps(&mut self, gps: &GpsData) -> TelemetryResult<()>;
}
