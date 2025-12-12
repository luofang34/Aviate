//! High-level telemetry interface for low-DAL tasks
//!
//! This module provides the TelemetryBackend trait for protocol-agnostic
//! telemetry transmission.
//!
//! ## DO-178C Usage Pattern
//!
//! - **High-DAL control code**: MUST NOT call implementations that perform I/O directly!
//!   - Instead: Call pure `format_*` helpers (in telemetry_mavlink.rs) and push to TelemetryQueue
//!   - Why: High-DAL code requires provable WCET, no external I/O allowed
//!
//! - **Low-DAL usage**: Use TelemetryBackend implementations to pop from queue + perform I/O
//!   - Why: Low-DAL code can fail (e.g., USB disconnected) without affecting control loop
//!
//! ## Example (Correct Pattern)
//!
//! ```ignore
//! // High-DAL control_task (provable WCET, no I/O)
//! pub fn control_task(ctx: &mut AppContext) {
//!     let state = ctx.kernel.estimate();
//!
//!     // FORMAT only - no I/O!
//!     let mut buf = [0u8; 256];
//!     if let Ok(len) = format_attitude(&state, ctx.time_ms, sys_id, comp_id, &mut ctx.seq, &mut buf) {
//!         let _ = ctx.telemetry_queue.push(&buf[..len]);  // Non-blocking O(1)
//!     }
//! }
//!
//! // Low-DAL telemetry_task (can fail, doesn't affect control)
//! pub fn telemetry_task(ctx: &mut AppContext, telemetry: &mut impl TelemetryBackend) {
//!     while ctx.telemetry_queue.pop_with(|frame| {
//!         let _ = telemetry.send_raw(frame);  // Failure OK, just increment counter
//!     }) {}
//! }
//! ```

use aviate_core::mixer::ActuatorCmd;
use aviate_core::state::StateEstimate;
use aviate_core::ChannelStatus;

use crate::errors::TelemetryResult;

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
    fn send_state(&mut self, state: &StateEstimate, time_ms: u32) -> TelemetryResult<()>;

    /// Send actuator commands (motor outputs)
    ///
    /// Maps to MAVLink ACTUATOR_OUTPUT_STATUS or similar messages.
    fn send_actuators(&mut self, cmd: &ActuatorCmd) -> TelemetryResult<()>;

    // Future: Add sensor-specific variants as needed
    // fn send_imu(&mut self, imu: &ImuData) -> TelemetryResult<()>;
    // fn send_gps(&mut self, gps: &GpsData) -> TelemetryResult<()>;
}
