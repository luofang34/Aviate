//! MAVLink Telemetry (Outbound: App → Ground)
//!
//! This module implements telemetry transmission using MAVLink protocol.
//!
//! ## DO-178C Data Flow Direction
//!
//! **Outbound ONLY** - This module sends telemetry to ground station.
//!
//! - ✅ Uses `FrameTx` for transmission
//! - ❌ MUST NOT use `FrameRx` (inbound is in command.rs)
//! - ❌ MUST NOT contain security logic (belongs in aviate-security)
//!
//! ## Criticality Level
//!
//! - **DAL D/E** (informational telemetry, no flight safety impact)
//! - Failures are acceptable (e.g., USB disconnected)
//! - Does not affect control loop timing
//!
//! ## Module Contents
//!
//! 1. **Pure format helpers** (safe for high-DAL control code):
//!    - `format_heartbeat()` - Format status → MAVLink HEARTBEAT
//!    - `format_attitude()` - Format state → MAVLink ATTITUDE_QUATERNION
//!    - No I/O, bounded runtime, can be called from control loop
//!
//! 2. **`MavlinkTelemetry<T>`** (low-DAL I/O sender):
//!    - Implements `TelemetryBackend` trait
//!    - Performs I/O via `FrameTx`
//!    - MUST NOT be called from high-DAL control code
//!
//! ## Audit Checklist
//!
//! When auditing this file, verify:
//! - ✅ No imports of `FrameRx` (only `FrameTx`)
//! - ✅ No imports from `aviate-security`
//! - ✅ No command parsing or reception logic
//! - ✅ Format helpers are pure (no I/O, no side effects)

use aviate_config::TelemetryConfig;
use aviate_core::mixer::ActuatorCmd;
use aviate_core::state::StateEstimate;
use aviate_core::ChannelStatus;
use aviate_hal_io::transport::FrameTx;

use super::protocol::{
    serialize_mavlink, AttitudeQuaternion, Heartbeat, LocalPositionNed, MavAutopilot, MavMessage,
    MavModeFlag, MavState, MavType,
};

use crate::errors::{TelemetryError, TelemetryResult};
use crate::queue::{DefaultTelemetryQueue, TELEMETRY_MAX_FRAME};
use crate::telemetry::{TelemetryBackend, TelemetryCycleFormatter, TelemetrySnapshot};

/// Pure helper - format heartbeat message (safe for high-DAL)
///
/// ## DO-178C Contract
///
/// - Non-blocking: YES (pure computation, no I/O)
/// - Time complexity: O(1), bounded by MAVLink serialization (~100 CPU cycles)
/// - WCET (engineering target): ~0.2 μs @ 480 MHz
/// - Memory: Uses provided buffer, no heap allocation
///
/// ## Parameters
///
/// - `status`: Channel status to encode
/// - `sys_id`: MAVLink system ID (1-255)
/// - `comp_id`: MAVLink component ID (1-255)
/// - `seq`: Sequence counter (auto-incremented)
/// - `buf`: Output buffer (must be at least 32 bytes)
///
/// ## Returns
///
/// - `Ok(len)`: Number of bytes written to `buf`
/// - `Err(TelemetryError::Protocol)`: Serialization failed
pub fn format_heartbeat(
    _status: &ChannelStatus,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let heartbeat = Heartbeat {
        mav_type: MavType::Quadrotor as u8,
        autopilot: MavAutopilot::Generic as u8,
        base_mode: MavModeFlag::SAFETY_ARMED.0, // TODO: derive from status
        custom_mode: 0,                         // Future: map from control mode
        system_status: MavState::Active as u8,
        mavlink_version: 3,
    };

    let msg = MavMessage::Heartbeat(heartbeat);
    let len =
        serialize_mavlink(&msg, *seq, sys_id, comp_id, buf).ok_or(TelemetryError::Protocol)?;

    *seq = seq.wrapping_add(1);
    Ok(len)
}

/// Pure helper - format attitude quaternion message (safe for high-DAL)
///
/// ## DO-178C Contract
///
/// - Non-blocking: YES (pure computation, no I/O)
/// - Time complexity: O(1), bounded by MAVLink serialization (~200 CPU cycles)
/// - WCET (engineering target): ~0.4 μs @ 480 MHz
/// - Memory: Uses provided buffer, no heap allocation
///
/// ## Parameters
///
/// - `state`: State estimate (attitude quaternion + body rates)
/// - `time_ms`: System time in milliseconds since boot
/// - `sys_id`: MAVLink system ID (1-255)
/// - `comp_id`: MAVLink component ID (1-255)
/// - `seq`: Sequence counter (auto-incremented)
/// - `buf`: Output buffer (must be at least 64 bytes)
///
/// ## Returns
///
/// - `Ok(len)`: Number of bytes written to `buf`
/// - `Err(TelemetryError::Protocol)`: Serialization failed
pub fn format_attitude(
    state: &StateEstimate,
    time_ms: u32,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let attitude = AttitudeQuaternion {
        time_boot_ms: time_ms,
        q1: state.attitude.w,
        q2: state.attitude.x,
        q3: state.attitude.y,
        q4: state.attitude.z,
        rollspeed: state.angular_velocity[0].0,
        pitchspeed: state.angular_velocity[1].0,
        yawspeed: state.angular_velocity[2].0,
        repr_offset_q: [0.0; 4], // Representation offset (not used)
    };

    let msg = MavMessage::AttitudeQuaternion(attitude);
    let len =
        serialize_mavlink(&msg, *seq, sys_id, comp_id, buf).ok_or(TelemetryError::Protocol)?;

    *seq = seq.wrapping_add(1);
    Ok(len)
}

/// Pure helper - format local position NED message (safe for high-DAL)
///
/// ## DO-178C Contract
///
/// - Non-blocking: YES (pure computation, no I/O)
/// - Time complexity: O(1), bounded by MAVLink serialization (~150 CPU cycles)
/// - WCET (engineering target): ~0.3 μs @ 480 MHz
/// - Memory: Uses provided buffer, no heap allocation
///
/// ## Parameters
///
/// - `state`: State estimate (position + velocity in NED frame)
/// - `time_ms`: System time in milliseconds since boot
/// - `sys_id`: MAVLink system ID (1-255)
/// - `comp_id`: MAVLink component ID (1-255)
/// - `seq`: Sequence counter (auto-incremented)
/// - `buf`: Output buffer (must be at least 48 bytes)
///
/// ## Returns
///
/// - `Ok(len)`: Number of bytes written to `buf`
/// - `Err(TelemetryError::Protocol)`: Serialization failed
pub fn format_local_position(
    state: &StateEstimate,
    time_ms: u32,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> TelemetryResult<usize> {
    let position = LocalPositionNed {
        time_boot_ms: time_ms,
        x: state.position_ned[0].0,
        y: state.position_ned[1].0,
        z: state.position_ned[2].0,
        vx: state.velocity_ned[0].0,
        vy: state.velocity_ned[1].0,
        vz: state.velocity_ned[2].0,
    };

    let msg = MavMessage::LocalPositionNed(position);
    let len =
        serialize_mavlink(&msg, *seq, sys_id, comp_id, buf).ok_or(TelemetryError::Protocol)?;

    *seq = seq.wrapping_add(1);
    Ok(len)
}

/// Pure helper - format actuator output message (safe for high-DAL)
///
/// ## DO-178C Contract
///
/// - Non-blocking: YES (pure computation, no I/O)
/// - Time complexity: O(1), bounded by MAVLink serialization (~150 CPU cycles)
/// - WCET (engineering target): ~0.3 μs @ 480 MHz
/// - Memory: Uses provided buffer, no heap allocation
///
/// ## Parameters
///
/// - `cmd`: Actuator commands (motor outputs)
/// - `time_ms`: System time in milliseconds since boot
/// - `sys_id`: MAVLink system ID (1-255)
/// - `comp_id`: MAVLink component ID (1-255)
/// - `seq`: Sequence counter (auto-incremented)
/// - `buf`: Output buffer (must be at least 64 bytes)
///
/// ## Returns
///
/// - `Ok(len)`: Number of bytes written to `buf`
/// - `Err(TelemetryError::Protocol)`: Serialization failed
pub fn format_actuators(
    _cmd: &ActuatorCmd,
    _time_ms: u32,
    _sys_id: u8,
    _comp_id: u8,
    _seq: &mut u8,
    _buf: &mut [u8],
) -> TelemetryResult<usize> {
    // TODO: Implement ACTUATOR_OUTPUT_STATUS message
    // For now, return zero length (no message sent)
    Ok(0)
}

/// MAVLink telemetry sender (LOW-DAL only!)
///
/// This struct performs I/O and MUST NOT be used in high-DAL control code.
///
/// ## Type Parameters
///
/// - `T`: Transport implementing `FrameTx` (e.g., USB CDC, UART, CAN)
///
/// ## Usage
///
/// ```ignore
/// // Create sender (in low-DAL telemetry task)
/// let mut telemetry = MavlinkTelemetry::new(usb_tx, 1, 1);
///
/// // Send telemetry (low-DAL only!)
/// telemetry.send_status(&status)?;
/// telemetry.send_state(&state, time_ms)?;
/// ```
pub struct MavlinkTelemetry<T: FrameTx> {
    /// Transport for sending frames
    tx: T,
    /// MAVLink system ID (1-255)
    sys_id: u8,
    /// MAVLink component ID (1-255)
    comp_id: u8,
    /// Sequence counter (wraps at 256)
    seq: u8,
}

impl<T: FrameTx> MavlinkTelemetry<T> {
    /// Create new MAVLink telemetry sender
    ///
    /// ## Parameters
    ///
    /// - `tx`: Transport implementing FrameTx
    /// - `sys_id`: MAVLink system ID (1-255, typically 1)
    /// - `comp_id`: MAVLink component ID (1-255, typically 1)
    pub fn new(tx: T, sys_id: u8, comp_id: u8) -> Self {
        Self {
            tx,
            sys_id,
            comp_id,
            seq: 0,
        }
    }

    /// Get mutable reference to transport (for configuration)
    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.tx
    }
}

impl<T: FrameTx> TelemetryBackend for MavlinkTelemetry<T> {
    fn send_status(&mut self, status: &ChannelStatus) -> TelemetryResult<()> {
        let mut buf = [0u8; 256];
        let len = format_heartbeat(status, self.sys_id, self.comp_id, &mut self.seq, &mut buf)?;
        self.tx
            .try_send(&buf[..len])
            .map_err(TelemetryError::Transport)
    }

    fn send_state(&mut self, state: &StateEstimate, time_ms: u32) -> TelemetryResult<()> {
        let mut buf = [0u8; 256];
        let len = format_attitude(
            state,
            time_ms,
            self.sys_id,
            self.comp_id,
            &mut self.seq,
            &mut buf,
        )?;
        self.tx
            .try_send(&buf[..len])
            .map_err(TelemetryError::Transport)
    }

    fn send_actuators(&mut self, cmd: &ActuatorCmd) -> TelemetryResult<()> {
        let mut buf = [0u8; 256];
        let len = format_actuators(
            cmd,
            0, // TODO: pass time_ms
            self.sys_id,
            self.comp_id,
            &mut self.seq,
            &mut buf,
        )?;

        if len > 0 {
            self.tx
                .try_send(&buf[..len])
                .map_err(TelemetryError::Transport)?;
        }

        Ok(())
    }
}

// ============================================================================
// MavlinkCycleFormatter (for TelemetryTask in aviate-runtime)
// ============================================================================

/// MAVLink cycle formatter for protocol-agnostic telemetry
///
/// This struct implements `TelemetryCycleFormatter` and is used by `TelemetryTask`
/// in aviate-runtime. It formats MAVLink messages at configured rates and pushes
/// them to the telemetry queue.
///
/// ## Usage
///
/// ```ignore
/// use aviate_link::mavlink::MavlinkCycleFormatter;
/// use aviate_runtime::TelemetryTask;
///
/// let formatter = MavlinkCycleFormatter::new(&telem_cfg, 1000);
/// let task = TelemetryTask::new(udp_tx, formatter);
/// ```
///
/// ## Rate Configuration
///
/// Message rates are configured via `TelemetryConfig`:
/// - `heartbeat_hz`: HEARTBEAT rate (default 1 Hz)
/// - `attitude_hz`: ATTITUDE_QUATERNION rate (default 10 Hz)
/// - `position_hz`: LOCAL_POSITION_NED rate (default 4 Hz)
pub struct MavlinkCycleFormatter {
    /// Heartbeat rate divider (loop_hz / heartbeat_hz)
    heartbeat_div: u32,
    /// Attitude rate divider (loop_hz / attitude_hz)
    attitude_div: u32,
    /// Position rate divider (loop_hz / position_hz)
    position_div: u32,
    /// MAVLink sequence counter
    seq: u8,
    /// MAVLink system ID
    sys_id: u8,
    /// MAVLink component ID
    comp_id: u8,
}

impl MavlinkCycleFormatter {
    /// Create a new MAVLink cycle formatter
    ///
    /// # Parameters
    /// - `cfg`: Telemetry configuration (rates)
    /// - `loop_hz`: Control loop frequency in Hz
    pub fn new(cfg: &TelemetryConfig, loop_hz: u32) -> Self {
        Self::with_ids(cfg, loop_hz, 1, 1)
    }

    /// Create a new MAVLink cycle formatter with custom system/component IDs
    ///
    /// # Parameters
    /// - `cfg`: Telemetry configuration (rates)
    /// - `loop_hz`: Control loop frequency in Hz
    /// - `sys_id`: MAVLink system ID (1-255)
    /// - `comp_id`: MAVLink component ID (1-255)
    pub fn with_ids(cfg: &TelemetryConfig, loop_hz: u32, sys_id: u8, comp_id: u8) -> Self {
        fn to_div(loop_hz: u32, msg_hz: u8) -> u32 {
            let hz = msg_hz.max(1) as u32; // Guard against zero
            (loop_hz / hz).max(1)
        }

        Self {
            heartbeat_div: to_div(loop_hz, cfg.heartbeat_hz),
            attitude_div: to_div(loop_hz, cfg.attitude_hz),
            position_div: to_div(loop_hz, cfg.position_hz),
            seq: 0,
            sys_id,
            comp_id,
        }
    }
}

impl TelemetryCycleFormatter for MavlinkCycleFormatter {
    fn format_cycle(&mut self, snapshot: &TelemetrySnapshot, queue: &mut DefaultTelemetryQueue) {
        let mut buf = [0u8; TELEMETRY_MAX_FRAME];

        // HEARTBEAT at configured rate (default 1 Hz)
        if snapshot.iteration.is_multiple_of(self.heartbeat_div) {
            if let Ok(len) = format_heartbeat(
                &snapshot.status,
                self.sys_id,
                self.comp_id,
                &mut self.seq,
                &mut buf,
            ) {
                let _ = queue.push(&buf[..len]);
            }
        }

        // ATTITUDE_QUATERNION at configured rate (default 10 Hz)
        if snapshot.iteration.is_multiple_of(self.attitude_div) {
            if let Ok(len) = format_attitude(
                &snapshot.state,
                snapshot.time_ms,
                self.sys_id,
                self.comp_id,
                &mut self.seq,
                &mut buf,
            ) {
                let _ = queue.push(&buf[..len]);
            }
        }

        // LOCAL_POSITION_NED at configured rate (default 4 Hz)
        if snapshot.iteration.is_multiple_of(self.position_div) {
            if let Ok(len) = format_local_position(
                &snapshot.state,
                snapshot.time_ms,
                self.sys_id,
                self.comp_id,
                &mut self.seq,
                &mut buf,
            ) {
                let _ = queue.push(&buf[..len]);
            }
        }
    }
}
