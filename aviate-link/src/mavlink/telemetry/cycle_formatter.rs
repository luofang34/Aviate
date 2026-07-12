//! MAVLink cycle formatter driving per-iteration telemetry emission.

use super::estimator;
use super::{format_heartbeat, TelemetryConfig};
use crate::errors::TelemetryError;
use crate::queue::{DefaultTelemetryQueue, TELEMETRY_MAX_FRAME};
use crate::telemetry::{TelemetryCycleFormatter, TelemetrySnapshot};

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
/// let formatter = MavlinkCycleFormatter::new(&telem_cfg, 1000)?;
/// let task = TelemetryTask::new(udp_tx, formatter);
/// ```
///
/// ## Rate Configuration
///
/// Message rates are configured via `TelemetryConfig`:
/// - `heartbeat_hz`: HEARTBEAT rate (default 1 Hz)
/// - `attitude_hz`: ATTITUDE_QUATERNION rate (default 10 Hz)
/// - `position_hz`: LOCAL_POSITION_NED rate (default 4 Hz)
/// - `estimator_status_hz`: minimum estimator-status rate (default 4 Hz)
///
/// Valid rates are 1..=255 Hz; a zero rate is a configuration error and
/// construction fails rather than reinterpreting it. Each stream emits
/// every `ceil(loop_hz / rate_hz)` iterations, so the achieved rate never
/// exceeds the requested rate; a rate above `loop_hz` is capped at one
/// emission per loop iteration. The iteration counter wraps at `u32::MAX`
/// (about 124 days at 400 Hz), and the one-second window containing the
/// wrap may carry a single extra frame per stream.
pub struct MavlinkCycleFormatter {
    /// Heartbeat rate divider (loop_hz / heartbeat_hz)
    heartbeat_div: u32,
    /// Attitude rate divider (loop_hz / attitude_hz)
    attitude_div: u32,
    /// Position rate divider (loop_hz / position_hz)
    position_div: u32,
    /// Estimator-status rate divider (loop_hz / estimator_status_hz)
    estimator_status_div: u32,
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
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::ZeroRate`] when `loop_hz` or any
    /// configured message rate is zero.
    pub fn new(cfg: &TelemetryConfig, loop_hz: u32) -> Result<Self, TelemetryError> {
        Self::with_ids(cfg, loop_hz, 1, 1)
    }

    /// Create a new MAVLink cycle formatter with custom system/component IDs
    ///
    /// # Parameters
    /// - `cfg`: Telemetry configuration (rates)
    /// - `loop_hz`: Control loop frequency in Hz
    /// - `sys_id`: MAVLink system ID (1-255)
    /// - `comp_id`: MAVLink component ID (1-255)
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::ZeroRate`] when `loop_hz` or any
    /// configured message rate is zero. A zero rate has no defined
    /// meaning; rejecting it keeps a typo from silently running a stream
    /// at a rate the config never requested.
    pub fn with_ids(
        cfg: &TelemetryConfig,
        loop_hz: u32,
        sys_id: u8,
        comp_id: u8,
    ) -> Result<Self, TelemetryError> {
        if loop_hz == 0 {
            return Err(TelemetryError::ZeroRate("loop_hz"));
        }

        // Ceiling division keeps the achieved rate at or below the
        // requested rate; flooring would overshoot every rate that does
        // not divide loop_hz. The zero check lives in the same call so
        // no divider can ever be computed from an unvalidated rate.
        fn to_div(loop_hz: u32, msg_hz: u8, field: &'static str) -> Result<u32, TelemetryError> {
            if msg_hz == 0 {
                return Err(TelemetryError::ZeroRate(field));
            }
            Ok(loop_hz.div_ceil(u32::from(msg_hz)))
        }

        Ok(Self {
            heartbeat_div: to_div(loop_hz, cfg.heartbeat_hz, "heartbeat_hz")?,
            attitude_div: to_div(loop_hz, cfg.attitude_hz, "attitude_hz")?,
            position_div: to_div(loop_hz, cfg.position_hz, "position_hz")?,
            estimator_status_div: to_div(loop_hz, cfg.estimator_status_hz, "estimator_status_hz")?,
            seq: 0,
            sys_id,
            comp_id,
        })
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
                queue.push(&buf[..len]).ok();
            }
        }

        let emit_attitude = snapshot.iteration.is_multiple_of(self.attitude_div);
        let emit_position = snapshot.iteration.is_multiple_of(self.position_div);
        let emit_status = snapshot.iteration.is_multiple_of(self.estimator_status_div);
        estimator::enqueue_estimate_group(
            snapshot,
            emit_attitude,
            emit_position,
            emit_status,
            (self.sys_id, self.comp_id),
            &mut self.seq,
            queue,
        );
    }
}
