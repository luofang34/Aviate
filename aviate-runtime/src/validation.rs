//! Configuration validation for runtime
//!
//! This module validates configuration values against compile-time limits.
//! Called during initialization (low-DAL) before starting the control loop.

use aviate_config::TelemetryConfig;
use aviate_link::{TELEMETRY_MAX_FRAME, TELEMETRY_MAX_QUEUE};

/// Telemetry configuration validation error
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TelemetryConfigError {
    /// frame_size exceeds TELEMETRY_MAX_FRAME
    FrameSizeTooLarge {
        /// Requested frame size in bytes.
        requested: usize,
        /// Compile-time maximum frame size.
        max: usize,
    },
    /// queue_len exceeds TELEMETRY_MAX_QUEUE
    QueueLenTooLarge {
        /// Requested queue length in frames.
        requested: usize,
        /// Compile-time maximum queue length.
        max: usize,
    },
}

impl core::fmt::Display for TelemetryConfigError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TelemetryConfigError::FrameSizeTooLarge { requested, max } => {
                write!(
                    f,
                    "telemetry frame_size {} exceeds maximum {}",
                    requested, max
                )
            }
            TelemetryConfigError::QueueLenTooLarge { requested, max } => {
                write!(
                    f,
                    "telemetry queue_len {} exceeds maximum {}",
                    requested, max
                )
            }
        }
    }
}

/// Validate telemetry configuration against compile-time limits
///
/// Call this during initialization to ensure config values don't exceed
/// the compile-time limits of the telemetry queue.
///
/// # Returns
///
/// - `Ok(())` if all values are within limits
/// - `Err(TelemetryConfigError)` if any value exceeds limits
///
/// # Example
///
/// ```ignore
/// use aviate_runtime::validation::validate_telemetry_config;
///
/// if let Err(e) = validate_telemetry_config(&cfg.telemetry) {
///     eprintln!("[ERROR] Invalid telemetry config: {}", e);
///     return Err(e);
/// }
/// ```
pub fn validate_telemetry_config(cfg: &TelemetryConfig) -> Result<(), TelemetryConfigError> {
    if cfg.frame_size > TELEMETRY_MAX_FRAME {
        return Err(TelemetryConfigError::FrameSizeTooLarge {
            requested: cfg.frame_size,
            max: TELEMETRY_MAX_FRAME,
        });
    }

    if cfg.queue_len > TELEMETRY_MAX_QUEUE {
        return Err(TelemetryConfigError::QueueLenTooLarge {
            requested: cfg.queue_len,
            max: TELEMETRY_MAX_QUEUE,
        });
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_valid_config() {
        let cfg = TelemetryConfig {
            frame_size: 256,
            queue_len: 16,
            heartbeat_hz: 1,
            attitude_hz: 10,
            position_hz: 4,
        };
        assert!(validate_telemetry_config(&cfg).is_ok());
    }

    #[test]
    fn test_frame_size_at_limit() {
        let cfg = TelemetryConfig {
            frame_size: TELEMETRY_MAX_FRAME,
            queue_len: TELEMETRY_MAX_QUEUE,
            heartbeat_hz: 1,
            attitude_hz: 10,
            position_hz: 4,
        };
        assert!(validate_telemetry_config(&cfg).is_ok());
    }

    #[test]
    fn test_frame_size_too_large() {
        let cfg = TelemetryConfig {
            frame_size: TELEMETRY_MAX_FRAME + 1,
            queue_len: 16,
            heartbeat_hz: 1,
            attitude_hz: 10,
            position_hz: 4,
        };
        assert!(matches!(
            validate_telemetry_config(&cfg),
            Err(TelemetryConfigError::FrameSizeTooLarge { .. })
        ));
    }

    #[test]
    fn test_queue_len_too_large() {
        let cfg = TelemetryConfig {
            frame_size: 256,
            queue_len: TELEMETRY_MAX_QUEUE + 1,
            heartbeat_hz: 1,
            attitude_hz: 10,
            position_hz: 4,
        };
        assert!(matches!(
            validate_telemetry_config(&cfg),
            Err(TelemetryConfigError::QueueLenTooLarge { .. })
        ));
    }
}
