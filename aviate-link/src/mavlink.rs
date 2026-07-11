//! MAVLink Protocol Module
//!
//! This module provides complete MAVLink 2.0 protocol support for Aviate.
//!
//! ## Module Organization (DO-178C 3-File Pattern)
//!
//! ```text
//! mavlink/
//!   protocol.rs    # Protocol specification (types, parser, serializer)
//!   telemetry.rs   # Outbound: StateEstimate → MAVLink (app → ground)
//!   command.rs     # Inbound: MAVLink → Command (ground → app)
//! ```
//!
//! ## DO-178C Separation of Concerns
//!
//! This module is **protocol-only**. It does NOT contain:
//! - ❌ Security logic (KeyStore, CryptoEngine, signature verification)
//! - ❌ Authentication (CommandAuth, SecureAuth)
//! - ❌ Anti-replay checks (sequence tracking, nonce validation)
//!
//! All security logic belongs in `aviate-security` crate.
//!
//! ## Data Flow Direction (Auditing)
//!
//! - **telemetry.rs**: Outbound ONLY (app → ground)
//!   - Checklist: Should NEVER read from FrameRx
//!   - Criticality: DAL D/E (informational, no flight safety impact)
//!
//! - **command.rs**: Inbound ONLY (ground → app)
//!   - Checklist: Should NEVER write to FrameTx
//!   - Checklist: Must NOT bypass CommandGateway
//!   - Criticality: DAL A/B (affects flight safety, requires verification)
//!
//! - **protocol.rs**: Pure translation (bytes ↔ structs)
//!   - Checklist: No I/O, no Aviate domain types, no security
//!
//! ## Audit Checklist (DO-178C)
//!
//! When auditing this module, verify:
//! - ✅ No imports from `aviate-security`
//! - ✅ No `KeyStore`, `CryptoEngine`, `CommandGateway` usage in protocol.rs
//! - ✅ telemetry.rs only uses `FrameTx` (never `FrameRx`)
//! - ✅ command.rs only uses `FrameRx` (never `FrameTx`)
//! - ✅ All commands from command.rs are UNVERIFIED (must use CommandGateway)
//!
//! ## Usage Example
//!
//! ```ignore
//! // Telemetry (app → ground)
//! use aviate_link::mavlink::{MavlinkTelemetry, format_heartbeat};
//!
//! // High-DAL: Format only
//! let mut buf = [0u8; 256];
//! let len = format_heartbeat(&status, sys_id, comp_id, &mut seq, &mut buf)?;
//! telemetry_queue.push(&buf[..len]);  // O(1), non-blocking
//!
//! // Low-DAL: I/O sender
//! let mut telemetry = MavlinkTelemetry::new(usb_tx, 1, 1);
//! telemetry.send_status(&status)?;
//!
//! // Commands (ground → app) - MUST use CommandGateway!
//! use aviate_link::mavlink::MavlinkCommandLink;
//! use aviate_security::CommandGateway;  // Required for verification!
//!
//! let link = MavlinkCommandLink::new(usb_rx);
//! let mut gateway = CommandGateway::new(link, auth);  // Adds security
//!
//! if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {
//!     kernel.execute(cmd);  // Safe: verified by gateway
//! }
//! ```

pub mod command;
pub mod protocol;
pub mod telemetry;

// Re-export protocol types for convenience
pub use protocol::{
    aviate_estimate_quality, estimator_status_flags, mav_cmd, mav_result, parse_mavlink,
    serialize_mavlink, AviateEstimatorStatus, EstimatorStatus, MavAutopilot, MavComponent,
    MavMessage, MavModeFlag, MavState, MavType,
};

// Re-export link implementations
pub use command::MavlinkCommandLink;
pub use telemetry::{
    format_actuators, format_attitude, format_aviate_estimator_status, format_estimator_status,
    format_heartbeat, format_local_position, MavlinkCycleFormatter, MavlinkTelemetry,
};
