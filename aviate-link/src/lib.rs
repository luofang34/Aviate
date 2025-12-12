//! Cross-protocol telemetry and command abstraction
//!
//! This crate provides protocol-agnostic abstractions for telemetry and commands.
//! It sits between the protocol layer (MAVLink, CCSDS, etc.) and the application layer.
//!
//! ## Architecture Position
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Application Layer (aviate-apps/*)                          │
//! │  - Uses CommandGateway for verified commands                │
//! │  - Uses TelemetryBackend for sending telemetry              │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │
//! ┌──────────────────▼──────────────────────────────────────────┐
//! │  aviate-security (Policy Layer)                             │
//! │  - CommandGateway (authentication + anti-replay)            │
//! │  - Uses KeyStore/CryptoEngine from HAL                      │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │
//! ┌──────────────────▼──────────────────────────────────────────┐
//! │  aviate-link (THIS CRATE - Protocol Abstraction)            │
//! │  - TelemetryBackend trait + MavlinkTelemetry                │
//! │  - CommandLink trait + MavlinkCommandLink                   │
//! │  - TelemetryQueue (time-deterministic ring buffer)          │
//! │  - Command struct (protocol-agnostic representation)        │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │
//! ┌──────────────────▼──────────────────────────────────────────┐
//! │  aviate-mavlink (Protocol Layer)                            │
//! │  - MAVLink message types (Heartbeat, AttitudeQuaternion)    │
//! │  - Parser + Serializer (pure protocol logic)                │
//! └──────────────────┬──────────────────────────────────────────┘
//!                    │
//! ┌──────────────────▼──────────────────────────────────────────┐
//! │  aviate-hal-io (Hardware Abstraction)                       │
//! │  - FrameTx/FrameRx traits (transport I/O)                   │
//! │  - KeyStore/CryptoEngine traits (security primitives)       │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Critical DO-178C Separation
//!
//! This crate has ZERO security knowledge:
//! - No signature verification (use `aviate-security::CommandGateway`)
//! - No anti-replay checks (use `aviate-security::CommandGateway`)
//! - No access control (use `aviate-security::CommandGateway`)
//!
//! All it does is:
//! 1. Parse protocol bytes → Command struct (syntax parsing)
//! 2. Format domain data → protocol bytes (serialization)
//! 3. Provide time-deterministic telemetry queue
//!
//! ## Key Abstractions
//!
//! ### Telemetry (App → Ground)
//!
//! - **TelemetryBackend trait**: High-level API for sending telemetry
//!   - LOW-DAL only! Performs I/O, can fail.
//! - **Pure format helpers**: For high-DAL control code
//!   - `format_heartbeat()`, `format_attitude()`, etc.
//!   - No I/O, bounded runtime, safe for provable WCET
//! - **TelemetryQueue**: Time-deterministic ring buffer
//!   - O(1) push/pop, statically allocated, no fragmentation
//!   - High-DAL pushes formatted frames, low-DAL pops and sends
//!
//! ### Commands (Ground → App)
//!
//! - **CommandLink trait**: Parse protocol bytes → Command struct
//!   - Returns UNVERIFIED commands! Always use CommandGateway.
//! - **Command struct**: Protocol-agnostic command representation
//!   - `CommandKind` enum + generic params
//!
//! ## Usage Example
//!
//! ```ignore
//! use aviate_link::{
//!     TelemetryBackend, MavlinkTelemetry, TelemetryQueue,
//!     CommandLink, MavlinkCommandLink, format_heartbeat,
//! };
//!
//! // Create telemetry queue (high-DAL control task uses this)
//! static mut TELEM_QUEUE: TelemetryQueue<16, 280> = TelemetryQueue::new();
//!
//! // High-DAL control task (provable WCET, no I/O):
//! fn control_task(ctx: &mut AppContext) {
//!     // Format telemetry (no I/O!)
//!     let mut buf = [0u8; 256];
//!     if let Ok(len) = format_heartbeat(&status, 1, 1, &mut ctx.seq, &mut buf) {
//!         let _ = ctx.telemetry_queue.push(&buf[..len]);  // O(1), non-blocking
//!     }
//! }
//!
//! // Low-DAL telemetry task (can fail, doesn't affect control):
//! fn telemetry_task(ctx: &mut AppContext, usb_tx: impl FrameTx) {
//!     while ctx.telemetry_queue.pop_with(|frame| {
//!         let _ = usb_tx.try_send(frame);  // Failure OK
//!     }) {}
//! }
//!
//! // Command reception (MUST use CommandGateway for verification!)
//! fn command_task(ctx: &mut AppContext, usb_rx: impl FrameRx) {
//!     let mut link = MavlinkCommandLink::new(usb_rx);
//!     let mut gateway = CommandGateway::new(link, auth);  // From aviate-security
//!
//!     if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {
//!         ctx.kernel.execute(cmd);  // Safe: command verified by gateway
//!     }
//! }
//! ```
//!
//! ## Feature Flags
//!
//! None currently. This crate has no optional features.

#![no_std]
#![forbid(unsafe_code)]
#![deny(clippy::panic)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]

pub mod command;
pub mod errors;
pub mod queue;
pub mod telemetry;

// Protocol implementations
pub mod mavlink;

// Re-export key types for convenience
pub use command::{Command, CommandKind, CommandLink};
pub use errors::{LinkError, LinkResult, TelemetryError, TelemetryResult};
pub use queue::{QueueError, TelemetryQueue};
pub use telemetry::TelemetryBackend;
