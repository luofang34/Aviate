//! Security policy layer for Aviate flight control system
//!
//! This crate provides command authentication, signature verification,
//! and anti-replay protection for external commands.
//!
//! ## Architecture (DO-178C 6-Layer Model)
//!
//! ```text
//! Layer 1: HAL (aviate-hal-io)
//!   ├─ KeyStore trait (OTP/flash key access)
//!   └─ CryptoEngine trait (HMAC/AES/Ed25519)
//!
//! Layer 2: Chip HAL (aviate-hal-stm32h7)
//!   ├─ Stm32h7KeyStore (OTP reads, flash const keys)
//!   └─ Stm32h7CryptoEngine (HMAC-SHA256 software/hardware)
//!
//! Layer 3: Link (aviate-link)
//!   ├─ Command struct (with optional SignatureMeta)
//!   └─ MavlinkCommandLink (parses MAVLink → Command)
//!
//! Layer 4: Security (THIS CRATE)
//!   ├─ CommandAuth trait (PlainAuth, SignedAuth)
//!   ├─ AntiReplayWindow (per-link_id monotonic counter)
//!   └─ CommandGateway (unified entry point)
//!
//! Layer 5: App
//!   └─ Uses CommandGateway to get verified commands
//! ```
//!
//! ## Usage Example
//!
//! ```ignore
//! use aviate_security::{CommandGateway, SignedAuth};
//! use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
//! use aviate_link::mavlink::MavlinkCommandLink;
//!
//! // Hardware layer
//! let keystore = Stm32h7KeyStore::new();
//! let crypto = Stm32h7CryptoEngine::new();
//!
//! // Link layer (protocol parsing)
//! let link = MavlinkCommandLink::new(usb_rx);
//!
//! // Security layer (verification)
//! let auth = SignedAuth::new(keystore, crypto);
//! let mut gateway = CommandGateway::new(link, auth);
//!
//! // Application layer
//! loop {
//!     if let Ok(Some(cmd)) = gateway.poll_command(now_ms) {
//!         // cmd is verified! Safe to execute
//!         kernel.execute(cmd);
//!     }
//! }
//! ```
//!
//! ## Security Model
//!
//! - **PlainAuth**: No verification (development/testing only)
//! - **SignedAuth**: Requires MAVLink message signing
//!   - HMAC-SHA256 verification per MAVLink spec
//!   - Per-link_id anti-replay (strict monotonic counter)
//!   - Key lookup: `KeySelector { link_id, purpose: Command }`
//!
//! ## DO-178C Criticality
//!
//! - **DAL A/B**: Flight-critical security policy
//! - Commands MUST go through CommandGateway
//! - Bypass paths are prohibited (enforced by API design)

#![no_std]
#![forbid(unsafe_code)]

pub mod anti_replay;
pub mod auth;
pub mod errors;
pub mod gateway;

// Re-export key types
pub use anti_replay::AntiReplayWindow;
pub use auth::{CommandAuth, PlainAuth, SignedAuth};
pub use errors::{AuthError, GatewayError};
pub use gateway::CommandGateway;
