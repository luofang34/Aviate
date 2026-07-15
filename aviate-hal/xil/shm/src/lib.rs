//! POSIX shared-memory transport for the [`aviate_xil_contract`]
//! block (#262, #265).
//!
//! This is the ONE place in the SITL data plane that touches raw
//! memory: `shm_open`/`mmap` plus volatile / atomic field access
//! over the mapping. Everything above it — the contract layout, the
//! flight controller, telemetry consumers — is `#![forbid(unsafe)]`
//! territory. Keep this crate small and boring; every `unsafe` block
//! carries a SAFETY note tied to the contract's invariants.
//!
//! Access is ROLE-SPECIFIC — a consumer cannot obtain mutable access
//! by accident, and the OS enforces it (read-only mapping):
//!
//! * [`SimWriterSession`] — creates and owns the block; publishes
//!   model state, reads motor commands, acks lifecycle requests
//!   (the gz plugin in production is C++ over the same layout; the
//!   Rust writer serves headless harnesses and tests).
//! * [`FcSession`] — the flight controller: writes motor commands,
//!   step acks, and its lifecycle status.
//! * [`HostSession`] — the session host / test harness: posts
//!   lifecycle requests and drives the runtime time controls.
//! * [`ConsumerSession`] — read-only telemetry observers, mapped
//!   `PROT_READ`.
//!
//! Every attach FAILS CLOSED unless magic, layout version, declared
//! size, and mapped size validate AND the writer has published
//! `plugin_ready` ([`AttachFailure`]).
//!
//! A consumer cannot obtain mutable access by accident: the type
//! carries no writer at all, and its mapping is `PROT_READ` so even
//! a raw store through it would fault.
//!
//! ```compile_fail
//! let c = aviate_xil_shm::ConsumerSession::attach("/aviate_gz_bridge").unwrap();
//! c.write_motor_command(&[100.0]); // no such method on a consumer
//! ```

mod mapping;
mod roles;

pub use mapping::{AttachFailure, ModelStateSnapshot};
pub use roles::{ConsumerSession, FcSession, HostSession, SimWriterSession};
