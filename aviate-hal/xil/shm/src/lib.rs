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
//! Roles:
//! * [`ShmSession::create`] — the simulation-side writer (the gz
//!   plugin in production is C++ and initializes the identical
//!   layout; the Rust creator exists for headless harnesses and
//!   tests).
//! * [`ShmSession::attach`] — the FC and read-only consumers.
//!   Attach FAILS CLOSED unless magic, layout version, declared
//!   size, and mapped size all validate ([`AttachFailure`]).

mod session;

pub use session::{AttachFailure, ModelStateSnapshot, ShmSession};
