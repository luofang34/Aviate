//! Rust-owned shared-memory contract between the Aviate gz-sim
//! plugin (writer of model state), the flight controller (writer of
//! motor commands), and read-only telemetry consumers (#262).
//!
//! This crate is the single source of truth for the `#[repr(C)]`
//! layout: the C++ plugin consumes the cbindgen-generated
//! `include/aviate_xil_contract.h` (checked in; a unit test fails if
//! it drifts from the Rust definitions), and every Rust consumer
//! uses these types directly. Layout changes bump
//! [`LAYOUT_VERSION`]; consumers fail closed on attach instead of
//! reading a foreign layout as plausible garbage.
//!
//! No I/O lives here — mapping the shared memory is the small,
//! auditable `aviate-xil-shm` crate; this crate is `no_std`, has no
//! unsafe code, and can be pinned as a git dependency by external
//! consumers (Pilotage) to replace hand-mirrored offsets.

#![no_std]

mod layout;
mod seqlock;

pub use layout::{
    pack_fc_status, pack_lifecycle_request, unpack_fc_status, unpack_lifecycle_request,
    validate_attach, AttachError, ControlBlock, FcState, LifecycleRequest, ModelStateBlock,
    MotorCommandBlock, SharedStateHeader, SharedStateV2, WriterState, EXPECTED_SIZE,
    LAYOUT_VERSION, MAGIC, SHM_NAME_BASE, SHM_NAME_INSTANCE_0,
};
pub use seqlock::{seqlock_read, seqlock_write, SEQLOCK_MAX_RETRIES};
