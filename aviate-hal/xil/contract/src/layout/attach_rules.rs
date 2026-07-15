//! Attach-time rules: what a session may conclude about a block
//! before it interprets a single payload field, and what the name it
//! attached to resolves to later.
//!
//! Split out of `layout.rs` so the wire structs read as a layout and
//! these read as a protocol.

use super::{EXPECTED_SIZE, LAYOUT_VERSION, MAGIC};

/// What the shm object a name resolves to RIGHT NOW is, relative to
/// the session holding a mapping of it.
///
/// A `bool` cannot express this: "the name does not resolve" and
/// "the name resolves to the same object" are opposite conclusions
/// that a boolean `writer_replaced` collapses into the same
/// `false` — leaving an orphaned mapping looking healthy forever.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WriterState {
    /// The name resolves to the very object this session mapped, and
    /// its writer is live. The only state in which reads are
    /// trustworthy.
    Current,
    /// The name does not resolve: the writer exited and unlinked.
    /// This session's mapping is an orphan kept alive only by
    /// itself — it must not be treated as healthy.
    Gone,
    /// The name resolves to an object that has not finished
    /// initialising (still zero-sized, or readiness not yet
    /// published). Retryable; not a failure.
    Initializing,
    /// The name resolves to a DIFFERENT object: the writer restarted
    /// and re-created the block. This session must re-attach; its
    /// mapping can only serve the dead world.
    Replaced,
    /// The name resolves to an object whose fingerprint this build
    /// does not accept (foreign or stale layout). Fail closed.
    ContractMismatch,
}

/// Attach-time validation failure. Every variant names what was
/// found so a mismatch is diagnosable from the error alone.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttachError {
    /// First eight bytes were not [`MAGIC`].
    BadMagic {
        /// Value found at offset 0.
        found: u64,
    },
    /// Layout version differs from this crate's.
    VersionMismatch {
        /// Version found in the header.
        found: u32,
    },
    /// The writer declared a different structure size than this
    /// crate compiled.
    DeclaredSizeMismatch {
        /// `declared_size` found in the header.
        found: u32,
    },
    /// The mapped object is smaller than the structure. (Larger is
    /// legal: macOS rounds `st_size` up to the page, so the check is
    /// `actual < expected` fails — never `==`.)
    ///
    /// A size of exactly ZERO is NOT this error: `shm_open(O_CREAT)`
    /// publishes the name before `ftruncate` runs, so a zero-sized
    /// object is a writer mid-creation. That window is reported as
    /// [`AttachError::Initializing`] and is retryable — calling it a
    /// contract violation turns a microsecond of normal startup into
    /// a permanent failure, because callers never retry a mismatch.
    MappingTooSmall {
        /// Object size reported by the OS.
        actual: usize,
    },
    /// The object exists but is still being created: the name is
    /// published (`shm_open`) before it is sized (`ftruncate`) and
    /// stamped. Retryable.
    Initializing,
}

/// Fail-closed attach validation (#262): magic, layout version,
/// declared size, and mapped-object size must all agree before a
/// single payload field is interpreted.
pub fn validate_attach(
    magic: u64,
    layout_version: u32,
    declared_size: u32,
    actual_object_size: usize,
) -> Result<(), AttachError> {
    // Zero is the `shm_open`-before-`ftruncate` window, not a
    // foreign object: retryable, not fatal.
    if actual_object_size == 0 {
        return Err(AttachError::Initializing);
    }
    if actual_object_size < EXPECTED_SIZE {
        return Err(AttachError::MappingTooSmall {
            actual: actual_object_size,
        });
    }
    if magic != MAGIC {
        return Err(AttachError::BadMagic { found: magic });
    }
    if layout_version != LAYOUT_VERSION {
        return Err(AttachError::VersionMismatch {
            found: layout_version,
        });
    }
    if declared_size as usize != EXPECTED_SIZE {
        return Err(AttachError::DeclaredSizeMismatch {
            found: declared_size,
        });
    }
    Ok(())
}
