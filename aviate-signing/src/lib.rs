#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
//! The MAVLink 2 `sha256_48` signing primitive.
//!
//! A crate of its own, and deliberately not part of `aviate-hal-io`:
//! that crate is inside `aviate-runtime`'s flight dependency graph, so
//! putting `sha2` there would pull the whole digest tree into the
//! certified surface (`scripts/check_runtime_boundary.sh` refuses it).
//! Only the board that actually verifies signatures depends on this.
//!
//! MAVLink 2 signs with SHA-256 over `secret_key ‖ frame`, truncated to
//! 48 bits — not HMAC-SHA256. The two are easy to confuse and produce
//! entirely different tags, so an implementation that picks the wrong one
//! rejects every frame from every conforming ground station.
//!
//! It lives in this crate, rather than in the board crate that runs it,
//! because board crates sit outside the workspace: CI can only
//! compile-check them, so a transposed `update(key)` / `update(msg)`
//! would pass every test and every gate and fail only on the aircraft.
//! Here the function the flight build calls is the one the pymavlink
//! golden vectors exercise on the host.

use sha2::{Digest, Sha256};

/// Bytes of the digest MAVLink 2 keeps as the signature.
pub const MAVLINK_SIGNATURE_LEN: usize = 6;

/// SHA-256 over `key ‖ msg`.
///
/// Callers truncate; MAVLink takes the leading
/// [`MAVLINK_SIGNATURE_LEN`] bytes.
pub fn sha256_keyed_prefix(key: &[u8], msg: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(key);
    hasher.update(msg);
    hasher.finalize().into()
}

/// Compare a possibly-truncated tag against [`sha256_keyed_prefix`].
///
/// Constant time in the tag bytes. A tag shorter than `min_tag_len` is
/// refused outright: an empty tag folds to zero difference and would
/// otherwise verify against anything.
pub fn verify_sha256_keyed_prefix(key: &[u8], msg: &[u8], tag: &[u8], min_tag_len: usize) -> bool {
    if tag.len() < min_tag_len {
        return false;
    }
    let digest = sha256_keyed_prefix(key, msg);
    let Some(prefix) = digest.get(..tag.len()) else {
        return false;
    };
    let diff = prefix
        .iter()
        .zip(tag.iter())
        .fold(0u8, |acc, (a, b)| acc | (a ^ b));
    diff == 0
}

#[cfg(test)]
mod tests;
