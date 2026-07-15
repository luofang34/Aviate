//! Atomic payload lanes — the primitives every shared access uses.
//!
//! Split out of `mapping.rs` so the mapping's lifecycle (create /
//! fail-closed attach / seqlock publish / staleness) reads without
//! the per-lane plumbing in the way. Both files are `unsafe`; this
//! one is where the "every lane is atomic" rule is implemented, and
//! `mapping.rs` is where it is used.

use core::sync::atomic::{AtomicU32, AtomicU64, Ordering};

// ---------------------------------------------------------------
// Atomic payload lanes.
//
// EVERY payload lane is read and written atomically (Relaxed) by
// BOTH sides. The seqlock supplies the ordering and the
// all-or-nothing snapshot; the atomics supply a defined memory
// model. A plain (or `volatile`) load racing the peer process's
// store is a data race — undefined behaviour in Rust and C++ alike —
// no matter how well the seqlock protocol behaves in practice, and
// `volatile` deliberately carries NO atomicity or ordering
// guarantees. The C++ plugin uses `__atomic_load_n` /
// `__atomic_store_n` on the same lanes for the same reason.
//
// f64 lanes are accessed as their bit patterns: f64 and u64 share
// size and alignment, and the layout's `double` stays readable in
// the generated C header.
// ---------------------------------------------------------------

/// # Safety
/// `p` must be an 8-byte-aligned lane inside a validated mapping,
/// accessed only through these helpers.
#[inline]
pub(crate) unsafe fn load_f64(p: *const f64) -> f64 {
    f64::from_bits(AtomicU64::from_ptr(p as *mut u64).load(Ordering::Relaxed))
}

/// # Safety
/// See [`load_f64`].
#[inline]
pub(crate) unsafe fn store_f64(p: *mut f64, v: f64) {
    AtomicU64::from_ptr(p.cast::<u64>()).store(v.to_bits(), Ordering::Relaxed);
}

/// # Safety
/// See [`load_f64`].
#[inline]
pub(crate) unsafe fn load_f64_lanes<const N: usize>(p: *const [f64; N]) -> [f64; N] {
    let base = p.cast::<f64>();
    core::array::from_fn(|i| load_f64(base.add(i)))
}

/// # Safety
/// See [`load_f64`].
#[inline]
pub(crate) unsafe fn store_f64_lanes<const N: usize>(p: *mut [f64; N], v: &[f64; N]) {
    let base = p.cast::<f64>();
    for (i, lane) in v.iter().enumerate() {
        store_f64(base.add(i), *lane);
    }
}

/// # Safety
/// See [`load_f64`].
#[inline]
pub(crate) unsafe fn load_u64(p: *const u64) -> u64 {
    AtomicU64::from_ptr(p as *mut u64).load(Ordering::Relaxed)
}

/// # Safety
/// See [`load_f64`].
#[inline]
pub(crate) unsafe fn store_u64(p: *mut u64, v: u64) {
    AtomicU64::from_ptr(p).store(v, Ordering::Relaxed);
}

/// # Safety
/// `p` must be a 4-byte-aligned lane inside a validated mapping.
#[inline]
pub(crate) unsafe fn load_u32(p: *const u32) -> u32 {
    AtomicU32::from_ptr(p as *mut u32).load(Ordering::Relaxed)
}

/// # Safety
/// See [`load_u32`].
#[inline]
pub(crate) unsafe fn store_u32(p: *mut u32, v: u32) {
    AtomicU32::from_ptr(p).store(v, Ordering::Relaxed);
}
