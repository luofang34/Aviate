//! Seqlock protocol over the shared blocks (#262).
//!
//! The writer makes `seq` odd before touching the payload and even
//! after; a reader takes a payload snapshot between two even, equal
//! reads of `seq`. This replaces the bare `seq += 1` the layout used
//! to carry, whose readers could miss a complete write landing
//! between their two reads (and whose Rust reader never re-checked
//! at all).
//!
//! These helpers own only the PROTOCOL; how the payload bytes are
//! copied (volatile reads out of the mapping) is the caller's
//! concern, so this crate stays free of unsafe code.

use core::sync::atomic::{AtomicU32, Ordering};

/// Retry budget for [`seqlock_read`]: with a 1 kHz writer a reader
/// virtually never observes more than one in-flight write; a bounded
/// budget keeps a crashed-mid-write writer (seq stuck odd) from
/// hanging the reader forever.
pub const SEQLOCK_MAX_RETRIES: u32 = 16;

/// One consistent snapshot of a seqlock-protected payload, or `None`
/// if the writer kept the payload in flight for the whole retry
/// budget (stale data must not be handed out on a torn read).
pub fn seqlock_read<T, F: FnMut() -> T>(seq: &AtomicU32, mut copy_payload: F) -> Option<T> {
    for _ in 0..SEQLOCK_MAX_RETRIES {
        let s1 = seq.load(Ordering::Acquire);
        if s1 & 1 != 0 {
            core::hint::spin_loop();
            continue;
        }
        let snapshot = copy_payload();
        // Acquire on the re-read orders it after the payload copy;
        // equal values prove no write started or completed inside
        // the window.
        let s2 = seq.load(Ordering::Acquire);
        if s1 == s2 {
            return Some(snapshot);
        }
        core::hint::spin_loop();
    }
    None
}

/// Publish one payload write under the seqlock: seq goes odd, the
/// payload is written, seq returns even.
pub fn seqlock_write<F: FnOnce()>(seq: &AtomicU32, write_payload: F) {
    // AcqRel: readers that see the odd value also see it BEFORE any
    // payload store that follows.
    let s = seq.fetch_add(1, Ordering::AcqRel);
    debug_assert!(s & 1 == 0, "seqlock writer re-entered mid-write");
    write_payload();
    seq.fetch_add(1, Ordering::Release);
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU64, Ordering};

    #[test]
    fn read_returns_consistent_snapshot() {
        let seq = AtomicU32::new(0);
        let payload = AtomicU64::new(42);
        let got = seqlock_read(&seq, || payload.load(Ordering::Relaxed)).unwrap();
        assert_eq!(got, 42);
    }

    #[test]
    fn read_refuses_in_flight_writer() {
        // seq stuck odd (writer crashed mid-write): the reader must
        // exhaust its budget and return None, never a torn payload.
        let seq = AtomicU32::new(1);
        let got = seqlock_read(&seq, || 7_u32);
        assert_eq!(got, None);
    }

    #[test]
    fn read_retries_across_a_write() {
        // Simulate a write completing between the reader's first
        // and second seq loads: first copy sees seq change, second
        // succeeds.
        let seq = AtomicU32::new(0);
        let mut calls = 0;
        let got = seqlock_read(&seq, || {
            calls += 1;
            if calls == 1 {
                // A full write lands during the first copy.
                seq.fetch_add(2, Ordering::Release);
            }
            calls
        });
        assert_eq!(got, Some(2), "first snapshot must be discarded");
    }

    #[test]
    fn write_toggles_odd_then_even() {
        let seq = AtomicU32::new(0);
        seqlock_write(&seq, || {
            assert_eq!(seq.load(Ordering::Relaxed) & 1, 1, "odd while writing");
        });
        assert_eq!(seq.load(Ordering::Relaxed), 2, "even after writing");
    }

    #[test]
    fn concurrent_reader_never_sees_torn_pair() {
        // Two u64 lanes written under the lock must always read as a
        // matched pair. std threads are available to tests even in a
        // no_std crate.
        extern crate std;
        use std::sync::atomic::AtomicBool;
        use std::sync::Arc;

        let seq = Arc::new(AtomicU32::new(0));
        let a = Arc::new(AtomicU64::new(0));
        let b = Arc::new(AtomicU64::new(0));
        let stop = Arc::new(AtomicBool::new(false));

        let w = {
            let (seq, a, b, stop) = (seq.clone(), a.clone(), b.clone(), stop.clone());
            std::thread::spawn(move || {
                let mut v = 0_u64;
                while !stop.load(Ordering::Relaxed) {
                    v = v.wrapping_add(1);
                    seqlock_write(&seq, || {
                        a.store(v, Ordering::Relaxed);
                        b.store(v.wrapping_mul(2), Ordering::Relaxed);
                    });
                }
            })
        };

        let mut consistent_reads = 0;
        while consistent_reads < 10_000 {
            if let Some((x, y)) = seqlock_read(&seq, || {
                (a.load(Ordering::Relaxed), b.load(Ordering::Relaxed))
            }) {
                assert_eq!(y, x.wrapping_mul(2), "torn read escaped the seqlock");
                consistent_reads += 1;
            }
        }
        stop.store(true, Ordering::Relaxed);
        w.join().unwrap();
    }
}
