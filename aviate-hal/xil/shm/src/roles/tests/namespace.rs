//! Namespace isolation: a stale pre-v3 world on the retired
//! unversioned rendezvous must coexist with the versioned block —
//! the versioned writer must not open, modify, or be blocked by it,
//! and no current consumer may fall back to it.
//!
//! WHY: the attach fingerprint protects readers that opt in, but a
//! stale WRITER built against an older layout writes at obsolete
//! offsets before any validation could reject it — the observed
//! failure was pre-v3 rotor speeds landing inside the v3 quaternion
//! lanes. The structural defence is that the versioned name is a
//! different rendezvous entirely; this test pins that isolation.
//!
//! The name pair is TEST-UNIQUE (`_<pid>` instance on both sides):
//! this test must never unlink, lease, or otherwise disturb the
//! canonical instance-0 rendezvous of a simulator that happens to be
//! live on this host. What the regression pins is the
//! unversioned/versioned RELATIONSHIP between the two rules; the
//! canonical strings themselves are pinned by the contract crate's
//! naming tests.

use std::os::fd::AsRawFd;

use aviate_xil_contract::shm_name;

use super::super::{ConsumerSession, FcSession, ModelStateSnapshot, SimWriterSession};
use crate::AttachFailure;

const LEGACY_SIZE: usize = 448;
const SENTINEL: u8 = 0xAB;

/// The retired pre-v3 naming rule, spelled out by hand ON PURPOSE:
/// a stale binary's knowledge must stay independent of the current
/// naming authority, or the regression proves nothing.
fn legacy_name(instance: u32) -> String {
    format!("/aviate_gz_bridge_{instance}")
}

/// The lease-path rule as the legacy writer derived it (identical
/// derivation, applied to the retired name).
fn legacy_lease_path(name: &str) -> String {
    format!("/tmp/{}.lease", name.trim_start_matches('/'))
}

/// A stale legacy writer in miniature: it holds its lease for its
/// whole life (acquired BEFORE the object exists, like any real
/// writer), owns a sentinel-filled object it created itself, and
/// tears down in writer order — unmap and unlink the owned object
/// while the lease is still held, release the lease last.
struct LegacyWriter {
    base: *mut libc::c_void,
    name: std::ffi::CString,
    /// Held exclusively for the struct's whole life. Fields drop
    /// AFTER `Drop::drop` runs, so the flock outlives the unlink and
    /// no successor can acquire the name and have ITS object
    /// unlinked. The lease file itself is never unlinked — removing
    /// a lock file races its next locker.
    _lease: std::fs::File,
}

impl LegacyWriter {
    fn create(name: &str) -> Self {
        // Lease FIRST, exactly like a real writer: nothing at this
        // name may be created, let alone unlinked, without owning it.
        let lease = std::fs::OpenOptions::new()
            .create(true)
            .truncate(false)
            .write(true)
            .open(legacy_lease_path(name))
            .expect("open legacy lease file");
        // SAFETY: advisory flock on an fd this struct owns.
        let locked = unsafe { libc::flock(lease.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
        assert_eq!(locked, 0, "the test-unique legacy lease must be free");

        let cname = std::ffi::CString::new(name).expect("literal has no NUL");
        // SAFETY: plain libc shm creation. O_EXCL is load-bearing:
        // this test only ever destroys an object it created itself —
        // a pre-existing object at the name is someone else's and
        // must FAIL the test, never be unlinked.
        unsafe {
            let fd = libc::shm_open(
                cname.as_ptr(),
                libc::O_CREAT | libc::O_EXCL | libc::O_RDWR,
                0o600 as libc::c_uint,
            );
            assert!(
                fd != -1,
                "legacy shm_open failed: {}",
                std::io::Error::last_os_error()
            );
            assert!(libc::ftruncate(fd, LEGACY_SIZE as libc::off_t) != -1);
            let ptr = libc::mmap(
                core::ptr::null_mut(),
                LEGACY_SIZE,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            assert!(ptr != libc::MAP_FAILED, "legacy mmap failed");
            core::ptr::write_bytes(ptr.cast::<u8>(), SENTINEL, LEGACY_SIZE);
            Self {
                base: ptr,
                name: cname,
                _lease: lease,
            }
        }
    }

    /// Re-open and re-map the object BY NAME: what the legacy
    /// rendezvous resolves to right now, not what this test mapped
    /// earlier.
    fn bytes_by_name(&self) -> Vec<u8> {
        // SAFETY: fresh read-only mapping of the named object,
        // unmapped before returning.
        unsafe {
            let fd = libc::shm_open(self.name.as_ptr(), libc::O_RDONLY, 0);
            assert!(fd != -1, "the legacy name no longer resolves");
            let ptr = libc::mmap(
                core::ptr::null_mut(),
                LEGACY_SIZE,
                libc::PROT_READ,
                libc::MAP_SHARED,
                fd,
                0,
            );
            libc::close(fd);
            assert!(ptr != libc::MAP_FAILED);
            let bytes = core::slice::from_raw_parts(ptr.cast::<u8>(), LEGACY_SIZE).to_vec();
            libc::munmap(ptr, LEGACY_SIZE);
            bytes
        }
    }
}

impl Drop for LegacyWriter {
    fn drop(&mut self) {
        // Writer teardown order: destroy the OWNED object while the
        // lease is still held; `_lease` drops after this body,
        // releasing the flock last.
        // SAFETY: this struct created both the mapping and the name.
        unsafe {
            libc::munmap(self.base, LEGACY_SIZE);
            libc::shm_unlink(self.name.as_ptr());
        }
    }
}

#[test]
fn versioned_namespace_isolates_unversioned_writer() {
    // Test-unique instance. Instance 0 IS the canonical rendezvous,
    // so a zero here would defeat the whole point of the pid.
    let instance = std::process::id();
    assert_ne!(instance, 0, "pid 0 would collide with the canonical names");

    let legacy = legacy_name(instance);
    let stale_writer = LegacyWriter::create(&legacy);

    // Different name, different lease: versioned creation must
    // succeed while the legacy world is fully "alive" (lease held).
    let versioned = shm_name(instance);
    let writer = SimWriterSession::create(&versioned)
        .expect("versioned create must not touch the legacy world");
    writer.write_model_state(&ModelStateSnapshot {
        reset_generation: writer.reset_generation(),
        sim_step: 1,
        time_us: 1_000,
        pos: [1.0, 2.0, 3.0],
        quat: [1.0, 0.0, 0.0, 0.0],
        vel: [0.1, 0.2, 0.3],
        ang_vel: [0.0, 0.0, 0.0],
    });

    // The observed corruption vector, replayed on the safe side:
    // rotor speeds that once landed inside pre-v3 quaternion lanes
    // now land in the versioned object's command block — and
    // nowhere else.
    let fc = FcSession::attach(&versioned).expect("FC attaches to the versioned name");
    fc.write_motor_command(&[733.0, 806.0, 733.0, 806.0]);

    // The legacy object is byte-for-byte as the stale world left it,
    // and its name still resolves to it.
    assert!(
        stale_writer.bytes_by_name().iter().all(|&b| b == SENTINEL),
        "the versioned side wrote into the legacy object"
    );

    // Handing a consumer the legacy name explicitly fails closed —
    // no fallback, no repair, no partial read.
    assert!(matches!(
        ConsumerSession::attach(&legacy),
        Err(AttachFailure::Contract(_))
    ));

    // And the versioned side stayed fully live throughout.
    let (velocities, count) = writer
        .read_motor_command()
        .expect("versioned command round-trips");
    assert_eq!(count, 4);
    assert_eq!(velocities[..4], [733.0, 806.0, 733.0, 806.0]);
}
