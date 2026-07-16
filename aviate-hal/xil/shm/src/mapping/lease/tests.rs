//! Errno-classification and liveness-verdict regression tests.
//!
//! The classification is a pure function precisely so every errno
//! branch — including `EINTR`, which a non-blocking call cannot be
//! made to produce on demand — has a direct test instead of relying
//! on provoking the kernel.

use std::os::unix::fs::PermissionsExt;

use super::{
    classify_flock, flock_nb, lease_path, writer_liveness, FlockOutcome, FlockVerdict, WriterLease,
    WriterLiveness,
};

fn unique_name(tag: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/avt_ls_{tag}_{}_{n}", std::process::id())
}

#[test]
fn every_errno_class_has_exactly_one_meaning() {
    assert_eq!(classify_flock(0, 0), FlockVerdict::Acquired);
    // A signal is not a verdict about the lock.
    assert_eq!(classify_flock(-1, libc::EINTR), FlockVerdict::Interrupted);
    // ONLY these two mean "a live exclusive holder exists".
    assert_eq!(classify_flock(-1, libc::EWOULDBLOCK), FlockVerdict::Held);
    assert_eq!(classify_flock(-1, libc::EAGAIN), FlockVerdict::Held);
    // Everything else is a broken probe, never a held lease: each of
    // these once collapsed into the same `-1` as Held, which turned
    // an environment fault into "another writer owns this name".
    assert_eq!(classify_flock(-1, libc::EBADF), FlockVerdict::Failed);
    assert_eq!(classify_flock(-1, libc::ENOLCK), FlockVerdict::Failed);
    assert_eq!(classify_flock(-1, libc::EINVAL), FlockVerdict::Failed);
    assert_eq!(classify_flock(-1, libc::EOPNOTSUPP), FlockVerdict::Failed);
}

#[test]
fn a_bad_descriptor_is_a_failure_not_a_held_lease() {
    // End-to-end through the real syscall: EBADF must classify as
    // Failed. Duplicate stdin to a HIGH descriptor and close it —
    // the kernel hands out lowest-available numbers, so no
    // concurrently running test can reclaim this one between the
    // close and the probe.
    // SAFETY: plain descriptor bookkeeping on a number this test
    // owns.
    let outcome = unsafe {
        let fd = libc::fcntl(0, libc::F_DUPFD_CLOEXEC, 923);
        assert!(fd >= 923, "fcntl(F_DUPFD_CLOEXEC) failed");
        libc::close(fd);
        flock_nb(fd, libc::LOCK_EX)
    };
    match outcome {
        FlockOutcome::Failed(e) => {
            assert_eq!(e.raw_os_error(), Some(libc::EBADF));
        }
        FlockOutcome::Acquired => panic!("a closed fd granted a lock"),
        FlockOutcome::Held => panic!("EBADF read as a held lease"),
    }
}

#[test]
fn liveness_is_alive_only_while_the_exclusive_lock_is_held() {
    let name = unique_name("lv");
    // No lease file yet: no writer has ever run.
    assert!(matches!(writer_liveness(&name), WriterLiveness::Dead));

    let lease = WriterLease::acquire(&name).unwrap();
    // Alive carries WHICH writer is alive: the probe's counter must
    // be the very incarnation this grant received, or callers could
    // not tell "my writer lives" from "somebody lives".
    match writer_liveness(&name) {
        WriterLiveness::Alive(incarnation) => assert_eq!(incarnation, lease.incarnation()),
        other => panic!("a held lease must probe Alive, got {other:?}"),
    }

    // Release: the file remains (it carries the counter and is
    // never unlinked), and the verdict flips to Dead. The flip is
    // not instantaneous under load: a fork() in ANY thread of this
    // test binary (the cross-process tests spawn children) pins the
    // just-released lock until the child execs, and the kernel
    // offers no event to wait on for that — so this convergence
    // check is a bounded poll, exactly like the consumers built on
    // it.
    drop(lease);
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        match writer_liveness(&name) {
            WriterLiveness::Dead => break,
            WriterLiveness::Alive(_) if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(4));
            }
            other => panic!("a released lease must converge to Dead, got {other:?}"),
        }
    }
}

#[test]
fn an_unreadable_lease_file_is_unknown_not_a_verdict() {
    // SAFETY: geteuid has no preconditions.
    if unsafe { libc::geteuid() } == 0 {
        // Root bypasses file modes; the EACCES this test constructs
        // cannot occur.
        return;
    }
    let name = unique_name("ur");
    let path = lease_path(&name);
    std::fs::write(&path, [0u8; 8]).unwrap();
    let mode_zero = std::fs::Permissions::from_mode(0o000);
    std::fs::set_permissions(&path, mode_zero).unwrap();

    match writer_liveness(&name) {
        WriterLiveness::Unknown(e) => {
            assert_eq!(e.raw_os_error(), Some(libc::EACCES));
        }
        WriterLiveness::Alive(_) => panic!("an unreadable lease read as a live writer"),
        WriterLiveness::Dead => {
            panic!("an unreadable lease read as a dead writer — data would flow on a broken probe")
        }
    }

    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
}

#[test]
fn the_lease_counter_is_monotonic_and_skips_zero() {
    let name = unique_name("ct");
    let first = WriterLease::acquire(&name).unwrap();
    let a = first.incarnation();
    assert_ne!(a, 0);
    drop(first);
    let second = WriterLease::acquire(&name).unwrap();
    assert_eq!(second.incarnation(), a.wrapping_add(1));
    drop(second);

    // Wrap: a counter at u64::MAX advances past the reserved zero.
    std::fs::write(lease_path(&name), u64::MAX.to_le_bytes()).unwrap();
    let wrapped = WriterLease::acquire(&name).unwrap();
    assert_eq!(wrapped.incarnation(), 1);
}
