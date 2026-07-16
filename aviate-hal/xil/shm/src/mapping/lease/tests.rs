//! Errno-classification and liveness-verdict regression tests.
//!
//! The classification is a pure function precisely so every errno
//! branch — including `EINTR`, which a non-blocking call cannot be
//! made to produce on demand — has a direct test instead of relying
//! on provoking the kernel.

use std::os::unix::fs::PermissionsExt;

use super::{
    classify_flock, flock_nb, lease_path, token_path, writer_liveness, FlockOutcome, FlockVerdict,
    WriterLease, WriterLiveness,
};

fn unique_name(tag: &str) -> String {
    use std::sync::atomic::{AtomicU32, Ordering};
    static COUNTER: AtomicU32 = AtomicU32::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("/avt_ls_{tag}_{}_{n}", std::process::id())
}

/// Poll until the probe converges on a fully-dead verdict. The flip
/// after a release is not instantaneous under load: a fork() in ANY
/// thread of this test binary (the cross-process tests spawn
/// children) pins a just-released lock until the child execs, and
/// the kernel offers no event to wait on for that — so convergence
/// is a bounded poll, exactly like the consumers built on it.
fn assert_converges_to_dead(name: &str, incarnation: u64) {
    let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
    loop {
        match writer_liveness(name, incarnation) {
            WriterLiveness::Dead {
                takeover_in_progress: false,
            } => break,
            WriterLiveness::Alive
            | WriterLiveness::Dead {
                takeover_in_progress: true,
            } if std::time::Instant::now() < deadline => {
                std::thread::sleep(std::time::Duration::from_millis(4));
            }
            other => panic!("a released lease must converge to Dead, got {other:?}"),
        }
    }
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
fn liveness_is_per_incarnation_not_per_name() {
    let name = unique_name("lv");
    // No lease has ever existed: every incarnation is dead.
    assert!(matches!(
        writer_liveness(&name, 1),
        WriterLiveness::Dead {
            takeover_in_progress: false
        }
    ));

    let lease = WriterLease::acquire(&name).unwrap();
    let granted = lease.incarnation();
    // The probe answers for ONE writer. The held token vouches for
    // the incarnation this grant received — and for no other, or
    // callers could not tell "my writer lives" from "somebody
    // lives".
    assert!(matches!(
        writer_liveness(&name, granted),
        WriterLiveness::Alive
    ));
    match writer_liveness(&name, granted.wrapping_sub(1)) {
        WriterLiveness::Dead {
            takeover_in_progress,
        } => assert!(
            takeover_in_progress,
            "the held global lease must qualify a dead predecessor as replaced"
        ),
        other => panic!("a foreign incarnation read as {other:?}, not Dead"),
    }

    // Release: the lease file remains (it carries the counter and is
    // never unlinked), and the verdict flips to fully dead.
    drop(lease);
    assert_converges_to_dead(&name, granted);
}

#[test]
fn a_grant_paused_before_its_counter_write_probes_dead() {
    // THE takeover window at its widest: the successor holds the
    // global lease but has not advanced the counter, so the lease
    // file still carries the predecessor's incarnation. A verdict
    // built on "the lock is held, and the counter says whose it is"
    // reads the corpse's own number back and calls it alive; the
    // token verdict must read Dead throughout.
    let name = unique_name("pw");
    let predecessor = WriterLease::acquire(&name).unwrap();
    let corpse = predecessor.incarnation();
    drop(predecessor); // the "crash"; the kernel released its locks

    let paused = WriterLease::acquire_global_only_for_test(&name).unwrap();
    match writer_liveness(&name, corpse) {
        WriterLiveness::Dead {
            takeover_in_progress,
        } => assert!(
            takeover_in_progress,
            "the paused grant holds the global lease; the death is a takeover"
        ),
        other => panic!("the pre-counter window revived the corpse: {other:?}"),
    }

    // The window closes: the paused grant completes as a real one
    // and only ITS incarnation reads alive.
    drop(paused);
    let successor = WriterLease::acquire(&name).unwrap();
    assert_eq!(successor.incarnation(), corpse.wrapping_add(1));
    assert!(matches!(
        writer_liveness(&name, successor.incarnation()),
        WriterLiveness::Alive
    ));
    assert!(matches!(
        writer_liveness(&name, corpse),
        WriterLiveness::Dead {
            takeover_in_progress: true
        }
    ));
}

#[test]
fn an_unreadable_token_file_is_unknown_not_a_verdict() {
    // SAFETY: geteuid has no preconditions.
    if unsafe { libc::geteuid() } == 0 {
        // Root bypasses file modes; the EACCES this test constructs
        // cannot occur.
        return;
    }
    let name = unique_name("ur");
    let path = token_path(&name, 7);
    std::fs::write(&path, b"").unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o000)).unwrap();

    match writer_liveness(&name, 7) {
        WriterLiveness::Unknown(e) => {
            assert_eq!(e.raw_os_error(), Some(libc::EACCES));
        }
        WriterLiveness::Alive => panic!("an unreadable token read as a live writer"),
        WriterLiveness::Dead { .. } => {
            panic!("an unreadable token read as a dead writer — data would flow on a broken probe")
        }
    }

    // The same discipline for the global lease: with the token
    // absent the writer is provably dead, but the takeover question
    // is answered by a probe that cannot run — Unknown, not a guess.
    let name2 = unique_name("ug");
    let global = lease_path(&name2);
    std::fs::write(&global, [0u8; 8]).unwrap();
    std::fs::set_permissions(&global, std::fs::Permissions::from_mode(0o000)).unwrap();
    match writer_liveness(&name2, 1) {
        WriterLiveness::Unknown(e) => {
            assert_eq!(e.raw_os_error(), Some(libc::EACCES));
        }
        other => panic!("an unreadable global lease read as a verdict: {other:?}"),
    }

    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o644)).unwrap();
    std::fs::set_permissions(&global, std::fs::Permissions::from_mode(0o644)).unwrap();
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
    // The successor retired its predecessor's token file: the value
    // can never be granted again, so the file could only accumulate.
    assert!(
        !std::path::Path::new(&token_path(&name, a)).exists(),
        "a grant must unlink its predecessor's token"
    );
    drop(second);

    // Wrap: a counter at u64::MAX advances past the reserved zero.
    std::fs::write(lease_path(&name), u64::MAX.to_le_bytes()).unwrap();
    let wrapped = WriterLease::acquire(&name).unwrap();
    assert_eq!(wrapped.incarnation(), 1);
}
