//! The versioned POSIX shm namespace for the gz-bridge block — the
//! single naming authority every endpoint derives from.
//!
//! WHY the name carries a version: the object name is the
//! rendezvous point, and any binary that ever knew it can
//! `shm_open` it read-write and WRITE before validating anything.
//! The attach-side fingerprint gate protects readers that opt in;
//! it cannot stop a stale writer built against an older layout from
//! scribbling at obsolete offsets — that writer predates the rule
//! it is breaking. Baking the contract major into the name makes
//! the collision impossible by construction: a pre-v3 binary
//! resolves `/aviate_gz_bridge` to nothing (or to its own dead
//! leftover) and can never reach this block.
//!
//! VERSIONING POLICY: the `_vN` suffix is the incompatible shm
//! contract major — [`crate::LAYOUT_VERSION`] — not an Aviate
//! release number. Compatible releases keep `_v3`; the next
//! breaking contract bump renames the namespace to `_v4` in the
//! same change that bumps `LAYOUT_VERSION` (a test pins the two
//! together). The unversioned name is retired: no endpoint may
//! create it, open it, or fall back to it.

/// Base POSIX shm object name. This IS instance 0's full name;
/// instance N > 0 appends `_N` — always via [`shm_name`], never by
/// hand. The C++ side derives the same name from the generated
/// header's `AviateSHM_NAME_BASE` / `aviate_shm_instance_name`.
pub const SHM_NAME_BASE: &str = "/aviate_gz_bridge_v3";

/// Strictest POSIX shm name limit across supported platforms:
/// macOS `PSHMNAMLEN` (31), which counts the leading slash —
/// `shm_open` rejects longer names with `ENAMETOOLONG`.
pub const SHM_NAME_MAX: usize = 31;

// Every u32 instance must fit: base + '_' + at most 10 decimal
// digits. This bound is what makes [`shm_name`] total (and its
// buffer writes provably in range); the worst case is pinned
// byte-exact by test.
const _: () = assert!(SHM_NAME_BASE.len() + 1 + 10 <= SHM_NAME_MAX);

/// An instance's shm object name, built without allocation so the
/// authority stays usable from `no_std` endpoints.
#[derive(Clone, Copy)]
pub struct ShmName {
    buf: [u8; SHM_NAME_MAX],
    len: usize,
}

impl ShmName {
    /// The name as the string `shm_open` takes.
    pub fn as_str(&self) -> &str {
        // Construction writes only ASCII (the base literal, '_',
        // decimal digits); the fallback is unreachable and exists
        // to keep this total without `unsafe`.
        core::str::from_utf8(&self.buf[..self.len]).unwrap_or(SHM_NAME_BASE)
    }
}

impl core::ops::Deref for ShmName {
    type Target = str;
    fn deref(&self) -> &str {
        self.as_str()
    }
}

impl AsRef<str> for ShmName {
    fn as_ref(&self) -> &str {
        self.as_str()
    }
}

impl core::fmt::Display for ShmName {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl core::fmt::Debug for ShmName {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        core::fmt::Debug::fmt(self.as_str(), f)
    }
}

/// Canonical shm object name for a bridge instance: `0` →
/// [`SHM_NAME_BASE`], `N` → `<base>_N`.
///
/// Total over all of `u32`: the worst case (`u32::MAX`) lands
/// exactly on [`SHM_NAME_MAX`] bytes, so no instance can produce a
/// name the strictest platform rejects.
pub fn shm_name(instance: u32) -> ShmName {
    let base = SHM_NAME_BASE.as_bytes();
    let mut buf = [0u8; SHM_NAME_MAX];
    let mut len = base.len();
    buf[..len].copy_from_slice(base);
    if instance > 0 {
        buf[len] = b'_';
        len += 1;
        // u32::MAX has 10 decimal digits.
        let mut digits = [0u8; 10];
        let mut v = instance;
        let mut n = 0;
        while v > 0 {
            digits[n] = b'0' + (v % 10) as u8;
            v /= 10;
            n += 1;
        }
        while n > 0 {
            n -= 1;
            buf[len] = digits[n];
            len += 1;
        }
    }
    ShmName { buf, len }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use crate::LAYOUT_VERSION;

    #[test]
    fn instance_zero_is_the_base_name() {
        assert_eq!(shm_name(0).as_str(), SHM_NAME_BASE);
    }

    #[test]
    fn instances_append_the_decimal_suffix() {
        assert_eq!(shm_name(1).as_str(), "/aviate_gz_bridge_v3_1");
        assert_eq!(shm_name(15).as_str(), "/aviate_gz_bridge_v3_15");
        assert_eq!(shm_name(4096).as_str(), "/aviate_gz_bridge_v3_4096");
    }

    #[test]
    fn maximum_instance_fits_the_macos_limit_exactly() {
        let name = shm_name(u32::MAX);
        assert_eq!(name.as_str(), "/aviate_gz_bridge_v3_4294967295");
        assert_eq!(name.as_str().len(), SHM_NAME_MAX);
    }

    #[test]
    fn namespace_version_tracks_layout_version() {
        // The `_vN` suffix IS the contract major: a LAYOUT_VERSION
        // bump that forgets to move the namespace fails here, and so
        // does a rename that forgets the version bump.
        let at = SHM_NAME_BASE.rfind("_v").unwrap();
        let major: u32 = SHM_NAME_BASE[at + 2..].parse().unwrap();
        assert_eq!(major, LAYOUT_VERSION);
    }

    #[test]
    fn no_instance_name_reuses_the_retired_unversioned_namespace() {
        // "/aviate_gz_bridge" and "/aviate_gz_bridge_1" belong to
        // pre-v3 binaries; nothing this module can produce may
        // collide with them.
        for instance in [0, 1, 15, u32::MAX] {
            assert_ne!(shm_name(instance).as_str(), "/aviate_gz_bridge");
            assert_ne!(shm_name(instance).as_str(), "/aviate_gz_bridge_1");
        }
    }
}
