//! Binds an authenticated principal to a command source (authority).
//!
//! The authority a command carries — RC, GCS/datalink, or offboard — is a
//! property of *which principal authenticated it*, never of anything the
//! payload claims. [`SourcePolicy`] is the gateway-owned table mapping a
//! [`Principal`] to a [`CommandSource`]. It is scheme-neutral: a MAVLink
//! signing principal and a future AEAD principal are authorized by the same
//! table, and the scheme tag inside the principal keeps them from colliding.
//!
//! A credential is a single secret (for MAVLink, one `link_id` key). It
//! cannot honestly speak for two different authorities, so the policy holds
//! at most ONE binding per `(scheme, credential)` — a second binding on the
//! same credential is rejected at configuration time.

use super::receipt::CommandSource;
use crate::principal::Principal;

/// Maximum number of principal→source bindings a policy holds.
///
/// Sized for an inner-loop flight controller's small set of authenticated
/// peers (an RC bridge, a GCS/datalink, an offboard companion).
pub const MAX_SOURCE_BINDINGS: usize = 8;

/// One `Principal` → [`CommandSource`] binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Binding {
    principal: Principal,
    source: CommandSource,
}

/// Configuration-time authorization errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CredentialError {
    /// Every binding slot is occupied.
    TableFull,
    /// A binding for this principal's `(scheme, credential)` already
    /// exists. One credential is one secret and speaks for one authority;
    /// a second binding on the same credential would let the key holder
    /// choose between authorities. Carries the offending credential id.
    DuplicateCredential {
        /// `Principal::credential()` of the binding that was rejected.
        credential: u16,
    },
}

/// Maps authenticated principals to their authorized command source.
///
/// Construct with [`SourcePolicy::new`] and add bindings with
/// [`SourcePolicy::bind`].
#[derive(Debug, Clone)]
pub struct SourcePolicy {
    bindings: [Option<Binding>; MAX_SOURCE_BINDINGS],
}

impl SourcePolicy {
    /// An empty policy: no principal is authorized until bound.
    pub const fn new() -> Self {
        Self {
            bindings: [None; MAX_SOURCE_BINDINGS],
        }
    }

    /// Authorize `principal` as `source`.
    ///
    /// Rejects a principal whose `(scheme, credential)` is already bound
    /// ([`CredentialError::DuplicateCredential`]) — one credential, one
    /// authority — and reports exhaustion ([`CredentialError::TableFull`])
    /// instead of silently dropping. Rotating a binding means rebuilding
    /// the policy, not editing it in place.
    pub fn bind(
        &mut self,
        principal: Principal,
        source: CommandSource,
    ) -> Result<(), CredentialError> {
        if self.bindings.iter().flatten().any(|b| {
            b.principal.scheme() == principal.scheme()
                && b.principal.credential() == principal.credential()
        }) {
            return Err(CredentialError::DuplicateCredential {
                credential: principal.credential(),
            });
        }
        match self.bindings.iter_mut().find(|slot| slot.is_none()) {
            Some(free) => {
                *free = Some(Binding { principal, source });
                Ok(())
            }
            None => Err(CredentialError::TableFull),
        }
    }

    /// Resolve the authorized source for an authenticated principal.
    ///
    /// The match is exact (scheme, credential, and asserted identity), so a
    /// command authenticated under one credential but claiming a different
    /// identity resolves to `None` — possession of a credential does not
    /// permit impersonating a different sender.
    pub fn resolve(&self, principal: Principal) -> Option<CommandSource> {
        self.bindings
            .iter()
            .flatten()
            .find_map(|b| (b.principal == principal).then_some(b.source))
    }
}

impl Default for SourcePolicy {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[allow(clippy::panic, clippy::expect_used)]
mod tests {
    use super::*;

    fn mav(system_id: u8, component_id: u8, link_id: u8) -> Principal {
        Principal::mavlink(system_id, component_id, link_id)
    }

    fn policy_with(bindings: &[(Principal, CommandSource)]) -> SourcePolicy {
        let mut policy = SourcePolicy::new();
        for (p, s) in bindings {
            policy.bind(*p, *s).expect("bind");
        }
        policy
    }

    #[test]
    fn resolves_bound_and_rejects_unbound() {
        let policy = policy_with(&[(mav(1, 1, 5), CommandSource::GcsDatalink)]);
        assert_eq!(
            policy.resolve(mav(1, 1, 5)),
            Some(CommandSource::GcsDatalink)
        );
        assert_eq!(policy.resolve(mav(1, 1, 6)), None);
        assert_eq!(policy.resolve(mav(2, 1, 5)), None);
    }

    /// One credential, one authority: a second binding on the same key slot
    /// (link_id) is refused, so the key holder cannot pick between
    /// authorities.
    #[test]
    fn second_binding_on_same_credential_rejected() {
        let mut policy = SourcePolicy::new();
        assert!(policy.bind(mav(1, 1, 5), CommandSource::Rc).is_ok());
        assert_eq!(
            policy.bind(mav(1, 2, 5), CommandSource::GcsDatalink),
            Err(CredentialError::DuplicateCredential { credential: 5 })
        );
        assert_eq!(policy.resolve(mav(1, 1, 5)), Some(CommandSource::Rc));
        assert_eq!(policy.resolve(mav(1, 2, 5)), None);
    }

    /// The identity check is exact: the key slot alone is not authority.
    #[test]
    fn key_possession_does_not_grant_identity_choice() {
        let policy = policy_with(&[(mav(1, 1, 5), CommandSource::Rc)]);
        assert_eq!(policy.resolve(mav(2, 1, 5)), None);
        assert_eq!(policy.resolve(mav(1, 2, 5)), None);
        assert_eq!(policy.resolve(mav(1, 1, 5)), Some(CommandSource::Rc));
    }

    #[test]
    fn distinct_credentials_carry_distinct_authorities() {
        let policy = policy_with(&[
            (mav(1, 1, 5), CommandSource::Rc),
            (mav(1, 2, 6), CommandSource::Offboard),
        ]);
        assert_eq!(policy.resolve(mav(1, 1, 5)), Some(CommandSource::Rc));
        assert_eq!(policy.resolve(mav(1, 2, 6)), Some(CommandSource::Offboard));
        assert_eq!(policy.resolve(mav(1, 2, 5)), None);
        assert_eq!(policy.resolve(mav(1, 1, 6)), None);
    }

    #[test]
    fn bind_reports_capacity_exhaustion() {
        let mut policy = SourcePolicy::new();
        for i in 0..MAX_SOURCE_BINDINGS as u8 {
            assert!(policy.bind(mav(1, 1, i), CommandSource::Rc).is_ok());
        }
        assert_eq!(
            policy.bind(mav(9, 9, 200), CommandSource::Rc),
            Err(CredentialError::TableFull)
        );
    }
}
