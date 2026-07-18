//! Binds an authenticated signing identity to a command source.
//!
//! The authority a command carries — RC, GCS/datalink, or offboard — is a
//! property of *which credential authenticated it*, never of anything the
//! payload claims. [`SourcePolicy`] is the gateway-owned table that maps a
//! verified `(system_id, component_id, link_id)` identity to a
//! [`CommandSource`]. An identity with no binding is unauthorized, so a
//! valid signature from an unexpected peer cannot pick its own authority.

use super::receipt::CommandSource;

/// Maximum number of identity→source bindings a policy holds.
///
/// Sized for an inner-loop flight controller's small set of authenticated
/// peers (an RC bridge, a GCS/datalink, an offboard companion).
pub const MAX_SOURCE_BINDINGS: usize = 8;

/// One `(system_id, component_id, link_id)` → [`CommandSource`] binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourceBinding {
    system_id: u8,
    component_id: u8,
    link_id: u8,
    source: CommandSource,
}

/// Maps authenticated signing identities to their authorized source.
///
/// Construct a flight policy with [`SourcePolicy::flight`] and add bindings
/// with [`SourcePolicy::bind`]. A development policy
/// ([`SourcePolicy::insecure_dev`]) additionally assigns a source to
/// unsigned frames — never use it in a flight assembly.
#[derive(Debug, Clone)]
pub struct SourcePolicy {
    bindings: [Option<SourceBinding>; MAX_SOURCE_BINDINGS],
    /// Source assigned to unsigned frames. `None` in a flight policy, so
    /// unsigned traffic is never authorized.
    unsigned_source: Option<CommandSource>,
}

impl SourcePolicy {
    /// A flight policy: no bindings yet, and unsigned frames unauthorized.
    pub const fn flight() -> Self {
        Self {
            bindings: [None; MAX_SOURCE_BINDINGS],
            unsigned_source: None,
        }
    }

    /// A development/SITL policy that authorizes unsigned frames as
    /// `unsigned_source`.
    ///
    /// ## Security Warning
    ///
    /// This authorizes traffic that carries no credential. It MUST NOT be
    /// used in a flight assembly; keep it behind the same non-flight gate as
    /// [`PlainAuth`](crate::PlainAuth).
    pub const fn insecure_dev(unsigned_source: CommandSource) -> Self {
        Self {
            bindings: [None; MAX_SOURCE_BINDINGS],
            unsigned_source: Some(unsigned_source),
        }
    }

    /// Bind an authenticated identity to a source.
    ///
    /// Returns `Err(source)` (handing the value back) if the table is full.
    /// A later bind of an already-bound identity overwrites it.
    pub fn bind(
        &mut self,
        system_id: u8,
        component_id: u8,
        link_id: u8,
        source: CommandSource,
    ) -> Result<(), CommandSource> {
        let new = SourceBinding {
            system_id,
            component_id,
            link_id,
            source,
        };
        // Overwrite an existing binding for the same identity.
        for existing in self.bindings.iter_mut().flatten() {
            if existing.system_id == system_id
                && existing.component_id == component_id
                && existing.link_id == link_id
            {
                *existing = new;
                return Ok(());
            }
        }
        // Otherwise take a free slot.
        match self.bindings.iter_mut().find(|slot| slot.is_none()) {
            Some(free) => {
                *free = Some(new);
                Ok(())
            }
            None => Err(source),
        }
    }

    /// Builder form of [`Self::bind`] that ignores overflow (drops the
    /// binding if the table is full). Convenient for `const`-ish setup.
    #[must_use]
    pub fn with_binding(
        mut self,
        system_id: u8,
        component_id: u8,
        link_id: u8,
        source: CommandSource,
    ) -> Self {
        let _ = self.bind(system_id, component_id, link_id, source);
        self
    }

    /// Resolve the authorized source for a verified identity, if bound.
    pub fn resolve(&self, system_id: u8, component_id: u8, link_id: u8) -> Option<CommandSource> {
        self.bindings.iter().find_map(|slot| match slot {
            Some(b)
                if b.system_id == system_id
                    && b.component_id == component_id
                    && b.link_id == link_id =>
            {
                Some(b.source)
            }
            _ => None,
        })
    }

    /// The source assigned to unsigned frames, if this is a dev policy.
    pub fn unsigned_source(&self) -> Option<CommandSource> {
        self.unsigned_source
    }
}

#[cfg(test)]
#[allow(clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn flight_policy_rejects_unbound_identity_and_unsigned() {
        let policy = SourcePolicy::flight().with_binding(1, 1, 5, CommandSource::GcsDatalink);
        assert_eq!(policy.resolve(1, 1, 5), Some(CommandSource::GcsDatalink));
        // A different identity is unbound → unauthorized.
        assert_eq!(policy.resolve(1, 1, 6), None);
        assert_eq!(policy.resolve(2, 1, 5), None);
        // Flight policy never authorizes unsigned traffic.
        assert_eq!(policy.unsigned_source(), None);
    }

    #[test]
    fn identity_is_the_full_tuple() {
        let policy = SourcePolicy::flight()
            .with_binding(1, 1, 5, CommandSource::Rc)
            .with_binding(1, 2, 5, CommandSource::GcsDatalink);
        // Same link_id, different component_id → distinct authorities.
        assert_eq!(policy.resolve(1, 1, 5), Some(CommandSource::Rc));
        assert_eq!(policy.resolve(1, 2, 5), Some(CommandSource::GcsDatalink));
    }

    #[test]
    fn dev_policy_authorizes_unsigned() {
        let policy = SourcePolicy::insecure_dev(CommandSource::Offboard);
        assert_eq!(policy.unsigned_source(), Some(CommandSource::Offboard));
    }

    #[test]
    fn bind_reports_capacity_exhaustion() {
        let mut policy = SourcePolicy::flight();
        for i in 0..MAX_SOURCE_BINDINGS as u8 {
            assert!(policy.bind(1, 1, i, CommandSource::Rc).is_ok());
        }
        assert_eq!(
            policy.bind(9, 9, 9, CommandSource::Rc),
            Err(CommandSource::Rc)
        );
        // Re-binding an existing identity still works (overwrite, no new slot).
        assert!(policy.bind(1, 1, 0, CommandSource::GcsDatalink).is_ok());
        assert_eq!(policy.resolve(1, 1, 0), Some(CommandSource::GcsDatalink));
    }
}
