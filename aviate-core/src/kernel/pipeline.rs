//! `KernelPipeline<E, V, M, S>` — the kernel's algorithm-identity bundle
//! (LLR-PIPE-101..103).
//!
//! Every Phase-2 field that's an *algorithm box* (estimator, controller,
//! mixer, sanitizer) plus the protector singleton lives here. The
//! pipeline carries no safety-relevant persistent state of its own —
//! that property becomes a hard rule in Phase 4 when the EKF state and
//! sanitizer fallback state are pulled out into `KernelState`. For
//! Phase 2, the algorithm objects still own their internal state for
//! source-compatibility reasons; this module's purpose is purely to
//! consolidate the substitutable surface so the kernel's top-level
//! struct exposes a single `pipeline` field instead of five
//! independent ones.
//!
//! What lives here: estimator (E), controller (V), mixer (M),
//! sanitizer (S), protector (SimpleEnvelopeProtector — singleton,
//! zero-size).
//!
//! What does NOT live here: configuration (`ResolvedKernelConfig`,
//! Phase 1 sub-struct), safety-relevant persistent state
//! (`KernelState`, Phase 3 sub-struct), the `mode` runtime variable
//! (Phase 3).
//!
//! Phantom-DA note: this module deliberately avoids `pub use submodule::*`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale (rustc's
//! coverage debug info attributes re-exported trait-impl items to the
//! re-exporter's translation unit with the defining file's line numbers,
//! producing entries no COV:EXCL marker can silence). Consumers import
//! via `aviate_core::kernel::pipeline::KernelPipeline`.

use crate::control::envelope::SimpleEnvelopeProtector;
use crate::control::VehicleController;
use crate::ekf::Estimator;
use crate::mixer::{ActuatorSanitizer, Mixer};

/// Algorithm-identity bundle. Type parameters mirror the kernel's:
/// `<E, V, M, S>` = estimator, vehicle controller, mixer, sanitizer.
pub struct KernelPipeline<E, V, M, S>
where
    E: Estimator,
    V: VehicleController,
    M: Mixer,
    S: ActuatorSanitizer,
{
    pub estimator: E,
    pub controller: V,
    pub mixer: M,
    pub sanitizer: S,
    pub protector: SimpleEnvelopeProtector,
}

impl<E, V, M, S> KernelPipeline<E, V, M, S>
where
    E: Estimator,
    V: VehicleController,
    M: Mixer,
    S: ActuatorSanitizer,
{
    /// Construct a pipeline from caller-supplied algorithm boxes. The
    /// `protector` is a zero-size singleton — no caller customization
    /// needed. New pipeline-level configuration knobs (scratch buffer
    /// sizing, algorithm tuning shared across multiple traits, etc.)
    /// land here in future PRs and add builder methods rather than
    /// positional arguments.
    pub fn new(estimator: E, controller: V, mixer: M, sanitizer: S) -> Self {
        Self {
            estimator,
            controller,
            mixer,
            sanitizer,
            protector: SimpleEnvelopeProtector,
        }
    }

    /// Deterministic identity hash of this pipeline's algorithm bundle.
    ///
    /// The 64-bit hash is folded over the four substitutable trait
    /// implementations' `ALGORITHM_ID` constants using FNV-1a (offset
    /// 0xcbf29ce484222325, prime 0x00000100000001b3). Each constant
    /// is encoded as 8 little-endian bytes; positional ordering (E,
    /// V, M, S) is part of the hash, so swapping two impls produces a
    /// different hash even when their `ALGORITHM_ID`s are unchanged.
    ///
    /// Scope: this hash is the *algorithm-identity* witness only — it
    /// does NOT cover algorithm internal tuning (`EkfConfig`,
    /// controller gains, mixer geometry). Cross-channel byte-equality
    /// of resolved configuration is HLR-CFG-001's job; this method
    /// answers the orthogonal question "are the channels running the
    /// same algorithm classes?".
    ///
    /// Determinism boundary: each `ALGORITHM_ID` is a compile-time
    /// `u64` constant declared at the impl site, so the hash is
    /// stable across compiler versions, target triples, and
    /// optimization levels — strictly stronger than the previous
    /// `core::any::type_name` derivation, which was best-effort and
    /// only stable within one build. Spec §16 cross-channel firmware
    /// verification SHALL require byte-equal hashes between channels;
    /// a mismatch blocks lockstep entry.
    pub fn algorithm_identity_hash(&self) -> u64 {
        const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
        const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

        let mut hash = FNV_OFFSET;
        for id in [
            E::ALGORITHM_ID,
            V::ALGORITHM_ID,
            M::ALGORITHM_ID,
            S::ALGORITHM_ID,
        ] {
            for byte in id.to_le_bytes() {
                hash ^= u64::from(byte);
                hash = hash.wrapping_mul(FNV_PRIME);
            }
        }
        hash
    }
}

#[cfg(test)]
mod tests {
    // TST-PIPE-103: structural witness for LLR-PIPE-103.
    use super::KernelPipeline;
    use crate::control::multirotor::MultirotorController;
    use crate::ekf::Ekf;
    use crate::mixer::{QuadXMixer, Sanitizer};
    use crate::time::{TimeSource, Timestamp};

    fn fake_ts() -> Timestamp {
        Timestamp {
            ticks: 0,
            source: TimeSource::Internal,
        }
    }

    fn fake_mixer() -> QuadXMixer {
        QuadXMixer {
            timestamp_source: fake_ts,
        }
    }

    fn make_pipeline() -> KernelPipeline<Ekf, MultirotorController, QuadXMixer, Sanitizer> {
        KernelPipeline::new(
            Ekf::default(),
            MultirotorController::default(),
            fake_mixer(),
            Sanitizer,
        )
    }

    #[test]
    fn identity_hash_is_deterministic_within_build() {
        let p = make_pipeline();
        // Exercise the embedded timestamp_source fn pointer once so the
        // test-fixture helper (`fake_ts`) is reached by coverage; the
        // hash itself is a pure function of type parameters and does
        // not depend on this call.
        let _ = (p.mixer.timestamp_source)();
        let h1 = p.algorithm_identity_hash();
        let h2 = make_pipeline().algorithm_identity_hash();
        assert_eq!(
            h1, h2,
            "two pipelines built with identical algorithm bundles must hash equal"
        );
    }

    #[test]
    fn identity_hash_differs_across_algorithm_bundles() {
        // Swap the controller type from MultirotorController → FixedWingController.
        use crate::control::fixed_wing::FixedWingController;
        let multirotor = make_pipeline().algorithm_identity_hash();
        let fixed_wing =
            KernelPipeline::new(Ekf::default(), FixedWingController, fake_mixer(), Sanitizer)
                .algorithm_identity_hash();
        assert_ne!(
            multirotor, fixed_wing,
            "swapping the controller type must change the identity hash"
        );
    }

    #[test]
    fn identity_hash_is_nonzero() {
        // FNV-1a starts from a nonzero offset basis and only multiplies
        // / xors thereafter — collapsing to 0 would indicate a coding
        // error in the loop.
        assert_ne!(make_pipeline().algorithm_identity_hash(), 0);
    }

    #[test]
    fn identity_hash_is_stable_across_builds() {
        // TST-PIPE-104 (LLR-PIPE-104): the hash is a deterministic
        // function of the four impls' compile-time `ALGORITHM_ID`
        // constants, so it must equal a fixed value across compiler
        // versions, target triples, and optimization levels.
        // Recomputing this assertion's RHS by hand from the four
        // `ALGORITHM_ID`s would catch a silent change to the FNV
        // constants or the field ordering. If this assertion ever
        // fails, do NOT update it without first checking whether a
        // production-channel hash mismatch was introduced — that is
        // exactly what spec §16 cross-channel firmware verification
        // exists to catch.
        //
        // This value reflects the Ekf identity `ETIMEKF4`
        // ("ekf.basic-15state.v3", no timeout adoption of rejected
        // aiding, sustained rejection surfaces as lost validity), the
        // controller identity `CTLMURV2` ("controller.multirotor.v2",
        // heading-frame tilt mapping), and the mixer identity
        // `MIXQUAD2` ("mixer.quad_x.v2", priority desaturation): each
        // moved deliberately off its retired predecessor because its
        // observable behavior changed while its state shape did not.
        const EXPECTED: u64 = 0x646b_55c0_745d_ab84;
        let actual = make_pipeline().algorithm_identity_hash();
        assert_eq!(
            actual, EXPECTED,
            "algorithm_identity_hash drifted; \
             check ALGORITHM_ID constants on Ekf / MultirotorController / \
             QuadXMixer / Sanitizer and the FNV folding loop"
        );
    }

    #[test]
    fn identity_hash_is_stable_across_builds_x500() {
        // The X500 production bundle swaps QuadXMixerX500 in for
        // QuadXMixer; every other member matches
        // `identity_hash_is_stable_across_builds`.
        // scripts/check_algorithm_identity.sh pins the same value
        // from the registry side, so an aggregate drift is caught
        // whether it enters through source constants or through
        // cert/algorithm_id_registry.toml — rotate a bundle member
        // and both pins must move in the same commit.
        use crate::mixer::QuadXMixerX500;
        const EXPECTED: u64 = 0x20ce_8c48_7287_24d5;
        let actual = KernelPipeline::new(
            Ekf::default(),
            MultirotorController::default(),
            QuadXMixerX500 {
                timestamp_source: fake_ts,
            },
            Sanitizer,
        )
        .algorithm_identity_hash();
        assert_eq!(
            actual, EXPECTED,
            "X500 algorithm_identity_hash drifted; \
             check ALGORITHM_ID constants on Ekf / MultirotorController / \
             QuadXMixerX500 / Sanitizer, the FNV folding loop, and the \
             X500_AGGREGATE pin in scripts/check_algorithm_identity.sh"
        );
    }
}
