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
    /// The 64-bit hash is derived from the `core::any::type_name` of the
    /// four substitutable algorithm parameters (`E`, `V`, `M`, `S`)
    /// using FNV-1a. Two channels running byte-identical firmware
    /// produce the same hash; a mismatch indicates the channels are
    /// running different algorithm bundles and SHALL block lockstep
    /// entry (spec §16, cross-channel firmware verification).
    ///
    /// Scope: this hash is the *algorithm-identity* witness only — it
    /// does NOT cover algorithm internal tuning (`EkfConfig`,
    /// controller gains, mixer geometry). Cross-channel byte-equality
    /// of resolved configuration is HLR-CFG-001's job; this method
    /// answers the orthogonal question "are the channels running the
    /// same algorithm classes?".
    ///
    /// Determinism boundary: `core::any::type_name` is documented as a
    /// best-effort symbol description that may differ across Rust
    /// compiler versions and target triples. The hash is therefore
    /// only deterministic *within a single firmware build*. Channels
    /// participating in lockstep are required to be byte-identical
    /// firmware images, so the within-build determinism is sufficient.
    pub fn algorithm_identity_hash(&self) -> u64 {
        const FNV_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
        const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

        let mut hash = FNV_OFFSET;
        for type_name in [
            core::any::type_name::<E>(),
            core::any::type_name::<V>(),
            core::any::type_name::<M>(),
            core::any::type_name::<S>(),
        ] {
            for byte in type_name.as_bytes() {
                hash ^= u64::from(*byte);
                hash = hash.wrapping_mul(FNV_PRIME);
            }
            // Field separator so concatenation collisions across the
            // four type-name boundaries become hash-distinguishable
            // (e.g. ("FooBar", "Baz") vs. ("Foo", "BarBaz")).
            hash ^= 0xff;
            hash = hash.wrapping_mul(FNV_PRIME);
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
        let h1 = make_pipeline().algorithm_identity_hash();
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
}
