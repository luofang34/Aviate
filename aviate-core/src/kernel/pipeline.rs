//! `KernelPipeline<E, V, M, S>` — the kernel's algorithm-identity bundle
//! (LLR-PIPE-101..102).
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
}
