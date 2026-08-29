//! Every arm request re-checks the condition identities that bind the run.

#![allow(clippy::expect_used)]

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use aviate_hal_xil::perturbation::{
    LoadedPerturbationArtifact, PerturbationArtifactIdentity, PerturbationCapability,
};
use aviate_runtime::ArmAuthorizer as _;
use sha2::{Digest as _, Sha256};

use super::super::config::PerturbationBinding;
use super::XPlaneArmAuthorizer;

const GOLDEN: &[u8] =
    include_bytes!("../../../../../aviate-hal/xil/fixtures/condition-v4.golden.json");
const RUN_SEED: u64 = 0x1112_1314_1516_1718;
const GOLDEN_HOVER_SCALE: u16 = 9_000;

fn digest(bytes: &[u8]) -> [u8; 32] {
    Sha256::digest(bytes).into()
}

fn artifact_path(name: &str) -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!("aviate-arm-authorization-{name}.json"));
    path
}

fn capabilities() -> [PerturbationCapability; 4] {
    [
        PerturbationCapability::SensorPerturbation,
        PerturbationCapability::ActuatorAuthority,
        PerturbationCapability::CommandHold,
        PerturbationCapability::HoverTrimUncertainty,
    ]
}

struct LoadedGolden {
    artifact: LoadedPerturbationArtifact,
    path: PathBuf,
}

impl Drop for LoadedGolden {
    fn drop(&mut self) {
        std::fs::remove_file(&self.path).ok();
    }
}

fn load_golden(name: &str) -> LoadedGolden {
    let path = artifact_path(name);
    std::fs::write(&path, GOLDEN).expect("write golden artifact");
    // The fixture bytes are already the canonical condition form, so the
    // condition digest is the digest of those bytes without the trailing LF.
    let condition_digest = digest(GOLDEN.strip_suffix(b"\n").unwrap_or(GOLDEN));
    let artifact = LoadedPerturbationArtifact::load(
        &path,
        digest(GOLDEN),
        condition_digest,
        RUN_SEED,
        &capabilities(),
    )
    .expect("golden artifact loads");
    LoadedGolden { artifact, path }
}

fn authorizer(
    binding: Option<PerturbationBinding>,
    guard_from: &LoadedGolden,
) -> XPlaneArmAuthorizer {
    XPlaneArmAuthorizer {
        runtime_identity_ready: true,
        tuning_trace_ready: true,
        perturbation_ready: true,
        perturbation_configured: true,
        binding,
        hover_scale_basis_points: GOLDEN_HOVER_SCALE,
        artifact_guard: Some(guard_from.artifact.live_guard()),
        artifact_failure: Arc::new(AtomicBool::new(false)),
    }
}

fn binding(loaded: &LoadedGolden) -> PerturbationBinding {
    PerturbationBinding {
        artifact: loaded.artifact.identity().clone(),
        manifest: loaded.artifact.identity().clone(),
        config: loaded.artifact.config().clone(),
    }
}

fn other_identity(base: &PerturbationArtifactIdentity) -> PerturbationArtifactIdentity {
    let mut other = base.clone();
    other.run_seed = base.run_seed.wrapping_add(1);
    other
}

#[test]
fn a_matched_condition_identity_authorizes_the_request() {
    let loaded = load_golden("matched");

    assert!(authorizer(Some(binding(&loaded)), &loaded)
        .authorize_arm()
        .is_ok());
}

#[test]
fn a_run_manifest_naming_another_artifact_refuses_the_request() {
    let loaded = load_golden("manifest-mismatch");
    let mut mismatched = binding(&loaded);
    mismatched.manifest = other_identity(&mismatched.artifact);

    assert!(authorizer(Some(mismatched), &loaded)
        .authorize_arm()
        .is_err());
}

#[test]
fn a_capability_the_run_does_not_execute_refuses_the_request() {
    let loaded = load_golden("capability-mismatch");
    let matched = binding(&loaded);

    // The kernel carries the nominal hover force, so the run executes no
    // hover-trim uncertainty even though the artifact declares it.
    let mut authorizer = authorizer(Some(matched), &loaded);
    authorizer.hover_scale_basis_points = 10_000;

    assert!(authorizer.authorize_arm().is_err());
}

#[test]
fn a_declared_capability_the_artifact_omits_refuses_the_request() {
    let loaded = load_golden("capability-omitted");
    let mut narrowed = binding(&loaded);
    narrowed.artifact.required_capabilities = vec![PerturbationCapability::SensorPerturbation];

    assert!(authorizer(Some(narrowed), &loaded).authorize_arm().is_err());
}

#[test]
fn an_unbound_condition_refuses_the_request() {
    let loaded = load_golden("unbound");

    assert!(authorizer(None, &loaded).authorize_arm().is_err());
}
