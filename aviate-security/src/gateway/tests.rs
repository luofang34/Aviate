//! Behavioral + structural tests for the verified-command boundary.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::auth::PlainAuth;
use crate::errors::{AuthError, GatewayError};
use crate::test_support::{signed_auth, signed_meta, valid_meta};
use aviate_hal_io::SystemCommand;

/// A flight gateway (signing auth) that binds one GCS identity.
fn flight_gateway() -> CommandGateway<
    crate::auth::SignedAuth<crate::test_support::MockKeyStore, crate::test_support::MockCrypto>,
> {
    let policy = SourcePolicy::flight().with_binding(1, 1, 5, CommandSource::GcsDatalink);
    CommandGateway::new(signed_auth(), policy)
}

fn unverified(
    command: SystemCommand,
    sig: Option<aviate_link::command::SignatureMeta>,
) -> UnverifiedSystemCommand {
    UnverifiedSystemCommand::new(command, sig)
}

#[test]
fn admit_mints_verified_command_binding_source_and_freshness_from_signature() {
    let mut gw = flight_gateway();
    let sig = valid_meta(1, 1, 5, 7000, &[0x01, 0x02, 0x03]);
    let verified = gw
        .admit(unverified(SystemCommand::Arm, Some(sig)), 42_000)
        .expect("valid signed command from a bound identity is admitted");
    let r = verified.receipt();
    // Source comes from the credential binding, not the payload.
    assert_eq!(r.source(), CommandSource::GcsDatalink);
    // Freshness counter is the authenticated signature timestamp.
    assert_eq!(r.sequence(), 7000);
    assert_eq!(r.authority_epoch(), 0);
    // Trusted receive time is the gateway's `now_us`, not the payload.
    assert_eq!(r.received_at_us(), 42_000);
    assert!(matches!(verified.command(), SystemCommand::Arm));
}

#[test]
fn signed_frame_from_unbound_identity_is_unauthorized() {
    // The signature is perfectly valid, but the identity maps to no source.
    // Because authority is bound to the credential (never the payload), the
    // command cannot pick its own source and is rejected.
    let mut gw = flight_gateway();
    let sig = valid_meta(9, 9, 9, 7000, &[0x01, 0x02]);
    assert!(matches!(
        gw.admit(unverified(SystemCommand::Arm, Some(sig)), 1_000),
        Err(GatewayError::Auth(AuthError::UnauthorizedSource))
    ));
}

#[test]
fn unsigned_frame_rejected_under_flight_policy() {
    let mut gw = flight_gateway();
    assert!(matches!(
        gw.admit(unverified(SystemCommand::Arm, None), 1_000),
        Err(GatewayError::Auth(AuthError::MissingSignature))
    ));
}

#[test]
fn bad_signature_mints_nothing() {
    let mut gw = flight_gateway();
    // Correct identity, but a wrong signature.
    let sig = signed_meta(1, 1, 5, 7000, &[0x01, 0x02], [0x00; 6]);
    assert!(matches!(
        gw.admit(unverified(SystemCommand::Arm, Some(sig)), 1_000),
        Err(GatewayError::Auth(AuthError::InvalidSignature))
    ));
}

#[test]
fn replayed_signature_is_rejected() {
    let mut gw = flight_gateway();
    let msg = [0x11, 0x22, 0x33];
    gw.admit(
        unverified(SystemCommand::Arm, Some(valid_meta(1, 1, 5, 5000, &msg))),
        1_000,
    )
    .unwrap();
    // Same identity, same (non-increasing) timestamp → replay.
    assert!(matches!(
        gw.admit(
            unverified(SystemCommand::Disarm, Some(valid_meta(1, 1, 5, 5000, &msg))),
            2_000
        ),
        Err(GatewayError::Auth(AuthError::ReplayAttack))
    ));
    // A strictly newer timestamp from the same identity is accepted.
    assert!(gw
        .admit(
            unverified(SystemCommand::Arm, Some(valid_meta(1, 1, 5, 5001, &msg))),
            3_000
        )
        .is_ok());
}

#[test]
fn distinct_bound_identities_carry_distinct_authorities() {
    // Same link_id, different component_id → different authorities. A
    // captured signature for one identity can never surface as the other's
    // source, because the source is resolved from the authenticated
    // identity itself.
    let policy = SourcePolicy::flight()
        .with_binding(1, 1, 5, CommandSource::Rc)
        .with_binding(1, 2, 5, CommandSource::Offboard);
    let mut gw = CommandGateway::new(signed_auth(), policy);

    let rc = gw
        .admit(
            unverified(SystemCommand::Arm, Some(valid_meta(1, 1, 5, 100, &[9]))),
            1,
        )
        .unwrap();
    assert_eq!(rc.receipt().source(), CommandSource::Rc);

    let off = gw
        .admit(
            unverified(SystemCommand::Arm, Some(valid_meta(1, 2, 5, 100, &[9]))),
            2,
        )
        .unwrap();
    assert_eq!(off.receipt().source(), CommandSource::Offboard);
}

#[test]
fn dev_policy_admits_unsigned_with_configured_source() {
    // PlainAuth + an insecure dev policy: unsigned traffic is admitted and
    // assigned the dev source. This path must never be in a flight build.
    let policy = SourcePolicy::insecure_dev(CommandSource::Offboard);
    let mut gw = CommandGateway::new(PlainAuth::new(), policy);
    let verified = gw
        .admit(unverified(SystemCommand::Arm, None), 500)
        .expect("dev policy admits unsigned");
    assert_eq!(verified.receipt().source(), CommandSource::Offboard);
    assert_eq!(verified.receipt().sequence(), 0);
}

#[test]
fn authority_epoch_advances_and_stamps_new_commands() {
    let mut gw = flight_gateway();
    assert_eq!(gw.authority_epoch(), 0);
    gw.begin_authority_epoch();
    assert_eq!(gw.authority_epoch(), 1);
    let v = gw
        .admit(
            unverified(SystemCommand::Arm, Some(valid_meta(1, 1, 5, 100, &[1]))),
            100,
        )
        .unwrap();
    assert_eq!(v.receipt().authority_epoch(), 1);
}

#[test]
fn into_command_erases_the_proof_exactly_once() {
    let mut gw = flight_gateway();
    let v = gw
        .admit(
            unverified(SystemCommand::Arm, Some(valid_meta(1, 1, 5, 100, &[1]))),
            100,
        )
        .unwrap();
    // The single erase point: consumes the wrapper, yields the bare cmd.
    let bare: SystemCommand = v.into_command();
    assert!(matches!(bare, SystemCommand::Arm));
    // `v` is moved out; it cannot be reused (enforced by the compiler).
}

#[test]
fn trusted_internal_command_is_a_distinct_type_from_verified() {
    // A self-generated failsafe is trusted but is NOT a
    // VerifiedSystemCommand and carries no external receipt. Minting one
    // requires the FailsafeAuthority capability.
    let authority = FailsafeAuthority::acquire();
    let internal = TrustedInternalCommand::mint(SystemCommand::Disarm, &authority);
    assert!(matches!(internal.command(), SystemCommand::Disarm));
    // There is deliberately no conversion internal -> verified.
}
