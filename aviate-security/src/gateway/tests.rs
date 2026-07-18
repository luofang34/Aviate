//! Behavioral tests for the scheme-neutral admission boundary.
//!
//! These drive [`CommandGateway::admit`] with sealed claims built directly
//! (no crypto), so they exercise authorization, anti-replay, and receipt
//! stamping independently of any scheme. The MAVLink adapter's end-to-end
//! behavior against real signed frames is covered in
//! `tests/mavlink_interop.rs`.

#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use super::*;
use crate::errors::{AuthError, GatewayError};
use crate::principal::Principal;
use aviate_hal_io::SystemCommand;

const FRESHNESS: FreshnessConfig = FreshnessConfig {
    initial_trusted_counter: 0,
    new_stream_max_age: crate::NEW_STREAM_MAX_AGE_10US,
};

/// A gateway that authorizes one GCS principal: MAVLink key slot 5,
/// identity (1, 1) → GcsDatalink.
fn gateway() -> CommandGateway {
    let mut policy = SourcePolicy::new();
    policy
        .bind(Principal::mavlink(1, 1, 5), CommandSource::GcsDatalink)
        .expect("bind");
    CommandGateway::new(policy, FRESHNESS)
}

/// Seal a claim as an admission adapter would (crate-internal test access).
fn claim(principal: Principal, counter: u64, command: SystemCommand) -> AuthenticatedCommand {
    AuthenticatedCommand::seal(principal, counter, command)
}

fn mav(system_id: u8, component_id: u8, link_id: u8) -> Principal {
    Principal::mavlink(system_id, component_id, link_id)
}

#[test]
fn admit_mints_binding_source_and_freshness_from_the_claim() {
    let mut gw = gateway();
    let verified = gw
        .admit(claim(mav(1, 1, 5), 7000, SystemCommand::Arm), 42_000)
        .expect("an authenticated command from a bound principal is admitted");
    let r = verified.receipt();
    // Source comes from the authorization policy, not the claim.
    assert_eq!(r.source(), CommandSource::GcsDatalink);
    // Freshness sequence is the claim's authenticated counter.
    assert_eq!(r.sequence(), 7000);
    assert_eq!(r.authority_epoch(), 0);
    // Trusted receive time is the gateway's `now_us`.
    assert_eq!(r.received_at_us(), 42_000);
    assert!(matches!(verified.command(), SystemCommand::Arm));
}

#[test]
fn unbound_principal_is_unauthorized() {
    let mut gw = gateway();
    assert!(matches!(
        gw.admit(claim(mav(9, 9, 9), 7000, SystemCommand::Arm), 1_000),
        Err(GatewayError::Auth(AuthError::UnauthorizedSource))
    ));
}

/// Admission order is authorize → commit: an authenticated principal that
/// is not authorized must NOT occupy a replay slot. If replay committed
/// before authorization, the flood below would exhaust the bounded table
/// and lock out the legitimate principal.
#[test]
fn unauthorized_principals_cannot_exhaust_replay_slots() {
    let mut gw = gateway();

    for sys in 0..(crate::anti_replay::MAX_SIGNING_PEERS as u8 + 4) {
        let p = mav(sys.wrapping_add(2), 7, 5);
        assert!(matches!(
            gw.admit(claim(p, 9000, SystemCommand::Arm), 1_000),
            Err(GatewayError::Auth(AuthError::UnauthorizedSource))
        ));
    }

    // The legitimate bound principal still admits: no slot was consumed.
    assert!(
        gw.admit(claim(mav(1, 1, 5), 9001, SystemCommand::Arm), 2_000)
            .is_ok(),
        "unauthorized claims consumed replay capacity"
    );
}

/// The credential on slot 5 names identity (1, 1); the same key slot under
/// any other header identity is a different principal and is unauthorized.
#[test]
fn key_possession_cannot_impersonate_another_identity() {
    let mut gw = gateway();
    assert!(matches!(
        gw.admit(claim(mav(1, 2, 5), 7000, SystemCommand::Arm), 1_000),
        Err(GatewayError::Auth(AuthError::UnauthorizedSource))
    ));
}

#[test]
fn replayed_counter_is_rejected() {
    let mut gw = gateway();
    gw.admit(claim(mav(1, 1, 5), 5000, SystemCommand::Arm), 1_000)
        .unwrap();
    // Same principal, same (non-increasing) counter → replay.
    assert!(matches!(
        gw.admit(claim(mav(1, 1, 5), 5000, SystemCommand::Disarm), 2_000),
        Err(GatewayError::Auth(AuthError::ReplayAttack))
    ));
    // A strictly newer counter from the same principal is accepted.
    assert!(gw
        .admit(claim(mav(1, 1, 5), 5001, SystemCommand::Arm), 3_000)
        .is_ok());
}

#[test]
fn stale_new_stream_rejected_after_reboot() {
    // Gateway rebooted with a trusted counter ahead of captured traffic.
    let mut policy = SourcePolicy::new();
    policy
        .bind(Principal::mavlink(1, 1, 5), CommandSource::GcsDatalink)
        .unwrap();
    let mut gw = CommandGateway::new(
        policy,
        FreshnessConfig {
            initial_trusted_counter: 100_000_000,
            new_stream_max_age: crate::NEW_STREAM_MAX_AGE_10US,
        },
    );
    assert!(matches!(
        gw.admit(claim(mav(1, 1, 5), 1000, SystemCommand::Arm), 1_000),
        Err(GatewayError::Auth(AuthError::StaleNewStream { .. }))
    ));
}

#[test]
fn authority_epoch_advances_and_stamps_new_commands() {
    let mut gw = gateway();
    assert_eq!(gw.authority_epoch(), 0);
    gw.begin_authority_epoch();
    assert_eq!(gw.authority_epoch(), 1);
    let v = gw
        .admit(claim(mav(1, 1, 5), 100, SystemCommand::Arm), 100)
        .unwrap();
    assert_eq!(v.receipt().authority_epoch(), 1);
}

#[test]
fn into_command_erases_the_proof_exactly_once() {
    let mut gw = gateway();
    let v = gw
        .admit(claim(mav(1, 1, 5), 100, SystemCommand::Arm), 100)
        .unwrap();
    let bare: SystemCommand = v.into_command();
    assert!(matches!(bare, SystemCommand::Arm));
    // `v` is moved out; it cannot be reused (enforced by the compiler).
}

/// The failsafe capability is a process-wide singleton: the first take
/// succeeds, every later one fails. This is the ONLY caller of
/// `FailsafeAuthority::take()` in the suite — a second would race it.
#[test]
fn failsafe_authority_is_taken_exactly_once() {
    let authority = FailsafeAuthority::take().expect("first take yields the capability");
    let internal = TrustedInternalCommand::mint(SystemCommand::Disarm, &authority);
    assert!(matches!(internal.command(), SystemCommand::Disarm));
    assert!(FailsafeAuthority::take().is_none());
}
