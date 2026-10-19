//! The law of a new flight period, and the degradations that must
//! outlive one.
//!
//! Disarm leaves the kernel on the backup law (HLR-FLT-205), so a
//! re-armed vehicle that kept it would sit on the safe pattern and
//! ignore every command. A degradation that must outlive the flight
//! period is carried by a latched fault instead, which refuses the next
//! arm until `ground_reset()`.

use aviate_core::checks::DegradationReason;
use aviate_core::control::{Command, ControlLawV1, ControlMode, Setpoint};
use aviate_core::fault::FaultFlags;
use aviate_core::mixer::ActuatorState;
use aviate_core::types::NormalizedThrust;
use aviate_core::{ArmError, ChannelId, UpdateResult, TIMING_VIOLATION_THRESHOLD};

use super::{
    armed_on_ground, hover_command, place_at, run_cycle, time_delta, timestamp, valid_sensors,
    TestKernel,
};

fn climb_command() -> Command {
    Command {
        mode: ControlMode::Attitude,
        setpoint: Setpoint {
            collective_thrust: NormalizedThrust(0.7),
            ..Default::default()
        },
        ..hover_command()
    }
}

fn climb_cycle(kernel: &mut TestKernel) -> UpdateResult {
    kernel.update(
        ChannelId::PRIMARY,
        time_delta(),
        &valid_sensors(),
        &climb_command(),
        0,
        &ActuatorState::default(),
        None,
    )
}

/// Run the init sequence on the ground until the kernel is armable.
fn reach_ready(kernel: &mut TestKernel) {
    let sensors = valid_sensors();
    for _ in 0..150 {
        kernel.init_step(&sensors, timestamp());
    }
}

/// Fly one cycle, land (the vehicle never left the ground), disarm.
fn fly_and_disarm(kernel: &mut TestKernel) {
    run_cycle(kernel);
    kernel.disarm().expect("a grounded disarm is accepted");
}

#[test]
fn re_arming_after_a_grounded_disarm_flies_the_primary_law() {
    let mut kernel = armed_on_ground();
    fly_and_disarm(&mut kernel);
    assert_eq!(kernel.state.control_law, ControlLawV1::Backup);

    reach_ready(&mut kernel);
    kernel.arm().expect("a fault-free kernel re-arms");
    let result = climb_cycle(&mut kernel);

    assert_eq!(result.status.law, ControlLawV1::Primary);
    assert_ne!(
        result.actuator.outputs,
        kernel.cfg().safe_output,
        "a climb command on the re-armed vehicle must drive the motors"
    );
}

/// The latch → refusal → ground-reset cycle for one degradation.
fn assert_latches_until_ground_reset(mut kernel: TestKernel, fault: FaultFlags) {
    assert!(
        kernel.state.faults.contains(fault),
        "{fault:?} must latch when its degradation engages"
    );
    kernel.disarm().expect("a grounded disarm is accepted");
    assert!(
        kernel.state.faults.contains(fault),
        "{fault:?} must survive disarm"
    );

    reach_ready(&mut kernel);
    assert_eq!(kernel.arm(), Err(ArmError::Faulted));
    assert!(kernel.state.faults.contains(fault));

    kernel.ground_reset();
    assert!(kernel.state.faults.is_empty());
    kernel.state.checks.pre_arm.update_throttle(true);
    place_at(&mut kernel, 0.0, 0.0);
    reach_ready(&mut kernel);
    kernel.arm().expect("ground reset clears the latch");
    assert_eq!(climb_cycle(&mut kernel).status.law, ControlLawV1::Primary);
}

#[test]
fn every_capability_loss_latches_a_fault_until_ground_reset() {
    let cases = [
        (DegradationReason::AttitudeLost, FaultFlags::ATTITUDE_LOST),
        (DegradationReason::ImuDegraded, FaultFlags::IMU_DEGRADED),
        (DegradationReason::PositionLost, FaultFlags::POSITION_LOST),
        (DegradationReason::VelocityLost, FaultFlags::VELOCITY_LOST),
        (DegradationReason::BaroDegraded, FaultFlags::BARO_DEGRADED),
        (
            DegradationReason::EnvelopeViolation,
            FaultFlags::ENVELOPE_VIOLATION,
        ),
        (DegradationReason::RcLost, FaultFlags::RC_LOST),
        (
            DegradationReason::TimingViolation,
            FaultFlags::TIMING_PERSISTENT,
        ),
    ];
    for (reason, fault) in cases {
        let mut kernel = armed_on_ground();
        run_cycle(&mut kernel);
        kernel.handle_degradation(reason, timestamp());
        assert_latches_until_ground_reset(kernel, fault);
    }
}

#[test]
fn recoverable_and_commanded_degradations_latch_nothing() {
    for reason in [
        DegradationReason::CommandTimeout,
        DegradationReason::LandRequested,
    ] {
        let mut kernel = armed_on_ground();
        run_cycle(&mut kernel);
        kernel.handle_degradation(reason, timestamp());
        assert!(kernel.state.faults.is_empty(), "{reason:?} latched a fault");
        fly_and_disarm(&mut kernel);
        reach_ready(&mut kernel);
        kernel
            .arm()
            .expect("nothing latched, so the next flight arms");
        assert_eq!(climb_cycle(&mut kernel).status.law, ControlLawV1::Primary);
    }
}

#[test]
fn persistent_timing_violations_latch_through_the_update_cycle() {
    let mut kernel = armed_on_ground();
    for _ in 0..TIMING_VIOLATION_THRESHOLD {
        kernel.report_timing_violation(true);
    }
    run_cycle(&mut kernel);
    assert_eq!(kernel.state.control_law, ControlLawV1::Alternate);
    assert_latches_until_ground_reset(kernel, FaultFlags::TIMING_PERSISTENT);
}
