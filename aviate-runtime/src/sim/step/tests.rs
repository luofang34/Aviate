//! Arm-authorization state-transition tests.

#![allow(clippy::expect_used)]

use aviate_core::control::multirotor::MultirotorController;
use aviate_core::ekf::Ekf;
use aviate_core::hal::ActuatorHal as _;
use aviate_core::kernel::builder::AviateKernelBuilder;
use aviate_core::mixer::{QuadXMixer, Sanitizer};
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::ArmError;
use aviate_hal_io::{
    BoardHal, CommandOutcome, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag, SystemCommand,
};
use aviate_hal_xil::{SitlIO, XilConfig, XilNetConfig};

use super::super::{ArmAuthorizer, SitlRunner, SitlTime};

struct RejectArm;

impl ArmAuthorizer for RejectArm {
    fn authorize_arm(&self) -> Result<(), ArmError> {
        Err(ArmError::NotReady)
    }
}

fn timestamp() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn runner() -> SitlRunner<MultirotorController, QuadXMixer> {
    let kernel = AviateKernelBuilder::new()
        .estimator(Ekf::default())
        .controller(MultirotorController::default())
        .mixer(QuadXMixer {
            timestamp_source: timestamp,
        })
        .sanitizer(Sanitizer)
        .build()
        .expect("valid test kernel");
    let transport = SitlIO::new(XilConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 16,
        },
    ))
    .expect("ephemeral SITL transport");
    let board_hal = BoardHal::new(
        FakeImu::new(),
        FakeBaro::new(),
        FakeMag::new(),
        FakeGnss::new(),
        SitlTime::new(),
        FakeActuator::new(),
    );
    SitlRunner::new(transport, board_hal, kernel)
}

#[test]
fn inbound_arm_rejection_changes_no_arm_state() {
    let mut runner = runner();
    let kernel_state = runner.kernel.state.init_state;
    assert!(!runner.board_hal.is_armed());
    assert!(!runner.transport.is_armed());

    let outcome = runner.enact_discrete(&SystemCommand::Arm, &RejectArm);

    assert!(matches!(
        outcome,
        Some(CommandOutcome::ArmRejected {
            error: ArmError::NotReady,
            ..
        })
    ));
    assert_eq!(runner.kernel.state.init_state, kernel_state);
    assert!(!runner.board_hal.is_armed());
    assert!(!runner.transport.is_armed());
}
