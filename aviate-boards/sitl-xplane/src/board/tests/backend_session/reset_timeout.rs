//! Check reset acknowledgment behavior after a timeout.

use super::*;
use aviate_hal_xil::SimulatorOperation;

pub(super) fn verify_timed_out_reset_refuses_retry() {
    let (address, bridge, bridge_thread) = start_bridge();
    let mut backend = make_backend(address);
    let connected = backend
        .connect(0, Duration::from_secs(1))
        .expect("backend connects");
    let (observed, request_seen) = mpsc::channel();
    bridge
        .send(BridgeCommand::HoldReset { observed })
        .expect("hold reset acknowledgment");

    let first_error = backend
        .execute(
            reset_directive(90, connected.generation),
            Duration::from_millis(10),
        )
        .expect_err("held acknowledgment must time out");
    let timed_out = backend.status();
    assert!(matches!(
        first_error,
        SimulatorError::Timeout {
            operation: SimulatorOperation::Reset,
            generation,
            ..
        } if generation == timed_out.generation
    ));
    assert_eq!(timed_out.lifecycle, SimulatorLifecycle::Resetting);
    request_seen
        .recv_timeout(Duration::from_secs(1))
        .expect("bridge saw reset request");

    assert_reset_refused(&mut backend, 91, timed_out);
    execute_kind(&mut backend, 92, SimulatorDirectiveKind::Stop);
    let stopped = backend.status();
    assert_eq!(stopped.lifecycle, SimulatorLifecycle::Stopped);
    assert_reset_refused(&mut backend, 93, stopped);

    bridge
        .send(BridgeCommand::ReleaseReset)
        .expect("release late reset acknowledgment");
    execute_kind(&mut backend, 94, SimulatorDirectiveKind::Start);
    let FrameEvent::Frame(late) = backend
        .next_frame(Duration::from_secs(1))
        .expect("late acknowledgment frame")
    else {
        panic!("expected a frame for the original generation");
    };
    assert_eq!(late.generation, timed_out.generation);
    assert_eq!(late.lifecycle, SimulatorLifecycle::Converging);
    bridge.send(BridgeCommand::Stop).expect("stop bridge");
    bridge_thread.join().expect("bridge thread finishes");
}

fn assert_reset_refused(
    backend: &mut AliaBackend,
    id: u64,
    expected: aviate_hal_xil::BackendStatus,
) {
    let error = backend
        .execute(
            reset_directive(id, expected.generation),
            Duration::from_secs(1),
        )
        .expect_err("unresolved reset must refuse a retry");
    assert!(matches!(
        error,
        SimulatorError::ResetFailed { generation, .. }
            if generation == expected.generation
    ));
    assert_eq!(backend.status(), expected);
}

fn execute_kind(backend: &mut AliaBackend, id: u64, kind: SimulatorDirectiveKind) {
    let generation = backend.status().generation;
    backend
        .execute(
            SimulatorDirective {
                id: DirectiveId(id),
                generation,
                kind,
            },
            Duration::from_secs(1),
        )
        .expect("lifecycle directive succeeds");
}

fn reset_directive(id: u64, generation: ResetGeneration) -> SimulatorDirective {
    SimulatorDirective {
        id: DirectiveId(id),
        generation,
        kind: SimulatorDirectiveKind::Reset,
    }
}
