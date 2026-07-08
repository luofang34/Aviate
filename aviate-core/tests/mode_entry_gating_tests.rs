//! Behavioral tests for #68 mode-entry gating: Position/Velocity-
//! family control modes are refused when the estimator can't back
//! them, falling back along Position -> Altitude -> Attitude
//! (Stabilized) rather than running an outer loop against invalid
//! state.
//!
//! Pure decision-table coverage (`required_validity`, `gate_mode_entry`,
//! `apply_mode_entry`) lives in `aviate-core/src/control/mode_gate.rs`.
//! This file witnesses the two things a unit test on the pure
//! functions can't: (1) the real cascade's behavior under a gated
//! command — no lateral tilt, real collective — and (2) that
//! `AviateKernelImpl::update()` actually wires the gate into loop
//! selection and reports it honestly on `ChannelStatus`.
//!
//! The real EKF (`ekf.rs`) only ever reports `POSITION`/`VELOCITY`
//! validity together (both valid post-init, both cleared only on a
//! non-finite numeric fault) — it cannot yet produce the
//! selectively-invalid combinations this gate needs to be exercised
//! against (a separate, already-tracked EKF-honesty gap). The
//! kernel-level tests below use `FixedEstimator`, a minimal
//! `Estimator` impl (same pattern as `mock_pipeline_tests.rs`'s
//! `MockEstimator`) that returns a caller-set `StateEstimate`
//! verbatim, so these tests can drive arbitrary `StateValidFlags`
//! combinations through the kernel's public surface without the
//! `test-hooks` feature (which CI compile-checks but does not run).

use aviate_core::checks::{KernelChecks, PreArmFlags};
use aviate_core::control::multirotor::{MultirotorController, MultirotorRuntimeState};
use aviate_core::control::{
    apply_mode_entry, gate_mode_entry, Command, CommandSource, ControlLawV1, ControlMode, Limits,
    ModeEntryDecision, Setpoint, VehicleControlMode, VehicleController,
};
use aviate_core::ekf::runtime::EstimatorRuntimeState;
use aviate_core::ekf::Estimator;
use aviate_core::kernel::config::ResolvedKernelConfig;
use aviate_core::kernel::pipeline::KernelPipeline;
use aviate_core::kernel::state::KernelState;
use aviate_core::kernel::{AviateKernelImpl, InitState};
use aviate_core::math::Quaternion;
use aviate_core::mixer::{ActuatorState, ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::sensor::{ImuData, SensorHealth, SensorReading, SensorSet};
use aviate_core::state::{EstimateQuality, StateEstimate, StateValidFlags};
use aviate_core::time::{TimeDelta, TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, MetersPerSecondSquared, RadiansPerSecond, Seconds,
};
use aviate_core::ChannelId;

// =============================================================================
// Cascade-level: the real MultirotorController under a gated command
// =============================================================================

fn cascade_limits() -> Limits {
    aviate_core::kernel::config::ResolvedKernelConfig::default().limits
}

fn cascade_state(valid_flags: StateValidFlags) -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-10.0)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags,
    }
}

fn position_command() -> Command {
    Command {
        mode: ControlMode::PositionHold,
        setpoint: Setpoint {
            // Offset target: if the position loop ran despite the
            // gate, the along-track error would tilt the vehicle.
            position: Some([Meters(10.0), Meters(10.0), Meters(-10.0)]),
            ..Default::default()
        },
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 1,
        source: CommandSource::Autopilot,
    }
}

fn step_cascade(state: &StateEstimate, command: Command) -> aviate_core::control::AxisCommand {
    let decision = gate_mode_entry(command.mode, state.valid_flags);
    let gated = apply_mode_entry(command, decision);
    let controller = MultirotorController::default();
    let mut runtime = MultirotorRuntimeState::default();
    controller.step(
        &mut runtime,
        state,
        &gated,
        &VehicleControlMode::from_control_mode(gated.mode),
        aviate_core::control::ConfigMode::Hover,
        &cascade_limits(),
    )
}

#[test]
fn falling_back_to_altitude_does_not_run_the_position_loop() {
    // POSITION invalid, ATTITUDE+VELOCITY valid: acceptance criterion
    // 3 — dropping position validity mid-flight must not continue
    // position control.
    let state = cascade_state(StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY);
    let axis = step_cascade(&state, position_command());

    assert!(
        axis.roll.0.abs() < 1e-3 && axis.pitch.0.abs() < 1e-3,
        "gated fallback must not run the position loop: roll={} pitch={}",
        axis.roll.0,
        axis.pitch.0
    );
    assert!(
        axis.collective.0 > 0.05,
        "AltitudeHold fallback must command real (closed-loop) \
         collective, not fall through to manual passthrough's \
         zero default: collective={}",
        axis.collective.0
    );
}

#[test]
fn falling_back_to_attitude_does_not_run_the_position_loop() {
    // POSITION and VELOCITY both invalid, ATTITUDE valid.
    let state = cascade_state(StateValidFlags::ATTITUDE);
    let axis = step_cascade(&state, position_command());

    assert!(
        axis.roll.0.abs() < 1e-3 && axis.pitch.0.abs() < 1e-3,
        "gated fallback must not run the position loop: roll={} pitch={}",
        axis.roll.0,
        axis.pitch.0
    );
}

#[test]
fn falling_back_to_attitude_inherits_the_raw_collective_setpoint_residual_risk() {
    // Documents a known, deliberate scope boundary (see PR notes):
    // `Attitude` has no closed-loop collective path in this
    // architecture (it always passes `Setpoint::collective_thrust`
    // through manually), so an autonomous PositionHold command that
    // never populated it — the normal case, since Position derives
    // collective from the velocity loop — falls through to zero once
    // gated this far down. The gate still correctly refuses the
    // position loop; synthesizing a safe open-loop collective for a
    // fully-autonomous Attitude fallback is out of this issue's
    // scope. This test pins the gap so a future fix must consciously
    // update it rather than silently regress it further.
    let state = cascade_state(StateValidFlags::ATTITUDE);
    let axis = step_cascade(&state, position_command());
    assert_eq!(axis.collective.0, 0.0);
}

#[test]
fn fully_valid_estimate_runs_position_control_unmodified() {
    // Sanity/regression: a healthy estimate must not be gated at
    // all — normal missions are unaffected.
    let state = cascade_state(StateValidFlags::all());
    let axis = step_cascade(&state, position_command());
    assert!(
        axis.pitch.0.abs() > 1e-3,
        "fully valid PositionHold must still tilt toward the offset \
         target: pitch={}",
        axis.pitch.0
    );
}

// =============================================================================
// Kernel-level: AviateKernelImpl::update() wiring + ChannelStatus honesty
// =============================================================================

/// Runtime state for `FixedEstimator`: just the `StateEstimate` a
/// test wants `estimate()` to project. `observe()` is a no-op, so a
/// test drives scenarios by writing `kernel.state.estimator.0`
/// directly before calling `update()`.
#[derive(Clone, Debug, Default)]
struct FixedEstimatorState(StateEstimate);

impl EstimatorRuntimeState for FixedEstimatorState {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

impl aviate_core::replicable::Replicable for FixedEstimatorState {
    const ENCODED_LEN: usize = 1;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        aviate_core::replicable::copy_into(buf, 0, &[0u8])
    }
}

#[derive(Default)]
struct FixedEstimator;

impl Estimator for FixedEstimator {
    type RuntimeState = FixedEstimatorState;

    const ALGORITHM_ID: u64 = 0x4649_5845_4553_5431; // "FIXEEST1"

    fn observe(
        &self,
        _state: &mut Self::RuntimeState,
        _sensors: &SensorSet,
        _overrides: Option<&aviate_core::control::SensorOverrides>,
        _dt: aviate_core::types::Scalar,
    ) {
    }

    fn estimate(&self, state: &Self::RuntimeState) -> StateEstimate {
        state.0.clone()
    }
}

fn timestamp_source() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn dt_100hz() -> TimeDelta {
    TimeDelta {
        dt_sec: Seconds(0.01),
        tick_delta: 10_000,
    }
}

type GatingKernel = AviateKernelImpl<FixedEstimator, MultirotorController, QuadXMixer, Sanitizer>;

fn make_kernel() -> GatingKernel {
    let mixer = QuadXMixer { timestamp_source };
    let mode_config = ModeConfig {
        mode: aviate_core::control::ConfigMode::Hover,
        groups: &[],
    };
    let mut kernel = AviateKernelImpl {
        pipeline: KernelPipeline::new(
            FixedEstimator,
            MultirotorController::default(),
            mixer,
            Sanitizer,
        ),
        state: KernelState::new(KernelChecks::with_pre_arm_required(PreArmFlags::empty())),
        cfg: ResolvedKernelConfig {
            mode_config,
            ..Default::default()
        },
    };
    // These tests exercise loop-selection gating specifically, not
    // the arm lifecycle (covered by kernel.rs / behavioral_tests.rs)
    // — bypass straight to Armed, matching mock_pipeline_tests.rs.
    kernel.state.init_state = InitState::Armed;
    kernel.state.control_law = ControlLawV1::Primary;
    kernel
}

/// One good IMU reading — enough to keep `ALL_IMU_FAILED` (a
/// `CRITICAL_FAULTS` bit) clear so `update()` reaches loop selection
/// instead of short-circuiting into the critical-fault safe-output
/// branch. GNSS/baro/mag are left invalid; neither gates
/// `CRITICAL_FAULTS` nor this gate (which reads `valid_flags`
/// directly from `FixedEstimator`, not from raw sensors).
fn minimal_sensors() -> SensorSet {
    let ts = timestamp_source();
    let mut sensors = SensorSet {
        imus: core::array::from_fn(|_| SensorReading::<ImuData>::default()),
        gnss: core::array::from_fn(|_| SensorReading::default()),
        mags: core::array::from_fn(|_| SensorReading::default()),
        baros: core::array::from_fn(|_| SensorReading::default()),
        airspeeds: core::array::from_fn(|_| SensorReading::default()),
        geometry: None,
    };
    sensors.imus[0] = SensorReading {
        value: ImuData {
            accel: [
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(0.0),
                MetersPerSecondSquared(-9.81),
            ],
            gyro: [RadiansPerSecond(0.0); 3],
        },
        valid: true,
        source_id: 0,
        timestamp: ts,
        health: SensorHealth::Good,
    };
    sensors
}

fn set_estimate(kernel: &mut GatingKernel, valid_flags: StateValidFlags) {
    kernel.state.estimator.0 = StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0), Meters(0.0), Meters(-10.0)],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: EstimateQuality::Good,
        valid_flags,
    };
}

#[test]
fn kernel_reports_fallen_back_mode_honestly_when_position_invalid() {
    let mut kernel = make_kernel();
    set_estimate(
        &mut kernel,
        StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY,
    );
    let sensors = minimal_sensors();
    let cmd = position_command();
    let actuator_state = ActuatorState::default();

    let result = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );

    assert_eq!(
        result.status.mode,
        ControlMode::AltitudeHold,
        "ChannelStatus.mode must report the mode actually flown, not \
         the raw PositionHold request (no silent lying)"
    );
    assert_eq!(
        result.status.mode_entry,
        ModeEntryDecision::FallenBack {
            requested: ControlMode::PositionHold,
            effective: ControlMode::AltitudeHold,
            missing: StateValidFlags::POSITION,
        }
    );
    assert!(
        result.actuator.outputs.iter().any(|o| o.0 > 0.05),
        "AltitudeHold fallback must keep motors running, not cut them: {:?}",
        result.actuator.outputs
    );
}

#[test]
fn kernel_reports_fallen_back_to_attitude_when_position_and_velocity_invalid() {
    let mut kernel = make_kernel();
    set_estimate(&mut kernel, StateValidFlags::ATTITUDE);
    let sensors = minimal_sensors();
    let cmd = position_command();
    let actuator_state = ActuatorState::default();

    let result = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );

    assert_eq!(result.status.mode, ControlMode::Attitude);
    assert_eq!(
        result.status.mode_entry,
        ModeEntryDecision::FallenBack {
            requested: ControlMode::PositionHold,
            effective: ControlMode::Attitude,
            missing: StateValidFlags::POSITION | StateValidFlags::VELOCITY,
        }
    );
}

#[test]
fn kernel_grants_position_mode_unmodified_when_estimate_is_fully_valid() {
    let mut kernel = make_kernel();
    set_estimate(&mut kernel, StateValidFlags::all());
    let sensors = minimal_sensors();
    let cmd = position_command();
    let actuator_state = ActuatorState::default();

    let result = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );

    assert_eq!(result.status.mode, ControlMode::PositionHold);
    assert_eq!(
        result.status.mode_entry,
        ModeEntryDecision::Granted(ControlMode::PositionHold)
    );
}

#[test]
fn mode_gate_recovers_once_validity_is_restored_not_latched() {
    // Design decision (see PR notes): mode-entry gating is a fresh,
    // per-cycle ENTRY check layered on top of the control-law
    // severity state machine (`handle_degradation`), not a second
    // sticky failsafe. `handle_degradation` only ever escalates
    // severity — deliberately, so a channel doesn't silently self-
    // heal out of a real failsafe mid-flight — but this gate answers
    // a narrower question ("is *this* mode legal *this* cycle?") and
    // must track the estimator honestly in both directions: an OEM
    // that reacquires GNSS should regain Position without a
    // disarm/rearm cycle.
    let mut kernel = make_kernel();
    let sensors = minimal_sensors();
    let cmd = position_command();
    let actuator_state = ActuatorState::default();

    set_estimate(
        &mut kernel,
        StateValidFlags::ATTITUDE | StateValidFlags::VELOCITY,
    );
    let degraded = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );
    assert_eq!(degraded.status.mode, ControlMode::AltitudeHold);

    set_estimate(&mut kernel, StateValidFlags::all());
    let recovered = kernel.update(
        ChannelId::PRIMARY,
        dt_100hz(),
        &sensors,
        &cmd,
        0,
        &actuator_state,
        None,
    );
    assert_eq!(
        recovered.status.mode,
        ControlMode::PositionHold,
        "mode-entry gating must recover the requested mode once its \
         validity requirement is met again, not latch the fallback"
    );
    assert_eq!(
        recovered.status.mode_entry,
        ModeEntryDecision::Granted(ControlMode::PositionHold)
    );
}
