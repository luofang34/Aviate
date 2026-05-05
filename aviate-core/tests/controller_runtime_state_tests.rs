//! TST-CTL-102: stateful controller regression suite.
//!
//! Defines a fake `TestStatefulController` whose `RuntimeState`
//! carries a counter incremented on every `step()` call. Drives the
//! kernel lifecycle methods (`ground_reset`, `disarm`,
//! `check_critical_faults`, `handle_degradation` to `Backup` /
//! `Alternate`) and asserts that the runtime counter is reset on
//! exactly the transitions LLR-CTL-101 enumerates — and not on
//! transitions outside that set.

use aviate_core::checks::in_flight::DegradationReason;
use aviate_core::checks::{KernelChecks, PreArmFlags};
use aviate_core::control::runtime::ControllerRuntimeState;
use aviate_core::control::{
    AxisCommand, Command, CommandSource, ConfigMode, ControlLawV1, ControlMode, Limits, Setpoint,
    VehicleController,
};
use aviate_core::ekf::Ekf;
use aviate_core::fault::FaultFlags;
use aviate_core::kernel::pipeline::KernelPipeline;
use aviate_core::kernel::AviateKernelImpl;
use aviate_core::math::Quaternion;
use aviate_core::mixer::{ModeConfig, QuadXMixer, Sanitizer};
use aviate_core::state::StateEstimate;
use aviate_core::time::{TimeSource, Timestamp};
use aviate_core::types::{
    Meters, MetersPerSecond, Normalized, NormalizedSigned, Radians, RadiansPerSecond,
};

#[derive(Clone, Debug, Default, PartialEq, Eq)]
struct TestRuntime {
    counter: u32,
}

impl ControllerRuntimeState for TestRuntime {
    fn reset(&mut self) {
        self.counter = 0;
    }
}

impl aviate_core::replicable::Replicable for TestRuntime {
    const ENCODED_LEN: usize = 4;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        aviate_core::replicable::copy_into(buf, 0, &self.counter.to_le_bytes())
    }
}

struct TestStatefulController;

impl VehicleController for TestStatefulController {
    type RuntimeState = TestRuntime;

    const ALGORITHM_ID: u64 = 0x4354_4C54_4553_5431; // "CTLTEST1"

    fn step(
        &self,
        runtime: &mut TestRuntime,
        _state: &StateEstimate,
        _command: &Command,
        _mode: ConfigMode,
        _limits: &Limits,
    ) -> AxisCommand {
        runtime.counter = runtime.counter.wrapping_add(1);
        AxisCommand {
            roll: NormalizedSigned(0.0),
            pitch: NormalizedSigned(0.0),
            yaw: NormalizedSigned(0.0),
            collective: Normalized(0.0),
        }
    }
}

fn dummy_ts() -> Timestamp {
    Timestamp {
        ticks: 0,
        source: TimeSource::Internal,
    }
}

fn make_kernel() -> AviateKernelImpl<Ekf, TestStatefulController, QuadXMixer, Sanitizer> {
    AviateKernelImpl {
        pipeline: KernelPipeline::new(
            Ekf::default(),
            TestStatefulController,
            QuadXMixer {
                timestamp_source: dummy_ts,
            },
            Sanitizer,
        ),
        state: aviate_core::kernel::state::KernelState::new(KernelChecks::with_pre_arm_required(
            PreArmFlags::empty(),
        )),
        cfg: aviate_core::kernel::config::ResolvedKernelConfig {
            mode_config: ModeConfig {
                mode: ConfigMode::Hover,
                groups: &[],
            },
            ..Default::default()
        },
    }
}

fn placeholder_state() -> StateEstimate {
    StateEstimate {
        attitude: Quaternion::IDENTITY,
        angular_velocity: [RadiansPerSecond(0.0); 3],
        position_ned: [Meters(0.0); 3],
        velocity_ned: [MetersPerSecond(0.0); 3],
        quality: aviate_core::state::EstimateQuality::Good,
        valid_flags: aviate_core::state::StateValidFlags::all(),
    }
}

fn placeholder_command() -> Command {
    Command {
        source: CommandSource::Pilot,
        mode: ControlMode::Rate,
        setpoint: Setpoint::default(),
        config_mode_request: None,
        sensor_overrides: None,
        sequence: 0,
    }
}

fn placeholder_limits() -> Limits {
    Limits {
        max_roll: Radians(0.7),
        max_pitch: Radians(0.7),
        max_roll_rate: RadiansPerSecond(3.5),
        max_pitch_rate: RadiansPerSecond(3.5),
        max_yaw_rate: RadiansPerSecond(2.0),
        max_horizontal_speed: MetersPerSecond(15.0),
        max_climb_rate: MetersPerSecond(5.0),
        max_descent_rate: MetersPerSecond(5.0),
        max_altitude: Meters(120.0),
        min_altitude: Meters(-1.0),
        min_airspeed: None,
        max_airspeed: None,
        max_load_factor: 4.0,
        min_load_factor: -1.0,
    }
}

#[test]
fn step_increments_runtime_counter() {
    let controller = TestStatefulController;
    let mut runtime = TestRuntime::default();
    assert_eq!(runtime.counter, 0);
    let _ = controller.step(
        &mut runtime,
        &placeholder_state(),
        &placeholder_command(),
        ConfigMode::Hover,
        &placeholder_limits(),
    );
    assert_eq!(runtime.counter, 1);
    let _ = controller.step(
        &mut runtime,
        &placeholder_state(),
        &placeholder_command(),
        ConfigMode::Hover,
        &placeholder_limits(),
    );
    assert_eq!(runtime.counter, 2);
}

#[test]
fn ground_reset_clears_runtime_counter() {
    let mut kernel = make_kernel();
    kernel.state.controller.counter = 42;
    kernel.ground_reset();
    assert_eq!(
        kernel.state.controller.counter, 0,
        "ground_reset must clear controller runtime state (LLR-CTL-101)"
    );
}

#[test]
fn disarm_clears_runtime_counter() {
    let mut kernel = make_kernel();
    kernel.state.controller.counter = 7;
    kernel.disarm();
    assert_eq!(
        kernel.state.controller.counter, 0,
        "disarm must clear controller runtime state (LLR-CTL-101)"
    );
}

#[test]
fn critical_fault_clears_runtime_counter() {
    let mut kernel = make_kernel();
    kernel.state.controller.counter = 99;
    kernel.state.faults |= FaultFlags::ALL_IMU_FAILED;
    let entered = kernel.check_critical_faults();
    assert!(entered, "ALL_IMU_FAILED is in CRITICAL_FAULTS");
    assert_eq!(
        kernel.state.controller.counter, 0,
        "check_critical_faults must clear controller runtime state on entering Fault (LLR-CTL-101)"
    );
}

#[test]
fn degradation_to_backup_clears_runtime_counter() {
    let mut kernel = make_kernel();
    kernel.state.controller.counter = 12;
    // AttitudeLost is the only DegradationReason that maps to Backup;
    // everything else maps to Alternate.
    let event = kernel.handle_degradation(DegradationReason::AttitudeLost, dummy_ts());
    assert!(event.is_some(), "Primary -> Backup is a degradation");
    assert_eq!(kernel.state.control_law, ControlLawV1::Backup);
    assert_eq!(
        kernel.state.controller.counter, 0,
        "handle_degradation to Backup must clear controller runtime state (LLR-CTL-101)"
    );
}

#[test]
fn degradation_to_alternate_keeps_runtime_counter() {
    let mut kernel = make_kernel();
    kernel.state.controller.counter = 5;
    // PositionLost -> Alternate, which is a degradation but NOT
    // Backup; controller runtime state is preserved across this
    // transition.
    let event = kernel.handle_degradation(DegradationReason::PositionLost, dummy_ts());
    assert!(event.is_some(), "Primary -> Alternate is a degradation");
    assert_eq!(kernel.state.control_law, ControlLawV1::Alternate);
    assert_eq!(
        kernel.state.controller.counter, 5,
        "handle_degradation to Alternate must preserve controller runtime state (LLR-CTL-101)"
    );
}
