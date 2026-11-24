//! TST-CTL-212: the direct board gives the actuator authority that the
//! host declares to the fixed-wing law.
//!
//! A pilot releases the servo clutch of a C172 and holds the controls
//! where they are for 6 s. The flight controller stays armed and its
//! command asks for a turn and a climb that the pilot does not fly.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use aviate_core::control::fixed_wing::{
    EnergyLoopState, FixedWingRuntimeState, LateralLoopState, RateLoopState,
};
use aviate_core::control::{ControlMode, Setpoint};
use aviate_core::sensor::ActuatorAuthority;
use aviate_xil_direct::{DirectActuatorCmd, DirectNoiseTier};

#[path = "plant/flight.rs"]
#[allow(dead_code)]
mod flight;
use flight::{course_speed_height, Flight, DT_S};

/// Steps of the 6 s that the pilot flies by hand.
const HAND_FLOWN_STEPS: u64 = 1_500;
/// Steps between two flight commands: 0.1 s.
const COMMAND_STEPS: u64 = 25;
/// Lane of the aileron command in the actuator outputs.
const AILERON: usize = 1;

const VELOCITY: ControlMode = ControlMode::VelocityControl;

fn cruise() -> Setpoint {
    course_speed_height(0.0, 55.0, 300.0)
}

/// A turn to the east and a climb of 100 m.
fn turn_and_climb() -> Setpoint {
    course_speed_height(core::f32::consts::FRAC_PI_2, 55.0, 400.0)
}

/// A C172 in settled cruise with the servos engaged.
fn settled_cruise() -> Flight {
    let mut flight = Flight::c172_with_probe(DirectNoiseTier::Off);
    let _settle = flight.fly(20.0, VELOCITY, |_| cruise());
    flight
}

fn law(flight: &Flight) -> &FixedWingRuntimeState {
    &flight.board.kernel().state.controller
}

/// Step the board on the next sensor samples while the pilot holds
/// `hands` on the controls, and return the command of the board.
fn hand_flown_step(flight: &mut Flight, step: u64, hands: DirectActuatorCmd) -> DirectActuatorCmd {
    if step.is_multiple_of(COMMAND_STEPS) {
        flight.send(VELOCITY, turn_and_climb());
    }
    let sensors = flight.next_sensors();
    let command = flight.board.step_sensors(&sensors);
    flight.apply(DirectActuatorCmd {
        timestamp_us: command.timestamp_us,
        ..hands
    });
    command
}

/// Fly 6 s by hand and return the first command after the pilot lets
/// the servos fly the turn and the climb again.
fn hand_fly_then_engage(flight: &mut Flight, released: ActuatorAuthority) -> DirectActuatorCmd {
    let hands = flight.last;
    flight.board.set_actuator_authority(released);
    for step in 0..HAND_FLOWN_STEPS {
        let _ = hand_flown_step(flight, step, hands);
    }
    flight
        .board
        .set_actuator_authority(ActuatorAuthority::Available);
    hand_flown_step(flight, HAND_FLOWN_STEPS, hands)
}

#[test]
fn the_board_reports_authority_until_the_host_declares_its_loss() {
    let mut flight = settled_cruise();
    assert_eq!(
        flight.board.actuator_authority(),
        ActuatorAuthority::Available
    );
    assert_eq!(
        law(&flight).actuator_authority,
        ActuatorAuthority::Available
    );

    flight
        .board
        .set_actuator_authority(ActuatorAuthority::Absent);
    assert_eq!(flight.board.actuator_authority(), ActuatorAuthority::Absent);
    // The law sees the discrete in the next kernel cycle.
    assert_eq!(
        law(&flight).actuator_authority,
        ActuatorAuthority::Available
    );
    let hands = flight.last;
    let _ = hand_flown_step(&mut flight, 0, hands);
    assert_eq!(law(&flight).actuator_authority, ActuatorAuthority::Absent);

    // A reset of the world clears the law, not the discrete of the host.
    let mut restart = flight.next_sensors();
    restart.reset_generation += 1;
    let _ = flight.board.step_sensors(&restart);
    assert_eq!(
        law(&flight).actuator_authority,
        ActuatorAuthority::Available
    );
    assert_eq!(flight.board.actuator_authority(), ActuatorAuthority::Absent);
}

#[test]
fn released_servos_leave_no_control_memory_in_the_law() {
    let mut flight = settled_cruise();
    // Settled cruise holds the elevator trim in the pitch-rate memory.
    assert_ne!(law(&flight).rates, RateLoopState::default());
    let hands = flight.last;
    let start_height = flight.plant.height();

    flight
        .board
        .set_actuator_authority(ActuatorAuthority::Absent);
    for step in 0..HAND_FLOWN_STEPS {
        let command = hand_flown_step(&mut flight, step, hands);
        let law = law(&flight);
        assert_eq!(law.actuator_authority, ActuatorAuthority::Absent);
        assert_eq!(law.rates, RateLoopState::default(), "step {step}");
        assert_eq!(law.energy, EnergyLoopState::default(), "step {step}");
        assert_eq!(law.lateral, LateralLoopState::default(), "step {step}");
        // The board still returns the command of the law: it asks for
        // the turn that the pilot does not fly.
        assert!(command.armed);
        assert!(law.targets.bank > 0.3, "bank target {}", law.targets.bank);
        assert!(command.outputs[AILERON] > hands.outputs[AILERON] + 0.05);
    }
    // The pilot, not the law, flew the aeroplane: no turn and no climb.
    assert!(flight.plant.roll.abs() < 0.05, "roll {}", flight.plant.roll);
    assert!((flight.plant.height() - start_height).abs() < 10.0);
    assert!((HAND_FLOWN_STEPS as f32 * DT_S - 6.0).abs() < 1.0e-3);
}

#[test]
fn a_host_that_declares_nothing_winds_the_law_up_and_one_that_does_gets_a_fresh_law() {
    // The host does not declare the release: the law believes that its
    // outputs fly the aeroplane, and its roll-rate and pitch-rate
    // memories run to the 0.3 limit of the C172 description.
    let mut undeclared = settled_cruise();
    let wound = hand_fly_then_engage(&mut undeclared, ActuatorAuthority::Available);
    let memory = law(&undeclared);
    let [roll_memory, pitch_memory, _yaw_memory] = memory.rates.integrators;
    assert!(roll_memory > 0.25, "{:?}", memory.rates);
    assert!(pitch_memory > 0.25, "{:?}", memory.rates);
    // The climb needs full throttle, so the throttle memory does not
    // run; the pitch memory of the energy balance does.
    assert!(memory.energy.pitch_integrator > 0.02, "{:?}", memory.energy);

    // The same flight with the release declared: the first cycle with
    // authority starts from empty memory, so its roll command is the
    // smaller one.
    let mut declared = settled_cruise();
    let fresh = hand_fly_then_engage(&mut declared, ActuatorAuthority::Absent);
    let memory = law(&declared);
    assert_eq!(memory.actuator_authority, ActuatorAuthority::Available);
    for held in memory.rates.integrators {
        // One cycle of integration is all that the memory holds.
        assert!(held.abs() < 0.005, "{:?}", memory.rates);
    }
    assert!(memory.energy.pitch_integrator.abs() < 0.001);
    assert!(
        fresh.outputs[AILERON] < wound.outputs[AILERON] - 0.05,
        "fresh {} wound {}",
        fresh.outputs[AILERON],
        wound.outputs[AILERON]
    );
}
