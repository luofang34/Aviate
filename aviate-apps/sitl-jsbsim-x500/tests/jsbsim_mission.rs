//! The X500 flies a closed-loop takeoff, hover and landing against a
//! live JSBSim sidecar through the typed simulator backend.
//!
//! Ignored by default: it needs `pilotage-jsbsim-sidecar` listening at
//! `AVIATE_JSBSIM_SIDECAR` (host:port), spawned at the default SITL
//! origin (47.3977419 N, 8.5455938 E, 488 m) with the X500 aircraft.
//! The CI SITL JSBSim job starts one and runs this with `--ignored`.

#![allow(clippy::expect_used, clippy::panic)]

use std::net::SocketAddr;
use std::time::Duration;

use aviate_app_sitl_jsbsim_x500::x500_controls;
use aviate_board_sitl_jsbsim::{JsbsimBoard, JsbsimConfig, JsbsimSimulatorBackend};
use aviate_hal_xil::synthesis::GeoOrigin;
use aviate_hal_xil::{
    Action, Criterion, Mission, MissionRunner, Phase, SitlConfig, VehicleConfig, XilNetConfig,
};
use world_core::DecimalYear;

const SIDECAR_ENV: &str = "AVIATE_JSBSIM_SIDECAR";

fn phase(name: &str, duration_ms: u64, action: Action, verify: Vec<Criterion>) -> Phase {
    Phase {
        name: name.to_owned(),
        duration: Duration::from_millis(duration_ms),
        action,
        verify,
    }
}

/// The closed-loop landing mission the Gazebo shards fly: settle on the
/// ground, take off under position control to 4 m, hover, land under
/// position control at no more than 1 m/s, disarm.
fn closed_loop_landing() -> Mission {
    let goto = |down: f32| Action::GoTo {
        position: [0.0, 0.0, down],
        heading: 0.0,
    };
    Mission {
        name: "jsbsim_closed_loop_landing".to_owned(),
        description: "X500 closed-loop takeoff, hover and landing on JSBSim".to_owned(),
        vehicle: VehicleConfig::default(),
        lockstep: true,
        phases: vec![
            phase("arm", 500, Action::Arm, vec![Criterion::Armed(true)]),
            phase(
                "settle",
                1_000,
                Action::Thrust(0.0),
                vec![
                    Criterion::Armed(true),
                    Criterion::StationKeeping {
                        center_ned: [0.0, 0.0, 0.0],
                        xy_tolerance: 0.2,
                        z_tolerance: 0.2,
                    },
                    Criterion::AttitudeBounded {
                        roll_pitch_max_deg: 5.0,
                    },
                ],
            ),
            phase(
                "takeoff_closed_loop",
                10_000,
                goto(-4.0),
                vec![
                    Criterion::Armed(true),
                    Criterion::MinAltitude(3.0),
                    Criterion::MaxExcursion {
                        center_ned: [0.0, 0.0, -2.0],
                        xy_max: 1.5,
                        z_max: 4.0,
                    },
                    Criterion::AttitudeBounded {
                        roll_pitch_max_deg: 25.0,
                    },
                ],
            ),
            phase(
                "hover_low",
                3_000,
                goto(-4.0),
                vec![
                    Criterion::Armed(true),
                    Criterion::MaxExcursion {
                        center_ned: [0.0, 0.0, -4.0],
                        xy_max: 3.0,
                        z_max: 3.0,
                    },
                    Criterion::AttitudeBounded {
                        roll_pitch_max_deg: 25.0,
                    },
                ],
            ),
            phase(
                "land",
                12_000,
                goto(0.0),
                vec![Criterion::TouchdownVelocity {
                    max_descent_mps: 1.0,
                    ground_tolerance: 0.30,
                }],
            ),
            phase("disarm", 500, Action::Disarm, vec![Criterion::Armed(false)]),
        ],
        reset_between_runs: true,
    }
}

#[test]
#[ignore = "needs a live JSBSim sidecar at AVIATE_JSBSIM_SIDECAR"]
fn the_x500_flies_the_closed_loop_landing_on_jsbsim() {
    let sidecar: SocketAddr = std::env::var(SIDECAR_ENV)
        .unwrap_or_else(|_| panic!("{SIDECAR_ENV} names the sidecar (host:port)"))
        .parse()
        .expect("the sidecar address is host:port");
    let mut config = JsbsimConfig::for_instance(0, GeoOrigin::DEFAULT, DecimalYear(2026.8));
    config.sidecar_addr = sidecar;
    // Port 0: the OS picks the GCS port.
    config.xil = SitlConfig::for_instance_with_net(
        0,
        XilNetConfig {
            base_port: 0,
            stride: 0,
        },
    );
    let kernel = aviate_app_sitl_gazebo_x500_kernel::build_x500_kernel().expect("X500 kernel");
    let board = JsbsimBoard::new(kernel, config, x500_controls).expect("board builds");
    let mut runner =
        MissionRunner::new(JsbsimSimulatorBackend::new(board, 0), "x500").expect("runner");
    let result = runner.run(&closed_loop_landing());
    for phase in &result.phases {
        for criterion in &phase.criteria_results {
            assert!(
                criterion.passed,
                "{}: expected {}, got {}",
                phase.name, criterion.expected, criterion.actual_value
            );
        }
    }
    assert!(result.passed, "mission failed: {result:?}");
    assert!(result.max_altitude > 3.0, "it never climbed to the hover");
}
