//! Mission-Based Test Framework
//!
//! This module provides a structured way to define and execute test missions
//! for SITL verification. Missions are defined as sequences of phases with
//! verification criteria.
//!
//! ## Design Goals
//! - Deterministic: All tests run under lockstep for reproducibility
//! - Configurable: Missions defined in TOML, not hardcoded
//! - Multi-vehicle: Support testing multiple aircraft simultaneously
//! - Extensible: Easy to add new mission types and verification criteria
//!
//! ## Example Mission (TOML)
//! ```toml
//! [mission]
//! name = "basic_takeoff_land"
//! description = "Verify takeoff to 5m and controlled landing"
//! vehicle = "x500"
//! lockstep = true
//!
//! [[phase]]
//! name = "arm"
//! duration_ms = 500
//! action = { type = "arm" }
//! verify = { armed = true }
//!
//! [[phase]]
//! name = "takeoff"
//! duration_ms = 5000
//! action = { type = "thrust", value = 0.8 }
//! verify = { min_altitude = 3.0 }
//!
//! [[phase]]
//! name = "hover"
//! duration_ms = 2000
//! action = { type = "thrust", value = 0.65 }
//! verify = { altitude_hold = { target = 5.0, tolerance = 0.5 } }
//!
//! [[phase]]
//! name = "land"
//! duration_ms = 3000
//! action = { type = "thrust", value = 0.0 }
//! verify = { max_altitude = 0.5 }
//!
//! [[phase]]
//! name = "disarm"
//! duration_ms = 500
//! action = { type = "disarm" }
//! verify = { armed = false }
//! ```

use std::time::Duration;

/// Mission definition
#[derive(Debug, Clone)]
pub struct Mission {
    pub name: String,
    pub description: String,
    pub vehicle: VehicleConfig,
    pub lockstep: bool,
    pub phases: Vec<Phase>,
    pub reset_between_runs: bool,
}

/// Vehicle configuration for a mission
#[derive(Debug, Clone)]
pub struct VehicleConfig {
    pub model: String,            // e.g., "x500", "x500_camera"
    pub instance: u8,             // For multi-vehicle (0, 1, 2...)
    pub spawn_position: [f32; 3], // Initial position [x, y, z]
    pub spawn_heading: f32,       // Initial heading (radians)
}

impl Default for VehicleConfig {
    fn default() -> Self {
        Self {
            model: "x500".to_string(),
            instance: 0,
            spawn_position: [0.0, 0.0, 0.0],
            spawn_heading: 0.0,
        }
    }
}

/// A phase in a mission (e.g., takeoff, hover, land)
#[derive(Debug, Clone)]
pub struct Phase {
    pub name: String,
    pub duration: Duration,
    pub action: Action,
    pub verify: Vec<Criterion>,
}

/// Actions that can be performed during a phase
#[derive(Debug, Clone)]
pub enum Action {
    /// Do nothing (wait)
    Wait,
    /// Arm the vehicle
    Arm,
    /// Disarm the vehicle
    Disarm,
    /// Set thrust level (0.0 - 1.0)
    Thrust(f32),
    /// Set attitude target (quaternion + thrust)
    AttitudeTarget {
        q: [f32; 4], // Quaternion [w, x, y, z]
        thrust: f32,
    },
    /// Go to position (NED)
    GoTo { position: [f32; 3], heading: f32 },
    /// Inject fault into a sensor (for SITL testing)
    InjectFault {
        /// Target sensor
        sensor: SensorTarget,
        /// Type of fault to inject
        fault: FaultSpec,
    },
    /// Clear all injected faults
    ClearFaults,
}

/// Target sensor for fault injection
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorTarget {
    Imu,
    Baro,
    Mag,
    Gnss,
}

/// Fault specification for injection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FaultSpec {
    /// Sensor reports degraded health
    HealthDegraded,
    /// Sensor has completely failed
    HealthFailed,
    /// Inject NaN values into readings
    NaN,
    /// Drop sensor data for N cycles
    Dropout { cycles: u32 },
    /// Add bias offset to readings
    BiasShift { offset: [f32; 3] },
    /// Add scalar bias (for single-value sensors like baro)
    BiasScalar { offset: f32 },
}

/// Verification criteria for a phase.
///
/// Three semantic shapes, in increasing strictness:
///
///   - **End-state**: one sample at phase end. (`Armed`,
///     `AltitudeHold`, `PositionHold`, `MaxAltitude`, `MaxDrift`,
///     `ReturnedNear`, `YawDriftBounded`, `SensorDataReceived`.)
///   - **At-some-point**: any sample in the trace must match.
///     (`MinAltitude`, `ReachedWaypoint`, `StableHover`.)
///   - **Throughout-phase**: every sample in the trace must
///     match. (`StationKeeping`, `MaxExcursion`, `AttitudeBounded`,
///     `AttitudeRateBounded`, `TelemetryAgreesWithTruth`.)
///
/// The "throughout-phase" shape is what enforces real flight —
/// a vehicle that briefly hit a target and then jumped 20 m
/// sideways fails a `StationKeeping` criterion even though it
/// would pass `MinAltitude` or `ReachedWaypoint`. New tests
/// should use the throughout-phase shape by default.
#[derive(Debug, Clone)]
pub enum Criterion {
    /// Vehicle must be armed at end of phase
    Armed(bool),
    /// Vehicle reached at least this altitude at some point during
    /// the phase (meters, positive up).
    MinAltitude(f32),
    /// Vehicle altitude at end of phase is at most this value
    MaxAltitude(f32),
    /// End-of-phase altitude is within `tolerance` of `target`
    AltitudeHold { target: f32, tolerance: f32 },
    /// End-of-phase 3D position is within `tolerance` of `target` (NED)
    PositionHold { target: [f32; 3], tolerance: f32 },
    /// End-of-phase horizontal drift from `start_position` ≤ `max`
    MaxDrift(f32),
    /// Sensor data received at least once
    SensorDataReceived,
    /// Vehicle was within `tolerance` metres of `target` at some
    /// point during the phase (NED). Pins "the FC actually reached
    /// the commanded waypoint", not "the FC ended where commanded".
    /// Useful when the next phase commands the vehicle to move on
    /// before this phase strictly ends.
    ReachedWaypoint { target: [f32; 3], tolerance: f32 },
    /// Vehicle held altitude within `[altitude-tolerance,
    /// altitude+tolerance]` continuously for at least `hold_secs`
    /// of the phase. Sliding window — the run can include
    /// transients before/after the held interval, but the
    /// interval must be contiguous.
    StableHover {
        altitude: f32,
        tolerance: f32,
        hold_secs: f32,
    },
    /// **Throughout phase**: every trace sample must be inside the
    /// box centred at `center_ned` with half-extents `xy_tolerance`
    /// (horizontal) and `z_tolerance` (vertical). A single sample
    /// outside the box FAILS the criterion. This is the canonical
    /// "the vehicle held position" check — strictly stricter than
    /// `StableHover` because it constrains XY as well as Z, and
    /// every sample as opposed to a sliding window.
    StationKeeping {
        center_ned: [f32; 3],
        xy_tolerance: f32,
        z_tolerance: f32,
    },
    /// **Throughout phase**: every trace sample's deviation from
    /// `center_ned` is bounded. Looser than `StationKeeping`
    /// (does not assert holding still); use to bound "runaway
    /// flight" while a transition or maneuver is in progress.
    MaxExcursion {
        center_ned: [f32; 3],
        xy_max: f32,
        z_max: f32,
    },
    /// **Trace-walking**: the vehicle visits each of `waypoints`
    /// in order, each within `tolerance` of its target NED point,
    /// completing the sequence within `max_time_s`. A waypoint is
    /// "visited" the first time a sample lands inside its
    /// tolerance ball; visits must occur in declaration order.
    TrajectoryTracking {
        waypoints: Vec<[f32; 3]>,
        tolerance: f32,
        max_time_s: f32,
    },
    /// **End-state**: end-of-phase 3D position within `tolerance`
    /// of `target_ned`. Used for "return to home" checks.
    ReturnedNear {
        target_ned: [f32; 3],
        tolerance: f32,
    },
    /// **Throughout phase**: every trace sample's roll and pitch
    /// (extracted from the body attitude quaternion) are below
    /// `roll_pitch_max_deg`. Yaw is unbounded — at low altitudes
    /// the vehicle yaws naturally as the EKF converges. A tumbling
    /// vehicle fails this criterion immediately.
    AttitudeBounded { roll_pitch_max_deg: f32 },
    /// **End-state**: at the moment the vehicle first touches the
    /// ground during the phase (z ≥ −`ground_tolerance` in NED),
    /// the vertical descent rate must not exceed `max_descent_mps`.
    /// A free-fall landing fails: `thrust=0` from 10 m gives ~14
    /// m/s impact, well above any safe threshold. The criterion
    /// makes "soft touchdown" a verifiable property of the
    /// controller rather than a hopeful side-effect of the
    /// mission profile.
    TouchdownVelocity {
        max_descent_mps: f32,
        ground_tolerance: f32,
    },
}

/// Result of a phase execution
#[derive(Debug)]
pub struct PhaseResult {
    pub name: String,
    pub passed: bool,
    pub duration_actual: Duration,
    pub max_altitude: f32,
    pub final_position: [f32; 3],
    pub criteria_results: Vec<CriterionResult>,
    /// Per-step trace samples from this phase. Retained on the
    /// result so the mission runner can write a CSV (or any other
    /// post-mortem artefact) covering every sample of every phase
    /// without duplicating the trace into a side channel.
    pub trace: Vec<crate::runner::TraceSample>,
    /// String describing the action commanded during this phase
    /// (e.g. `thrust=0.85`, `goto=[0,0,-5]`). Tagged at phase
    /// start so the CSV reader can see which input drove each
    /// sample.
    pub action_tag: String,
}

#[derive(Debug)]
pub struct CriterionResult {
    pub criterion: String,
    pub passed: bool,
    pub actual_value: String,
    pub expected: String,
}

/// Result of a complete mission
#[derive(Debug)]
pub struct MissionResult {
    pub mission_name: String,
    pub passed: bool,
    pub phases: Vec<PhaseResult>,
    pub total_duration: Duration,
    pub max_altitude: f32,
}

impl Mission {
    /// Create a basic takeoff-land mission (the current default test)
    pub fn basic_takeoff_land() -> Self {
        Self {
            name: "basic_takeoff_land".to_string(),
            description: "Verify takeoff to ~10m and landing".to_string(),
            vehicle: VehicleConfig::default(),
            lockstep: true,
            reset_between_runs: true,
            phases: vec![
                Phase {
                    name: "arm".to_string(),
                    duration: Duration::from_millis(500),
                    action: Action::Arm,
                    verify: vec![Criterion::Armed(true)],
                },
                Phase {
                    name: "takeoff".to_string(),
                    duration: Duration::from_secs(5),
                    action: Action::Thrust(0.8),
                    verify: vec![Criterion::MinAltitude(5.0)],
                },
                Phase {
                    name: "land".to_string(),
                    duration: Duration::from_secs(3),
                    action: Action::Thrust(0.0),
                    verify: vec![], // Just wait for descent
                },
                Phase {
                    name: "disarm".to_string(),
                    duration: Duration::from_millis(500),
                    action: Action::Disarm,
                    verify: vec![Criterion::Armed(false)],
                },
            ],
        }
    }

    /// Create a hover hold mission (tests attitude stability)
    pub fn hover_hold() -> Self {
        Self {
            name: "hover_hold".to_string(),
            description: "Takeoff, hold altitude for 10s, verify stability".to_string(),
            vehicle: VehicleConfig::default(),
            lockstep: true,
            reset_between_runs: true,
            phases: vec![
                Phase {
                    name: "arm".to_string(),
                    duration: Duration::from_millis(500),
                    action: Action::Arm,
                    verify: vec![Criterion::Armed(true)],
                },
                Phase {
                    name: "takeoff".to_string(),
                    duration: Duration::from_secs(3),
                    action: Action::Thrust(0.8),
                    verify: vec![Criterion::MinAltitude(3.0)],
                },
                Phase {
                    name: "hover".to_string(),
                    duration: Duration::from_secs(10),
                    action: Action::Thrust(0.65), // Hover thrust
                    verify: vec![
                        Criterion::AltitudeHold {
                            target: 5.0,
                            tolerance: 1.0,
                        },
                        Criterion::MaxDrift(2.0),
                    ],
                },
                Phase {
                    name: "land".to_string(),
                    duration: Duration::from_secs(5),
                    action: Action::Thrust(0.0),
                    verify: vec![Criterion::MaxAltitude(0.5)],
                },
                Phase {
                    name: "disarm".to_string(),
                    duration: Duration::from_millis(500),
                    action: Action::Disarm,
                    verify: vec![Criterion::Armed(false)],
                },
            ],
        }
    }

    /// Verify total mission duration
    pub fn total_duration(&self) -> Duration {
        self.phases.iter().map(|p| p.duration).sum()
    }
}

/// Multi-vehicle mission for swarm/formation testing
#[derive(Debug, Clone)]
pub struct MultiVehicleMission {
    pub name: String,
    pub description: String,
    pub lockstep: bool,
    pub vehicles: Vec<VehicleConfig>,
    pub phases: Vec<MultiVehiclePhase>,
}

/// Phase for multi-vehicle missions
#[derive(Debug, Clone)]
pub struct MultiVehiclePhase {
    pub name: String,
    pub duration: Duration,
    /// Actions per vehicle (indexed by vehicle instance)
    pub actions: Vec<Action>,
    /// Verification criteria (can reference multiple vehicles)
    pub verify: Vec<MultiVehicleCriterion>,
}

/// Verification criteria for multi-vehicle scenarios
#[derive(Debug, Clone)]
pub enum MultiVehicleCriterion {
    /// All vehicles meet a single-vehicle criterion
    All(Criterion),
    /// Minimum separation between vehicles
    MinSeparation(f32),
    /// Formation shape maintained
    FormationHold {
        offsets: Vec<[f32; 3]>,
        tolerance: f32,
    },
}

impl MultiVehicleMission {
    /// Create a basic two-vehicle formation test
    pub fn two_vehicle_formation() -> Self {
        Self {
            name: "two_vehicle_formation".to_string(),
            description: "Two vehicles takeoff and maintain 5m separation".to_string(),
            lockstep: true,
            vehicles: vec![
                VehicleConfig {
                    model: "x500".to_string(),
                    instance: 0,
                    spawn_position: [0.0, 0.0, 0.0],
                    spawn_heading: 0.0,
                },
                VehicleConfig {
                    model: "x500".to_string(),
                    instance: 1,
                    spawn_position: [5.0, 0.0, 0.0], // 5m east
                    spawn_heading: 0.0,
                },
            ],
            phases: vec![
                MultiVehiclePhase {
                    name: "arm_all".to_string(),
                    duration: Duration::from_millis(500),
                    actions: vec![Action::Arm, Action::Arm],
                    verify: vec![MultiVehicleCriterion::All(Criterion::Armed(true))],
                },
                MultiVehiclePhase {
                    name: "takeoff_all".to_string(),
                    duration: Duration::from_secs(5),
                    actions: vec![Action::Thrust(0.8), Action::Thrust(0.8)],
                    verify: vec![
                        MultiVehicleCriterion::All(Criterion::MinAltitude(3.0)),
                        MultiVehicleCriterion::MinSeparation(4.0),
                    ],
                },
                MultiVehiclePhase {
                    name: "land_all".to_string(),
                    duration: Duration::from_secs(5),
                    actions: vec![Action::Thrust(0.0), Action::Thrust(0.0)],
                    verify: vec![MultiVehicleCriterion::All(Criterion::MaxAltitude(0.5))],
                },
                MultiVehiclePhase {
                    name: "disarm_all".to_string(),
                    duration: Duration::from_millis(500),
                    actions: vec![Action::Disarm, Action::Disarm],
                    verify: vec![MultiVehicleCriterion::All(Criterion::Armed(false))],
                },
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_mission_duration() {
        let mission = Mission::basic_takeoff_land();
        let total = mission.total_duration();
        assert!(total >= Duration::from_secs(8));
    }

    #[test]
    fn test_mission_has_phases() {
        let mission = Mission::hover_hold();
        assert!(mission.phases.len() >= 4);
        assert!(mission.lockstep);
    }
}
