# Aviate Spec v0.5 (Architecture-Complete)

**Minimal Deterministic Vehicle Motion Control Core**

---

## 0. Document Status

| Version | Date | Status |
|---------|------|--------|
| v0.1 | - | Initial architecture |
| v0.2 | - | Redundancy-aware interfaces |
| v0.3 | - | Complete type definitions |
| v0.4 | - | Behavioral semantics, fault binding |
| v0.5 | - | Language profile, vector safety, morphing support |

**Companion Documents**:
- `AVIATE_LANGUAGE_PROFILE.md` — LLM/CI constraint specification

---

## 1. Project Definition (One Sentence)

Aviate is a minimal, deterministic, hard-real-time motion control kernel responsible only for state estimation and stabilization control — not navigation, communication, mission management, or human-machine interface.

---

## 2. Design Philosophy

### 2.1 Aviate Does Exactly Three Things

1. **State Estimation** — Attitude, position, velocity, angular rate estimation
2. **Stabilization & Control** — Rate loop, attitude loop, velocity/altitude/position hold
3. **Actuation Output** — Force/torque commands mapped to actuator outputs via mixer

### 2.2 Aviate Never Does

- ❌ Navigation (waypoints, procedures, LNAV/VNAV)
- ❌ Mission systems / autopilot management
- ❌ Maps / charts / databases
- ❌ Networking (TCP/UDP/WiFi/LTE)
- ❌ File systems / logging
- ❌ UI / GCS / cloud platforms
- ❌ Operating system dependencies

---

## 3. Language & Implementation Profile

*See companion document: `AVIATE_LANGUAGE_PROFILE.md` for complete LLM-friendly specification.*

### 3.1 Summary of Constraints (Flight Build)

| Constraint | Rule |
|------------|------|
| Runtime | `#![no_std]`, `core` only |
| Memory | No heap allocation |
| Safety | `#![forbid(unsafe_code)]` in core |
| Recursion | Forbidden |
| Loops | Statically bounded |
| Panics | Forbidden (`unwrap`, `expect`, `panic!`) |
| Concurrency | No threads, no async, no interrupt-driven state |

### 3.2 Numeric Policy

- Base type: `Scalar = f32`
- Physical quantities: Dimensional newtypes only
- NaN/Inf: Never propagate, trigger fault on detection

---

## 4. Configuration Modes & Morphing Support

### 4.1 Design Rationale

For morphing aircraft (VTOL, folding-wing, etc.), the same physical actuators have **different coupling semantics** in different flight phases:

| Phase | Quadrotor Motors | Coupling | Single Failure |
|-------|------------------|----------|----------------|
| Hover | Lift + attitude | **Strong** | Catastrophic |
| Cruise (as pullers) | Distributed thrust | **Weak** | Degraded performance |

Therefore: **Coupling is per-mode, not a fixed actuator property.**

### 4.2 Configuration Mode (Discrete State)

```rust
/// Flight configuration mode (discrete)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigMode {
    /// Multirotor hover mode
    Hover,
    /// Transitioning between modes
    Transition,
    /// Fixed-wing cruise mode
    Cruise,
    /// Degraded configuration after morphing failure
    Degraded,
}

/// Complete configuration for one flight mode
#[derive(Clone, Debug)]
pub struct ModeConfig {
    pub mode: ConfigMode,
    pub mixer: MixerConfig,
    pub groups: &'static [ActuatorGroupConfig],
    pub limits: Limits,
    pub law_profile: LawProfile,
}
```

### 4.3 Geometry State (Continuous)

```rust
/// Continuous geometry state for morphing aircraft
#[derive(Clone, Debug)]
pub struct GeometryState {
    /// Arm/wing fold angles [rad]
    pub fold_angles: [Radians; MAX_FOLD_JOINTS],
    /// Rotor positions relative to CoG, body frame [m]
    pub rotor_positions: [[Meters; 3]; MAX_ROTORS],
    /// Wing sweep angle [rad]
    pub wing_sweep: Radians,
    /// Geometry validity flag
    pub valid: bool,
}

pub const MAX_FOLD_JOINTS: usize = 8;
pub const MAX_ROTORS: usize = 8;
```

### 4.4 Configuration Transition State Machine

```rust
/// Configuration transition state
#[derive(Clone, Debug)]
pub enum ConfigTransitionState {
    /// Stable in a configuration mode
    Stable(ConfigMode),
    
    /// Actively transitioning between modes
    Switching {
        from: ConfigMode,
        to: ConfigMode,
        /// Progress 0.0 (start) to 1.0 (complete)
        progress: Scalar,
        /// Geometry state during transition
        geometry: GeometryState,
    },
    
    /// Transition failed, in degraded configuration
    Failed {
        intended: ConfigMode,
        actual: ConfigMode,
        reason: TransitionFailure,
    },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransitionFailure {
    /// Morphing actuator stuck
    ActuatorStuck,
    /// Left/right asymmetry detected
    Asymmetry,
    /// Transition timeout
    Timeout,
    /// Unsafe flight conditions
    UnsafeConditions,
}
```

### 4.5 Transition Safety Rules

```
RULE 1: Mode transitions only allowed when:
        - Airspeed/altitude within transition envelope
        - Attitude within limits
        - All morphing actuators reporting healthy

RULE 2: During transition:
        - Use dedicated Transition ModeConfig
        - More conservative Limits
        - Mixer interpolates between source/target geometry

RULE 3: On transition failure:
        - Abort to nearest safe ConfigMode
        - Set FaultFlags::CONFIG_TRANSITION_FAILED
        - Enter Degraded ModeConfig
        
RULE 4: ConfigMode determines which ModeConfig is active,
        which determines actuator grouping and coupling semantics.
```

---

## 5. Authority & Law Profile

### 5.1 Authority Philosophy

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AuthorityProfile {
    HardEnvelope,  // Airbus-style: inviolable boundary
    SoftEnvelope,  // Boeing-style: resistance + warnings
}
```

### 5.2 Law Profile

```rust
#[derive(Clone, Debug)]
pub struct LawProfile {
    pub authority: AuthorityProfile,
    pub chain: &'static [ControlLaw],
    pub capabilities: &'static [LawCapabilities],
}

bitflags::bitflags! {
    pub struct LawCapabilities: u32 {
        const ATTITUDE_PROTECTION = 0x0001;
        const LOAD_FACTOR_PROTECTION = 0x0002;
        const STALL_PROTECTION = 0x0004;
        const OVERSPEED_PROTECTION = 0x0008;
        const ALTITUDE_PROTECTION = 0x0010;
        const BANK_ANGLE_PROTECTION = 0x0020;
        const AUTOTRIM = 0x0040;
        const YAW_DAMPER = 0x0080;
        const TURN_COORDINATION = 0x0100;
        const POSITION_HOLD = 0x0200;
        const ALTITUDE_HOLD = 0x0400;
        const HEADING_HOLD = 0x0800;
        
        const FULL = Self::all().bits();
        const REDUCED_ENVELOPE = 0x0007;
        const BASIC_STABILIZATION = 0x0180;
        const NONE = 0;
    }
}
```

---

## 5. Numeric Representation

### 5.1 Base Types

```rust
pub type Scalar = f32;

// Dimensional newtypes (mandatory for physics)
pub struct Meters(pub Scalar);
pub struct MetersPerSecond(pub Scalar);
pub struct MetersPerSecondSquared(pub Scalar);
pub struct RadiansPerSecond(pub Scalar);
pub struct Radians(pub Scalar);
pub struct Seconds(pub Scalar);
pub struct Normalized(pub Scalar);      // [0.0, 1.0]
pub struct NormalizedSigned(pub Scalar); // [-1.0, 1.0]
```

### 5.2 Validation Trait

```rust
pub trait Validated {
    fn is_valid(&self) -> bool;
    fn sanitize_or_default(&self, default: Self) -> Self;
}

impl Validated for Scalar {
    fn is_valid(&self) -> bool { self.is_finite() }
    fn sanitize_or_default(&self, default: Self) -> Self {
        if self.is_finite() { *self } else { default }
    }
}
```

---

## 6. Time Model

```rust
pub const TICK_FREQUENCY_HZ: u64 = 1_000_000;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeSource { Internal, Gps, Ptp }

#[derive(Copy, Clone, Debug)]
pub struct Timestamp {
    pub ticks: u64,
    pub source: TimeSource,
}

#[derive(Copy, Clone, Debug)]
pub struct TimeDelta {
    pub dt_sec: Seconds,
    pub tick_delta: u64,
}
```

---

## 7. Sensor Model

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SensorHealth { Good, Degraded, Failed, NotAvailable }

#[derive(Copy, Clone, Debug)]
pub struct SensorReading<T> {
    pub value: T,
    pub valid: bool,
    pub source_id: u8,
    pub timestamp: Timestamp,
    pub health: SensorHealth,
}

pub const MAX_IMU: usize = 3;
pub const MAX_GNSS: usize = 2;
pub const MAX_MAG: usize = 2;
pub const MAX_BARO: usize = 2;
pub const MAX_AIRSPEED: usize = 2;

#[derive(Clone, Debug)]
pub struct SensorSet {
    pub imus: [SensorReading<ImuData>; MAX_IMU],
    pub gnss: [SensorReading<GnssData>; MAX_GNSS],
    pub mags: [SensorReading<MagData>; MAX_MAG],
    pub baros: [SensorReading<BaroData>; MAX_BARO],
    pub airspeeds: [SensorReading<AirspeedData>; MAX_AIRSPEED],
}
```

---

## 8. Actuator Model

### 8.1 Actuator Types

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ActuatorKind {
    Motor,
    MotorBidirectional,
    Servo,
    TiltServo,
    Wheel,
    Flap,
    Spoiler,
    MorphingJoint,  // NEW: for folding mechanisms
    Custom(u8),
}

#[derive(Copy, Clone, Debug)]
pub struct ActuatorChannelConfig {
    pub kind: ActuatorKind,
    pub output_min: u16,
    pub output_max: u16,
    pub safe_output: Normalized,
    pub enabled: bool,
    // NOTE: coupling_group is NOT here - it's in ActuatorGroupConfig per mode
}

pub const MAX_ACTUATORS: usize = 16;
```

### 8.2 Actuator Group Configuration (Per-Mode)

**Critical**: Coupling semantics are determined by flight mode, not fixed actuator properties.

```rust
/// Group kind - semantic role of this actuator group
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GroupKind {
    /// Multirotor lift rotors (hover mode)
    Multirotor,
    /// Distributed thrust (cruise mode pullers)
    DistributedThrust,
    /// Fixed-wing control surfaces
    ControlSurfaces,
    /// Morphing/folding mechanisms
    Morphing,
    /// Landing gear, doors, etc.
    Auxiliary,
    Custom(u8),
}

/// Coupling semantics within a group
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CouplingKind {
    /// Any channel fault → entire group fallback
    /// Use for: hover-mode quadrotor, differential ailerons
    Strong,
    
    /// Per-channel fallback allowed
    /// Use for: cruise-mode distributed thrust, independent servos
    Weak,
}

/// Fallback strategy when fault detected
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FallbackPolicy {
    /// Hold last known good vector for entire group
    HoldLastGood,
    /// Decay to safe pattern with time constant
    DecayToSafe { tau_ms: u16 },
    /// Immediately jump to safe pattern
    SafePattern,
}

/// Configuration for one actuator group (defined per ConfigMode)
#[derive(Clone, Debug)]
pub struct ActuatorGroupConfig {
    pub kind: GroupKind,
    pub coupling: CouplingKind,
    pub fallback: FallbackPolicy,
    /// Member channel indices
    pub members: &'static [u8],
    /// Safe pattern for this group (used by SafePattern/DecayToSafe)
    pub safe_pattern: [Normalized; MAX_ACTUATORS],
}
```

### 8.3 Example: Same Motors, Different Coupling by Mode

```rust
// HOVER MODE: Quadrotor motors are strongly coupled
const HOVER_ROTOR_GROUP: ActuatorGroupConfig = ActuatorGroupConfig {
    kind: GroupKind::Multirotor,
    coupling: CouplingKind::Strong,  // ← Any fault = all fallback
    fallback: FallbackPolicy::HoldLastGood,
    members: &[0, 1, 2, 3],
    safe_pattern: [Normalized(0.15); MAX_ACTUATORS], // Idle
};

// CRUISE MODE: Same motors as distributed thrust, weakly coupled
const CRUISE_THRUST_GROUP: ActuatorGroupConfig = ActuatorGroupConfig {
    kind: GroupKind::DistributedThrust,
    coupling: CouplingKind::Weak,    // ← Per-channel fallback OK
    fallback: FallbackPolicy::SafePattern,
    members: &[0, 1, 2, 3],
    safe_pattern: [Normalized(0.0); MAX_ACTUATORS], // Feathered/idle
};

// Different ModeConfigs reference different group configs
const HOVER_MODE: ModeConfig = ModeConfig {
    mode: ConfigMode::Hover,
    groups: &[HOVER_ROTOR_GROUP, /* ... */],
    // ...
};

const CRUISE_MODE: ModeConfig = ModeConfig {
    mode: ConfigMode::Cruise,
    groups: &[CRUISE_THRUST_GROUP, /* ... */],
    // ...
};
```

### 8.4 Coupling Rationale by Phase

| Mode | Actuators | Coupling | Single-Failure Consequence |
|------|-----------|----------|---------------------------|
| Hover | Quad motors | **Strong** | Loss of attitude/lift control → catastrophic |
| Cruise | Same as pullers | **Weak** | Asymmetric thrust → trim + reduced performance |
| Hover | Ailerons | N/A (inactive) | - |
| Cruise | Ailerons | **Strong** | Roll authority loss |
| Any | Landing gear | **Weak** | Degraded but manageable |
```

### 8.5 Actuator Commands

```rust
#[derive(Clone, Debug)]
pub struct ActuatorCmd {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub active_mask: u16,
    pub sequence: u32,
    pub timestamp: Timestamp,
    /// Which groups had fallback this cycle (bitmask by group index)
    pub fallback_mask: u8,
    /// Was any sanitization performed?
    pub sanitized: bool,
}
```

### 8.6 Vector-Level Fallback State

```rust
/// Tracks last-known-good actuator vectors per group
#[derive(Clone, Debug)]
pub struct ActuatorFallbackState {
    /// Last valid output for each group (up to 8 groups)
    pub last_good: [GroupVector; MAX_GROUPS],
    /// Age of last good vector (cycles since update)
    pub age: [u16; MAX_GROUPS],
}

pub const MAX_GROUPS: usize = 8;

#[derive(Clone, Debug)]
pub struct GroupVector {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub mask: u16,
    pub valid: bool,
}
```

---

## 9. Actuator Output Sanitization (CRITICAL SAFETY)

### 9.1 Design Rationale

**Problem**: Per-channel sanitization (replacing one bad channel with a default while keeping others) creates catastrophic torque imbalance on coupled systems like quadrotors.

**Solution**: Vector-level sanitization — if any channel in a coupled group fails numeric validation, the ENTIRE group falls back to a coherent safe vector.

### 9.2 Sanitization Rules

```
RULE 1: Numeric validation is per-channel, but fallback is per-group.

RULE 2: For any strongly coupled actuator group (e.g., multirotor lift rotors),
        detection of a numeric error in ANY member SHALL cause rejection of
        the ENTIRE newly computed actuator vector for that group in this cycle.

RULE 3: A coherent fallback vector SHALL be used instead. Options (in order):
        a) Last known good vector for this group (if age < MAX_FALLBACK_AGE)
        b) Predefined safe vector for this group (e.g., symmetric descent thrust)
        c) Zero output (only if safe vector undefined and last good expired)

RULE 4: Independent actuators (landing gear, etc.) MAY use per-channel fallback.

RULE 5: Fallback events MUST be reported via:
        - ActuatorCmd.fallback_groups flags
        - FaultFlags::ACTUATOR_NUMERIC_ERROR
        - Status reporting to upper systems
```

### 9.3 Sanitization Implementation

```rust
/// Sanitization result for one coupling group
#[derive(Copy, Clone, Debug)]
pub enum GroupSanitizeResult {
    /// All channels valid, no action needed
    AllValid,
    /// Minor clamping performed (within tolerance)
    Clamped,
    /// Fallback required - used last good vector
    FallbackLastGood,
    /// Fallback required - used predefined safe vector
    FallbackSafe,
    /// Fallback required - no valid fallback available (CRITICAL)
    FallbackUnavailable,
}

pub trait ActuatorSanitizer {
    /// Sanitize actuator command before hardware output
    /// 
    /// For each coupling group:
    /// 1. Check all channels in group for NaN/Inf/out-of-range
    /// 2. If ANY channel invalid → reject entire group's new values
    /// 3. Replace with coherent fallback vector
    /// 4. Update last_good if this cycle was valid
    fn sanitize(
        &mut self,
        cmd: &mut ActuatorCmd,
        config: &MixerConfig,
    ) -> SanitizeReport;
}

#[derive(Clone, Debug)]
pub struct SanitizeReport {
    pub group_results: [GroupSanitizeResult; MAX_COUPLING_GROUPS],
    pub any_fallback: bool,
    pub critical_failure: bool,  // true if FallbackUnavailable occurred
}
```

### 9.4 Fallback Aging and Limits

```rust
/// How many cycles a "last good" vector remains usable
pub const MAX_FALLBACK_AGE_CYCLES: u16 = 100;  // 100ms at 1kHz

/// Maximum consecutive fallback cycles before triggering degradation
pub const MAX_CONSECUTIVE_FALLBACK: u16 = 10;  // 10ms

/// After MAX_CONSECUTIVE_FALLBACK, force ControlLaw degradation
/// to prevent indefinite operation on stale/safe vectors
```

### 9.5 Per-Vehicle Safe Vectors

```rust
/// MixerConfig must define safe fallback vectors
#[derive(Clone, Debug)]
pub struct MixerConfig {
    pub channels: [ActuatorChannelConfig; MAX_ACTUATORS],
    
    /// Safe fallback vectors per coupling group
    /// These represent physically reasonable states, not just "zero"
    pub safe_vectors: SafeVectorSet,
}

#[derive(Clone, Debug)]
pub struct SafeVectorSet {
    /// Lift rotors: symmetric thrust for controlled descent
    /// e.g., [0.3, 0.3, 0.3, 0.3] for gentle descent
    pub lift_rotors: Option<[Normalized; MAX_ACTUATORS]>,
    
    /// Primary controls: neutral/centered
    /// e.g., [0.5, 0.5, 0.5, 0.5] for centered servos
    pub primary_controls: Option<[Normalized; MAX_ACTUATORS]>,
    
    /// Tilt mechanism: hover position
    pub tilt_mechanism: Option<[Normalized; MAX_ACTUATORS]>,
}

impl SafeVectorSet {
    /// Quadrotor default: 30% throttle symmetric descent
    pub const QUADROTOR_DEFAULT: Self = Self {
        lift_rotors: Some([
            Normalized(0.3), Normalized(0.3), 
            Normalized(0.3), Normalized(0.3),
            // ... rest zeros
            Normalized(0.0), Normalized(0.0), Normalized(0.0), Normalized(0.0),
            Normalized(0.0), Normalized(0.0), Normalized(0.0), Normalized(0.0),
            Normalized(0.0), Normalized(0.0), Normalized(0.0), Normalized(0.0),
        ]),
        primary_controls: None,
        tilt_mechanism: None,
    };
}
```

### 9.6 Integration with Control Law

```
Sanitization is the LAST line of defense, not a control strategy.

                    ┌─────────────────────────────────────────┐
                    │              Control Flow               │
                    └─────────────────────────────────────────┘
                                      │
    Command → Envelope → Controller → Mixer → [Sanitizer] → Hardware
                                                   │
                                          ┌───────┴───────┐
                                          │ If fallback:  │
                                          │ - Set fault   │
                                          │ - Report      │
                                          │ - May trigger │
                                          │   degradation │
                                          └───────────────┘

If sanitizer triggers fallback:
1. Set FaultFlags::ACTUATOR_NUMERIC_ERROR
2. Increment consecutive fallback counter
3. If counter > MAX_CONSECUTIVE_FALLBACK:
   - Trigger FaultHandlingTable lookup
   - Likely degrade to Alternate or Frozen law
4. Let degraded control law handle recovery
```

---

## 10. State Representation

```rust
#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    pub w: Scalar,
    pub x: Scalar,
    pub y: Scalar,
    pub z: Scalar,
}

#[derive(Clone, Debug)]
pub struct StateEstimate {
    pub attitude: Quaternion,
    pub angular_velocity: [RadiansPerSecond; 3],
    pub position_ned: [Meters; 3],
    pub velocity_ned: [MetersPerSecond; 3],
    pub euler_deg: EulerAngles,
    pub quality: EstimateQuality,
    pub valid_flags: StateValidFlags,
}
```

---

## 11. Command & Setpoint Model

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Rate,
    Attitude,
    AltitudeHold,
    PositionHold,
    VelocityControl,
    DeviationTracking,
}

#[derive(Clone, Debug)]
pub struct Setpoint {
    pub mode: ControlMode,
    pub attitude: Option<Quaternion>,
    pub angular_rate: Option<[RadiansPerSecond; 3]>,
    pub altitude: Option<Meters>,
    pub vertical_speed: Option<MetersPerSecond>,
    pub heading: Option<Radians>,
    pub position: Option<[Meters; 3]>,
    pub velocity: Option<[MetersPerSecond; 3]>,
    pub lateral_deviation: Option<Meters>,
    pub vertical_deviation: Option<Meters>,
    pub collective_thrust: Normalized,
}

#[derive(Clone, Debug)]
pub struct Command {
    pub mode: ControlMode,
    pub setpoint: Setpoint,
    pub timestamp: Timestamp,
    pub sequence: u32,
    pub source: CommandSource,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommandSource { Pilot, Autopilot, Gcs, Failsafe }
```

---

## 12. Envelope Protection

```rust
#[derive(Clone, Debug)]
pub struct Limits {
    pub max_roll: Radians,
    pub max_pitch: Radians,
    pub max_roll_rate: RadiansPerSecond,
    pub max_pitch_rate: RadiansPerSecond,
    pub max_yaw_rate: RadiansPerSecond,
    pub max_horizontal_speed: MetersPerSecond,
    pub max_climb_rate: MetersPerSecond,
    pub max_descent_rate: MetersPerSecond,
    pub max_altitude: Meters,
    pub min_altitude: Meters,
    pub min_airspeed: Option<MetersPerSecond>,
    pub max_airspeed: Option<MetersPerSecond>,
    pub max_load_factor: Scalar,
    pub min_load_factor: Scalar,
}

pub trait EnvelopeProtector {
    fn constrain(
        &self,
        raw_sp: &Setpoint,
        state: &StateEstimate,
        limits: &Limits,
        authority: AuthorityProfile,
    ) -> (Setpoint, ProtectionStatus);
}
```

---

## 13. Control Law Degradation

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ControlLaw {
    Normal = 0,
    Alternate1 = 1,
    Alternate2 = 2,
    Direct = 3,
    Frozen = 4,
}

#[derive(Copy, Clone, Debug)]
pub struct DegradationEvent {
    pub from: ControlLaw,
    pub to: ControlLaw,
    pub reason: DegradationReason,
    pub timestamp: Timestamp,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DegradationReason {
    SensorLoss,
    ActuatorFault,
    ActuatorNumericError,  // NEW: sanitizer triggered fallback
    EstimatorDivergence,
    EnvelopeExceedance,
    CommandTimeout,
    TimingViolation,
    NumericError,
    ExplicitRequest,
}
```

---

## 14. Fault Model & Handling

### 14.1 Fault Categories

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FaultCategory {
    // Sensor faults
    ImuFailed,
    ImuAllFailed,
    GnssLost,
    GnssAllLost,
    BaroFailed,
    MagFailed,
    AirspeedFailed,
    
    // Actuator faults
    ActuatorFailed,
    ActuatorSaturated,
    ActuatorDisagreement,
    ActuatorNumericError,       // NEW: NaN/Inf in actuator path
    ActuatorFallbackPersistent, // NEW: too many consecutive fallbacks
    
    // Estimation faults
    EstimatorDiverged,
    AttitudeUncertain,
    PositionUncertain,
    NumericError,
    
    // Command/timing faults
    CommandTimeout,
    CommandInvalid,
    TimingViolation,
    TimingViolationPersistent,
    ConfigInvalid,
}

bitflags::bitflags! {
    pub struct FaultFlags: u64 {
        const IMU0_FAILED = 1 << 0;
        const IMU1_FAILED = 1 << 1;
        const IMU2_FAILED = 1 << 2;
        const ALL_IMU_FAILED = 1 << 3;
        const GNSS0_LOST = 1 << 4;
        const GNSS1_LOST = 1 << 5;
        const ALL_GNSS_LOST = 1 << 6;
        const BARO_FAILED = 1 << 7;
        const MAG_FAILED = 1 << 8;
        const AIRSPEED_FAILED = 1 << 9;
        
        const ACTUATOR_FAULT = 1 << 16;
        const ACTUATOR_NUMERIC = 1 << 17;      // NEW
        const ACTUATOR_FALLBACK = 1 << 18;     // NEW
        
        const ESTIMATOR_DIVERGED = 1 << 24;
        const ATTITUDE_UNCERTAIN = 1 << 25;
        const POSITION_UNCERTAIN = 1 << 26;
        const NUMERIC_ERROR = 1 << 27;
        
        const COMMAND_TIMEOUT = 1 << 32;
        const COMMAND_INVALID = 1 << 33;
        const TIMING_VIOLATION = 1 << 40;
        const TIMING_PERSISTENT = 1 << 41;
        const CONFIG_INVALID = 1 << 48;
    }
}
```

### 14.2 Fault Response Table

```rust
impl FaultHandlingTable {
    pub const DEFAULT: Self = Self {
        entries: &[
            // ... existing entries ...
            
            // NEW: Actuator numeric error (single occurrence)
            FaultResponse { 
                fault: FaultCategory::ActuatorNumericError, 
                action: FaultAction::Monitor,  // Just log, fallback handles it
                degrade_to: None,
                max_response_time_ms: 0,
            },
            // NEW: Persistent actuator fallback
            FaultResponse { 
                fault: FaultCategory::ActuatorFallbackPersistent, 
                action: FaultAction::Degrade, 
                degrade_to: Some(ControlLaw::Alternate1),
                max_response_time_ms: 10,
            },
        ],
    };
}
```

---

## 15. Channel & Redundancy Model

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ChannelId(pub u8);

impl ChannelId {
    pub const PRIMARY: Self = Self(0);
    pub const SECONDARY: Self = Self(1);
    pub const TERTIARY: Self = Self(2);
    pub const MAX_CHANNELS: usize = 3;
}

#[derive(Clone, Debug)]
pub struct CrossChannelData {
    pub estimates: [Option<StateEstimate>; ChannelId::MAX_CHANNELS],
    pub health: [Option<ChannelHealth>; ChannelId::MAX_CHANNELS],
    pub commands: [Option<ActuatorCmd>; ChannelId::MAX_CHANNELS],
    pub sequences: [Option<u32>; ChannelId::MAX_CHANNELS],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChannelHealth { Operative, Degraded, Failed, Testing }

#[derive(Clone, Debug)]
pub struct ChannelStatus {
    pub mode: ControlMode,
    pub law: ControlLaw,
    pub health: ChannelHealth,
    pub faults: FaultFlags,
    pub confidence: EstimateQuality,
    pub envelope_margin: EnvelopeMargin,
    pub sequence: u32,
    pub protection: ProtectionStatus,
    pub sanitize_report: SanitizeReport,  // NEW: sanitization status
}
```

---

## 16. Initialization

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InitState {
    PowerOn,
    ConfigLoading,
    SensorInit,
    EstimatorConverging,
    PreArm,
    Ready,
    Armed,
    Disarmed,
    Fault,
}

/// Init ↔ Update coupling:
/// Non-Armed states → ControlLaw::Frozen → safe_output only
impl InitState {
    pub fn allows_active_control(&self) -> bool {
        matches!(self, InitState::Armed)
    }
    
    pub fn forced_control_law(&self) -> Option<ControlLaw> {
        if self.allows_active_control() { None } 
        else { Some(ControlLaw::Frozen) }
    }
}
```

---

## 17. Timing & Watchdog

```rust
pub const CONTROL_LOOP_PERIOD_US: u64 = 1000;
pub const CONTROL_LOOP_DEADLINE_US: u64 = 800;
pub const WATCHDOG_PERIOD_MS: u32 = 10;
pub const COMMAND_TIMEOUT_MS: u32 = 100;
pub const TIMING_VIOLATION_THRESHOLD: u32 = 3;

#[derive(Copy, Clone, Debug)]
pub struct TimingStats {
    pub last_cycle_us: u32,
    pub max_cycle_us: u32,
    pub min_cycle_us: u32,
    pub deadline_violations: u32,
    pub consecutive_violations: u32,
    pub total_cycles: u64,
}

pub trait Watchdog {
    fn kick(&mut self);
    fn check_deadline(&self) -> bool;
}
```

---

## 18. Configuration

```rust
#[derive(Clone, Debug)]
pub struct Config {
    pub law_profile: LawProfile,
    pub airframe: AirframeConfig,
    pub tuning: TuningConfig,
    pub limits: Limits,  // Default limits (per-mode limits in ModeConfig)
    pub failsafe: FailsafeConfig,
    pub sensors: SensorConfig,
    pub fault_table: FaultHandlingTable,
    pub invalid_command_policy: InvalidCommandPolicy,
}

#[derive(Clone, Debug)]
pub struct AirframeConfig {
    pub vehicle_type: VehicleType,
    pub mass_kg: Scalar,
    pub inertia: [Scalar; 3],
    /// Mode configurations (for morphing aircraft, multiple modes)
    pub modes: &'static [ModeConfig],
    /// Default/fallback mixer (used if no mode-specific config)
    pub default_mixer: MixerConfig,
    /// Safe output for non-armed states
    pub safe_output: [Normalized; MAX_ACTUATORS],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VehicleType { Multirotor, FixedWing, VTOL, MorphingVTOL, Rover, Boat }
```

---

## 19. Core Interface

```rust
#[derive(Clone, Debug)]
pub struct UpdateResult {
    pub actuator: ActuatorCmd,
    pub status: ChannelStatus,
    pub estimate: StateEstimate,
    pub timing: CycleTiming,
    pub degradation: Option<DegradationEvent>,
}

pub fn update(
    channel: ChannelId,
    time: TimeDelta,
    sensors: &SensorSet,
    command: &Command,
    actuator_state: &ActuatorState,
    cross_channel: Option<&CrossChannelData>,
) -> UpdateResult;

pub trait AviateKernel {
    fn init_step(&mut self, sensors: &SensorSet, time: Timestamp) -> InitResult;
    fn init_state(&self) -> InitState;
    fn is_ready(&self) -> bool;
    fn arm(&mut self) -> Result<(), ArmError>;
    fn disarm(&mut self);
    
    fn update(
        &mut self,
        channel: ChannelId,
        time: TimeDelta,
        sensors: &SensorSet,
        command: &Command,
        actuator_state: &ActuatorState,
        cross_channel: Option<&CrossChannelData>,
    ) -> UpdateResult;
    
    fn load_config(&mut self, config: &ConfigBlock) -> Result<(), ConfigError>;
    fn get_config(&self) -> &Config;
    fn get_health(&self) -> HealthReport;
    fn get_faults(&self) -> FaultFlags;
    fn get_control_law(&self) -> ControlLaw;
    fn kick_watchdog(&mut self);
    fn ground_reset(&mut self);
    
    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate);
    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultCategory);
}
```

---

## 20. Architecture Invariants

1. Aviate never parses maps, routes, or procedures
2. Aviate never contains network protocol stacks
3. Aviate only operates on physical state and control error
4. Aviate is a "black box control core" driven by external world
5. Euler angles never enter EKF or controller internals
6. Control law degradation is monotonic (ground reset to restore)
7. All external inputs are validated before use
8. All outputs include diagnostic/health information
9. Time is explicit, never implicit wall-clock
10. Configuration changes are transactional with rollback
11. Physics calculations use dimensional newtypes, not raw Scalar
12. NaN/Inf never propagates to actuator output
13. Non-Armed states produce only safe/neutral output
14. Fault→Degradation mapping is explicit in FaultHandlingTable
15. Authority philosophy is visible in Config
16. No unsafe code in flight build (HAL boundary only)
17. No dynamic memory allocation at runtime
18. No recursion
19. All loops have statically bounded iteration count
20. No panic-based error handling; all errors explicit
21. No non-deterministic concurrency in core
22. **Actuator coupling is per-mode, not fixed actuator property**
23. **Strongly coupled groups use vector-level fallback, never per-channel**
24. **Sanitizer is last defense, not control strategy**
25. **ConfigMode determines active ModeConfig (mixer, groups, limits)**

---

## 21. Minimal Implementation Strategy

| Component | v0.5 Minimal Implementation |
|-----------|----------------------------|
| Channels | Only `ChannelId(0)` |
| Sensors | Only `imus[0]` |
| Cross-channel | Always `None` |
| ConfigMode | Single mode (no morphing) |
| Envelope | Simple clamping |
| Control law | Always `Normal` |
| Coupling groups | All motors in single Strong group |
| Fallback | Last good vector only |
| Geometry | Static (no morphing) |

---

## Appendix A: Fault → Degradation Quick Reference

| Fault | Action | Degrade To | Response Time |
|-------|--------|------------|---------------|
| Single IMU failed | Isolate | - | 10 ms |
| All IMU failed | Emergency | Frozen | 0 ms |
| All GNSS lost | Degrade | Alternate1 | 100 ms |
| Estimator diverged | Degrade | Alternate2 | 10 ms |
| Numeric error | Emergency | Frozen | 0 ms |
| Command timeout | Degrade | Alternate1 | 100 ms |
| Actuator numeric | Monitor | - | 0 ms |
| Actuator fallback persistent | Degrade | Alternate1 | 10 ms |
| Config transition failed | Degrade | Alternate1 | 0 ms |
| Timing violation (persistent) | Degrade | Alternate2 | 50 ms |

---

## Appendix B: Coupling Examples by Mode

| Vehicle | Mode | Group | Coupling | Rationale |
|---------|------|-------|----------|-----------|
| Quadrotor | Hover | Motors 0-3 | **Strong** | Single failure = catastrophic |
| 4+1 VTOL | Hover | Motors 0-3 | **Strong** | Same as quad |
| 4+1 VTOL | Cruise | Motors 0-3 (pullers) | **Weak** | Single failure = trim + degrade |
| 4+1 VTOL | Cruise | Ailerons | **Strong** | Roll authority critical |
| Fixed-wing | Any | Elevator | **Weak** | Single channel, independent |
| Folding quad | Transition | Fold servos | **Strong** | Asymmetry = disaster |

---

## Appendix C: Version History

| Version | Changes |
|---------|---------|
| v0.1 | Initial architecture |
| v0.2 | Redundancy-aware interfaces |
| v0.3 | Complete type definitions |
| v0.4 | Behavioral semantics locked |
| v0.5 | Language profile, per-mode coupling, morphing support |

---

*End of Spec v0.5*