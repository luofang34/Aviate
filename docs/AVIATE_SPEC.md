# Aviate Spec v0.5.1 (Architecture-Complete)

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
| v0.5.1 | - | Control-law/safety split; SafetyLevelV1 and ChannelHealthV1 semantics; 16-bit center-code; SEU resilience rules; FCR/frame integrity; TMR/lockstep support; command authority; record/replay; HAL boundary (§3.3); time monotonicity (§7); sensor/command buffering (§8,12); invariants 31-37 |

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
| Units | No bare `Scalar` for physical quantities in control/estimation; use dimensional newtypes |

### 3.2 Numeric Policy

- Base type: `Scalar = f32`
- Physical quantities: Dimensional newtypes only
- NaN/Inf: Never propagate, trigger fault on detection

### 3.3 Hardware Abstraction Boundary

The Aviate core does not own or manage hardware resources directly. All interaction
with physical devices (sensors, actuators, buses, timers, DMA, interrupts) occurs
through platform-specific HAL or OS layers outside this specification.

The only interfaces between Aviate and hardware are the typed data structures defined
in this document (e.g., `SensorSet`, `ActuatorCmd`, `ActuatorState`, `CycleTiming`) and
the `AviateKernel` trait. Implementations SHALL NOT introduce hidden, hardware-specific
side effects inside the core beyond these interfaces.

Reference HAL implementations SHOULD use `embedded-hal` traits where available, but this
is not normative for the core specification. The spec remains platform-agnostic and may
have implementations in other languages (e.g., C).

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

// Contract: The number of actuator groups in one ModeConfig shall not exceed
// MAX_GROUPS, otherwise config loading fails with FaultFlags::CONFIG_INVALID.
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
        geometry: Option<GeometryState>,
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

RULE 5: Transition to ConfigTransitionState::Failed SHALL raise FaultCategory::ConfigTransitionFailed (and any actuator-related fault implied by TransitionFailure); recovery requires explicit reset or valid re-transition.

Transition triggers:
- External systems request mode changes via `Command.config_mode_request` or `AviateKernel::request_config_mode`.
- The kernel exposes `transition_state()`/`config_mode()` for status reporting; transitions are performed only when RULE 1 conditions are met.
- Re-entrant requests while already Switching SHALL be rejected with TransitionError::NotReady; requesting the current stable mode is a no-op; when in Failed, only a reset or valid re-transition to a safe mode is allowed.
- Transition ModeConfig MUST exist in `airframe.modes`; its mixer may be precomputed offline (e.g., interpolated between source/target geometry) but must be deterministic and static-size (no dynamic allocation).
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
    pub chain: &'static [ControlLawV1],
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

## 6. Numeric Representation

### 6.1 Base Types

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
pub struct Pascals(pub Scalar);
pub struct Celsius(pub Scalar);
pub struct Degrees(pub Scalar);
pub struct Microtesla(pub Scalar);
pub struct Kilograms(pub Scalar);
pub struct KilogramMeterSquared(pub Scalar);
```

### 6.2 Validation Trait

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

// All dimensional newtypes SHALL implement Validated by delegating to inner Scalar
// Example pattern:
// impl Validated for Meters {
//     fn is_valid(&self) -> bool { self.0.is_finite() }
//     fn sanitize_or_default(&self, default: Self) -> Self {
//         if self.0.is_finite() { *self } else { default }
//     }
// }
```

---

## 7. Time Model

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

**Time monotonicity and wrap-around:**

Within a flight segment, `Timestamp.ticks` and control-loop timing fields
(e.g., `CycleTiming.cycle_start_us`, `cycle_end_us`, `duration_us`) SHOULD form
monotonic sequences as observed by the Aviate core. The HAL or platform layer is
responsible for handling any hardware timer wrap-around behavior and presenting
values to the core such that:

- `TimeDelta.tick_delta` is non-negative and represents the elapsed ticks between
  consecutive `update()` calls, and
- `TimeDelta.dt_sec` is consistent with `tick_delta` and the configured
  `TICK_FREQUENCY_HZ`.

Implementations SHALL NOT pass negative or zero-duration `TimeDelta` values into
`update()` except during explicitly defined initialization sequences. Large,
unexpected jumps in timing (e.g., due to wrap-around mis-handling) SHALL be treated
as timing faults and mapped to appropriate `FaultCategory::TimingViolation` /
`FaultCategory::NumericError` responses.

---

## 8. Sensor Model

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

/// Unified air-data payload: baro, pitot, or air-data computer
#[derive(Copy, Clone, Debug)]
pub struct AirData {
    /// Static pressure [Pa] if available
    pub static_pressure: Option<Pascals>,
    /// Dynamic pressure [Pa] if available
    pub dynamic_pressure: Option<Pascals>,
    /// Total pressure [Pa] if provided directly
    pub total_pressure: Option<Pascals>,
    pub temperature: Option<Celsius>,
    /// Sensor-provided indicated airspeed (derived)
    pub indicated_airspeed: Option<MetersPerSecond>,
    /// Sensor-provided true airspeed (derived)
    pub true_airspeed: Option<MetersPerSecond>,
}

#[derive(Copy, Clone, Debug)]
pub struct ImuData {
    pub accel: [MetersPerSecondSquared; 3],
    pub gyro: [RadiansPerSecond; 3],
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GnssFix {
    None,
    TwoD,
    ThreeD,
    RtkFloat,
    RtkFixed,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GnssHealth {
    Good,
    Suspect, // propagated for diagnostics only; not fused for control/estimation
    Lost,
}

#[derive(Copy, Clone, Debug)]
pub struct GnssData {
    pub position_ned: [Meters; 3],
    pub velocity_ned: [MetersPerSecond; 3],
    pub fix: GnssFix,
    pub health: GnssHealth,
}

#[derive(Copy, Clone, Debug)]
pub struct MagData {
    /// Magnetic field in microtesla
    pub field_ut: [Microtesla; 3],
}

#[derive(Copy, Clone, Debug)]
pub struct BaroData {
    /// Optional sensor-provided altitude (for display/debug)
    pub altitude: Option<Meters>,
    /// Static air data (static_pressure + temperature expected to be Some)
    pub air: AirData,
}

#[derive(Copy, Clone, Debug)]
pub struct AirspeedData {
    /// Air data focused on dynamic/total pressure or sensor-derived IAS/TAS
    pub air: AirData,
}

#[derive(Clone, Debug)]
pub struct SensorSet {
    pub imus: [SensorReading<ImuData>; MAX_IMU],
    pub gnss: [SensorReading<GnssData>; MAX_GNSS],
    pub mags: [SensorReading<MagData>; MAX_MAG],
    pub baros: [SensorReading<BaroData>; MAX_BARO],
    pub airspeeds: [SensorReading<AirspeedData>; MAX_AIRSPEED],
    /// Optional geometry feedback for morphing aircraft (fold angles, etc.)
    pub geometry: Option<GeometryState>,
}

/// External/upper-layer sensor overrides (e.g., force GNSS trust state)
#[derive(Copy, Clone, Debug)]
pub struct SensorOverrides {
    pub gnss_force_state: Option<GnssHealth>, // None = no override
}
```

**Air-data usage rules**:
- The EKF and control logic treat `static_pressure`, `dynamic_pressure`, or `total_pressure` as the authoritative sources. Derived fields (`altitude`, `indicated_airspeed`, `true_airspeed`) are optional convenience values and must never override raw pressures.
- In morphing/configurable aircraft, the role of each physical pressure port (static/dynamic/total) is defined per `ConfigMode`/`SensorConfig`; HAL populates `AirData` accordingly rather than hardcoding roles in `SensorSet` types.
- For BaroData used in EKF, `air.static_pressure` MUST be `Some`; invalid/missing static pressure is a config/init fault.
- If both `dynamic_pressure` and `indicated_airspeed` are present, EKF shall treat `dynamic_pressure` as authoritative and may recompute IAS to check consistency; disagreements beyond tolerance raise `FaultCategory::AirspeedFailed`.

**Data trust levels:**

All external inputs (Command, sensor streams, cross-channel data) SHALL be conceptually
classified by trust level (e.g., trusted hardware path, untrusted network, diagnostic-only).
Only inputs from trusted paths MAY directly influence inner control loops; untrusted or
diagnostic-only data MUST go through additional validation / gating logic before affecting
setpoints or state estimates.

Existing mechanisms such as GnssHealth::Suspect and SensorOverrides are examples of this
gating: they prevent low-trust inputs from directly affecting estimator or controller state.

**Sampling and buffering model:**

Aviate's core consumes at most one `SensorSet` snapshot per control cycle. It is the
responsibility of the HAL or upper layers to provide the latest available measurements
in each cycle; the core SHALL NOT depend on draining historical queues of sensor samples
inside `update()`.

Sensor producers (drivers, HAL tasks, DMA handlers) MAY overwrite older samples with newer
ones in their own buffers (mailbox semantics). From the core's perspective, each element
of `SensorSet` represents the most recent valid reading for that sensor class at the time
the control cycle begins.

Temporal filtering, resampling, or ring-buffer management (e.g., IMU sample queues) are
implementation details of the HAL and estimator; they SHALL NOT change the Aviate core's
contract of "latest-value" consumption per control cycle.

---

## 9. Actuator Model

### 9.1 Actuator Types

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
    MorphingJoint,
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

### 9.2 Actuator Group Configuration (Per-Mode)

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

/// Group-level actuator vector (shared by config and runtime fallback)
#[derive(Clone, Debug)]
pub struct GroupVector {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub mask: u16,
    pub valid: bool,
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
    /// Populated per mode from MixerConfig.safe_vectors or mode-specific data
    pub safe_pattern: GroupVector,
}
```

### 9.3 Example: Same Motors, Different Coupling by Mode

```rust
// HOVER MODE: Quadrotor motors are strongly coupled
const HOVER_ROTOR_GROUP: ActuatorGroupConfig = ActuatorGroupConfig {
    kind: GroupKind::Multirotor,
    coupling: CouplingKind::Strong,  // ← Any fault = all fallback
    fallback: FallbackPolicy::HoldLastGood,
    members: &[0, 1, 2, 3],
    safe_pattern: GroupVector { 
        outputs: [Normalized(0.15); MAX_ACTUATORS], 
        mask: 0b0000_0000_0000_1111, 
        valid: true,
    }, // Idle
};

// CRUISE MODE: Same motors as distributed thrust, weakly coupled
const CRUISE_THRUST_GROUP: ActuatorGroupConfig = ActuatorGroupConfig {
    kind: GroupKind::DistributedThrust,
    coupling: CouplingKind::Weak,    // ← Per-channel fallback OK
    fallback: FallbackPolicy::SafePattern,
    members: &[0, 1, 2, 3],
    safe_pattern: GroupVector { 
        outputs: [Normalized(0.0); MAX_ACTUATORS], 
        mask: 0b0000_0000_0000_1111, 
        valid: true,
    }, // Feathered/idle
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

### 9.4 Coupling Rationale by Phase

| Mode | Actuators | Coupling | Single-Failure Consequence |
|------|-----------|----------|---------------------------|
| Hover | Quad motors | **Strong** | Loss of attitude/lift control → catastrophic |
| Cruise | Same as pullers | **Weak** | Asymmetric thrust → trim + reduced performance |
| Hover | Ailerons | N/A (inactive) | - |
| Cruise | Ailerons | **Strong** | Roll authority loss |
| Any | Landing gear | **Weak** | Degraded but manageable |

### 9.5 Actuator Commands

```rust
#[derive(Clone, Debug)]
pub struct ActuatorCmd {
    pub outputs: [Normalized; MAX_ACTUATORS],
    pub active_mask: u16,
    pub sequence: u32,
    pub timestamp: Timestamp,
    /// Which groups had fallback this cycle (bitmask by group index)
    /// Limited to MAX_GROUPS entries (bit i corresponds to groups[i])
    pub fallback_mask: u8,
    /// Was any sanitization performed?
    pub sanitized: bool,
}

/// Feedback from actuators (e.g., positions for servos/tilts)
#[derive(Clone, Debug)]
pub struct ActuatorState {
    pub feedback: [Normalized; MAX_ACTUATORS],
    pub timestamp: Timestamp,
}
// Minimal feedback; per-actuator health/load feedback may extend this in later versions.
```

### 9.6 Vector-Level Fallback State

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
```

---

## 10. Actuator Output Sanitization (CRITICAL SAFETY)

### 10.1 Design Rationale

**Problem**: Per-channel sanitization (replacing one bad channel with a default while keeping others) creates catastrophic torque imbalance on coupled systems like quadrotors.

**Solution**: Vector-level sanitization — if any channel in a coupled group fails numeric validation, the ENTIRE group falls back to a coherent safe vector.

### 10.2 Sanitization Rules

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
        - ActuatorCmd.fallback_mask (bit i corresponds to groups[i])
        - FaultFlags::ACTUATOR_NUMERIC
        - Status reporting to upper systems
```

### 10.3 Sanitization Implementation

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
    /// For each coupling group in the active ModeConfig:
    /// 1. Check all channels in group for NaN/Inf/out-of-range
    /// 2. If ANY channel invalid → reject entire group's new values
    /// 3. Replace with coherent fallback vector
    /// 4. Update last_good if this cycle was valid
    fn sanitize(
        &mut self,
        cmd: &mut ActuatorCmd,
        mode: &ModeConfig,
    ) -> SanitizeReport;
}

#[derive(Clone, Debug)]
pub struct SanitizeReport {
    pub group_results: [GroupSanitizeResult; MAX_GROUPS],
    pub any_fallback: bool,
    pub critical_failure: bool,  // true if FallbackUnavailable occurred
}
```

### 10.4 Fallback Aging and Limits

```rust
/// How many cycles a "last good" vector remains usable
pub const MAX_FALLBACK_AGE_CYCLES: u16 = 100;  // 100ms at 1kHz

/// Maximum consecutive fallback cycles before triggering degradation
pub const MAX_CONSECUTIVE_FALLBACK: u16 = 10;  // 10ms

/// After MAX_CONSECUTIVE_FALLBACK, force ControlLawV1 degradation
/// to prevent indefinite operation on stale/safe vectors
```

### 10.5 Per-Vehicle Safe Vectors

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

// SafeVectorSet is a template; configuration tooling shall project these into
// per-mode ActuatorGroupConfig.safe_pattern values. Runtime uses the per-group
// safe_pattern only.

// Fallback vector precedence:
// 1) ModeConfig/ActuatorGroupConfig.safe_pattern.valid == true (per-mode, authoritative)
// 2) MixerConfig.safe_vectors template (used to populate group safe_pattern during config load)
// 3) Zero output only if no valid safe_pattern exists (config load should reject before flight)
```

### 10.6 Integration with Control Law

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
1. Set FaultFlags::ACTUATOR_NUMERIC
2. Increment consecutive fallback counter
3. If counter > MAX_CONSECUTIVE_FALLBACK:
   - Trigger FaultHandlingTable lookup
   - Likely degrade to Alternate or Frozen law
4. Let degraded control law handle recovery
```

---

## 11. State Representation

```rust
#[derive(Copy, Clone, Debug)]
pub struct Quaternion {
    pub w: Scalar,
    pub x: Scalar,
    pub y: Scalar,
    pub z: Scalar,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EstimateQuality {
    Good,
    Degraded,
    Unusable,
}

bitflags::bitflags! {
    pub struct StateValidFlags: u8 {
        const ATTITUDE = 0x01;
        const ANGULAR_RATE = 0x02;
        const POSITION = 0x04;
        const VELOCITY = 0x08;
    }
}

#[derive(Clone, Debug)]
pub struct StateEstimate {
    pub attitude: Quaternion,
    pub angular_velocity: [RadiansPerSecond; 3],
    pub position_ned: [Meters; 3],
    pub velocity_ned: [MetersPerSecond; 3],
    pub quality: EstimateQuality,
    pub valid_flags: StateValidFlags,
}
```

---

## 12. Command & Setpoint Model

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
    pub attitude: Option<Quaternion>,
    pub angular_rate: Option<[RadiansPerSecond; 3]>,
    pub altitude: Option<Meters>,
    pub vertical_speed: Option<MetersPerSecond>,
    pub heading: Option<Radians>,
    pub position: Option<[Meters; 3]>,
    pub velocity: Option<[MetersPerSecond; 3]>,
    pub lateral_deviation: Option<Meters>,
    pub vertical_deviation: Option<Meters>,
    pub collective_thrust: Normalized, // semantics interpreted per active ModeConfig; not comparable across modes
}

#[derive(Clone, Debug)]
pub struct Command {
    pub mode: ControlMode,
    /// Setpoint values must be consistent with `mode`; no duplicate mode inside
    pub setpoint: Setpoint,
    /// Optional request to change configuration mode (morphing)
    pub config_mode_request: Option<ConfigMode>,
    /// Optional overrides from higher-level systems (e.g., GNSS trust)
    pub sensor_overrides: Option<SensorOverrides>,
    pub timestamp: Timestamp,
    pub sequence: u32,
    pub source: CommandSource,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CommandSource { Pilot, Autopilot, Gcs, Failsafe }
```

**Command buffering and freshness:**

External command sources may produce commands at rates different from the control-loop
period. Implementations SHOULD treat per-source command buffers as single-element
mailboxes: newer commands overwrite older ones, and the Aviate core consumes at most one
`Command` instance per control cycle.

When multiple commands from different `CommandSource` values are available for the same
cycle, the active command for that cycle SHALL be selected according to:

1. `CommandSource` precedence (see *Command authority* below), and
2. Freshness criteria (e.g., largest `sequence` value within an acceptable age window).

The core SHALL NOT depend on draining long queues of historical commands inside `update()`.
Intermediate commands that are overwritten before a cycle begins are considered never
applied.

The age of the selected `Command` for a cycle SHALL still be checked against
`COMMAND_TIMEOUT_MS` (or an equivalent configured timeout); a mailbox that stops
receiving fresh commands SHALL therefore lead to `FaultCategory::CommandTimeout`,
even if it continues to hold an old value.

**Command authority:**

Implementations SHOULD define a simple, static precedence between CommandSource values
(e.g., Pilot > Autopilot > Gcs > Failsafe). Lower-precedence sources MUST NOT override
higher-precedence commands within the same time window.

The precedence mapping between CommandSource values SHOULD be part of configuration or
system design data, not hard-coded magic behavior.

Any command or configuration change that affects arming state, ConfigMode, or control law
selection SHALL only be accepted from trusted command sources. Generic GCS or network-linked
sources MUST NOT be allowed to arm/disarm or change configuration modes without an
out-of-band authorization mechanism.

**Command consistency validation:**

For each Command, implementations SHALL validate that the setpoint fields are consistent
with mode (e.g., no conflicting attitude/position requests) and within configured physical
limits. Violations (including impossible combinations that may result from SEU) SHALL be
treated as FaultCategory::CommandInvalid and handled according to InvalidCommandPolicy,
not applied to the control loop.

When FaultCategory::CommandInvalid is raised due to inconsistency, the resulting behavior
(reject / clamp / freeze) SHALL follow the configured InvalidCommandPolicy and SHALL be
observable via FaultFlags and ChannelStatus.

---

## 13. Envelope Protection

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
    pub max_load_factor: Scalar, // dimensionless
    pub min_load_factor: Scalar, // dimensionless
}

bitflags::bitflags! {
    pub struct AxisLimitFlags: u8 {
        const ROLL = 0x01;
        const PITCH = 0x02;
        const YAW = 0x04;
        const ALTITUDE = 0x08;
        const SPEED = 0x10;
        const LOAD_FACTOR = 0x20;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct EnvelopeMargin {
    /// Positive values mean remaining margin before limit breach
    pub roll_rad: Radians,
    pub pitch_rad: Radians,
    pub yaw_rate_rad_s: RadiansPerSecond,
    pub altitude_m: Meters,
    pub airspeed_mps: MetersPerSecond,
    pub load_factor: Scalar, // dimensionless
}

#[derive(Copy, Clone, Debug)]
pub struct ProtectionStatus {
    pub limited_axes: AxisLimitFlags,
    pub saturated: bool,
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

## 14. Control Law Degradation

```rust
/// Control law capability: what control strategies are available.
/// NOTE: ControlLawV1 describes flight control capability, NOT safety/risk level.
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum ControlLawV1 {
    /// Full envelope protection, all loops active
    Primary = 0,
    /// Reduced protections, degraded but flyable
    Alternate = 1,
    /// Manual with minimal augmentation
    Direct = 2,
    /// Last-ditch stability only
    Backup = 3,
}

/// Safety level: whole-aircraft situational risk assessment.
/// Orthogonal to control law capability and channel health.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum SafetyLevelV1 {
    /// Normal flight with adequate margins (altitude, fuel, divert options)
    FlightNormal,
    /// Margins noticeably reduced (takeoff/landing, config change, oceanic, low fuel)
    FlightMarginal,
    /// Urgent but controllable, analogous to "PAN-PAN"
    FlightUrgent,
    /// Life/platform threatening, analogous to "MAYDAY"
    FlightEmergency,
}
```

Implementations MAY derive and maintain a `SafetyLevelV1` from envelope margins, energy state
(altitude + speed + configuration), sensor/plant health, and external cues (e.g., GPWS). This
label is advisory and intended for higher-level decisions and mode selection. It MUST NOT replace
proper envelope protection or fault/degradation logic, and it SHALL NOT directly trigger control-law
degradation.

```rust
#[derive(Copy, Clone, Debug)]
pub struct DegradationEvent {
    pub from: ControlLawV1,
    pub to: ControlLawV1,
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

### 14.1 Recommended 16-bit center-code scheme

For implementations that share buses between different Aviate versions or independent control boxes, a 16-bit center-code scheme is RECOMMENDED:

- Assign four 16-bit center codewords with pairwise Hamming distance ≥ 7 for each of `ControlLawV1` and `SafetyLevelV1`.
- These center codewords define Voronoi regions in the 16-bit space; future versions MAY add finer codewords within each region.

**Decoding rules for v1 decoders:**

1. Map any received 16-bit value to the nearest center codeword (minimum Hamming distance).
2. If the distance is 0–2:
   - Treat as a valid code; an implementation MAY correct the bits and record that a 1–2 bit error occurred.
3. If the distance is ≥ 3:
   - Raise a protocol/numeric fault; but still map to the corresponding coarse enum value (`ControlLawV1` / `SafetyLevelV1`) associated with the nearest center.

Under these rules, for the v1 center codewords, a d_min ≥ 7 yields double-error-correcting and triple-error-detecting behavior: any 1–2 bit error is decoded back to the original center codeword; any 3+ bit error is detected and flagged as a fault.

**Forward compatibility:**

Future profiles MAY define additional "fine" codewords within each center's Voronoi region, provided that any such codeword remains strictly closer to its own center than to any other center. V1 decoders will automatically project such fine codewords back to the correct coarse value, while still being able to flag them as non-center (implementation choice).

**Example center codewords (non-normative):**

| `ControlLawV1` | 16-bit center |
|----------------|---------------|
| Primary        | 0x0000        |
| Alternate      | 0x00FF        |
| Direct         | 0x0F0F        |
| Backup         | 0xFFFF        |

| `SafetyLevelV1`  | 16-bit center |
|------------------|---------------|
| FlightNormal     | 0x0000        |
| FlightMarginal   | 0x00FF        |
| FlightUrgent     | 0x0F0F        |
| FlightEmergency  | 0xFFFF        |

(These example codewords have pairwise Hamming distance 8 ≥ 7; they are non-normative and may be replaced by any set satisfying the constraints.)

**Enum wire encoding:**

For ControlLawV1 and SafetyLevelV1, the 16-bit center-code scheme in §14.1 defines their
normative on-wire representation for safety-critical links. Implementations MAY still use
different in-memory representations internally, but on-wire encodings SHALL follow §14.1.

For all other enums, this specification does not fix in-memory discriminant values.
Any on-wire or cross-chip protocol MUST define its own explicit encoding (with checksum/CRC)
and MUST NOT rely on Rust enum discriminants.

---

## 15. Fault Model & Handling

### 15.1 Fault Categories

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
    ActuatorNumericError,
    ActuatorFallbackPersistent,
    
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
    ConfigTransitionFailed,

    // Memory/integrity faults (for future ECC/lockstep reporting)
    MemoryError,
    EnumInvalid,
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
        const ACTUATOR_NUMERIC = 1 << 17;
        const ACTUATOR_FALLBACK = 1 << 18;
        
        const ESTIMATOR_DIVERGED = 1 << 24;
        const ATTITUDE_UNCERTAIN = 1 << 25;
        const POSITION_UNCERTAIN = 1 << 26;
        const NUMERIC_ERROR = 1 << 27;
        
        const COMMAND_TIMEOUT = 1 << 32;
        const COMMAND_INVALID = 1 << 33;
        const TIMING_VIOLATION = 1 << 40;
        const TIMING_PERSISTENT = 1 << 41;
        const CONFIG_INVALID = 1 << 48;
        const CONFIG_TRANSITION_FAILED = 1 << 49;

        const MEMORY_ERROR = 1 << 56;
        const ENUM_INVALID = 1 << 57;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FaultAction {
    Monitor,
    Isolate,
    Degrade,
    Emergency,
}

#[derive(Copy, Clone, Debug)]
pub struct FaultResponse {
    pub fault: FaultCategory,
    pub action: FaultAction,
    pub degrade_to: Option<ControlLawV1>,
    pub max_response_time_ms: u32,
}

#[derive(Copy, Clone, Debug)]
pub struct FaultHandlingTable {
    pub entries: &'static [FaultResponse],
}
```

### 15.2 Fault Response Table

```rust
impl FaultHandlingTable {
    pub const DEFAULT: Self = Self {
        entries: &[
            // ... existing entries ...
            
            // Actuator numeric error (single occurrence)
            FaultResponse { 
                fault: FaultCategory::ActuatorNumericError, 
                action: FaultAction::Monitor,  // Just log, fallback handles it
                degrade_to: None,
                max_response_time_ms: 0,
            },
            // Persistent actuator fallback
            FaultResponse { 
                fault: FaultCategory::ActuatorFallbackPersistent, 
                action: FaultAction::Degrade, 
                degrade_to: Some(ControlLawV1::Alternate),
                max_response_time_ms: 10,
            },
            FaultResponse {
                fault: FaultCategory::ConfigTransitionFailed,
                action: FaultAction::Degrade,
                degrade_to: Some(ControlLawV1::Alternate),
                max_response_time_ms: 0,
            },
        ],
    };
}
```

### 15.3 SEU Resilience Rules

Single Event Upsets (SEU) can flip bits in RAM, registers, or flash, potentially causing enum fields to become invalid values or "valid but wrong" values. The following rules establish defense-in-depth against SEU-induced mode confusion.

**Enum validation (all control-plane enums):**

Any enum field received from off-chip memory, non-ECC RAM, or external buses MUST be checked for "known variant". Unknown or out-of-range values SHALL be treated as `FaultCategory::EnumInvalid` and trigger:
- `FaultFlags::ENUM_INVALID`
- Immediate fallback to `ControlLawV1::Backup` + safe actuator output
- The invalid value SHALL NOT be silently interpreted as a different valid mode

This rule applies to: `ControlLawV1`, `SafetyLevelV1`, `ChannelHealthV1`, `ConfigMode`, `ControlMode`, `CommandSource`, `InitState`, `VehicleType`, and any future control-plane enums.

**FaultFlags pessimistic semantics:**

FaultFlags and similar safety-critical bitfields SHALL be designed with pessimistic semantics:
1. **Set-only during flight**: Safety-critical fault bits (e.g., `ALL_IMU_FAILED`, `ESTIMATOR_DIVERGED`) SHALL only be cleared through an explicit recovery sequence or ground maintenance action, never solely because the bit reads as 0.
2. **SEU bias toward conservatism**: If an SEU flips a bit to indicate "fault present" when none exists, the system becomes more conservative (acceptable). If an SEU clears a fault bit, the explicit recovery requirement prevents false "all clear" states.
3. **Monotonic counters**: Fields like `deadline_violations`, `consecutive_violations`, and `sequence` SHALL only increase monotonically during flight. A decrease or large jump SHALL raise `FaultCategory::NumericError`.

**Numeric validation frequency:**

All `Scalar` and dimensional newtype values participating in control loops MUST be validated (`is_finite()`) every control cycle. Any validation failure triggers the appropriate fault category and actuator fallback. This catches SEU-induced NaN/Inf/extreme values before they propagate.

**Internal numeric self-checks:**

Estimation and control algorithms SHALL include internal numeric self-checks (e.g.,
positive-definiteness of covariance matrices, bounded gains, condition-number checks).
Violations SHALL be mapped to FaultCategory::EstimatorDiverged or FaultCategory::NumericError,
never silently ignored.

---

## 16. Channel & Redundancy Model

A "channel" in Aviate denotes a single end-to-end control path:

- the sensor inputs as seen by one MCU/flight controller instance,
- that channel's estimator and control logic,
- and that channel's actuator command outputs onto its connected buses.

Multiple channels MAY drive the same physical actuators (e.g., via cross-strapped buses). `ChannelHealthV1` describes the internal health and capability of one channel, not the whole vehicle and not individual actuators.

**Frame integrity for cross-channel / external links:**

Any serialization of Command, UpdateResult, CrossChannelData, or related control-plane
structures onto off-chip links or shared buses MUST:
- include an end-to-end integrity check (e.g., CRC-32 or stronger), and
- include a monotonically increasing sequence number or equivalent freshness indicator.

Sequence numbers SHOULD be chosen large enough to avoid wraparound during a single flight.
If wraparound is possible, implementations SHALL define an explicit comparison rule
(e.g., modular arithmetic with bounded reordering window) to distinguish fresh from stale
frames.

Implementations MAY maintain a small acceptance window to tolerate limited reordering but
SHALL reject frames with sequence numbers older than the last accepted value beyond that
window.

Frames that fail integrity or freshness checks SHALL be treated as unavailable data from
that source (i.e., ignored for voting / control), not as evidence that the source is
healthier or less healthy than other channels.

`CrossChannelData.sequences[i]` represents the last accepted sequence number from channel i
after integrity checks.

**Fault containment regions (FCR):**

A fault-containment region (FCR) is the smallest unit within which a single software or
hardware fault may arbitrarily corrupt all state. In Aviate deployments, each MCU + its
local memory + its local I/O are treated as one FCR. Cross-FCR corruption is assumed
impossible except via defined communication links.

Cross-FCR influence SHALL only occur through explicitly defined data structures (Command,
CrossChannelData, UpdateResult, etc.) and SHALL be subject to the integrity and freshness
checks described in §15.3 and this section.

Shared-memory mechanisms between FCRs (including MMU-mapped regions or DMA-accessible RAM)
SHALL NOT be used for safety-related data exchange.

```rust
/// A "channel" is one end-to-end control path:
///   sensors/inputs → one Aviate core instance on one MCU/FC → output buses.
/// Multiple channels may exist in parallel for redundancy and voting.
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
    pub health: [Option<ChannelHealthV1>; ChannelId::MAX_CHANNELS],
    pub commands: [Option<ActuatorCmd>; ChannelId::MAX_CHANNELS],
    pub sequences: [Option<u32>; ChannelId::MAX_CHANNELS],
}

/// Health of one end-to-end control channel:
///   sensors/inputs → Aviate core on one MCU/FC → output buses.
///
/// ChannelHealthV1 reflects only this channel's compute and local I/O capability.
/// External actuator failures (motors/servos/structure) SHALL NOT directly cause
/// ChannelHealthV1::Failed — use FaultFlags / ActuatorStatus / SafetyLevelV1 instead.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ChannelHealthV1 {
    /// Channel fully capable:
    /// - Required local sensors available within spec;
    /// - Control-loop timing within limits;
    /// - At least one output bus reachable.
    Operative,
    /// Channel degraded but can still safely close a control loop:
    /// - Some local sensors/buses lost, but at least one inertial reference
    ///   and one output path remain; or
    /// - Persistent but bounded timing issues; or
    /// - Permanent estimator/controller degradation.
    Degraded,
    /// Channel cannot safely close a control loop under any law:
    /// - No valid inertial reference for this channel; or
    /// - All its output paths are unavailable; or
    /// - Severe timing or numeric failure.
    ///
    /// Failed channels SHALL NOT be selected as active and SHALL NOT drive
    /// any actuator bus.
    Failed,
    /// Channel in self-test / maintenance / explicitly offline:
    /// - Power-on self-test, ground testing, or administratively offlined;
    /// - May run EKF/control on test data, but MUST NOT drive actuators
    ///   or be eligible for active voting.
    Offline,
}

#[derive(Clone, Debug)]
pub struct ChannelStatus {
    pub mode: ControlMode,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub law: ControlLawV1,
    pub health: ChannelHealthV1,
    pub faults: FaultFlags,
    pub confidence: EstimateQuality,
    pub envelope_margin: EnvelopeMargin,
    pub sequence: u32,
    pub protection: ProtectionStatus,
    pub sanitize_report: SanitizeReport,
}
```

---

## 17. Initialization

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
/// Non-Armed states → ControlLawV1::Backup → safe_output only
impl InitState {
    pub fn allows_active_control(&self) -> bool {
        matches!(self, InitState::Armed)
    }

    pub fn forced_control_law(&self) -> Option<ControlLawV1> {
        if self.allows_active_control() { None }
        else { Some(ControlLawV1::Backup) }
    }
}

#[derive(Clone, Debug)]
pub struct InitResult {
    pub state: InitState,
    pub faults: FaultFlags,
    pub ready: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ArmError {
    NotReady,
    Faulted,
    AlreadyArmed,
    ConfigInvalid,
}
```

**Reset contract:**

In-flight software reset of the Aviate core SHALL only be initiated by an external
supervisory function and SHALL treat the core as unavailable until it re-enters
InitState::Ready. Aviate itself SHALL NOT autonomously perform a full reset; instead
it transitions to Backup law and safe outputs, awaiting external intervention.

External supervisory functions are expected to monitor HealthReport, FaultFlags, and
SafetyLevelV1 and decide when a reset or channel handover is appropriate.

---

## 18. Timing & Watchdog

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

#[derive(Copy, Clone, Debug)]
pub struct CycleTiming {
    pub cycle_start_us: u32,
    pub cycle_end_us: u32,
    pub duration_us: u32,
    pub deadline_met: bool,
}

pub trait Watchdog {
    fn kick(&mut self);
    fn check_deadline(&self) -> bool;
}
```

**Overload handling:**

The control loop SHALL maintain a fixed period; overload handling is performed by reducing
functionality (e.g., selecting a lower ControlLawV1 or disabling optional LawCapabilities-
flagged features) rather than stretching the control-loop period. For production builds,
each ControlLawV1 / LawCapabilities profile SHOULD have a documented worst-case execution
time (WCET) budget. Degradation from Primary → Alternate → Direct → Backup MAY reduce
computational load to guarantee deadlines under fault or overload.

---

## 19. Configuration

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
    pub mass: Kilograms,
    pub inertia: [KilogramMeterSquared; 3],
    /// Mode configurations (for morphing aircraft, multiple modes)
    pub modes: &'static [ModeConfig],
    /// Default/fallback mixer (used if no mode-specific config)
    pub default_mixer: MixerConfig,
    /// Safe output for non-armed states
    pub safe_output: [Normalized; MAX_ACTUATORS],
}

#[derive(Clone, Debug)]
pub struct TuningConfig {
    pub rate_gains: [Scalar; 3],
    pub attitude_gains: [Scalar; 3],
    pub position_gains: [Scalar; 3],
}
// Minimal placeholder; full implementations may include I/D terms, damping, feedforward, and response shaping.

#[derive(Clone, Debug)]
pub struct FailsafeConfig {
    /// Command timeout before failsafe triggers
    pub command_timeout_ms: u32,
    /// Descent rate during land failsafe
    pub land_descent_rate: MetersPerSecond,
    /// Optional return-to height for fixed-wing/VTOL
    pub return_altitude: Option<Meters>,
}

#[derive(Clone, Debug)]
pub struct SensorConfig {
    pub imu_used: usize,
    pub gnss_used: usize,
    pub mag_used: usize,
    pub baro_used: usize,
    pub airspeed_used: usize,
    pub geometry_used: bool,
}
// Contract: indices in each sensor array < *_used are eligible for estimation; the rest are ignored.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InvalidCommandPolicy {
    Reject,   // Drop command and set fault
    Clamp,    // Clamp to limits and continue
    Freeze,   // Hold last valid command
}

#[derive(Clone, Debug)]
pub struct ConfigBlock {
    /// Raw serialized config blob (e.g., from flash)
    pub data: &'static [u8],
    pub version: u16,
    /// CRC-32 or stronger checksum over data
    pub checksum: u32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ConfigError {
    InvalidFormat,
    UnsupportedVersion,
    OutOfRange,
    ChecksumMismatch,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TransitionError {
    InvalidRequest,
    NotReady,
    UnsafeConditions,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum VehicleType { Multirotor, FixedWing, VTOL, MorphingVTOL, Rover, Boat }
```

**ModeConfig precedence**:
- If `airframe.modes` is non-empty, the active `ModeConfig` selected by `ConfigMode` SHALL provide the authoritative `mixer`, `groups`, `limits`, and `law_profile`. The top-level `Config.limits` and `Config.law_profile` serve only as defaults for non-morphing/single-mode vehicles and SHALL NOT conflict with any `ModeConfig`; conflicts are `ConfigError::InvalidFormat`.
- Each `ConfigMode` SHALL appear at most once in `airframe.modes`; missing mandatory modes (e.g., `Degraded` when morphing is enabled) are `ConfigError::InvalidFormat`.

**Configuration integrity (SEU/flash protection)**:

1. **Mandatory checksum**: All configuration blocks MUST include a CRC-32 (or stronger) checksum. Checksum mismatch SHALL raise `ConfigError::ChecksumMismatch` and prevent arming.

2. **Recommended mirroring**: Implementations SHOULD store at least two independent copies of configuration in non-volatile memory and verify consistency at boot. On mismatch between copies, the core SHALL treat configuration as invalid and refuse to arm.

3. **Calibration bounds checking**: Sensor calibration data (e.g., IMU biases, magnetometer offsets, barometric pressure offsets) SHALL be validated against physically reasonable bounds at load time. Out-of-range calibration values are `ConfigError::OutOfRange`.

4. **Read-only at runtime**: Once loaded and validated, configuration data SHALL be treated as read-only during flight. Any detected modification to configuration memory during flight SHALL raise `FaultCategory::MemoryError`.

**Spec extension strategy:**

Future extensions to structs in this spec SHOULD follow a "reserved/extension" pattern
(e.g., adding optional fields, reserved bits, or versioned payloads) rather than
reinterpreting existing fields. Existing fields SHALL NOT change meaning between minor
spec versions.

Reserved or extension fields SHALL have well-defined default semantics (e.g., treated as
zero/None when absent) so that older implementations can safely ignore them.

---

## 20. Core Interface

```rust
#[derive(Clone, Debug)]
pub struct UpdateResult {
    pub actuator: ActuatorCmd,
    pub status: ChannelStatus,
    pub estimate: StateEstimate,
    pub timing: CycleTiming,
    pub degradation: Option<DegradationEvent>,
}

#[derive(Clone, Debug)]
pub struct HealthReport {
    pub init_state: InitState,
    pub control_law: ControlLawV1,
    pub config_mode: ConfigMode,
    pub transition_state: ConfigTransitionState,
    pub faults: FaultFlags,
    pub channel_health: ChannelHealthV1,
}
// HealthReport is a lightweight snapshot API; ChannelStatus is the per-cycle detailed status returned by update().

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
    fn config_mode(&self) -> ConfigMode;
    fn transition_state(&self) -> ConfigTransitionState;
    fn request_config_mode(&mut self, to: ConfigMode) -> Result<(), TransitionError>;
    
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
    fn get_control_law(&self) -> ControlLawV1;
    fn kick_watchdog(&mut self);
    fn ground_reset(&mut self);
    
    #[cfg(feature = "test-hooks")]
    fn inject_state(&mut self, state: &StateEstimate);
    #[cfg(feature = "test-hooks")]
    fn inject_fault(&mut self, fault: FaultCategory);
}
```

**Record/replay facility:**

Implementations SHOULD provide a record/replay facility where a time-ordered log of
SensorSet, Command, and ActuatorState (plus configuration snapshot) can be fed into
the core to deterministically reproduce UpdateResult sequences for debugging and
verification.

Replay logs SHOULD contain enough information (configuration snapshot, version identifiers,
and all external inputs) to reproduce UpdateResult sequences bit-for-bit for a given core
implementation.

All internal safety-related decisions (law changes, sanitization fallback, major fault
transitions) SHALL be observable via ChannelStatus and/or HealthReport, so that external
monitors and verification tools do not need to infer behavior from actuator outputs alone.

---

## 21. Architecture Invariants

1. Aviate never parses maps, routes, or procedures
2. Aviate never contains network protocol stacks
3. Aviate only operates on physical state and control error
4. Aviate is a "black box control core" driven by external world
5. Control law degradation is monotonic (ground reset to restore)
6. All external inputs are validated before use
7. All outputs include diagnostic/health information
8. Time is explicit, never implicit wall-clock
9. Configuration changes are transactional with rollback
10. Physics calculations use dimensional newtypes, not raw Scalar
11. NaN/Inf never propagates to actuator output
12. Non-Armed states produce only safe/neutral output
13. Fault→Degradation mapping is explicit in FaultHandlingTable
14. Authority philosophy is visible in Config
15. No unsafe code in flight build (HAL boundary only)
16. No dynamic memory allocation at runtime
17. No recursion
18. All loops have statically bounded iteration count
19. No panic-based error handling; all errors explicit
20. No non-deterministic concurrency in core
21. **Actuator coupling is per-mode, not fixed actuator property**
22. **Strongly coupled groups use vector-level fallback, never per-channel**
23. **Sanitizer is last defense, not control strategy**
24. **ConfigMode determines active ModeConfig (mixer, groups, limits)**
25. Core control/estimation uses dimensional newtypes (no bare Scalar) and obeys the language profile summary in §3 (no unsafe, no panics, bounded loops, no alloc, no recursion)
26. GNSS SHALL NOT drive inner attitude/rate loops; its effect on position/velocity estimates SHALL be bounded over finite time windows and may be removed entirely without destabilizing the control core
27. Quaternions used in StateEstimate/Setpoint SHALL be normalized to unit length within tolerance ε; violation is a numeric fault (e.g., NumericError/EstimatorDiverged)
28. Control-law capability (ControlLawV1) and whole-aircraft safety level (SafetyLevelV1) are orthogonal: changing SafetyLevelV1 SHALL NOT implicitly force control-law degradation, and changing ControlLawV1 SHALL NOT, by itself, declare an Emergency safety level.
29. ChannelHealthV1 applies only to a single end-to-end control channel; multiple channels may disagree. External voters and higher layers MUST NOT confuse ChannelHealthV1 with SafetyLevelV1 or ControlLawV1, and SHOULD consider all three dimensions when selecting which channel and law to grant authority.
30. The 16-bit encoding for ControlLawV1 / SafetyLevelV1 SHALL be designed such that future profiles can add finer-grain states as additional codewords within the Voronoi region of each v1 center codeword, and v1 decoders will still project any such codeword to the same coarse ControlLawV1 / SafetyLevelV1 value via nearest-center mapping.
31. All control-plane enum fields received from external memory or buses SHALL be validated for known variants; unknown values trigger `FaultCategory::EnumInvalid` and immediate fallback to safe state, never silent reinterpretation as a different valid mode.
32. Safety-critical fault flags SHALL only be cleared through explicit recovery sequences, not by reading a zero value; monotonic counters SHALL only increase during flight, and decreases or large jumps trigger `FaultCategory::NumericError`.
33. Aviate core SHALL be deterministic, side-effect free, and idempotent over a single control cycle with respect to its public interfaces: given identical inputs at the same logical time (including configuration, InitState, and any internal mode flags encoded in the inputs), multiple independent instances SHALL produce identical outputs. Implementations MAY therefore wrap the core in higher-level redundancy (e.g., TMR, lockstep, hardware ECC) without changing functional behavior.
34. Cross-FCR influence SHALL only occur through explicitly defined interfaces subject to integrity and freshness checks; shared memory or implicit global state between FCRs is forbidden.
35. The control loop SHALL maintain a fixed period; overload is handled by functionality degradation, not by stretching the loop period.
36. Aviate core SHALL NOT autonomously perform a full software reset in flight; it transitions to Backup law and safe outputs, awaiting external supervisory action.
37. All persistent state relevant to control or estimation SHALL be owned by explicit Aviate core instances (e.g., an `AviateKernel` implementation) or documented modules referenced through their interfaces; hidden global mutable state or implicit singleton patterns inside the core are forbidden. Hardware-facing resources (timers, buses, interrupts, DMA) are managed exclusively by HAL/OS layers outside the core boundary.

---

## 22. Minimal Implementation Strategy

| Component | v0.5.1 Minimal Implementation |
|-----------|----------------------------|
| Channels | Only `ChannelId(0)` |
| Sensors | Only `imus[0]` |
| Cross-channel | Always `None` |
| ConfigMode | Single mode (no morphing) |
| Envelope | Simple clamping |
| Control law | Always `Primary` |
| Coupling groups | All motors in single Strong group |
| Fallback | Last good vector only |
| Geometry | Static (no morphing) |

---

## 23. GNSS Usage Boundaries (Spoofing-Resilient)

1. **Scope**: GNSS SHALL NOT be used in inner attitude/rate loops. GNSS is only a low-frequency aid for position/velocity/heading drift; primary altitude/vertical-speed sources are baro + IMU.
2. **Kinematic Gate**: Each GNSS sample MUST be kinematically consistent with recent IMU linear acceleration/angular rates, barometric altitude changes, and airspeed plus a configured max-wind assumption. Violations SHALL be rejected and MAY downgrade GNSS health.
3. **Bounded Authority**: The cumulative correction applied from GNSS to INS position/velocity SHALL be bounded over any finite window (e.g., max Δpos/Δvel per 10s). Beyond the bound, additional GNSS innovation is ignored/down-weighted (effectively INS-only).
4. **Suspect State**: GNSS may enter a Suspect state where measurements are forwarded for diagnostics/telemetry but NOT used to drive control or estimator corrections.
5. **External Override**: Higher-level systems may request GNSS Ignore/Suspect via a minimal override interface; the core shall honor overrides by treating GNSS as lost or suspect accordingly.

---

## Appendix A: Fault → Degradation Quick Reference

| Fault | Action | Degrade To | Response Time |
|-------|--------|------------|---------------|
| Single IMU failed | Isolate | - | 10 ms |
| All IMU failed | Emergency | Backup | 0 ms |
| All GNSS lost | Degrade | Alternate | 100 ms |
| Estimator diverged | Degrade | Alternate | 10 ms |
| Numeric error | Emergency | Backup | 0 ms |
| Command timeout | Degrade | Alternate | 100 ms |
| Actuator numeric | Monitor | - | 0 ms |
| Actuator fallback persistent | Degrade | Alternate | 10 ms |
| Config transition failed | Degrade | Alternate | 0 ms |
| Timing violation (persistent) | Degrade | Alternate | 50 ms |

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
| v0.5.1 | ControlLawV1 / SafetyLevelV1 split; 16-bit center-code; ChannelHealthV1 semantics; SEU resilience (§15.3); FCR/frame integrity (§16); TMR/lockstep support (invariant 33); overload/reset contracts (§17-18); data trust/command authority (§8,12); record/replay (§20); extension strategy (§19); HAL boundary (§3.3); time monotonicity (§7); sensor/command buffering (§8,12); invariants 31-37 |

---

*End of Spec v0.5.1*
