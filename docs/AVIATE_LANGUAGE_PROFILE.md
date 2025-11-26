# Aviate Language & Implementation Profile

**LLM-Friendly, Verification-Oriented Specification**

---

## 1. Purpose

This document defines the language constraints for Aviate, a flight control kernel. These rules ensure code is:

- Statically analyzable (control flow, data flow, stack depth, WCET)
- Suitable for safety certification (DO-178C)
- Safe for LLM generation and review (no dangerous patterns)

This describes **behavioral requirements**, not specific linter configurations. Tools may change; these constraints do not.

---

## 2. Rust Usage & Build Profiles

### 2.1 Language & Runtime

- Implementation language: **Rust**
- Runtime: **`#![no_std]`** mandatory
- Standard library: **`core` only** in flight code
- `alloc` crate: **Test/simulation only**, never in flight builds

### 2.2 Build Profiles

| Profile | Purpose | Constraints |
|---------|---------|-------------|
| **Flight** | Real hardware, test flights, HIL | All rules enforced |
| **Dev/Test** | Unit tests, property testing, SITL | Relaxed (but isolated) |

**Critical**: CI must guarantee flight builds never include test code.

---

## 3. Prohibited Features (Flight Build)

### 3.1 No `unsafe`

**Rule**: No `unsafe fn`, `unsafe { }` blocks, or `unsafe trait` implementations in Aviate core.

Hardware interaction (registers, assembly, FFI) must be isolated in a separate HAL crate with its own safety review. HAL exposes only safe interfaces to Aviate.

### 3.2 No Dynamic Memory Allocation

**Rule**: Aviate shall not allocate or free heap memory at runtime.

- No `alloc` crate
- No `Box`, `Vec`, `String`, `Rc`, `Arc`
- All memory: static or stack-allocated
- All array sizes: compile-time constants

This ensures deterministic memory usage and eliminates allocation failure modes.

### 3.3 No Recursion

**Rule**: All functions must be non-recursive. No direct or indirect recursive calls.

This guarantees bounded call depth and predictable stack usage.

### 3.4 No Unbounded Loops

**Rule**: Every loop must have a statically bounded iteration count.

**Prohibited patterns**:
```rust
// BAD: unbounded loop
loop {
    if converged { break; }
}

// BAD: condition not statically analyzable
while !converged { ... }
```

**Allowed patterns**:
```rust
// GOOD: fixed bound
for i in 0..MAX_ITERATIONS { ... }

// GOOD: fixed-size array
for sensor in sensors.iter() { ... }

// GOOD: bounded with explicit maximum
for _ in 0..MAX_NEWTON_ITERATIONS {
    if converged { break; }
}
// Must still produce valid output if max reached
```

For numeric algorithms (Newton-Raphson, etc.):
- Use compile-time maximum iteration count
- Return valid (possibly degraded) result even if not converged
- Report non-convergence as a fault condition

### 3.5 No Panic-Based Error Handling

**Rule**: Flight code shall not use `unwrap()`, `expect()`, `panic!()`, `unreachable!()`, `todo!()`, or `unimplemented!()`.

All errors must be handled explicitly:
- `Result<T, E>` → Map to fault flags, degradation, or init blockers
- `Option<T>` → Explicit match or safe defaults

### 3.6 Panic Strategy

**Rule**: Flight builds use `panic = "abort"`. Any panic is a design error.

Production binaries must be verified panic-free via static analysis.

### 3.7 No Non-Deterministic Concurrency

**Rule**: Aviate core does not use threads, async/await, or interrupt-driven state modification.

All state updates occur within explicit `update()` calls with explicit `TimeDelta` time input.

HAL may use interrupts for sensor sampling, but must present a stable `SensorSet` snapshot to Aviate.

---

## 4. Recommended Lint Configuration

For Flight Build crates (example, may evolve with tooling):

```rust
#![no_std]
#![forbid(unsafe_code)]
#![deny(unused_must_use)]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
#![deny(clippy::unreachable)]
#![deny(clippy::todo)]
#![deny(clippy::unimplemented)]
```

The specification constrains **behavior**. Specific lints are implementation details.

---

## 5. Numeric Representation

### 5.1 Base Type

- `Scalar = f32` as the base floating-point type
- Hardware FPU assumed (STM32H7, etc.)

### 5.2 Dimensional Newtypes (Mandatory)

All physical quantities must use dimensional newtypes:

```rust
Meters, Seconds, Radians
MetersPerSecond, MetersPerSecondSquared
RadiansPerSecond
Newtons, NewtonMeters
Pascals, Celsius
Normalized      // [0.0, 1.0]
NormalizedSigned // [-1.0, 1.0]
```

### 5.3 Key Constraint

**Rule**: Control and estimation code shall not use raw `Scalar` for physical quantities. Only dimensional newtypes are permitted.

Raw `Scalar` is allowed only for dimensionless values (coefficients, damping factors, quality metrics) with explicit comments.

This enables simple static analysis to find unit-mixing bugs.

### 5.4 NaN/Inf Policy

**Rule**: Aviate shall not allow NaN or Inf to propagate through the system.

| Location | Action |
|----------|--------|
| External input | Validate with `is_finite()`, reject invalid |
| Internal computation | Detect → Fault (EstimatorDiverged, NumericError) |
| Actuator output | Sanitize before hardware, never emit NaN/Inf |

---

## 6. Actuator Output Sanitization

### 6.1 The Problem

Per-channel sanitization (replacing one bad channel's value while keeping others) creates catastrophic torque imbalance on coupled systems like quadrotors.

### 6.2 Vector-Level Rule

**Rule**: For strongly coupled actuator groups (e.g., hover-mode quadrotor motors), detection of a numeric error in ANY member shall cause rejection of the ENTIRE newly computed actuator vector for that group.

A coherent fallback vector shall be used instead:
1. Last known good vector (if recent enough)
2. Predefined safe vector (e.g., symmetric descent thrust)
3. Zero output (only if no valid fallback available)

**Never**: Mix new values with sanitized defaults in a coupled group.

### 6.3 Coupling is Per-Mode

**Critical**: The same physical actuators may have different coupling semantics in different flight modes.

| Mode | Quad Motors | Coupling | Rationale |
|------|-------------|----------|-----------|
| Hover | Lift + attitude | **Strong** | Single failure = catastrophic |
| Cruise (as pullers) | Distributed thrust | **Weak** | Single failure = degraded performance |

This is configured via `ModeConfig` in the Spec, not hardcoded per actuator.

### 6.3 Sanitizer Role

Sanitization is the **last line of defense**, not a control strategy.

When sanitizer triggers fallback:
1. Set appropriate fault flag
2. Increment fallback counter
3. If persistent, trigger control law degradation
4. Let degraded control law handle recovery

---

## 7. SITL & no_std Boundary

### 7.1 Aviate Core Responsibility

- Always `no_std`, depends only on `core`
- Receives time via explicit `TimeDelta` parameter
- Receives sensors via `SensorSet` snapshot
- Never directly accesses: system clock, network, filesystem, OS APIs

### 7.2 SITL/Simulation Responsibility

- May use `std`, networking, threads, files
- Obtains simulator time (real-time or lockstep)
- Samples sensors, builds `SensorSet`
- Calls Aviate `update()`, receives `ActuatorCmd`
- Translates commands to simulator format

### 7.3 Lockstep vs RealTime

| Mode | Time Source | Aviate Behavior |
|------|-------------|-----------------|
| Lockstep | Simulator controls | No sleep, no wall-clock reads |
| RealTime | System clock → `TimeDelta` | Still validates dt bounds |

---

## 8. Static Analysis Requirements

CI pipeline must verify:

| Check | Method |
|-------|--------|
| No unsafe | `#![forbid(unsafe_code)]` |
| No unwrap/expect/panic | Clippy lints |
| No recursion | Custom lint or code review |
| Bounded loops | Review + annotation |
| No alloc | Build without alloc feature |
| Stack usage | Static analysis tool |
| WCET | Measurement + analysis |

---

## 9. LLM Code Generation Guidelines

When generating Aviate code, LLMs must:

**Never use**:
- `unwrap()`, `expect()`, `panic!()`, `todo!()`, `unimplemented!()`
- `unsafe`
- `Vec`, `Box`, `String`, or any heap-allocating types
- Recursive functions
- Unbounded loops (`loop {}` without explicit max, `while condition`)

**Always use**:
- Dimensional newtypes for physical quantities (`Meters`, `Seconds`, etc.)
- Explicit error handling with `Result`/`Option`
- `for` loops with fixed bounds or fixed-size array iteration
- Explicit maximum iteration counts for numeric algorithms

**When iterative convergence is needed**:
- Define `MAX_ITERATIONS` constant
- Return valid output even if max iterations reached
- Report non-convergence as fault/quality degradation

---

## 10. Summary

These constraints transform Rust into a verifiable subset suitable for safety-critical flight control:

| Property | How Achieved |
|----------|--------------|
| Bounded memory | No heap, static/stack only |
| Bounded stack | No recursion |
| Bounded execution | No unbounded loops |
| No runtime failure | No panics, explicit errors |
| Deterministic timing | No concurrency, explicit time |
| Numeric safety | Dimensional types, NaN rejection |
| Physical safety | Vector-level actuator fallback |

This profile is analogous to MISRA-C or JSF++ but leverages Rust's type system for stronger compile-time guarantees.
