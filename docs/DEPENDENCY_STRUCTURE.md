# Aviate Dependency Structure

This document explains the dependency layering philosophy for Aviate flight control software.

## Core Principle: Separation of Traits and Implementations

```
Apps depend on:
  - Traits (interfaces) → Can write generic code
  - Abstractions (board, airframe) → Select hardware/dynamics

Boards depend on:
  - Implementations (chip HAL) → Provide hardware-specific code
  - Traits (HAL interfaces) → Implement the contracts

Chips depend on:
  - Traits (HAL interfaces) → Implement for specific hardware
```

## Dependency Graph (Revised)

### Layer 1: HAL Traits (Interface Definitions)

**Crate**: `aviate-hal-io`

**Purpose**: Define hardware abstraction traits (no implementations)

**Exports**:
- `KeyStore` - Key storage abstraction
- `CryptoEngine` - Cryptographic operations abstraction
- `FrameTx` / `FrameRx` - Transport abstractions
- Sensor/actuator driver traits

**Dependencies**: None

**Who uses it**:
- ✅ Chip HAL implementations (to implement traits)
- ✅ Apps (to write generic code over traits)
- ✅ Boards (to re-export or compose)

### Layer 2: Chip Runtime HAL (Hardware Implementations)

**Crate**: `aviate-hal-stm32h7`

**Purpose**: STM32H7 family implementations of HAL traits

**Exports**:
- `Stm32h7KeyStore` - OTP/flash key storage for STM32H7
- `Stm32h7CryptoEngine` - HMAC-SHA256 using `sha2` + `hmac`
- `Stm32h7UsbCdcTx/Rx` - USB CDC transport (future)

**Dependencies**:
- `aviate-hal-io` (implements these traits)
- `stm32h7xx-hal` (vendor HAL)
- `sha2`, `hmac` (crypto crates)

**Who uses it**:
- ✅ Board crates (select which chip HAL to use)
- ❌ Apps (should NOT depend directly - use board abstraction)

### Layer 2.5: Board Configuration

**Crate**: `aviate-board-micoair-h743-v2`

**Purpose**: Board-specific pin mappings, sensor configuration, and HAL capability interface

**Exports**:
- Pin definitions (LED, motor, sensor CS pins)
- Sensor configurations (I2C addresses, rotation)
- Board metadata (name, variant, MCU)
- **HAL re-exports** (`hal` module) ⭐ **Apps use these instead of direct HAL dependencies**

**HAL Re-exports** (`pub mod hal`):
```rust
// Transport traits
pub use aviate_hal_io::transport::{FrameTx, FrameRx, TransportError};

// Security traits
pub use aviate_hal_io::security::{KeyStore, CryptoEngine, ...};

// Board's chip implementations
pub use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
```

**Dependencies**:
- `aviate-hal-io` (trait definitions) - **re-exported via `hal` module**
- `aviate-hal-stm32h7` ⭐ **Chip HAL (board's choice) - re-exported via `hal` module**
- `aviate-drivers` (sensor driver library)
- `aviate-core` (data types)

**Who uses it**:
- ✅ Apps (select which board to build for, get HAL traits via re-exports)
- ❌ Other boards (each board is independent)

### Layer 3: Core Data Types

**Crate**: `aviate-core`

**Purpose**: Shared data structures (protocol-agnostic)

**Exports**:
- `StateEstimate` - EKF output
- `ActuatorCmd` - Motor commands
- `ChannelStatus` - System status

**Dependencies**: None (pure data types)

**Who uses it**: Everyone (data interchange format)

### Layer 4: Protocol Abstraction

**Crate**: `aviate-link`

**Purpose**: Protocol-agnostic telemetry and command abstraction

**Exports**:
- `CommandLink` trait - Protocol parsing → Command
- `TelemetryQueue` - Bounded ring buffer for telemetry
- `Command` - Domain-level command representation

**Dependencies**:
- `aviate-core` (data types)
- `aviate-hal-io` (transport traits)

**Who uses it**:
- ✅ Apps (protocol abstraction)
- ✅ Security layer (command verification)

### Layer 5: Security Policy

**Crate**: `aviate-security`

**Purpose**: Command authentication and verification

**Exports**:
- `CommandGateway<L, A>` - Unified command entry point
- `PlainAuth` / `SignedAuth` - Authentication implementations
- `AntiReplayWindow` - Replay attack prevention

**Dependencies**:
- `aviate-hal-io` (trait definitions for KeyStore/CryptoEngine)
- `aviate-link` (Command type)

**Who uses it**:
- ✅ Apps (security enforcement)

### Layer 6: Airframe Dynamics

**Crate**: `aviate-airframe-multirotor`

**Purpose**: Flight dynamics and control algorithms

**Exports**:
- Mixer configurations (quad-x, quad-plus, hex, etc.)
- Control laws (PID, LQR, etc.)

**Dependencies**:
- `aviate-core` (data types)

**Who uses it**:
- ✅ Apps (select airframe type)

### Layer 7: Application

**Crate**: `aviate-app-micoair-h743-v2-test`

**Purpose**: Wire all components together for specific use case

**Dependencies** (simplified via board re-exports):
```toml
# Board (provides HAL traits + implementations via re-exports)
aviate-board-micoair-h743-v2 = { path = "..." }

# Airframe selection (flight dynamics)
aviate-airframe-multirotor = { path = "...", features = ["quad-x"] }

# Protocol and security layers
aviate-link = { path = "..." }
aviate-security = { path = "..." }
```

**How app gets HAL traits**:
```rust
// Import from board's re-exports, not directly from aviate-hal-io
use aviate_board_micoair_h743_v2::hal::{FrameTx, FrameRx};

pub fn telemetry_task<T: FrameTx>(transport: &mut T) {
    // Generic code using board's capabilities
}
```

**What app does NOT depend on**:
- ❌ `aviate-hal-io` - HAL traits (accessed via board re-exports)
- ❌ `aviate-hal-stm32h7` - Chip HAL (board's responsibility)
- ❌ `aviate-drivers` - Sensor drivers (board's responsibility)

## Dependency Rules

### ✅ Good Practices

1. **Apps import HAL traits from board re-exports**:
   ```rust
   // App imports from board's hal module
   use aviate_board_micoair_h743_v2::hal::{FrameTx, FrameRx};

   // Generic code works with any board that provides these traits
   fn telemetry_task<T: FrameTx>(transport: &mut T) { ... }
   ```

2. **Board re-exports HAL capabilities**:
   ```rust
   // In board's lib.rs
   pub mod hal {
       // Re-export traits this board supports
       pub use aviate_hal_io::transport::{FrameTx, FrameRx};

       // Re-export board's implementations
       pub use aviate_hal_stm32h7::{Stm32h7KeyStore, Stm32h7CryptoEngine};
   }
   ```

3. **Board encapsulates chip choice**:
   ```toml
   # In aviate-board-micoair-h743-v2/Cargo.toml
   aviate-hal-io = { path = "..." }       # ✅ Board depends on traits
   aviate-hal-stm32h7 = { path = "..." }  # ✅ Board chooses chip implementation

   # Apps DON'T need to know which chip (use board re-exports)
   ```

4. **Cross-cutting concerns (protocol, security) at app level**:
   ```toml
   # App explicitly declares these dependencies
   aviate-link = { path = "..." }
   aviate-security = { path = "..." }
   ```

### ❌ Anti-Patterns

1. **App importing HAL traits directly** (instead of via board):
   ```rust
   // ❌ WRONG - bypasses board abstraction
   use aviate_hal_io::transport::FrameTx;

   // ✅ CORRECT - use board's re-exports
   use aviate_board_micoair_h743_v2::hal::FrameTx;
   ```

2. **App depending on HAL crates in Cargo.toml**:
   ```toml
   # ❌ WRONG - app shouldn't depend on HAL directly
   aviate-hal-io = { path = "..." }
   aviate-hal-stm32h7 = { path = "..." }

   # ✅ CORRECT - only board dependency
   aviate-board-micoair-h743-v2 = { path = "..." }
   ```

3. **App depending on chip HAL directly**:
   ```toml
   # ❌ WRONG - app shouldn't know which chip board uses
   aviate-hal-stm32h7 = { path = "..." }
   ```

4. **App depending on vendor HAL directly**:
   ```toml
   # ❌ WRONG - this is board's responsibility
   stm32h7xx-hal = { version = "..." }
   ```

5. **Board hiding cross-cutting concerns**:
   ```toml
   # ❌ WRONG - security is app-level policy, not board-level
   # Board should NOT pull in aviate-security
   ```

## Rationale

### Why Apps Import HAL Traits via Board Re-exports?

Apps need trait definitions to write generic code, but should get them from the board's declared capabilities:

```rust
// App imports from board's hal module (not directly from aviate-hal-io)
use aviate_board_micoair_h743_v2::hal::FrameTx;

pub fn telemetry_task<T: FrameTx>(transport: &mut T) {
    // Can work with any FrameTx implementation the board provides
}
```

**Benefits**:
- **Single source of truth**: Board declares what it supports
- **Encapsulation**: App doesn't need to know about aviate-hal-io or chip HAL
- **Flexibility**: Different boards can provide same traits with different implementations
- **Clear contract**: "This board provides these capabilities"

### Why Apps DON'T Depend on `aviate-hal-stm32h7`?

Chip HAL is an implementation detail hidden by board:

```rust
// App creates board-provided transport (type comes from board)
use aviate_board_micoair_h743_v2 as board;

// Board documentation tells you which types to use
// App doesn't need to know it's STM32H7-specific
```

This allows:
- Switching boards without changing app code
- Multiple boards sharing same app codebase
- Board encapsulation of hardware details

### Why Boards Depend on Chip HAL?

Boards are the "glue" that selects hardware:

```toml
# Board's Cargo.toml
aviate-hal-stm32h7 = { path = "..." }  # This board uses STM32H7
```

Different boards can use different chips:

```toml
# aviate-board-pixhawk6x/Cargo.toml
aviate-hal-stm32h7 = { path = "..." }  # STM32H753

# aviate-board-kakuteh7/Cargo.toml
aviate-hal-stm32h7 = { path = "..." }  # STM32H743

# aviate-board-matekf405/Cargo.toml
aviate-hal-stm32f4 = { path = "..." }  # STM32F405 (different chip!)
```

## Example: Adding a New Board

1. **Create board crate**: `aviate-boards/pixhawk6x/`

2. **Board selects chip HAL**:
   ```toml
   [dependencies]
   aviate-hal-io = { path = "..." }
   aviate-hal-stm32h7 = { path = "..." }  # This board uses H7
   ```

3. **App switches boards** (no other changes needed):
   ```toml
   # Change this line only:
   aviate-board-pixhawk6x = { path = "..." }
   ```

4. **App code unchanged** (uses generic traits):
   ```rust
   // Still works with new board
   fn telemetry_task<T: FrameTx>(transport: &mut T) { ... }
   ```

## Summary

**Final clean dependency hierarchy** (via board re-exports):
```
App
 ├─ Board (owns chip choice, re-exports HAL traits)
 │   ├─ HAL Traits (interface definitions)
 │   └─ Chip HAL (implementations)
 ├─ Airframe (flight dynamics)
 ├─ Link (protocol abstraction)
 └─ Security (policy enforcement)
```

**Key insights**:
1. Apps get HAL traits **via board re-exports**, not direct dependencies
2. Board is the **single source of truth** for hardware capabilities
3. Apps depend on **abstractions** (board, airframe, protocols), board depends on **implementations** (chip HAL)

**Actual app dependencies** (4 crates):
```toml
aviate-board-xxx        # Board (provides hal module with re-exports)
aviate-airframe-xxx     # Airframe
aviate-link             # Protocol
aviate-security         # Security
```

This structure enables:
- ✅ **Simplified dependencies**: Apps don't know about HAL crates
- ✅ **Board portability**: Change board → app recompiles with new capabilities
- ✅ **Chip portability**: Board encapsulates chip choice
- ✅ **Clean layering**: Clear separation of concerns
- ✅ **Testability**: Apps can use mock boards for testing
- ✅ **Single source of truth**: Board declares what it supports via `hal` module
