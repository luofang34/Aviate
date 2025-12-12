//! Example flight application demonstrating DO-178C architecture
//!
//! This file shows the complete wiring of the 6-layer architecture:
//!
//! ```text
//! Layer 1: HAL Traits (aviate-hal-io)
//! Layer 2: Chip HAL (aviate-hal-stm32h7)
//! Layer 3: Core (aviate-core)
//! Layer 4: Link (aviate-link)
//! Layer 5: Security (aviate-security)
//! Layer 6: App (THIS FILE)
//! ```
//!
//! ## Usage
//!
//! To build this example instead of the test app:
//! 1. Update Cargo.toml [[bin]] section:
//!    ```toml
//!    [[bin]]
//!    name = "flight-app"
//!    path = "src/example_flight_app.rs"
//!    ```
//! 2. Build: `cargo build --bin flight-app`
//!
//! ## Architecture Demonstrated
//!
//! - AppContext struct (no static mut globals)
//! - Task separation (control/telemetry/command)
//! - CommandGateway as sole command entry point
//! - TelemetryQueue for high-DAL→low-DAL communication
//! - Security layer integration (PlainAuth for development)

#![no_std]
#![no_main]
#![allow(unused)]

use panic_halt as _;

use cortex_m_rt::entry;

// Import context and tasks modules
mod context;
mod tasks;

use context::AppContext;
use tasks::{command_task, control_task, telemetry_task};

// HAL imports via board re-exports (apps should not depend on aviate-hal-io directly)
use aviate_board_micoair_h743_v2::hal::{
    FrameRx, FrameTx, KeyStore, Stm32h7CryptoEngine, Stm32h7KeyStore, TransportError,
};
use aviate_link::command::{Command, CommandLink};
use aviate_link::errors::LinkResult;
use aviate_security::{CommandGateway, PlainAuth};

/// Mock transport for demonstration (replace with real USB/UART in production)
struct MockTransport;

impl FrameTx for MockTransport {
    fn try_send(&mut self, _frame: &[u8]) -> Result<(), TransportError> {
        // TODO: Replace with real USB CDC or UART
        Ok(())
    }
}

impl FrameRx for MockTransport {
    fn try_recv(&mut self, _buf: &mut [u8]) -> Result<usize, TransportError> {
        // TODO: Replace with real USB CDC or UART
        Ok(0) // No data available
    }
}

/// Mock command link for demonstration (replace with MavlinkCommandLink in production)
struct MockCommandLink {
    transport: MockTransport,
}

impl CommandLink for MockCommandLink {
    fn poll_command(&mut self, _now_ms: u32) -> LinkResult<Option<Command>> {
        // TODO: Replace with real MavlinkCommandLink<T: FrameRx>
        Ok(None)
    }
}

#[entry]
fn main() -> ! {
    // ============================================================================
    // Layer 1-2: Initialize hardware (KeyStore, CryptoEngine, Transports)
    // ============================================================================

    let keystore = Stm32h7KeyStore::new();
    let crypto = Stm32h7CryptoEngine::new();

    // TODO: Initialize real transports (USB CDC, UART, etc.)
    let telemetry_transport = MockTransport;

    // ============================================================================
    // Layer 4: Initialize link layer (protocol parsing)
    // ============================================================================

    let command_link = MockCommandLink {
        transport: MockTransport,
    };

    // ============================================================================
    // Layer 5: Initialize security layer (authentication + gateway)
    // ============================================================================

    // For development: Use PlainAuth (no verification)
    // For production: Use SignedAuth::new(keystore, crypto)
    let auth = PlainAuth::new();

    let command_gateway = CommandGateway::new(command_link, auth);

    // ============================================================================
    // Layer 6: Create application context
    // ============================================================================

    let mut ctx = AppContext::new(command_gateway);
    let mut telemetry_transport = telemetry_transport;

    // ============================================================================
    // Main loop: Call tasks in sequence
    // ============================================================================

    let mut tick_count: u32 = 0;

    loop {
        // Update uptime
        ctx.uptime_ms = tick_count;

        // High-DAL: Control loop (1kHz rate, provable WCET)
        //
        // This task MUST complete in bounded time and never block.
        // It reads sensors, runs state estimation, computes control outputs,
        // and formats telemetry (pushes to queue).
        control_task(&mut ctx);

        // Low-DAL: Telemetry transmission (best-effort)
        //
        // This task pops frames from queue and transmits via transport.
        // Failures are logged but don't affect flight.
        telemetry_task(&mut ctx, &mut telemetry_transport);

        // Low-DAL: Command reception (best-effort)
        //
        // This task polls CommandGateway for new commands.
        // ALL commands go through security verification before execution.
        command_task(&mut ctx);

        // Increment tick counter (1ms ticks in real system)
        tick_count = tick_count.wrapping_add(1);

        // TODO: Add proper timing (SysTick interrupt or RTIC framework)
        // For now, just spin
    }
}

// ============================================================================
// Production Configuration Notes
// ============================================================================

/// For production flight systems, make these changes:
///
/// 1. **Replace PlainAuth with SignedAuth**:
///    ```rust
///    let auth = SignedAuth::new(keystore, crypto);
///    ```
///
/// 2. **Use real transports** (USB CDC, UART):
///    ```rust
///    use aviate_hal_stm32h7::transport::{Stm32h7UsbCdcTx, Stm32h7UsbCdcRx};
///    let usb_tx = Stm32h7UsbCdcTx::new(usb_serial);
///    let usb_rx = Stm32h7UsbCdcRx::new(usb_serial);
///    ```
///
/// 3. **Use MavlinkCommandLink**:
///    ```rust
///    use aviate_link::mavlink::MavlinkCommandLink;
///    let command_link = MavlinkCommandLink::new(usb_rx);
///    ```
///
/// 4. **Add real sensor drivers**:
///    - BMI088 IMU (SPI2)
///    - SPL06 barometer (I2C2)
///    - QMC5883L magnetometer (I2C1)
///
/// 5. **Integrate control loop**:
///    - EKF state estimation
///    - Attitude/rate controller
///    - Motor mixing
///    - PWM output
///
/// 6. **Add telemetry formatting**:
///    ```rust
///    use aviate_link::telemetry_mavlink::{format_heartbeat, format_attitude};
///    ```
///
/// 7. **Use RTIC or similar framework** for deterministic scheduling:
///    - Control loop: 1kHz (1ms period)
///    - Telemetry: 50Hz (20ms period)
///    - Commands: As available (event-driven)
