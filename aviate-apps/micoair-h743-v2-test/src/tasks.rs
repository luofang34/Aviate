//! Task separation for DO-178C compliance
//!
//! This module separates high-DAL control logic from low-DAL I/O operations.
//!
//! ## Criticality Levels (DO-178C)
//!
//! ### High-DAL (Development Assurance Level A/B)
//! - `control_task()`: Flight-critical computations
//! - Must have provable WCET (Worst-Case Execution Time)
//! - NO I/O operations (no USB, no UART, no blocking calls)
//! - Only: sensor reads, state estimation, control law, actuator output, telemetry formatting
//!
//! ### Low-DAL (Development Assurance Level D/E)
//! - `telemetry_task()`: Telemetry transmission (can fail without affecting flight)
//! - `command_task()`: Command reception (can fail without affecting flight)
//! - Can perform I/O, can fail, errors logged but don't crash system
//!
//! ## Task Flow
//!
//! ```text
//! control_task() [High-DAL]
//!   ├─ Read sensors (bounded time)
//!   ├─ Run EKF (bounded time)
//!   ├─ Compute control (bounded time)
//!   ├─ Write actuators (bounded time)
//!   └─ Format telemetry → push to queue (bounded time, never blocks)
//!
//! telemetry_task() [Low-DAL]
//!   └─ Pop from queue → transmit via USB/UART (can fail)
//!
//! command_task() [Low-DAL]
//!   └─ Poll CommandGateway → execute if verified (can fail)
//! ```

// Import HAL traits from board re-exports (not directly from aviate-hal-io)
// This ensures apps only depend on board's declared capabilities
use aviate_board_micoair_h743_v2::hal::FrameTx;
use aviate_link::command::CommandLink;
use aviate_security::CommandAuth;

use crate::context::{execute_command, AppContext};

/// High-DAL control task (flight-critical, provable WCET)
///
/// This function must complete in bounded time and never block on I/O.
///
/// ## DO-178C Requirements
///
/// - **WCET**: Provable worst-case execution time (e.g., < 1ms for 1kHz loop)
/// - **NO I/O**: No USB, UART, or other unbounded operations
/// - **Deterministic**: Same inputs → same execution time
///
/// ## Processing Steps
///
/// 1. Read sensors (via DMA or memory-mapped I/O, bounded time)
/// 2. Update state estimator (EKF, fixed iteration count)
/// 3. Compute control outputs (PID/LQR, fixed computation)
/// 4. Write actuator commands (PWM registers, bounded time)
/// 5. Format telemetry frames (pure computation, bounded)
/// 6. Push to TelemetryQueue (O(1), never blocks, may drop if full)
///
/// ## Telemetry Pattern (DO-178C Compliant)
///
/// ```ignore
/// // Format attitude telemetry (pure function, no I/O)
/// let mut buf = [0u8; 280];
/// if let Ok(len) = format_attitude(&state, time_ms, sys_id, comp_id, &mut seq, &mut buf) {
///     // Push to queue (non-blocking, O(1))
///     if ctx.telemetry_queue.push(&buf[..len]).is_err() {
///         ctx.counters.telemetry_dropped += 1; // Count drops, but don't fail
///     }
/// }
/// ```
///
/// ## Error Handling
///
/// - Sensor errors: Use last good value + increment error counter
/// - Queue full: Increment drop counter, continue (old data not critical)
/// - Never panic, never block
pub fn control_task<L, A>(_ctx: &mut AppContext<L, A>) {
    // TODO: Implement control loop when sensors are integrated
    //
    // 1. Read IMU via DMA (bounded time)
    // 2. Read barometer via I2C (bounded time, timeout)
    // 3. Update EKF (fixed iteration count)
    // 4. Compute attitude/rate control
    // 5. Write motor PWM outputs
    // 6. Format telemetry (pure function)
    // 7. Push to telemetry_queue (non-blocking)
    //
    // Example:
    // let state = ekf.update(&imu_data, &baro_data, dt);
    // let actuator_cmd = controller.compute(&state, &setpoint);
    // motors.write_pwm(&actuator_cmd);
    //
    // let mut buf = [0u8; 280];
    // if let Ok(len) = format_attitude(&state, ctx.uptime_ms, 1, 1, &mut ctx.telemetry_seq, &mut buf) {
    //     if ctx.telemetry_queue.push(&buf[..len]).is_err() {
    //         ctx.counters.telemetry_dropped += 1;
    //     }
    // }
}

/// Low-DAL telemetry task (can fail without affecting flight)
///
/// This function pops telemetry frames from the queue and transmits them
/// via the transport layer (USB, UART, etc.).
///
/// ## DO-178C Properties
///
/// - **Criticality**: Low (DAL D/E)
/// - **Can fail**: Transport errors don't affect flight
/// - **Can block**: Waiting for USB/UART is acceptable (but use try_send)
///
/// ## Error Handling
///
/// - Transport error (USB disconnect): Increment counter, continue
/// - Buffer full: Drop frame, increment counter, continue
/// - Never panic, never crash system
///
/// ## Parameters
///
/// - `ctx`: Application context (for telemetry queue and counters)
/// - `transport`: Transport layer implementing FrameTx (USB, UART, etc.)
pub fn telemetry_task<L, A, T: FrameTx>(ctx: &mut AppContext<L, A>, transport: &mut T) {
    // Pop all available frames from queue and transmit
    while ctx.telemetry_queue.pop_with(|frame| {
        match transport.try_send(frame) {
            Ok(()) => {
                ctx.counters.telemetry_sent += 1;
            }
            Err(_) => {
                // Transport error (USB disconnected, buffer full, etc.)
                // This is OK - telemetry is best-effort
                ctx.counters.transport_errors += 1;
            }
        }
    }) {
        // Continue popping until queue empty
    }
}

/// Low-DAL command task (can fail without affecting flight)
///
/// This function polls the CommandGateway for new commands and executes
/// them if verified.
///
/// ## DO-178C Properties
///
/// - **Criticality**: Low (DAL D/E)
/// - **Can fail**: Command errors don't affect flight (control loop continues)
/// - **Security**: ALL commands go through CommandGateway (mandatory)
///
/// ## Processing Steps
///
/// 1. Poll CommandGateway (protocol parse + signature verify)
/// 2. If command verified, execute based on system state
/// 3. Log errors but don't crash
///
/// ## Error Handling
///
/// - Transport error: Log, continue
/// - Parse error: Log, continue
/// - Auth error: Log security alert, increment counter, continue
/// - Execution error: Log, continue
///
/// ## Parameters
///
/// - `ctx`: Application context (for CommandGateway and state)
pub fn command_task<L: CommandLink, A: CommandAuth>(ctx: &mut AppContext<L, A>) {
    // Poll for a verified command
    match ctx.command_gateway.poll_command(ctx.uptime_ms) {
        Ok(Some(cmd)) => {
            // Command verified! Safe to execute
            ctx.counters.commands_received += 1;

            if !execute_command(ctx, &cmd) {
                // Command rejected (e.g., not armed, not allowed in current mode)
                ctx.counters.commands_rejected += 1;
                // TODO: Log rejection for debugging
            }
        }
        Ok(None) => {
            // No command available (not an error)
        }
        Err(_e) => {
            // Error in transport, parsing, or verification
            ctx.counters.commands_rejected += 1;

            // TODO: Log error for debugging
            // match _e {
            //     GatewayError::Link(_) => { /* Transport or parse error */ }
            //     GatewayError::Auth(_) => { /* Security violation - log alert! */ }
            //     _ => {}
            // }
        }
    }
}

/// Example: Format heartbeat telemetry (pure function, safe for high-DAL)
///
/// This demonstrates the pattern for telemetry formatting in high-DAL code.
/// Pure functions like this can be called from control_task() because they:
/// - Have bounded execution time
/// - Don't perform I/O
/// - Don't block
///
/// ## Parameters
///
/// - `armed`: System armed state
/// - `uptime_ms`: System uptime in milliseconds
/// - `sys_id`: MAVLink system ID
/// - `comp_id`: MAVLink component ID
/// - `seq`: Sequence counter (will be incremented)
/// - `buf`: Output buffer for formatted frame
///
/// ## Returns
///
/// - `Ok(len)`: Number of bytes written to buf
/// - `Err(...)`: Formatting error (buffer too small, etc.)
#[allow(unused)]
fn format_heartbeat(
    armed: bool,
    uptime_ms: u32,
    sys_id: u8,
    comp_id: u8,
    seq: &mut u8,
    buf: &mut [u8],
) -> Result<usize, ()> {
    // TODO: Use aviate_link::telemetry_mavlink::format_heartbeat() when available
    // For now, return placeholder
    Err(())
}
