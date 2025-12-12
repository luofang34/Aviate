//! Application context - aggregates all system components
//!
//! This module defines the `AppContext` struct which replaces `static mut` globals
//! with a structured, owned context passed between tasks.
//!
//! ## DO-178C Pattern
//!
//! ```text
//! AppContext (owned state)
//!   ├─ CommandGateway<L, A> (security layer)
//!   ├─ TelemetryQueue (bounded ring buffer)
//!   ├─ System state (armed, mode, etc.)
//!   └─ Counters (telemetry drops, command rejects, etc.)
//! ```
//!
//! ## Critical Safety Rule
//!
//! - High-DAL control code: Read sensors, run EKF, compute actuator outputs, format telemetry
//! - Low-DAL I/O code: Send telemetry, receive commands, handle USB/UART errors
//!
//! Separation ensures control loop WCET is provable and not affected by I/O failures.

use aviate_link::command::Command;
use aviate_link::queue::TelemetryQueue;
use aviate_security::CommandGateway;

/// Application context for MicoAir H743-V2 test application
///
/// This struct aggregates all system components and state. It is passed
/// mutably to each task function, eliminating the need for `static mut` globals.
///
/// ## Ownership Model
///
/// - Created once in main()
/// - Passed as `&mut AppContext` to each task
/// - NO static mut, NO globals (except hardware peripherals if unavoidable)
///
/// ## Benefits
///
/// - Clear ownership and lifetimes
/// - Easier to test (can create AppContext in tests)
/// - Safer (no data races from static mut)
/// - DO-178C auditable (explicit state flow)
pub struct AppContext<L, A> {
    /// Command gateway (protocol parsing + security verification)
    ///
    /// This is the ONLY way commands should enter the system.
    /// Bypass paths are prohibited.
    pub command_gateway: CommandGateway<L, A>,

    /// Telemetry queue (bounded ring buffer for high-DAL→low-DAL)
    ///
    /// High-DAL control code pushes formatted frames here (non-blocking).
    /// Low-DAL telemetry task pops and transmits (can fail safely).
    pub telemetry_queue: TelemetryQueue<32, 280>, // 32 frames, 280 bytes each

    /// System armed state
    ///
    /// - true: Motors can spin (commands accepted)
    /// - false: Motors locked (disarm commands only)
    pub armed: bool,

    /// Telemetry sequence counter (MAVLink requirement)
    pub telemetry_seq: u8,

    /// System uptime (milliseconds since boot)
    ///
    /// Updated by main loop, used for timestamps
    pub uptime_ms: u32,

    /// Diagnostic counters
    pub counters: DiagnosticCounters,
}

impl<L, A> AppContext<L, A> {
    /// Create new application context
    ///
    /// ## Parameters
    ///
    /// - `command_gateway`: Configured CommandGateway (link + auth)
    pub fn new(command_gateway: CommandGateway<L, A>) -> Self {
        Self {
            command_gateway,
            telemetry_queue: TelemetryQueue::new(),
            armed: false,
            telemetry_seq: 0,
            uptime_ms: 0,
            counters: DiagnosticCounters::new(),
        }
    }
}

/// Diagnostic counters for telemetry and debugging
///
/// These counters help track system health and performance.
/// They can be included in periodic status telemetry.
#[derive(Clone, Copy, Debug)]
pub struct DiagnosticCounters {
    /// Number of commands received (before security check)
    pub commands_received: u32,

    /// Number of commands accepted (after security verification)
    pub commands_accepted: u32,

    /// Number of commands rejected (failed auth/anti-replay)
    pub commands_rejected: u32,

    /// Number of telemetry frames dropped (queue full)
    pub telemetry_dropped: u32,

    /// Number of telemetry frames sent successfully
    pub telemetry_sent: u32,

    /// Number of transport errors (USB disconnect, etc.)
    pub transport_errors: u32,
}

impl DiagnosticCounters {
    pub const fn new() -> Self {
        Self {
            commands_received: 0,
            commands_accepted: 0,
            commands_rejected: 0,
            telemetry_dropped: 0,
            telemetry_sent: 0,
            transport_errors: 0,
        }
    }
}

/// Process a verified command
///
/// This function is called by command_task() after CommandGateway verification.
/// It executes the command based on system state and safety checks.
///
/// ## Safety Checks (DO-178C)
///
/// - Arm command: Only if system healthy (sensors OK, EKF converged, etc.)
/// - Disarm command: Always allowed (safety escape)
/// - Flight commands: Only if armed
///
/// ## Parameters
///
/// - `ctx`: Application context (for state updates)
/// - `cmd`: Verified command from CommandGateway
///
/// ## Returns
///
/// - true: Command executed successfully
/// - false: Command rejected (not allowed in current state)
pub fn execute_command<L, A>(ctx: &mut AppContext<L, A>, cmd: &Command) -> bool {
    use aviate_link::command::CommandKind;

    match cmd.kind {
        CommandKind::Arm => {
            // Safety check: Only arm if system is ready
            // TODO: Add sensor health checks, EKF convergence, etc.
            if !ctx.armed {
                ctx.armed = true;
                ctx.counters.commands_accepted += 1;
                true
            } else {
                // Already armed, ignore
                false
            }
        }

        CommandKind::Disarm => {
            // Always allow disarm (safety escape)
            if ctx.armed {
                ctx.armed = false;
            }
            ctx.counters.commands_accepted += 1;
            true
        }

        CommandKind::SetMode => {
            // TODO: Implement flight mode switching
            ctx.counters.commands_accepted += 1;
            true
        }

        CommandKind::SetAttitude | CommandKind::SetRate | CommandKind::SetThrust => {
            // Only accept flight commands if armed
            if ctx.armed {
                // TODO: Send to control loop (attitude/rate setpoint)
                ctx.counters.commands_accepted += 1;
                true
            } else {
                // Not armed, reject
                ctx.counters.commands_rejected += 1;
                false
            }
        }
    }
}
