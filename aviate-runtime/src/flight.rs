//! Flight runtime for real hardware (DO-178C path)
//!
//! This module provides the runtime infrastructure for hardware flight.
//! It re-exports `FlightRunner` and related types from the generic runner,
//! and provides hardware-specific utilities.
//!
//! # Safety Requirements (DO-178C)
//!
//! - NO simulator dependencies (enforced by feature guards)
//! - NO heap allocations in flight path
//! - Bounded execution time for all operations
//! - Deterministic control loop timing
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Application (e.g., quadcopter-stm32h7)                    │
//! │  - Calls board.into_runner().run(period_us)                │
//! └─────────────────────────────────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Board Crate (e.g., micoair-h743-v2)                       │
//! │  - Creates BoardHal with real sensors                      │
//! │  - Creates Stm32h7Time (DWT-based)                         │
//! │  - Creates Stm32h7Transport (USB/UART)                     │
//! │  - Provides into_runner() -> FlightRunner<...>             │
//! └─────────────────────────────────────────────────────────────┘
//!                           │
//!                           ▼
//! ┌─────────────────────────────────────────────────────────────┐
//! │  FlightRunner<Board, Time, Transport, Cmd> (this module)   │
//! │  - Generic control loop with bounded catch-up              │
//! │  - step(tick_us, now_us, dt_us) for each control tick      │
//! │  - Health tracking (link_ok, sensors_ok, ekf_converged)    │
//! │  - Watchdog kick per tick                                  │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Usage
//!
//! ```ignore
//! // In application main.rs (no_std, no_main)
//! use aviate_board_micoair_h743_v2::MicoAirH743Board;
//!
//! #[entry]
//! fn main() -> ! {
//!     let dp = pac::Peripherals::take().unwrap();
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!
//!     // Board creates sensors, time source, and transport
//!     let board = MicoAirH743Board::new(dp, cp, &APP_CONFIG);
//!
//!     // Get runner and start control loop
//!     let mut runner = board.into_runner();
//!     runner.run(1000)  // 1kHz = 1000us period
//! }
//! ```

// Re-export FlightRunner and related types for hardware use
pub use crate::runner::{BoardStep, FlightRunner, RunnerHealth, LINK_TIMEOUT_US, MAX_CATCH_UP};

/// Run control loop at specified period (hardware version)
///
/// This is the hardware-equivalent of `sim::run_control_loop()`.
/// Delegates to `FlightRunner::run()`.
///
/// # Arguments
///
/// * `runner` - FlightRunner with board, time, and transport
/// * `period_us` - Control loop period in microseconds (e.g., 1000 for 1kHz)
///
/// # Returns
///
/// Never returns (`-> !`). The control loop runs forever.
///
/// # Example
///
/// ```ignore
/// let mut runner = board.into_runner();
/// aviate_runtime::flight::run_control_loop(&mut runner, 1000);
/// ```
pub fn run_control_loop<Board, Time, Transport, Watchdog, Cmd>(
    runner: &mut FlightRunner<Board, Time, Transport, Watchdog, Cmd>,
    period_us: u32,
) -> !
where
    Board: BoardStep<Cmd = Cmd>,
    Time: aviate_hal_io::TimeHal,
    Transport: aviate_hal_io::TransportHal<Cmd>,
    Watchdog: aviate_hal_io::WatchdogHal,
    Cmd: Clone + crate::command_ingress::ClassifyCommand,
{
    runner.run(period_us)
}

/// Hardware board info (static metadata)
///
/// Boards provide this to identify themselves for logging and telemetry.
#[derive(Debug, Clone, Copy)]
pub struct HwBoardInfo {
    /// Short board identifier (e.g., "micoair-h743-v2")
    pub name: &'static str,
    /// Human-readable description
    pub description: &'static str,
    /// MCU identifier (e.g., "STM32H743VIH6")
    pub mcu: &'static str,
}

/// Common control loop periods for hardware (microseconds)
pub mod loop_periods {
    /// 1kHz control loop (1000us period) - typical for multirotors
    pub const HZ_1000: u32 = 1_000;

    /// 500Hz control loop (2000us period) - lower CPU load
    pub const HZ_500: u32 = 2_000;

    /// 400Hz control loop (2500us period) - PX4 default
    pub const HZ_400: u32 = 2_500;

    /// 250Hz control loop (4000us period) - fixed-wing typical
    pub const HZ_250: u32 = 4_000;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loop_periods() {
        assert_eq!(loop_periods::HZ_1000, 1_000);
        assert_eq!(loop_periods::HZ_500, 2_000);
        assert_eq!(loop_periods::HZ_400, 2_500);
        assert_eq!(loop_periods::HZ_250, 4_000);
    }

    #[test]
    fn test_hw_board_info() {
        let info = HwBoardInfo {
            name: "test-board",
            description: "Test board",
            mcu: "STM32H743",
        };
        assert_eq!(info.name, "test-board");
    }
}
