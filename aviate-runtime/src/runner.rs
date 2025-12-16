//! Generic Flight Runner
//!
//! Provides the core control loop infrastructure shared between SITL and hardware.
//! This module is `#![no_std]` compatible and defines the scheduler, health tracking,
//! and timing contracts.
//!
//! ## Architecture
//!
//! ```text
//! FlightRunner<Board, Time, Transport, Watchdog>
//! │
//! ├── run(period_us) -> !
//! │   ├── poll transport
//! │   ├── bounded catch-up loop (max 3 steps)
//! │   │   └── step(tick_us, now_us, dt_us)
//! │   └── sleep_until_us(next_tick)
//! │
//! └── step(tick_us, now_us, dt_us)
//!     ├── try_recv_command (non-blocking)
//!     ├── update link_ok from timeout
//!     ├── delegate to board_step() for sensor/actuator handling
//!     └── watchdog.kick()
//! ```
//!
//! ## Time Parameters
//!
//! - `tick_us`: Scheduled tick timestamp (deterministic, for telemetry timestamps)
//! - `now_us`: Actual monotonic time (for safety/health: link timeout, sensor staleness)
//! - `dt_us`: Fixed period (for EKF/controller math - determinism)
//!
//! ## DO-178C Compliance
//!
//! - No heap allocations in run() or step()
//! - Bounded catch-up (max 3 steps) prevents spiral on stalls
//! - Single time read per tick execution
//! - Watchdog kicked once per tick (windowed-safe)

use aviate_hal_io::{TimeHal, TransportHal, TransportStatus, WatchdogHal};

/// Link timeout in microseconds (1 second)
///
/// If no valid command received within this window, `link_ok` becomes false
/// and failsafe is triggered.
pub const LINK_TIMEOUT_US: u64 = 1_000_000;

/// Maximum catch-up steps when scheduler falls behind
///
/// If more than MAX_CATCH_UP ticks behind, resync to current time
/// and increment overrun counter.
pub const MAX_CATCH_UP: u8 = 3;

/// Health status for arming gates and failsafe
///
/// Tracks system health independently of transport `connected` status.
/// Failsafe decisions use `link_ok` (command timeout), not transport status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RunnerHealth {
    /// Sensors returning fresh data
    pub sensors_ok: bool,
    /// EKF has converged
    pub ekf_converged: bool,
    /// Valid command received within timeout (for failsafe)
    ///
    /// `link_ok = now_us.wrapping_sub(last_cmd_time_us) < LINK_TIMEOUT_US`
    pub link_ok: bool,
    /// Timestamp of last valid command (microseconds)
    pub last_cmd_time_us: u64,
    /// Count of scheduler overruns (fell behind by >MAX_CATCH_UP ticks)
    pub overrun_count: u32,
}

impl RunnerHealth {
    /// Create new health status
    pub fn new() -> Self {
        Self::default()
    }

    /// Update link_ok based on command timeout (wrapping-safe)
    ///
    /// # Arguments
    ///
    /// * `now_us` - Current time in microseconds
    pub fn update_link_status(&mut self, now_us: u64) {
        self.link_ok = now_us.wrapping_sub(self.last_cmd_time_us) < LINK_TIMEOUT_US;
    }

    /// Record that a valid command was received
    ///
    /// # Arguments
    ///
    /// * `now_us` - Current time in microseconds
    pub fn command_received(&mut self, now_us: u64) {
        self.last_cmd_time_us = now_us;
        self.link_ok = true;
    }
}

/// Trait for board-specific stepping logic
///
/// This trait allows FlightRunner to be generic over different board implementations
/// while keeping the scheduler and timing logic shared.
///
/// ## Contract
///
/// - `board_step()` MUST complete in bounded time
/// - `board_step()` MUST NOT block on I/O
/// - Sensor reads return `Sample<T>` with `fresh` flag (no blocking)
pub trait BoardStep {
    /// Command type for this board
    type Cmd;

    /// Perform one control step
    ///
    /// This is called by `FlightRunner::step()` after transport polling.
    /// The board should:
    /// 1. Read sensors (non-blocking, return cached if not ready)
    /// 2. Update EKF/controller with dt_us
    /// 3. Compute actuator outputs
    /// 4. Write to actuators
    ///
    /// # Arguments
    ///
    /// * `tick_us` - Scheduled tick time (deterministic timestamps)
    /// * `now_us` - Actual time (health/timeout calculations)
    /// * `dt_us` - Fixed period for controller math
    /// * `cmd` - Last command (or default/failsafe)
    /// * `link_ok` - Whether link is healthy (for failsafe decisions)
    fn board_step(
        &mut self,
        tick_us: u64,
        now_us: u64,
        dt_us: u32,
        cmd: &Self::Cmd,
        link_ok: bool,
    );

    /// Check if sensors are returning fresh data
    fn sensors_ok(&self) -> bool;

    /// Check if EKF has converged
    fn ekf_converged(&self) -> bool;
}

/// Generic flight runner for control loop scheduling
///
/// Encapsulates the timing, scheduling, and health tracking logic
/// shared between SITL and hardware environments.
///
/// ## Type Parameters
///
/// - `Board`: Implements `BoardStep` for sensor/actuator handling
/// - `Time`: Implements `TimeHal` for monotonic time and sleep
/// - `Transport`: Implements `TransportHal<Cmd>` for command/telemetry I/O
/// - `Watchdog`: Implements `WatchdogHal` for hardware watchdog
/// - `Cmd`: Command type (e.g., `aviate_link::Command`)
pub struct FlightRunner<Board, Time, Transport, Watchdog, Cmd>
where
    Board: BoardStep<Cmd = Cmd>,
    Time: TimeHal,
    Transport: TransportHal<Cmd>,
    Watchdog: WatchdogHal,
{
    /// Board HAL (sensors, actuators, kernel)
    pub board: Board,
    /// Time source (DWT on hardware, std::time in SITL)
    pub time: Time,
    /// Transport for commands and telemetry
    pub transport: Transport,
    /// Hardware watchdog (kicked once per control tick)
    pub watchdog: Watchdog,
    /// Last command received (or default/failsafe)
    pub last_cmd: Cmd,
    /// Health status for arming gates and failsafe
    pub health: RunnerHealth,
}

impl<Board, Time, Transport, Watchdog, Cmd> FlightRunner<Board, Time, Transport, Watchdog, Cmd>
where
    Board: BoardStep<Cmd = Cmd>,
    Time: TimeHal,
    Transport: TransportHal<Cmd>,
    Watchdog: WatchdogHal,
    Cmd: Clone,
{
    /// Create a new flight runner
    ///
    /// # Arguments
    ///
    /// * `board` - Board HAL with sensors and actuators
    /// * `time` - Time source for scheduling
    /// * `transport` - Transport for commands and telemetry
    /// * `watchdog` - Hardware watchdog (kicked once per tick)
    /// * `default_cmd` - Default/failsafe command when no input received
    pub fn new(
        board: Board,
        time: Time,
        transport: Transport,
        watchdog: Watchdog,
        default_cmd: Cmd,
    ) -> Self {
        Self {
            board,
            time,
            transport,
            watchdog,
            last_cmd: default_cmd,
            health: RunnerHealth::new(),
        }
    }

    /// Execute one control tick
    ///
    /// Called by `run()` for each scheduled tick. This method:
    /// 1. Polls transport for commands (non-blocking)
    /// 2. Updates health status (link_ok, sensors_ok, etc.)
    /// 3. Delegates to board for sensor/actuator handling
    /// 4. Kicks watchdog
    ///
    /// # Arguments
    ///
    /// * `tick_us` - Scheduled tick time (deterministic, for telemetry timestamps)
    /// * `now_us` - Actual monotonic time (for safety/health: link timeout)
    /// * `dt_us` - Fixed period for controller math (determinism)
    pub fn step(&mut self, tick_us: u64, now_us: u64, dt_us: u32) {
        // 1. Poll transport for commands (non-blocking)
        if let Some(cmd) = self.transport.try_recv_command() {
            self.last_cmd = cmd;
            self.health.command_received(now_us);
        }

        // 2. Update link_ok from command timeout (wrapping-safe)
        self.health.update_link_status(now_us);

        // 3. Delegate to board for sensor/actuator handling
        self.board.board_step(
            tick_us,
            now_us,
            dt_us,
            &self.last_cmd,
            self.health.link_ok,
        );

        // 4. Update health from board
        self.health.sensors_ok = self.board.sensors_ok();
        self.health.ekf_converged = self.board.ekf_converged();

        // 5. Kick watchdog (once per tick, windowed-safe)
        self.watchdog.kick();
    }

    /// Run the control loop (never returns)
    ///
    /// Schedules control ticks at fixed intervals using `next_tick` scheduling
    /// (no drift). Implements bounded catch-up: if behind by more than
    /// MAX_CATCH_UP ticks, resyncs to current time.
    ///
    /// # Arguments
    ///
    /// * `period_us` - Control loop period in microseconds (e.g., 1000 for 1kHz)
    ///
    /// # Control Flow
    ///
    /// ```text
    /// loop {
    ///     transport.poll()                    // Service hardware
    ///     while now >= next_tick && steps < MAX_CATCH_UP:
    ///         step(next_tick, now, period_us) // Execute tick
    ///         next_tick += period_us          // Advance schedule
    ///         steps++
    ///     if steps == MAX_CATCH_UP:
    ///         next_tick = now                 // Resync
    ///         overrun_count++
    ///     time.sleep_until_us(next_tick)      // Wait for next tick
    /// }
    /// ```
    pub fn run(&mut self, period_us: u32) -> ! {
        let mut next_tick = self.time.now_us();

        loop {
            // Service transport (RX/interrupts) - may need frequent polling
            self.transport.poll();

            // Bounded catch-up loop: run up to MAX_CATCH_UP steps if behind
            let mut steps: u8 = 0;
            loop {
                let now = self.time.now_us(); // Single time read per tick execution

                // Check if we've reached next_tick (wrapping-safe)
                if (now.wrapping_sub(next_tick) as i64) < 0 {
                    break; // Not yet time for next tick
                }

                // Execute one control tick - pass both tick_us and now_us
                self.step(next_tick, now, period_us);

                // Advance schedule (no drift)
                next_tick = next_tick.wrapping_add(period_us as u64);

                steps += 1;
                if steps >= MAX_CATCH_UP {
                    // Resync to now and record overrun
                    self.health.overrun_count = self.health.overrun_count.saturating_add(1);
                    next_tick = now;
                    break;
                }
            }

            // Sleep until next tick (timer compare + WFI on hardware, not polling)
            self.time.sleep_until_us(next_tick);
        }
    }

    /// Get current transport status
    pub fn transport_status(&self) -> TransportStatus {
        self.transport.status()
    }

    /// Get current health status
    pub fn health(&self) -> &RunnerHealth {
        &self.health
    }

    /// Check if link is healthy (command received within timeout)
    pub fn link_ok(&self) -> bool {
        self.health.link_ok
    }

    /// Get scheduler overrun count
    pub fn overrun_count(&self) -> u32 {
        self.health.overrun_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_runner_health_default() {
        let health = RunnerHealth::default();
        assert!(!health.sensors_ok);
        assert!(!health.ekf_converged);
        assert!(!health.link_ok);
        assert_eq!(health.last_cmd_time_us, 0);
        assert_eq!(health.overrun_count, 0);
    }

    #[test]
    fn test_runner_health_command_received() {
        let mut health = RunnerHealth::new();
        health.command_received(1_000_000);
        assert!(health.link_ok);
        assert_eq!(health.last_cmd_time_us, 1_000_000);
    }

    #[test]
    fn test_runner_health_link_timeout() {
        let mut health = RunnerHealth::new();
        health.command_received(0);

        // Just before timeout
        health.update_link_status(LINK_TIMEOUT_US - 1);
        assert!(health.link_ok);

        // At timeout
        health.update_link_status(LINK_TIMEOUT_US);
        assert!(!health.link_ok);

        // After timeout
        health.update_link_status(LINK_TIMEOUT_US + 1000);
        assert!(!health.link_ok);
    }

    #[test]
    fn test_runner_health_link_timeout_wrapping() {
        let mut health = RunnerHealth::new();

        // Command received near u64::MAX
        health.command_received(u64::MAX - 100);

        // Update just after wrap (within timeout)
        health.update_link_status(100);
        assert!(health.link_ok);

        // Update way after wrap (beyond timeout)
        health.update_link_status(LINK_TIMEOUT_US + 100);
        assert!(!health.link_ok);
    }
}
