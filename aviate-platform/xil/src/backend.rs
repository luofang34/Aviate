//! Backend Trait Definitions
//!
//! Defines the interface for kinematics/physics backends (Gazebo, Unity, Chrono, etc.).
//! The xil core does NOT depend on any specific backend implementation.

use crate::world::World;
use std::time::Duration;

/// Simulation timing mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimingMode {
    /// Run as fast as possible (multi-core/GPU accelerated)
    /// No artificial rate limiting - step as fast as hardware allows
    Unlimited,

    /// Cap at real-time (1x speed)
    /// Simulation time matches wall-clock time
    RealTime,

    /// Fixed multiplier of real-time
    /// e.g., 2.0 = 2x faster than real-time
    Scaled(f64),
}

/// Lockstep synchronization mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LockstepMode {
    /// No synchronization - backend runs independently
    Async,

    /// Barrier-based lockstep - backend waits for FC acknowledgment each step
    /// Provides deterministic, reproducible simulation
    Lockstep {
        /// Timeout for FC acknowledgment (microseconds)
        timeout_us: u64,
    },
}

/// Backend configuration
#[derive(Debug, Clone)]
pub struct BackendConfig {
    /// Physics step size
    pub dt: Duration,

    /// Timing mode (unlimited, real-time, or scaled)
    pub timing: TimingMode,

    /// Lockstep synchronization mode
    pub lockstep: LockstepMode,

    /// Number of vehicle instances
    pub num_instances: u8,
}

impl Default for BackendConfig {
    fn default() -> Self {
        Self {
            dt: Duration::from_millis(1), // 1ms = 1kHz physics
            timing: TimingMode::RealTime,
            lockstep: LockstepMode::Lockstep { timeout_us: 50000 },
            num_instances: 1,
        }
    }
}

/// Trait for kinematics/physics backends
///
/// Implementations:
/// - `aviate-backend-gz`: Gazebo Harmonic via shared memory
/// - Future: Unity, Chrono, custom world kernel
pub trait KinematicsBackend: Send {
    /// Backend identifier (e.g., "gazebo", "unity", "chrono")
    fn name(&self) -> &str;

    /// Initialize the backend with configuration
    fn start(&mut self, cfg: &BackendConfig) -> Result<(), BackendError>;

    /// Advance simulation by one step
    ///
    /// In lockstep mode, this blocks until:
    /// 1. Physics step completes
    /// 2. World state is updated
    /// 3. (Optional) FC acknowledges the step
    ///
    /// Returns the actual time advanced (may differ from dt in async mode)
    fn step(&mut self, world: &mut World) -> Result<Duration, BackendError>;

    /// Check if backend is ready for next step (non-blocking)
    fn poll_ready(&self) -> bool;

    /// Get current simulation time
    fn sim_time(&self) -> Duration;

    /// Get current step count
    fn step_count(&self) -> u64;

    /// Shutdown the backend
    fn stop(&mut self) -> Result<(), BackendError>;

    /// Reset to initial state (for test reruns)
    fn reset(&mut self) -> Result<(), BackendError>;
}

/// Backend errors
#[derive(Debug)]
pub enum BackendError {
    /// Backend not initialized
    NotInitialized,
    /// Backend feature not available/supported
    NotSupported(String),
    /// Connection to external simulator failed
    ConnectionFailed(String),
    /// Lockstep timeout - FC didn't acknowledge in time
    LockstepTimeout { step: u64, timeout_us: u64 },
    /// Physics step failed
    StepFailed(String),
    /// Configuration error
    ConfigError(String),
    /// Generic error
    Other(String),
}

impl std::fmt::Display for BackendError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BackendError::NotInitialized => write!(f, "Backend not initialized"),
            BackendError::NotSupported(msg) => write!(f, "Not supported: {}", msg),
            BackendError::ConnectionFailed(msg) => write!(f, "Connection failed: {}", msg),
            BackendError::LockstepTimeout { step, timeout_us } => {
                write!(f, "Lockstep timeout at step {} ({}us)", step, timeout_us)
            }
            BackendError::StepFailed(msg) => write!(f, "Step failed: {}", msg),
            BackendError::ConfigError(msg) => write!(f, "Config error: {}", msg),
            BackendError::Other(msg) => write!(f, "{}", msg),
        }
    }
}

impl std::error::Error for BackendError {}
