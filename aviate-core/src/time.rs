use crate::types::Seconds;

pub const TICK_FREQUENCY_HZ: u64 = 1_000_000;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TimeSource {
    Internal,
    Gps,
    Ptp,
}

impl Default for TimeSource {
    fn default() -> Self {
        Self::Internal
    }
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Timestamp {
    pub ticks: u64,
    pub source: TimeSource,
}

#[derive(Copy, Clone, Debug)]
pub struct TimeDelta {
    pub dt_sec: Seconds,
    pub tick_delta: u64,
}

impl TimeDelta {
    /// Returns the time delta in microseconds
    ///
    /// Used for watchdog timing and deadline checking.
    /// Assumes tick_delta is already in microseconds (TICK_FREQUENCY_HZ = 1_000_000).
    pub fn as_micros(&self) -> u64 {
        self.tick_delta
    }
}
