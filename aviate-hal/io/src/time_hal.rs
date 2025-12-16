//! Time abstraction for Aviate flight controller
//!
//! Provides monotonic time source and sleep capability, decoupled from transport.
//! Different implementations for different environments:
//! - Hardware: DWT cycle counter + TIM compare for sleep
//! - SITL: std::time::Instant + thread::sleep
//!
//! ## DO-178C Compliance
//!
//! - `now_us()` MUST be monotonic (never goes backwards)
//! - Rollover handling MUST be correct (32-bit counter → 64-bit)
//! - `sleep_until_us()` MUST NOT busy-poll on hardware (use timer compare + WFI)

/// Time source trait for flight controller
///
/// Uses `&mut self` to allow updating internal rollover tracking state
/// without requiring interior mutability (Cell, Atomic, etc.).
///
/// ## Implementation Requirements
///
/// 1. **Monotonic**: `now_us()` MUST never return a value less than a previous call
/// 2. **Rollover-safe**: Handle hardware counter rollover (e.g., 32-bit DWT CYCCNT)
/// 3. **No interior mutability needed**: `&mut self` allows state updates
///
/// ## Sleep Implementation Requirements
///
/// - **SITL**: `std::thread::sleep()` is acceptable (blocking is OK)
/// - **Hardware**: Use timer compare interrupt + WFI (NOT polling loop)
pub trait TimeHal {
    /// Get current time in microseconds (monotonic, handles rollover)
    ///
    /// Takes `&mut self` to allow updating internal rollover tracking state.
    /// Single point of truth for time - only `run()` should call this.
    ///
    /// # Returns
    ///
    /// Monotonic timestamp in microseconds. Handles 32-bit counter rollover
    /// by tracking last value and extending to 64-bit.
    fn now_us(&mut self) -> u64;

    /// Sleep until the specified time (or return immediately if past)
    ///
    /// # Implementation Requirements
    ///
    /// - **SITL**: Use `std::thread::sleep()` for the delta
    /// - **Hardware**: Use timer compare interrupt + WFI (single wake, not polling)
    ///
    /// # Arguments
    ///
    /// * `target_us` - Target wake time in microseconds
    ///
    /// # Behavior
    ///
    /// - If `target_us <= now_us()`, returns immediately
    /// - Otherwise, sleeps until approximately `target_us`
    fn sleep_until_us(&mut self, target_us: u64);
}

/// Sample wrapper for sensor reads - encodes non-blocking contract in types
///
/// This type ensures that sensor reads are non-blocking:
/// - If sensor data is ready, `fresh = true` and `value` contains new data
/// - If sensor is busy, `fresh = false` and `value` contains last cached sample
///
/// ## DO-178C Compliance
///
/// This pattern eliminates blocking sensor reads at the type level.
/// The runner can check `fresh` to determine if data is new and track
/// sensor staleness for health monitoring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample<T> {
    /// The sensor value (new if fresh, cached if not)
    pub value: T,
    /// Whether this is a fresh reading (true) or cached from previous read (false)
    pub fresh: bool,
}

impl<T> Sample<T> {
    /// Create a fresh sample (new data from sensor)
    pub const fn fresh(value: T) -> Self {
        Self { value, fresh: true }
    }

    /// Create a stale sample (cached data, sensor not ready)
    pub const fn stale(value: T) -> Self {
        Self {
            value,
            fresh: false,
        }
    }

    /// Map the value while preserving freshness
    pub fn map<U, F: FnOnce(T) -> U>(self, f: F) -> Sample<U> {
        Sample {
            value: f(self.value),
            fresh: self.fresh,
        }
    }
}

impl<T: Default> Default for Sample<T> {
    fn default() -> Self {
        Self::stale(T::default())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sample_fresh() {
        let s = Sample::fresh(42u32);
        assert!(s.fresh);
        assert_eq!(s.value, 42);
    }

    #[test]
    fn test_sample_stale() {
        let s = Sample::stale(42u32);
        assert!(!s.fresh);
        assert_eq!(s.value, 42);
    }

    #[test]
    fn test_sample_map() {
        let s = Sample::fresh(21u32);
        let s2 = s.map(|v| v * 2);
        assert!(s2.fresh);
        assert_eq!(s2.value, 42);
    }

    #[test]
    fn test_sample_default() {
        let s: Sample<u32> = Sample::default();
        assert!(!s.fresh);
        assert_eq!(s.value, 0);
    }
}
