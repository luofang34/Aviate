//! Minimal Flight Log for SITL Testing
//!
//! Provides a circular buffer flight log for trajectory monitoring during SITL tests.
//! The log stores position samples with timestamps, enabling post-flight analysis.
//!
//! ## Features
//!
//! - Fixed-size circular buffer (no allocations after init)
//! - Configurable sample rate and buffer size
//! - Auto-cleanup: oldest samples are overwritten
//! - Binary serialization for efficient storage
//!
//! ## Usage
//!
//! ```rust,ignore
//! use aviate_platform_xil::flight_log::{FlightLog, FlightLogConfig};
//!
//! let mut log = FlightLog::new(FlightLogConfig::default());
//!
//! // Record samples during flight
//! log.record(1000, [0.0, 0.0, -1.0], [0.0, 0.0, 0.5]);
//!
//! // Analyze after flight
//! let stats = log.analyze();
//! println!("Max altitude: {:.2}m", stats.max_altitude);
//! ```

/// Flight log configuration
#[derive(Clone, Debug)]
pub struct FlightLogConfig {
    /// Maximum number of samples to store (circular buffer)
    pub max_samples: usize,
    /// Minimum interval between samples in milliseconds
    pub sample_interval_ms: u32,
}

impl Default for FlightLogConfig {
    fn default() -> Self {
        Self {
            max_samples: 1000,      // ~20 seconds at 50Hz
            sample_interval_ms: 20, // 50 Hz
        }
    }
}

/// A single flight log sample
#[derive(Clone, Copy, Debug, Default)]
pub struct FlightSample {
    /// Timestamp in milliseconds since start
    pub time_ms: u32,
    /// Position in NED frame [x, y, z] (meters)
    /// z is down, so altitude = -z
    pub position: [f32; 3],
    /// Velocity in NED frame [vx, vy, vz] (m/s)
    pub velocity: [f32; 3],
}

impl FlightSample {
    /// Get altitude (positive up, in meters)
    #[inline]
    pub fn altitude(&self) -> f32 {
        -self.position[2]
    }

    /// Get horizontal speed (m/s)
    #[inline]
    pub fn horizontal_speed(&self) -> f32 {
        (self.velocity[0].powi(2) + self.velocity[1].powi(2)).sqrt()
    }

    /// Get vertical speed (positive up, m/s)
    #[inline]
    pub fn vertical_speed(&self) -> f32 {
        -self.velocity[2]
    }
}

/// Flight statistics computed from the log
#[derive(Clone, Debug, Default)]
pub struct FlightStats {
    /// Number of samples recorded
    pub sample_count: usize,
    /// Duration of flight in milliseconds
    pub duration_ms: u32,
    /// Maximum altitude reached (meters)
    pub max_altitude: f32,
    /// Maximum horizontal speed (m/s)
    pub max_horizontal_speed: f32,
    /// Maximum vertical speed (m/s, positive up)
    pub max_vertical_speed: f32,
    /// Final altitude (meters)
    pub final_altitude: f32,
    /// Altitude at peak (highest point)
    pub peak_time_ms: u32,
}

/// Circular buffer flight log
pub struct FlightLog {
    config: FlightLogConfig,
    samples: Vec<FlightSample>,
    write_idx: usize,
    count: usize,
    last_sample_time_ms: u32,
}

impl FlightLog {
    /// Create a new flight log with the given configuration
    pub fn new(config: FlightLogConfig) -> Self {
        let mut samples = Vec::with_capacity(config.max_samples);
        samples.resize(config.max_samples, FlightSample::default());
        Self {
            config,
            samples,
            write_idx: 0,
            count: 0,
            last_sample_time_ms: 0,
        }
    }

    /// Reset the log (clear all samples)
    pub fn reset(&mut self) {
        self.write_idx = 0;
        self.count = 0;
        self.last_sample_time_ms = 0;
    }

    /// Record a new sample
    ///
    /// Returns true if the sample was recorded, false if skipped (rate limiting)
    pub fn record(&mut self, time_ms: u32, position: [f32; 3], velocity: [f32; 3]) -> bool {
        // Rate limiting
        if self.count > 0 {
            let elapsed = time_ms.saturating_sub(self.last_sample_time_ms);
            if elapsed < self.config.sample_interval_ms {
                return false;
            }
        }

        let sample = FlightSample {
            time_ms,
            position,
            velocity,
        };

        self.samples[self.write_idx] = sample;
        self.write_idx = (self.write_idx + 1) % self.config.max_samples;
        if self.count < self.config.max_samples {
            self.count += 1;
        }
        self.last_sample_time_ms = time_ms;

        true
    }

    /// Get the number of samples in the log
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if the log is empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Iterate over all samples in chronological order
    pub fn iter(&self) -> impl Iterator<Item = &FlightSample> {
        let start = if self.count < self.config.max_samples {
            0
        } else {
            self.write_idx
        };

        (0..self.count).map(move |i| {
            let idx = (start + i) % self.config.max_samples;
            &self.samples[idx]
        })
    }

    /// Get the most recent sample
    pub fn last(&self) -> Option<&FlightSample> {
        if self.count == 0 {
            return None;
        }
        let idx = if self.write_idx == 0 {
            self.config.max_samples - 1
        } else {
            self.write_idx - 1
        };
        Some(&self.samples[idx])
    }

    /// Analyze the flight log and compute statistics
    pub fn analyze(&self) -> FlightStats {
        if self.count == 0 {
            return FlightStats::default();
        }

        let mut stats = FlightStats {
            sample_count: self.count,
            ..Default::default()
        };

        let mut first_time_ms = u32::MAX;
        let mut last_time_ms = 0u32;
        let mut max_altitude = f32::MIN;
        let mut peak_time_ms = 0u32;

        for sample in self.iter() {
            first_time_ms = first_time_ms.min(sample.time_ms);
            last_time_ms = last_time_ms.max(sample.time_ms);

            let altitude = sample.altitude();
            if altitude > max_altitude {
                max_altitude = altitude;
                peak_time_ms = sample.time_ms;
            }

            let h_speed = sample.horizontal_speed();
            let v_speed = sample.vertical_speed();

            stats.max_horizontal_speed = stats.max_horizontal_speed.max(h_speed);
            stats.max_vertical_speed = stats.max_vertical_speed.max(v_speed);
        }

        stats.duration_ms = last_time_ms.saturating_sub(first_time_ms);
        stats.max_altitude = max_altitude.max(0.0);
        stats.peak_time_ms = peak_time_ms;

        if let Some(last) = self.last() {
            stats.final_altitude = last.altitude();
        }

        stats
    }

    /// Check if the flight meets success criteria
    ///
    /// Returns (success, reason)
    pub fn verify_flight(&self, min_altitude: f32, min_samples: usize) -> (bool, &'static str) {
        if self.count < min_samples {
            return (false, "insufficient samples");
        }

        let stats = self.analyze();

        if stats.max_altitude < min_altitude {
            return (false, "altitude too low");
        }

        (true, "PASSED")
    }

    /// Export log to binary format (for post-processing)
    pub fn export_binary(&self) -> Vec<u8> {
        let mut data = Vec::with_capacity(4 + self.count * 28);

        // Header: sample count (4 bytes)
        data.extend_from_slice(&(self.count as u32).to_le_bytes());

        // Samples (28 bytes each: 4 + 12 + 12)
        for sample in self.iter() {
            data.extend_from_slice(&sample.time_ms.to_le_bytes());
            for &p in &sample.position {
                data.extend_from_slice(&p.to_le_bytes());
            }
            for &v in &sample.velocity {
                data.extend_from_slice(&v.to_le_bytes());
            }
        }

        data
    }

    /// Print a summary to stderr
    pub fn print_summary(&self) {
        let stats = self.analyze();
        eprintln!("[FlightLog] Summary:");
        eprintln!("  Samples: {}", stats.sample_count);
        eprintln!("  Duration: {:.1}s", stats.duration_ms as f32 / 1000.0);
        eprintln!(
            "  Max altitude: {:.2}m @ {:.1}s",
            stats.max_altitude,
            stats.peak_time_ms as f32 / 1000.0
        );
        eprintln!("  Max h-speed: {:.2} m/s", stats.max_horizontal_speed);
        eprintln!("  Max v-speed: {:.2} m/s", stats.max_vertical_speed);
        eprintln!("  Final altitude: {:.2}m", stats.final_altitude);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flight_log_basic() {
        let mut log = FlightLog::new(FlightLogConfig {
            max_samples: 10,
            sample_interval_ms: 0, // No rate limiting for test
        });

        assert!(log.is_empty());

        // Record some samples
        log.record(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        log.record(100, [0.0, 0.0, -1.0], [0.0, 0.0, -1.0]);
        log.record(200, [0.0, 0.0, -2.0], [0.0, 0.0, -0.5]);
        log.record(300, [0.0, 0.0, -1.5], [0.0, 0.0, 0.5]);

        assert_eq!(log.len(), 4);

        let stats = log.analyze();
        assert_eq!(stats.sample_count, 4);
        assert!((stats.max_altitude - 2.0).abs() < 0.01);
        assert_eq!(stats.peak_time_ms, 200);
    }

    #[test]
    fn test_circular_buffer() {
        let mut log = FlightLog::new(FlightLogConfig {
            max_samples: 3,
            sample_interval_ms: 0,
        });

        log.record(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        log.record(1, [0.0, 0.0, -1.0], [0.0, 0.0, 0.0]);
        log.record(2, [0.0, 0.0, -2.0], [0.0, 0.0, 0.0]);
        log.record(3, [0.0, 0.0, -3.0], [0.0, 0.0, 0.0]); // Overwrites first

        assert_eq!(log.len(), 3);

        // Should have samples 1, 2, 3 (0 was overwritten)
        let samples: Vec<_> = log.iter().collect();
        assert_eq!(samples[0].time_ms, 1);
        assert_eq!(samples[1].time_ms, 2);
        assert_eq!(samples[2].time_ms, 3);
    }

    #[test]
    fn test_rate_limiting() {
        let mut log = FlightLog::new(FlightLogConfig {
            max_samples: 100,
            sample_interval_ms: 20,
        });

        assert!(log.record(0, [0.0; 3], [0.0; 3]));
        assert!(!log.record(10, [0.0; 3], [0.0; 3])); // Too soon
        assert!(log.record(20, [0.0; 3], [0.0; 3])); // OK
        assert!(!log.record(25, [0.0; 3], [0.0; 3])); // Too soon
        assert!(log.record(40, [0.0; 3], [0.0; 3])); // OK

        assert_eq!(log.len(), 3);
    }

    #[test]
    fn test_verify_flight() {
        let mut log = FlightLog::new(FlightLogConfig {
            max_samples: 100,
            sample_interval_ms: 0,
        });

        // Empty log
        let (ok, _) = log.verify_flight(0.5, 10);
        assert!(!ok);

        // Add samples but not enough altitude
        for i in 0..20 {
            log.record(i * 100, [0.0, 0.0, -0.3], [0.0, 0.0, 0.0]);
        }
        let (ok, reason) = log.verify_flight(0.5, 10);
        assert!(!ok);
        assert_eq!(reason, "altitude too low");

        // Reset and add good samples
        log.reset();
        for i in 0..20 {
            log.record(i * 100, [0.0, 0.0, -1.0], [0.0, 0.0, 0.0]);
        }
        let (ok, reason) = log.verify_flight(0.5, 10);
        assert!(ok);
        assert_eq!(reason, "PASSED");
    }
}
