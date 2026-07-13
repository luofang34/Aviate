//! LED Heartbeat Module
//!
//! Visual status indication via RGB LEDs.
//!
//! ## Pin Assignments
//!
//! | LED   | GPIO | Active |
//! |-------|------|--------|
//! | Green | PE2  | Low    |
//! | Red   | PE3  | Low    |
//! | Blue  | PE4  | Low    |
//!
//! ## State Machine
//!
//! | State       | Pattern               | Meaning                        |
//! |-------------|----------------------|--------------------------------|
//! | Boot        | Blue solid           | System initializing            |
//! | Calibrating | Blue fast blink (5Hz)| Sensors calibrating            |
//! | Standby     | Green slow blink (1Hz)| Ready, disarmed               |
//! | Active      | Green solid          | Armed and flying               |
//! | Critical    | Red fast blink (5Hz) | Critical error (land ASAP)     |
//! | Emergency   | Red solid            | Emergency (motors cut)         |
//!
//! ## DO-178C Compliance
//!
//! - Counter-based timing (no floating point)
//! - Bounded execution (O(1) update)
//! - No heap allocation

use stm32h7xx_hal::gpio::{Output, PushPull, PE2, PE3, PE4};

/// LED update rate (calls per second)
const UPDATE_RATE_HZ: u32 = 1000;

/// Fast blink period in ticks (5Hz = 200ms period = 100ms half-period)
const FAST_BLINK_HALF_PERIOD: u32 = UPDATE_RATE_HZ / 10; // 100 ticks

/// Slow blink period in ticks (1Hz = 1000ms period = 500ms half-period)
const SLOW_BLINK_HALF_PERIOD: u32 = UPDATE_RATE_HZ / 2; // 500 ticks

/// LED state for visual feedback
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum LedState {
    /// Boot: Blue solid (system initializing)
    #[default]
    Boot,
    /// Calibrating: Blue fast blink (sensors calibrating)
    Calibrating,
    /// Standby: Green slow blink (ready, disarmed)
    Standby,
    /// Active: Green solid (armed and flying)
    Active,
    /// Critical: Red fast blink (land ASAP)
    Critical,
    /// Emergency: Red solid (motors cut)
    Emergency,
}

/// LED heartbeat controller
///
/// Manages RGB LEDs based on system state.
/// Call `update()` at 1kHz for correct timing.
pub struct LedHeartbeat {
    /// Green LED (PE2, active low)
    green: PE2<Output<PushPull>>,
    /// Red LED (PE3, active low)
    red: PE3<Output<PushPull>>,
    /// Blue LED (PE4, active low)
    blue: PE4<Output<PushPull>>,
    /// Current state
    state: LedState,
    /// Tick counter for blink timing
    tick: u32,
}

impl LedHeartbeat {
    /// Create new LED heartbeat controller
    ///
    /// LEDs start in Boot state (blue solid).
    pub fn new(
        mut green: PE2<Output<PushPull>>,
        mut red: PE3<Output<PushPull>>,
        mut blue: PE4<Output<PushPull>>,
    ) -> Self {
        // Start with blue on (Boot state)
        // Active low: set_low() turns LED ON
        green.set_high(); // OFF
        red.set_high(); // OFF
        blue.set_low(); // ON (Boot)

        Self {
            green,
            red,
            blue,
            state: LedState::Boot,
            tick: 0,
        }
    }

    /// Set LED state
    ///
    /// State change takes effect immediately on next update().
    pub fn set_state(&mut self, state: LedState) {
        if self.state != state {
            self.state = state;
            self.tick = 0; // Reset blink phase on state change
        }
    }

    /// Get current LED state
    pub fn state(&self) -> LedState {
        self.state
    }

    /// Update LED outputs based on current state
    ///
    /// Call at 1kHz for correct timing. Bounded O(1) execution.
    pub fn update(&mut self) {
        // Increment tick counter (wraps at u32::MAX, which is fine)
        self.tick = self.tick.wrapping_add(1);

        // Determine blink phase
        let fast_on = (self.tick / FAST_BLINK_HALF_PERIOD).is_multiple_of(2);
        let slow_on = (self.tick / SLOW_BLINK_HALF_PERIOD).is_multiple_of(2);

        // Set LEDs based on state (active low: set_low = ON)
        match self.state {
            LedState::Boot => {
                // Blue solid
                self.green.set_high(); // OFF
                self.red.set_high(); // OFF
                self.blue.set_low(); // ON
            }
            LedState::Calibrating => {
                // Blue fast blink
                self.green.set_high(); // OFF
                self.red.set_high(); // OFF
                if fast_on {
                    self.blue.set_low(); // ON
                } else {
                    self.blue.set_high(); // OFF
                }
            }
            LedState::Standby => {
                // Green slow blink
                self.red.set_high(); // OFF
                self.blue.set_high(); // OFF
                if slow_on {
                    self.green.set_low(); // ON
                } else {
                    self.green.set_high(); // OFF
                }
            }
            LedState::Active => {
                // Green solid
                self.green.set_low(); // ON
                self.red.set_high(); // OFF
                self.blue.set_high(); // OFF
            }
            LedState::Critical => {
                // Red fast blink
                self.green.set_high(); // OFF
                self.blue.set_high(); // OFF
                if fast_on {
                    self.red.set_low(); // ON
                } else {
                    self.red.set_high(); // OFF
                }
            }
            LedState::Emergency => {
                // Red solid
                self.green.set_high(); // OFF
                self.red.set_low(); // ON
                self.blue.set_high(); // OFF
            }
        }
    }

    /// Turn all LEDs off
    ///
    /// Use for power-down or entering DFU mode.
    pub fn all_off(&mut self) {
        self.green.set_high();
        self.red.set_high();
        self.blue.set_high();
    }

    /// Set RGB directly (for testing)
    ///
    /// Arguments are logical (true = LED on), handles active-low internally.
    #[cfg(test)]
    pub fn set_rgb(&mut self, green: bool, red: bool, blue: bool) {
        if green {
            self.green.set_low();
        } else {
            self.green.set_high();
        }
        if red {
            self.red.set_low();
        } else {
            self.red.set_high();
        }
        if blue {
            self.blue.set_low();
        } else {
            self.blue.set_high();
        }
    }
}
