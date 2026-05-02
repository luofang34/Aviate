//! `KernelState` — every safety-relevant runtime field the kernel
//! mutates per cycle (LLR-STATE-101..104).
//!
//! Phase 3 collects seven formerly-direct fields on `AviateKernelImpl`
//! into a single sub-struct so the kernel's runtime-state surface has
//! one anchor:
//!
//!   - `init_state`     — lifecycle state machine cursor (spec §17)
//!   - `mode`           — current `ConfigMode` (spec §4)
//!   - `faults`         — fault flag latch (spec §15)
//!   - `control_law`    — active control authority profile (spec §14)
//!   - `checks`         — pre-arm / in-flight / transition gate state
//!   - `actuator_state` — cached actuator commanded state (for transition checks)
//!   - `timing_stats`   — per-cycle timing instrumentation (spec §18)
//!
//! Phase 4 will additionally pull EKF persistent state (quat, pos,
//! vel, biases, p_cov, initialized, quat_fault, last_gyro_body) and
//! sanitizer fallback state (last_good, age, consecutive_fallback)
//! into `KernelState` and flip the `Estimator` / `ActuatorSanitizer`
//! trait surfaces to take `&mut <State>` arguments. After Phase 4,
//! "every safety-relevant persistent state field has exactly one owner
//! (`KernelState`)" becomes a hard, structurally-enforced rule —
//! prerequisite for redundant-channel snapshot replication, voting,
//! and hot-spare takeover (spec §16).
//!
//! ## Borrow destructuring idiom
//!
//! When a function reads and writes multiple `KernelState` fields in
//! the same scope, destructure at function top to avoid
//! `&mut self.state` alias conflicts:
//!
//! ```ignore
//! let KernelState {
//!     ref mut checks,
//!     ref actuator_state,
//!     ..
//! } = self.state;
//! checks.transition.update_from_actuators(actuator_state, mask);
//! ```
//!
//! Reviewers should prefer this idiom over `let x = &mut self.state.x;`
//! chains — the destructure makes the read/write split explicit at the
//! function head and surfaces unintended cross-field aliasing as a
//! compile error rather than a runtime aliasing bug.
//!
//! Phantom-DA note: this module avoids `pub use submodule::*`
//! re-exports — see `aviate-core/src/lib.rs` for the rationale.

use crate::checks::KernelChecks;
use crate::control::{ConfigMode, ControlLawV1};
use crate::fault::FaultFlags;
use crate::kernel_types::{InitState, TimingStats};
use crate::mixer::ActuatorState;

/// Kernel runtime state. Each field's mutation locus is documented at
/// its declaration site.
// COV:EXCL_START(phantom DA: struct-init lines for the `Default` impl
// below have no executable code beyond the struct literal; rustc's
// coverage attribution places phantom DAs on the field declarations
// under grcov, same artifact class documented in
// `aviate-core/src/ekf.rs` and `aviate-core/src/kernel/config.rs`.)
pub struct KernelState {
    /// Init/arm/disarm/fault state machine cursor (spec §17).
    /// Mutated by `kernel_logic.rs::init_step`, `arm`, `disarm`,
    /// `ground_reset`, `handle_critical_fault`.
    pub init_state: InitState,

    /// Current vehicle configuration mode (spec §4). Transitions are
    /// gated by `request_config_mode()` in `kernel_logic.rs`.
    pub mode: ConfigMode,

    /// Latched fault flags (spec §15). Set by `update_sensor_faults`,
    /// `check_critical_faults`, command-validation guards in
    /// `kernel_update.rs`. Cleared by `ground_reset`.
    pub faults: FaultFlags,

    /// Currently-active control authority profile (spec §14).
    /// Transitions through `handle_degradation` and `disarm`.
    pub control_law: ControlLawV1,

    /// Pre-arm / in-flight / transition gate aggregator. The
    /// sub-flag bits are owned by `KernelChecks`; this struct only
    /// holds the aggregator container.
    pub checks: KernelChecks,

    /// Cached commanded-actuator snapshot for transition checks.
    /// Updated each cycle by `kernel_update.rs::update`'s actuator
    /// stage; consumed by `kernel_logic.rs::request_config_mode`'s
    /// transition-gate evaluation.
    pub actuator_state: ActuatorState,

    /// Per-cycle timing instrumentation (spec §18). The cycle-time
    /// counters here are inputs to the persistent-violation
    /// degradation trigger in `kernel_update.rs`.
    pub timing_stats: TimingStats,
}
// COV:EXCL_STOP

impl KernelState {
    /// Construct a fresh kernel state with the given `KernelChecks`.
    /// All other fields take their `Default` values: `PowerOn` init
    /// state, `Hover` mode, no faults, `Primary` control law, default
    /// actuator and timing state.
    pub fn new(checks: KernelChecks) -> Self {
        Self {
            init_state: InitState::PowerOn,
            mode: ConfigMode::Hover,
            faults: FaultFlags::empty(),
            control_law: ControlLawV1::Primary,
            checks,
            actuator_state: ActuatorState::default(),
            timing_stats: TimingStats::default(),
        }
    }
}

impl Default for KernelState {
    fn default() -> Self {
        Self::new(KernelChecks::new())
    }
}
