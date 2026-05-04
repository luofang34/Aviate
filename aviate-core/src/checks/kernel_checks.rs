//! Aggregate check bundle owned by the kernel.
//!
//! `KernelChecks` holds the three check categories (pre-arm, in-flight,
//! transition) as a single owned value so kernel code can borrow them
//! together without juggling three fields.

use super::in_flight::InFlightStatus;
use super::pre_arm::{PreArmFlags, PreArmStatus};
use super::transition::TransitionStatus;

/// All checks managed by the kernel
#[derive(Clone, Debug, Default)]
pub struct KernelChecks {
    /// Pre-arm checks (InitState transitions)
    pub pre_arm: PreArmStatus,
    /// In-flight checks (continuous monitoring)
    pub in_flight: InFlightStatus,
    /// Transition checks (ConfigMode changes)
    pub transition: TransitionStatus,
}

impl KernelChecks {
    pub fn new() -> Self {
        Self::default()
    }

    /// Create with custom pre-arm requirements
    pub fn with_pre_arm_required(required: PreArmFlags) -> Self {
        Self {
            pre_arm: PreArmStatus::with_required(required),
            ..Default::default()
        }
    }
}

impl crate::replicable::Replicable for KernelChecks {
    const ENCODED_LEN: usize =
        PreArmStatus::ENCODED_LEN + InFlightStatus::ENCODED_LEN + TransitionStatus::ENCODED_LEN;
    fn encode_canonical(&self, buf: &mut [u8]) -> usize {
        let mut written = self.pre_arm.encode_canonical(buf);
        if written < buf.len() {
            written += self.in_flight.encode_canonical(&mut buf[written..]);
        }
        if written < buf.len() {
            written += self.transition.encode_canonical(&mut buf[written..]);
        }
        written
    }
}
