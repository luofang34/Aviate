//! MAVLink HIL transport and simulator-neutral conversion.
//!
//! World data uses NED coordinates. Inertial and magnetic data uses the
//! flight-controller body-FRD frame.

#![forbid(unsafe_code)]
#![forbid(clippy::panic)]
#![forbid(clippy::unwrap_used)]
#![forbid(clippy::expect_used)]

mod backend;
pub mod geodetic;
mod link;
pub mod messages;
mod sensor_fields;
pub mod transport;
pub mod transport_tcp;
pub mod wire;

pub use backend::{
    ActuatorSendReceipt, HilBackend, HilBackendConfig, LOCKSTEP_ACTUATOR_FLAG,
    RESET_ACK_SENSOR_FLAG, RESET_REQUEST_ACTUATOR_FLAG,
};
pub use messages::{
    Heartbeat, HilActuatorControls, HilGps, HilMessage, HilSensor, HilStateQuaternion,
};
pub use sensor_fields::SensorFields;
pub use transport::{HilTransport, HilTransportConfig};
pub use transport_tcp::{HilTcpConfig, HilTcpTransport};
pub use wire::{parse_frame, serialize_frame, MavFrame, ParseError};
