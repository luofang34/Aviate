//! Transport selection for one MAVLink HIL session.

use std::io;

use crate::messages::{Heartbeat, HilActuatorControls, HilGps, HilSensor, HilStateQuaternion};
use crate::transport::HilTransport;
use crate::transport_tcp::HilTcpTransport;

pub(super) enum Link {
    Udp(HilTransport),
    Tcp(HilTcpTransport),
}

impl Link {
    pub(super) fn poll(&mut self) {
        match self {
            Self::Udp(transport) => transport.poll(),
            Self::Tcp(transport) => transport.poll(),
        }
    }

    pub(super) fn take_sensor(&mut self) -> Option<HilSensor> {
        match self {
            Self::Udp(transport) => transport.take_sensor(),
            Self::Tcp(transport) => transport.take_sensor(),
        }
    }

    pub(super) fn take_gps(&mut self) -> Option<HilGps> {
        match self {
            Self::Udp(transport) => transport.take_gps(),
            Self::Tcp(transport) => transport.take_gps(),
        }
    }

    pub(super) fn take_state_quaternion(&mut self) -> Option<HilStateQuaternion> {
        match self {
            Self::Udp(transport) => transport.take_state_quaternion(),
            Self::Tcp(transport) => transport.take_state_quaternion(),
        }
    }

    pub(super) fn now_us(&self) -> u64 {
        match self {
            Self::Udp(transport) => transport.now_us(),
            Self::Tcp(transport) => transport.now_us(),
        }
    }

    pub(super) fn send_actuator_controls(
        &mut self,
        controls: &HilActuatorControls,
    ) -> io::Result<()> {
        match self {
            Self::Udp(transport) => transport.send_actuator_controls(controls),
            Self::Tcp(transport) => transport.send_actuator_controls(controls),
        }
    }

    pub(super) fn send_heartbeat(&mut self, heartbeat: &Heartbeat) -> io::Result<()> {
        match self {
            Self::Udp(transport) => transport.send_heartbeat(heartbeat),
            Self::Tcp(transport) => transport.send_heartbeat(heartbeat),
        }
    }
}
