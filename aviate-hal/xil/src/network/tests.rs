//! XIL port-allocation tests.

use super::*;

#[test]
fn default_ports_have_one_instance_stride() {
    let net = XilNetConfig::default();
    assert_eq!(net.base_port, 20_000);
    assert_eq!(net.stride, 16);
    assert_eq!(net.port(0, PortSlot::SensorIn), 20_000);
    assert_eq!(net.port(0, PortSlot::ActuatorOut), 20_001);
    assert_eq!(net.port(0, PortSlot::FaultCmd), 20_002);
    assert_eq!(net.port(1, PortSlot::SensorIn), 20_016);
    assert_eq!(net.port(2, PortSlot::SensorIn), 20_032);
}

#[test]
fn port_calculation_saturates() {
    let net = XilNetConfig {
        base_port: 65_000,
        stride: 16,
    };
    assert_eq!(net.port(100, PortSlot::SensorIn), u16::MAX);
}

#[test]
fn instance_configuration_uses_the_selected_range() {
    let zero = XilConfig::for_instance_with_net(0, XilNetConfig::default());
    assert_eq!(zero.sensor_port(), 20_000);
    assert_eq!(zero.actuator_port(), 20_001);
    assert_eq!(zero.fault_cmd_port(), 20_002);
    assert_eq!(zero.simulator_addr(), ([127, 0, 0, 1], 20_001).into());
    assert_eq!(zero.gcs_addr.port(), 14_550);

    let one = XilConfig::for_instance_with_net(1, XilNetConfig::default());
    assert_eq!(one.sensor_port(), 20_016);
    assert_eq!(one.actuator_port(), 20_017);
}

#[test]
fn instance_port_ranges_do_not_overlap() {
    let net = XilNetConfig::default();
    let mut ports = std::collections::HashSet::new();
    for instance in 0..100_u16 {
        for slot in [
            PortSlot::SensorIn,
            PortSlot::ActuatorOut,
            PortSlot::FaultCmd,
        ] {
            assert!(ports.insert(net.port(instance, slot)));
        }
    }
}
