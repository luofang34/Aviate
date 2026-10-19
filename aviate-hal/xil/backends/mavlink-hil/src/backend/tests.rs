//! MAVLink HIL conversion tests.

use aviate_hal_xil::{SimActuatorCmd, SimGnssFix, SimSensorPacket};

use super::*;

fn sensor(fields_updated: u32) -> HilSensor {
    HilSensor {
        time_usec: 1_000_000,
        xacc: 1.0,
        yacc: 2.0,
        zacc: 3.0,
        xgyro: 4.0,
        ygyro: 5.0,
        zgyro: 6.0,
        xmag: 0.2,
        ymag: 0.3,
        zmag: 0.4,
        abs_pressure: 1_013.25,
        diff_pressure: 12.5,
        pressure_alt: 321.0,
        temperature: 25.0,
        fields_updated,
        id: 0,
    }
}

#[test]
fn sensor_conversion_uses_body_frd_and_si_units() {
    let mut packet = SimSensorPacket::default();
    apply_sensor(&mut packet, sensor(0x0fff));

    assert!(packet.imu.is_some());
    let Some(imu) = packet.imu else {
        return;
    };
    assert_eq!(imu.accel, [1.0, 2.0, 3.0]);
    assert_eq!(imu.gyro, [4.0, 5.0, 6.0]);
    assert!(packet.mag.is_some());
    let Some(mag) = packet.mag else {
        return;
    };
    assert_eq!(mag.field_ut, [20.0, 30.000002, 40.0]);
    assert!(packet.baro.is_some());
    let Some(baro) = packet.baro else {
        return;
    };
    assert_eq!(baro.pressure_pa, 101_325.0);
    assert_eq!(baro.differential_pressure_pa, Some(1_250.0));
    assert_eq!(baro.pressure_altitude_m, Some(321.0));
    assert_eq!(packet.presence_mask, 0x0fff);
}

#[test]
fn partial_vectors_and_pressure_declarations_remain_absent() {
    let mut vector = SimSensorPacket::default();
    apply_sensor(&mut vector, sensor(0b11_1110 | (0b110 << 6)));
    assert!(vector.imu.is_none());
    assert!(vector.mag.is_none());

    let mut pressure = SimSensorPacket::default();
    apply_sensor(&mut pressure, sensor(1 << 10));
    assert!(pressure.baro.is_none());
    assert_eq!(pressure.presence_mask, 1 << 10);
}

#[test]
fn gps_conversion_latches_one_ned_origin() {
    let mut packet = SimSensorPacket::default();
    let mut origin = NedOrigin::default();
    apply_gps(
        &mut packet,
        &mut origin,
        HilGps {
            time_usec: 1_000_000,
            lat: 473_977_420,
            lon: 85_455_940,
            alt: 488_000,
            eph: 100,
            epv: 150,
            vel: 500,
            vn: 100,
            ve: 200,
            vd: -50,
            cog: 9_000,
            fix_type: 3,
            satellites_visible: 12,
            id: 0,
            yaw: 0,
        },
    );

    assert!(packet.gnss.is_some());
    let Some(gnss) = packet.gnss else {
        return;
    };
    assert_eq!(gnss.fix, SimGnssFix::ThreeD);
    assert_eq!(gnss.position_ned, [0.0; 3]);
    assert_eq!(gnss.vel_ned, [1.0, 2.0, -0.5]);
    assert_eq!(gnss.h_acc, 1.0);
    assert_eq!(gnss.v_acc, 1.5);
}

#[test]
fn actuator_message_has_exact_lockstep_evidence() {
    let mut command = SimActuatorCmd::new(99, 4, true);
    command.outputs[..4].copy_from_slice(&[0.1, 0.2, 0.3, 0.4]);
    let message = actuator_message(&command, 42, 7);

    assert_eq!(message.time_usec, 42);
    assert_eq!(message.flags, LOCKSTEP_ACTUATOR_FLAG);
    assert!(message.is_armed());
    assert_eq!(&message.controls[..4], &[0.1, 0.2, 0.3, 0.4]);

    let fallback_clock = actuator_message(&command, 0, 7);
    assert_eq!(fallback_clock.time_usec, 7);
}

#[test]
fn a_truth_paced_reply_echoes_the_truth_time_with_the_lockstep_flag() {
    let command = SimActuatorCmd {
        outputs: [0.25; 16],
        count: 4,
        armed: true,
        ..SimActuatorCmd::default()
    };
    let message = actuator_message(&command, 4_000, 4_000);
    assert_eq!(message.time_usec, 4_000);
    assert_eq!(
        message.flags & LOCKSTEP_ACTUATOR_FLAG,
        LOCKSTEP_ACTUATOR_FLAG
    );
    assert_eq!(message.mode, HilActuatorControls::MODE_FLAG_ARMED);
    assert_eq!(&message.controls[..4], &[0.25; 4]);
    assert_eq!(&message.controls[4..], &[0.0; 12]);
}

#[test]
fn a_udp_link_has_no_session_to_restart() {
    let backend = HilBackend::new(HilBackendConfig {
        local_port: 0,
        simulator_addr: std::net::SocketAddr::from(([127, 0, 0, 1], 9)),
        ..HilBackendConfig::default()
    });
    assert!(backend.is_ok());
    let Ok(mut backend) = backend else {
        return;
    };
    assert_eq!(backend.sessions(), 1);
    let refused = backend.restart_session();
    assert!(refused.is_err_and(|e| e.kind() == std::io::ErrorKind::Unsupported));
}
