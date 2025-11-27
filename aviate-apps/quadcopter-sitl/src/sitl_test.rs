//! SITL Flight Test Binary
//!
//! This test verifies the SITL command path by:
//! 1. Sending heartbeat/arm/disarm commands to Aviate via MAVLink
//! 2. Sending attitude/thrust commands to control the quadcopter
//! 3. Verifying motor commands are forwarded to Gazebo (via gz-bridge)
//! 4. Recording flight trajectory via FlightLog for analysis
//!
//! Usage:
//!   # Automated test (headless):
//!   ./scripts/run_sitl.sh --test
//!
//!   # Manual test with GUI:
//!   ./scripts/run_sitl.sh
//!
//! The test requires:
//! - Gazebo running with the x500_quadcopter world
//! - gz-bridge running (bridges Gazebo ↔ MAVLink)
//! - Aviate SITL running (flight controller)

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use aviate_mavlink::{
    serialize_mavlink, parse_mavlink, MavMessage,
    Heartbeat, CommandLong, SetAttitudeTarget,
    MavType, MavState, MavModeFlag,
    mav_cmd,
};

use aviate_platform_sitl::{FlightLog, FlightLogConfig};

/// Test configuration
const AVIATE_PORT: u16 = 14560;
const LISTEN_PORT: u16 = 14562;  // For receiving responses (different from bridge)

/// Test result
#[derive(Debug)]
struct TestResult {
    armed: bool,
    received_actuator_response: bool,
    test_duration_ms: u64,
    max_altitude: f64,
}

/// SITL Test Client
struct SitlTestClient {
    socket: UdpSocket,
    seq: u8,
    start_time: Instant,
    flight_log: FlightLog,
}

impl SitlTestClient {
    fn new() -> std::io::Result<Self> {
        let socket = UdpSocket::bind(("0.0.0.0", LISTEN_PORT))?;
        socket.set_nonblocking(true)?;

        // Configure flight log: 50Hz sampling, 1000 samples (20 seconds of data)
        let log_config = FlightLogConfig {
            max_samples: 1000,
            sample_interval_ms: 20,
        };

        Ok(Self {
            socket,
            seq: 0,
            start_time: Instant::now(),
            flight_log: FlightLog::new(log_config),
        })
    }

    fn send_heartbeat(&mut self) {
        let hb = Heartbeat {
            mav_type: MavType::Gcs as u8,
            autopilot: 8,  // MAV_AUTOPILOT_INVALID
            base_mode: 0,
            custom_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 3,
        };
        self.send_message(&MavMessage::Heartbeat(hb));
    }

    fn send_arm_command(&mut self, arm: bool) {
        let cmd = CommandLong {
            target_system: 1,
            target_component: 1,
            command: mav_cmd::COMPONENT_ARM_DISARM,
            confirmation: 0,
            param1: if arm { 1.0 } else { 0.0 },
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
        };
        self.send_message(&MavMessage::CommandLong(cmd));
    }

    fn send_attitude_target(&mut self, thrust: f32) {
        let time_ms = self.start_time.elapsed().as_millis() as u32;
        let target = SetAttitudeTarget {
            time_boot_ms: time_ms,
            target_system: 1,
            target_component: 1,
            type_mask: 0,
            q: [1.0, 0.0, 0.0, 0.0],  // Level attitude (identity quaternion)
            body_roll_rate: 0.0,
            body_pitch_rate: 0.0,
            body_yaw_rate: 0.0,
            thrust,
            thrust_body: [0.0, 0.0, 0.0],
        };
        self.send_message(&MavMessage::SetAttitudeTarget(target));
    }

    fn send_message(&mut self, msg: &MavMessage) {
        let mut buf = [0u8; 300];
        if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
            self.seq = self.seq.wrapping_add(1);
            let _ = self.socket.send_to(&buf[..len], ("127.0.0.1", AVIATE_PORT));
        }
    }

    fn receive_messages(&mut self) -> Vec<MavMessage> {
        let mut messages = Vec::new();
        let mut buf = [0u8; 512];
        let time_ms = self.start_time.elapsed().as_millis() as u32;

        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((len, _)) => {
                    if let Ok((msg, _)) = parse_mavlink(&buf[..len]) {
                        // Record position data to flight log
                        if let MavMessage::LocalPositionNed(pos) = &msg {
                            let position = [pos.x, pos.y, pos.z];
                            let velocity = [pos.vx, pos.vy, pos.vz];
                            self.flight_log.record(time_ms, position, velocity);
                        }
                        messages.push(msg);
                    }
                }
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
        messages
    }

    /// Get the flight log for analysis
    fn flight_log(&self) -> &FlightLog {
        &self.flight_log
    }
}


/// Verify trajectory based on flight log
fn verify_trajectory(log: &FlightLog) -> (bool, &'static str) {
    const MIN_ALTITUDE: f32 = 0.5;  // At least 0.5m altitude required
    const MIN_SAMPLES: usize = 10;  // At least 10 samples required

    println!("\n[Trajectory Verification]");

    let (ok, reason) = log.verify_flight(MIN_ALTITUDE, MIN_SAMPLES);

    let stats = log.analyze();
    println!("  Samples: {}", stats.sample_count);
    println!("  Max altitude: {:.2} m", stats.max_altitude);
    println!("  Required: >= {:.2} m, >= {} samples", MIN_ALTITUDE, MIN_SAMPLES);

    if ok {
        println!("  ✓ Flight trajectory verified!");
    } else {
        println!("  ✗ Flight trajectory verification failed: {}", reason);
    }

    (ok, reason)
}

fn main() {
    println!("SITL Flight Test");
    println!("================");
    println!();
    println!("Prerequisites:");
    println!("  1. Gazebo running: gz sim -r aviate-apps/quadcopter-sitl/worlds/x500_quadcopter.sdf");
    println!("  2. Bridge running: ./target/debug/gz-bridge");
    println!("  3. Aviate running: ./target/debug/aviate-app-quadcopter-sitl");
    println!();

    let mut client = match SitlTestClient::new() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("Failed to create test client: {}", e);
            std::process::exit(1);
        }
    };

    // Run the test but capture the flight log before consuming the result
    match run_flight_test_with_log(&mut client) {
        Ok(result) => {
            println!("\n=== Test Results ===");
            println!("Duration: {} ms", result.test_duration_ms);
            println!("Armed: {}", if result.armed { "YES" } else { "NO (warning)" });
            println!("Actuator response: {}", if result.received_actuator_response { "YES" } else { "NO" });
            println!("Max altitude: {:.2} m", result.max_altitude);

            // Verify trajectory
            let (trajectory_ok, trajectory_status) = verify_trajectory(client.flight_log());
            println!("Trajectory: {}", trajectory_status.to_uppercase());

            if trajectory_ok {
                println!("\n✅ SITL Test Complete");
                std::process::exit(0);
            } else {
                eprintln!("\n❌ SITL Test Failed: trajectory verification failed");
                std::process::exit(1);
            }
        }
        Err(e) => {
            eprintln!("\n❌ Test Failed: {}", e);
            std::process::exit(1);
        }
    }
}

/// Run flight test with access to the client's flight log
fn run_flight_test_with_log(client: &mut SitlTestClient) -> Result<TestResult, String> {
    println!("=== SITL Flight Test ===");
    println!("Connecting to Aviate at 127.0.0.1:{}", AVIATE_PORT);
    println!("Listening for position data on port {}", LISTEN_PORT);

    let start = Instant::now();
    let mut armed = false;
    let mut received_position = false;

    // Phase 1: Send heartbeats and drain any pending messages
    println!("\n[Phase 1] Sending heartbeats...");
    for _ in 0..10 {
        client.send_heartbeat();
        client.receive_messages();
        std::thread::sleep(Duration::from_millis(100));
    }

    // Phase 2: Arm
    println!("\n[Phase 2] Arming...");
    client.send_arm_command(true);
    std::thread::sleep(Duration::from_millis(500));

    // Check for responses
    let msgs = client.receive_messages();
    for msg in &msgs {
        if let MavMessage::Heartbeat(hb) = msg {
            if hb.base_mode & MavModeFlag::SAFETY_ARMED.0 != 0 {
                armed = true;
                println!("  Received armed heartbeat");
            }
        }
    }

    // Phase 3: Takeoff (send thrust commands)
    println!("\n[Phase 3] Taking off (60% thrust for 5 seconds)...");
    let takeoff_duration = Duration::from_secs(5);
    let takeoff_start = Instant::now();
    let mut last_print = Instant::now();

    while takeoff_start.elapsed() < takeoff_duration {
        client.send_attitude_target(0.6);
        let _msgs = client.receive_messages();

        if client.flight_log().len() > 0 {
            received_position = true;
        }

        // Print status every second
        if last_print.elapsed() >= Duration::from_secs(1) {
            let stats = client.flight_log().analyze();
            if stats.sample_count > 0 {
                println!("  Altitude: {:.2} m (max: {:.2} m, {} samples)",
                    stats.final_altitude, stats.max_altitude, stats.sample_count);
            }
            last_print = Instant::now();
        }

        std::thread::sleep(Duration::from_millis(20));  // 50 Hz
    }

    // Print takeoff summary
    let takeoff_stats = client.flight_log().analyze();
    println!("  Takeoff complete: max alt {:.2} m ({} samples)",
        takeoff_stats.max_altitude, takeoff_stats.sample_count);

    // Phase 4: Land (zero thrust)
    println!("\n[Phase 4] Landing (0% thrust)...");
    for _ in 0..10 {
        client.send_attitude_target(0.0);
        client.receive_messages();
        std::thread::sleep(Duration::from_millis(100));
    }

    // Phase 5: Disarm
    println!("\n[Phase 5] Disarming...");
    client.send_arm_command(false);
    std::thread::sleep(Duration::from_millis(500));

    let test_duration = start.elapsed();

    // Print flight log summary
    println!("\n[Flight Summary]");
    client.flight_log().print_summary();

    let final_stats = client.flight_log().analyze();

    Ok(TestResult {
        armed,
        received_actuator_response: received_position,
        test_duration_ms: test_duration.as_millis() as u64,
        max_altitude: final_stats.max_altitude as f64,
    })
}
