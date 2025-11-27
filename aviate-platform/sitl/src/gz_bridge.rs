//! Gazebo Transport Bridge for SITL
//!
//! This module bridges Gazebo Sim sensor topics to MAVLink HIL protocol.
//! It subscribes to Gazebo IMU and odometry topics, converts them to
//! HIL_SENSOR/HIL_GPS MAVLink messages, and sends them via UDP.
//!
//! This module is only compiled when the `gz-bridge` feature is enabled.

#[cfg(feature = "gz-bridge")]
mod inner {
    use std::net::UdpSocket;
    use std::sync::{Arc, Mutex};
    use std::time::Instant;

    use gz::transport::Node;
    use gz_msgs::imu::IMU;
    use gz_msgs::odometry::Odometry;
    use gz_msgs::actuators::Actuators;

    use aviate_mavlink::{
        serialize_mavlink, HilSensor, HilGps, HilActuatorControls,
        MavMessage, parse_mavlink,
    };

    /// Gazebo bridge configuration
    #[derive(Clone, Debug)]
    pub struct GzBridgeConfig {
        /// IMU topic name in Gazebo
        pub imu_topic: String,
        /// Odometry topic name in Gazebo
        pub odom_topic: String,
        /// Motor command topic in Gazebo
        pub motor_topic: String,
        /// UDP port to send HIL data to Aviate
        pub aviate_port: u16,
        /// UDP port to receive actuator commands from Aviate
        pub actuator_port: u16,
    }

    impl Default for GzBridgeConfig {
        fn default() -> Self {
            Self {
                imu_topic: "/X3/imu".to_string(),
                odom_topic: "/model/X3/odometry".to_string(),
                motor_topic: "/X3/gazebo/command/motor_speed".to_string(),
                aviate_port: 14560,
                actuator_port: 14561,
            }
        }
    }

    /// Shared sensor state between callbacks and main loop
    #[derive(Default)]
    struct SensorState {
        // IMU data
        accel: [f32; 3],
        gyro: [f32; 3],
        // Odometry data
        position: [f32; 3],
        velocity: [f32; 3],
        orientation: [f32; 4], // quaternion [w, x, y, z]
        // Timestamps
        last_imu_us: u64,
        last_odom_us: u64,
    }

    /// Gazebo-MAVLink bridge
    pub struct GzBridge {
        config: GzBridgeConfig,
        node: Node,
        state: Arc<Mutex<SensorState>>,
        start_time: Instant,
        send_socket: UdpSocket,
        recv_socket: UdpSocket,
        seq: u8,
        // Statistics
        imu_count: u64,
        hil_sent: u64,
        motor_recv: u64,
    }

    impl GzBridge {
        /// Create a new Gazebo bridge
        pub fn new(config: GzBridgeConfig) -> std::io::Result<Self> {
            let node = Node::new().ok_or_else(|| {
                std::io::Error::new(std::io::ErrorKind::Other, "Failed to create Gazebo node")
            })?;

            let send_socket = UdpSocket::bind("0.0.0.0:0")?;
            send_socket.set_nonblocking(true)?;

            let recv_socket = UdpSocket::bind(("0.0.0.0", config.actuator_port))?;
            recv_socket.set_nonblocking(true)?;

            Ok(Self {
                config,
                node,
                state: Arc::new(Mutex::new(SensorState::default())),
                start_time: Instant::now(),
                send_socket,
                recv_socket,
                seq: 0,
                imu_count: 0,
                hil_sent: 0,
                motor_recv: 0,
            })
        }

        /// Subscribe to Gazebo topics
        pub fn subscribe(&mut self) -> Result<(), String> {
            let state = Arc::clone(&self.state);
            let start = self.start_time;

            // Subscribe to IMU topic
            let imu_ok = self.node.subscribe(&self.config.imu_topic, move |msg: IMU| {
                let now_us = start.elapsed().as_micros() as u64;
                if let Ok(mut s) = state.lock() {
                    // Extract angular velocity (MessageField)
                    if let Some(av) = msg.angular_velocity.as_ref() {
                        s.gyro = [av.x as f32, av.y as f32, av.z as f32];
                    }
                    // Extract linear acceleration
                    if let Some(la) = msg.linear_acceleration.as_ref() {
                        s.accel = [la.x as f32, la.y as f32, la.z as f32];
                    }
                    s.last_imu_us = now_us;
                }
            });
            if !imu_ok {
                return Err("Failed to subscribe to IMU topic".to_string());
            }

            let state = Arc::clone(&self.state);
            let start = self.start_time;

            // Subscribe to odometry topic
            let odom_ok = self.node.subscribe(&self.config.odom_topic, move |msg: Odometry| {
                let now_us = start.elapsed().as_micros() as u64;
                if let Ok(mut s) = state.lock() {
                    // Extract pose (MessageField has as_ref() method)
                    if let Some(pose) = msg.pose.as_ref() {
                        if let Some(pos) = pose.position.as_ref() {
                            s.position = [pos.x as f32, pos.y as f32, pos.z as f32];
                        }
                        if let Some(orient) = pose.orientation.as_ref() {
                            s.orientation = [
                                orient.w as f32,
                                orient.x as f32,
                                orient.y as f32,
                                orient.z as f32,
                            ];
                        }
                    }
                    // Extract twist (velocity)
                    if let Some(twist) = msg.twist.as_ref() {
                        if let Some(lin) = twist.linear.as_ref() {
                            s.velocity = [lin.x as f32, lin.y as f32, lin.z as f32];
                        }
                    }
                    s.last_odom_us = now_us;
                }
            });
            if !odom_ok {
                return Err("Failed to subscribe to odometry topic".to_string());
            }

            Ok(())
        }

        /// Run one iteration of the bridge loop
        pub fn step(&mut self) {
            let now_us = self.start_time.elapsed().as_micros() as u64;

            // Read current sensor state
            let (accel, gyro, position, velocity) = {
                let s = self.state.lock().unwrap();
                (s.accel, s.gyro, s.position, s.velocity)
            };

            // Build and send HIL_SENSOR
            let hil_sensor = HilSensor {
                time_usec: now_us,
                xacc: accel[0],
                yacc: accel[1],
                zacc: accel[2],
                xgyro: gyro[0],
                ygyro: gyro[1],
                zgyro: gyro[2],
                xmag: 0.2,  // Simulated magnetometer
                ymag: 0.0,
                zmag: 0.4,
                abs_pressure: 1013.25,  // Sea level pressure (mbar)
                diff_pressure: 0.0,
                pressure_alt: -position[2],  // NED z is down
                temperature: 25.0,
                fields_updated: 0x1FFF,
                id: 0,
            };

            self.send_mavlink(&MavMessage::HilSensor(hil_sensor));
            self.hil_sent += 1;

            // Print stats every second (250 iterations at 250 Hz)
            if self.hil_sent % 250 == 0 {
                let s = self.state.lock().unwrap();
                eprintln!("[DEBUG] hil={}, motors={}, imu={}us, accel=[{:.2},{:.2},{:.2}]",
                    self.hil_sent, self.motor_recv, s.last_imu_us,
                    s.accel[0], s.accel[1], s.accel[2]);
            }

            // Send HIL_GPS at lower rate (every 25th iteration ≈ 10Hz at 250Hz loop)
            if self.seq % 25 == 0 {
                // Convert local position to fake GPS coordinates
                let lat = (position[1] / 111000.0 * 1e7) as i32;
                let lon = (position[0] / 111000.0 * 1e7) as i32;
                let alt = (-position[2] * 1000.0) as i32;

                let hil_gps = HilGps {
                    time_usec: now_us,
                    lat,
                    lon,
                    alt,
                    eph: 100,
                    epv: 100,
                    vel: ((velocity[0].powi(2) + velocity[1].powi(2)).sqrt() * 100.0) as u16,
                    vn: (velocity[1] * 100.0) as i16,
                    ve: (velocity[0] * 100.0) as i16,
                    vd: (-velocity[2] * 100.0) as i16,
                    cog: 0,
                    fix_type: 3,
                    satellites_visible: 10,
                    id: 0,
                    yaw: 0,
                };

                self.send_mavlink(&MavMessage::HilGps(hil_gps));
            }

            // Receive actuator commands from Aviate
            self.receive_actuators();
        }

        /// Send a MAVLink message to Aviate
        fn send_mavlink(&mut self, msg: &MavMessage) {
            let mut buf = [0u8; 300];
            if let Some(len) = serialize_mavlink(msg, self.seq, &mut buf) {
                self.seq = self.seq.wrapping_add(1);
                let addr = ("127.0.0.1", self.config.aviate_port);
                let _ = self.send_socket.send_to(&buf[..len], addr);
            }
        }

        /// Receive and process actuator commands
        fn receive_actuators(&mut self) {
            let mut buf = [0u8; 512];

            loop {
                match self.recv_socket.recv_from(&mut buf) {
                    Ok((len, _)) => {
                        if let Ok((msg, _)) = parse_mavlink(&buf[..len]) {
                            if let MavMessage::HilActuatorControls(ctrl) = msg {
                                self.motor_recv += 1;
                                self.send_motor_command(&ctrl);
                            }
                        }
                    }
                    Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }

        /// Send motor velocity command to Gazebo
        fn send_motor_command(&mut self, ctrl: &HilActuatorControls) {
            // Convert normalized thrust (0-1) to motor velocity (0-800 rad/s)
            let velocities: Vec<f64> = ctrl.controls[..4]
                .iter()
                .map(|&c| (c.max(0.0) * 800.0) as f64)
                .collect();

            // Debug: print motor commands occasionally
            if self.motor_recv % 500 == 1 {
                eprintln!("[DEBUG] Motor cmd: [{:.1},{:.1},{:.1},{:.1}] rad/s",
                    velocities[0], velocities[1], velocities[2], velocities[3]);
            }

            // Create Actuators message
            let mut actuators = Actuators::default();
            actuators.velocity = velocities;

            // Publish to Gazebo (advertise returns Option, not Result)
            if let Some(mut publisher) = self.node.advertise::<Actuators>(&self.config.motor_topic) {
                let _ = publisher.publish(&actuators);
            }
        }

        /// Get timestamp in microseconds
        pub fn now_us(&self) -> u64 {
            self.start_time.elapsed().as_micros() as u64
        }
    }
}

#[cfg(feature = "gz-bridge")]
pub use inner::*;

// Stub when gz-bridge feature is not enabled
#[cfg(not(feature = "gz-bridge"))]
pub struct GzBridgeConfig;

#[cfg(not(feature = "gz-bridge"))]
impl Default for GzBridgeConfig {
    fn default() -> Self {
        Self
    }
}
