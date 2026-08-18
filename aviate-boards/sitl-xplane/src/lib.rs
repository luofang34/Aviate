//! X-Plane SITL board.
//!
//! The simulator's bridge plugin listens; this board dials it, feeds
//! the received HIL sensor stream into the kernel through the
//! simulator-neutral `SimSensorPacket` seam, and answers each sample
//! with the mixer's actuator command. Airframe selection stays with
//! the application: the board takes the kernel by injection.

mod board;

pub use board::{XPlaneBoard, XPlaneConfig, BOARD_INFO};
