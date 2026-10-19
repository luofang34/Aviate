//! The JSBSim X500: the airframe's actuator map for the JSBSim board,
//! shared by the `sitl-jsbsim-x500` binary and its live mission test.

mod actuators;

pub use actuators::x500_controls;
