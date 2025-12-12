//! MicoAir H743-V2 application library
//!
//! This library exports reusable components for the MicoAir H743-V2 board.
//!
//! ## DO-178C Architecture Example
//!
//! The modules in this library demonstrate the proper layering for flight-critical
//! software according to DO-178C guidelines:
//!
//! - `context`: Application state management (replaces static mut globals)
//! - `tasks`: Task separation (high-DAL control vs low-DAL I/O)
//!
//! See `example_flight_app.rs` for a complete wiring example.

#![no_std]

pub mod context;
pub mod tasks;
