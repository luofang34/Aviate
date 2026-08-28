//! Construct the simulator input and output devices.

use std::io;

use aviate_hal_io::{BoardHal, FakeActuator, FakeBaro, FakeGnss, FakeImu, FakeMag};
use aviate_hal_xil::{SitlConfig, SitlIO};
use aviate_runtime::{SitlBoardHal, SitlTime};

pub(super) fn build_simulator_io() -> io::Result<(SitlIO, SitlBoardHal)> {
    let transport = SitlIO::new(SitlConfig::default())?;
    let board = BoardHal::new(
        FakeImu::new(),
        FakeBaro::new(),
        FakeMag::new(),
        FakeGnss::new(),
        SitlTime::new(),
        FakeActuator::new(),
    );
    Ok((transport, board))
}
