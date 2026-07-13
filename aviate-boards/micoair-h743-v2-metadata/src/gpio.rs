/// STM32 GPIO port designator.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GpioPort {
    /// GPIO port A.
    A,
    /// GPIO port B.
    B,
    /// GPIO port C.
    C,
    /// GPIO port D.
    D,
    /// GPIO port E.
    E,
    /// GPIO port F.
    F,
    /// GPIO port G.
    G,
    /// GPIO port H.
    H,
    /// GPIO port I.
    I,
    /// GPIO port J.
    J,
    /// GPIO port K.
    K,
}

impl GpioPort {
    /// Returns the schematic port letter.
    pub const fn as_char(self) -> char {
        match self {
            Self::A => 'A',
            Self::B => 'B',
            Self::C => 'C',
            Self::D => 'D',
            Self::E => 'E',
            Self::F => 'F',
            Self::G => 'G',
            Self::H => 'H',
            Self::I => 'I',
            Self::J => 'J',
            Self::K => 'K',
        }
    }
}

/// Board-level GPIO identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GpioPin {
    /// GPIO port.
    pub port: GpioPort,
    /// Zero-based pin number within the port.
    pub number: u8,
}

impl GpioPin {
    /// Creates a GPIO identity.
    pub const fn new(port: GpioPort, number: u8) -> Self {
        Self { port, number }
    }

    /// Returns the board crate's stable tuple representation.
    pub const fn as_tuple(self) -> (char, u8) {
        (self.port.as_char(), self.number)
    }
}
