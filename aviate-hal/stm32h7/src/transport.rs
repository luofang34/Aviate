//! STM32H7 Transport Implementations
//!
//! Hardware transport implementations for the STM32H7 family.
//! Supports USB CDC, UART, and CAN transports.
//!
//! ## Architecture
//!
//! All transports implement `TransportHal<Command>` from `aviate-hal-io`:
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │  TransportHal<Command> (trait from aviate-hal-io)          │
//! │  - try_recv_command() -> Option<Command>                   │
//! │  - try_send_telemetry(&[u8]) -> bool                       │
//! │  - set_system_state(SystemState)                           │
//! │  - set_armed(bool)                                         │
//! │  - poll()                                                   │
//! │  - status() -> TransportStatus                             │
//! └─────────────────────────────────────────────────────────────┘
//!                           ↑
//!           implements TransportHal
//!                           ↑
//! ┌─────────────────────────────────────────────────────────────┐
//! │  Stm32h7Transport (enum)                                   │
//! │  ├── Usb(UsbCdcTransport)                                  │
//! │  ├── Uart(UartTransport)                                   │
//! │  ├── Can(CanTransport)                                     │
//! │  └── UsbUart { usb, uart }                                 │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Static Resources
//!
//! USB requires static buffers and device handles. The board init code
//! must allocate these statically and pass references to the transport.
//!
//! ## Non-blocking Contract
//!
//! All methods are non-blocking:
//! - `try_recv_command()` returns `None` if no command available
//! - `try_send_telemetry()` drops on buffer full (returns false)
//! - `poll()` services hardware but never blocks
//!
//! ## Command Type
//!
//! The transport is generic over the command type `C`. For Aviate,
//! this is typically `aviate_link::Command`.

use aviate_hal_io::{SystemState, TransportHal, TransportStatus};

/// MAVLink framing constants
const MAVLINK_MAX_FRAME_LEN: usize = 280;

/// USB CDC transport for STM32H7
///
/// Uses the USB OTG FS/HS peripheral with CDC ACM class.
/// Requires static buffers for USB descriptors and endpoints.
///
/// ## Initialization
///
/// The board init code must:
/// 1. Enable HSI48 or use PLL for 48MHz USB clock
/// 2. Configure USB OTG peripheral
/// 3. Allocate static USB buffers
/// 4. Pass configured USB device to this transport
///
/// ## Buffer Sizes
///
/// - RX buffer: 512 bytes (multiple MAVLink frames)
/// - TX buffer: 512 bytes
/// - Endpoint buffers: 64 bytes (USB FS) or 512 bytes (USB HS)
#[allow(dead_code)] // Stub - fields used when implementation is complete
pub struct UsbCdcTransport {
    /// Receive ring buffer for incoming data
    rx_buf: [u8; 512],
    rx_head: usize,
    rx_tail: usize,

    /// Transmit ring buffer for outgoing data
    tx_buf: [u8; 512],
    tx_head: usize,
    tx_tail: usize,

    /// MAVLink frame accumulator
    frame_buf: [u8; MAVLINK_MAX_FRAME_LEN],
    frame_len: usize,

    /// Status tracking
    rx_errors: u32,
    tx_errors: u32,
    connected: bool,

    /// System state for heartbeat
    system_state: SystemState,
    armed: bool,
}

impl UsbCdcTransport {
    /// Create a new USB CDC transport (stub)
    ///
    /// # TODO
    ///
    /// Full implementation requires USB device stack integration.
    /// This stub allows compilation and testing of the architecture.
    pub fn new() -> Self {
        // COV:EXCL_START(STUB) - Hardware-only stub
        Self {
            rx_buf: [0; 512],
            rx_head: 0,
            rx_tail: 0,
            tx_buf: [0; 512],
            tx_head: 0,
            tx_tail: 0,
            frame_buf: [0; MAVLINK_MAX_FRAME_LEN],
            frame_len: 0,
            rx_errors: 0,
            tx_errors: 0,
            connected: false,
            system_state: SystemState::Uninit,
            armed: false,
        }
        // COV:EXCL_STOP
    }
}

impl Default for UsbCdcTransport {
    fn default() -> Self {
        Self::new()
    }
}

/// UART transport for STM32H7
///
/// Uses USART/UART peripherals with DMA for efficient transfers.
/// Supports baud rates from 9600 to 921600.
///
/// ## Pin Configuration
///
/// The board config specifies which UART and pins to use.
/// This transport is generic over the UART peripheral.
///
/// ## DMA Configuration
///
/// For best performance, use DMA for both TX and RX:
/// - RX: Circular DMA with idle line detection
/// - TX: Normal DMA with completion interrupt
#[allow(dead_code)] // Stub - fields used when implementation is complete
pub struct UartTransport {
    /// Receive ring buffer
    rx_buf: [u8; 512],
    rx_head: usize,
    rx_tail: usize,

    /// Transmit ring buffer
    tx_buf: [u8; 512],
    tx_head: usize,
    tx_tail: usize,

    /// MAVLink frame accumulator
    frame_buf: [u8; MAVLINK_MAX_FRAME_LEN],
    frame_len: usize,

    /// Status tracking
    rx_errors: u32,
    tx_errors: u32,
    connected: bool, // UART: always true (no handshake)

    /// System state for heartbeat
    system_state: SystemState,
    armed: bool,

    /// Baud rate (for status reporting)
    baud_rate: u32,
}

impl UartTransport {
    /// Create a new UART transport (stub)
    ///
    /// # Arguments
    ///
    /// - `baud_rate`: UART baud rate (e.g., 57600, 115200)
    ///
    /// # TODO
    ///
    /// Full implementation requires UART peripheral integration.
    pub fn new(baud_rate: u32) -> Self {
        // COV:EXCL_START(STUB) - Hardware-only stub
        Self {
            rx_buf: [0; 512],
            rx_head: 0,
            rx_tail: 0,
            tx_buf: [0; 512],
            tx_head: 0,
            tx_tail: 0,
            frame_buf: [0; MAVLINK_MAX_FRAME_LEN],
            frame_len: 0,
            rx_errors: 0,
            tx_errors: 0,
            connected: true, // UART has no connection state
            system_state: SystemState::Uninit,
            armed: false,
            baud_rate,
        }
        // COV:EXCL_STOP
    }
}

/// CAN transport for STM32H7
///
/// Uses FDCAN peripherals for DroneCAN/UAVCAN communication.
/// Supports both classic CAN (1 Mbps) and CAN FD (up to 8 Mbps).
///
/// ## Node ID
///
/// CAN transport requires a node ID (1-127). This can be:
/// - Fixed in board config
/// - Dynamically assigned via DNA protocol
#[allow(dead_code)] // Stub - fields used when implementation is complete
pub struct CanTransport {
    /// Status tracking
    rx_errors: u32,
    tx_errors: u32,
    connected: bool, // CAN: true if not bus-off

    /// System state
    system_state: SystemState,
    armed: bool,

    /// CAN node ID (1-127)
    node_id: u8,
}

impl CanTransport {
    /// Create a new CAN transport (stub)
    ///
    /// # Arguments
    ///
    /// - `node_id`: CAN node ID (1-127)
    ///
    /// # TODO
    ///
    /// Full implementation requires FDCAN peripheral integration.
    pub fn new(node_id: u8) -> Self {
        // COV:EXCL_START(STUB) - Hardware-only stub
        Self {
            rx_errors: 0,
            tx_errors: 0,
            connected: false,
            system_state: SystemState::Uninit,
            armed: false,
            node_id,
        }
        // COV:EXCL_STOP
    }
}

/// Runtime-selectable hardware transport
///
/// Board constructs the appropriate variant from compile-time config.
/// This avoids trait objects while supporting multiple transport types.
///
/// ## Variants
///
/// - `Usb`: USB CDC only
/// - `Uart`: UART only (e.g., telemetry radio)
/// - `Can`: CAN only (DroneCAN)
/// - `UsbUart`: USB + UART (most common)
///
/// ## Command Reception
///
/// For multi-transport variants (e.g., `UsbUart`), commands are received
/// from whichever transport has data available (USB checked first).
///
/// ## Telemetry Transmission
///
/// Telemetry is sent to all active transports (broadcast).
///
/// ## Size Note
///
/// This enum is intentionally large (~2.7KB) because we avoid heap allocation
/// in `#![no_std]` embedded code. The buffers are embedded directly in the struct.
/// This is acceptable for flight controllers with sufficient RAM.
#[allow(clippy::large_enum_variant)] // Intentional for no_std - no Box
pub enum Stm32h7Transport {
    /// USB CDC only
    Usb(UsbCdcTransport),
    /// UART only
    Uart(UartTransport),
    /// CAN only (DroneCAN)
    Can(CanTransport),
    /// USB + UART (most common combo)
    UsbUart {
        usb: UsbCdcTransport,
        uart: UartTransport,
    },
}

impl Stm32h7Transport {
    /// Create USB-only transport
    pub fn usb() -> Self {
        Self::Usb(UsbCdcTransport::new())
    }

    /// Create UART-only transport
    pub fn uart(baud_rate: u32) -> Self {
        Self::Uart(UartTransport::new(baud_rate))
    }

    /// Create CAN-only transport
    pub fn can(node_id: u8) -> Self {
        Self::Can(CanTransport::new(node_id))
    }

    /// Create USB + UART transport
    pub fn usb_uart(uart_baud_rate: u32) -> Self {
        Self::UsbUart {
            usb: UsbCdcTransport::new(),
            uart: UartTransport::new(uart_baud_rate),
        }
    }
}

/// Implement TransportHal for the STM32H7 transport enum
///
/// The command type `C` must be parseable from MAVLink frames.
/// For Aviate, this is `aviate_link::Command`.
impl<C> TransportHal<C> for Stm32h7Transport {
    fn try_recv_command(&mut self) -> Option<C> {
        // COV:EXCL_START(STUB) - Hardware-only stub
        // TODO: Parse MAVLink frames from rx_buf and return Command
        // For multi-transport, check USB first, then UART
        match self {
            Self::Usb(_usb) => None,
            Self::Uart(_uart) => None,
            Self::Can(_can) => None,
            Self::UsbUart { usb: _usb, uart: _uart } => None,
        }
        // COV:EXCL_STOP
    }

    fn try_send_telemetry(&mut self, frame: &[u8]) -> bool {
        // COV:EXCL_START(STUB) - Hardware-only stub
        // TODO: Queue frame to tx_buf, trigger DMA if idle
        // For multi-transport, send to all active transports
        let _ = frame;
        match self {
            Self::Usb(_usb) => false,
            Self::Uart(_uart) => false,
            Self::Can(_can) => false,
            Self::UsbUart { usb: _usb, uart: _uart } => false,
        }
        // COV:EXCL_STOP
    }

    fn set_system_state(&mut self, state: SystemState) {
        // COV:EXCL_START(STUB) - Hardware-only stub
        match self {
            Self::Usb(usb) => usb.system_state = state,
            Self::Uart(uart) => uart.system_state = state,
            Self::Can(can) => can.system_state = state,
            Self::UsbUart { usb, uart } => {
                usb.system_state = state;
                uart.system_state = state;
            }
        }
        // COV:EXCL_STOP
    }

    fn set_armed(&mut self, armed: bool) {
        // COV:EXCL_START(STUB) - Hardware-only stub
        match self {
            Self::Usb(usb) => usb.armed = armed,
            Self::Uart(uart) => uart.armed = armed,
            Self::Can(can) => can.armed = armed,
            Self::UsbUart { usb, uart } => {
                usb.armed = armed;
                uart.armed = armed;
            }
        }
        // COV:EXCL_STOP
    }

    fn poll(&mut self) {
        // COV:EXCL_START(STUB) - Hardware-only stub
        // TODO: Service USB device, DMA completions, etc.
        // This must be fast and non-blocking
        match self {
            Self::Usb(_usb) => {
                // Poll USB device
            }
            Self::Uart(_uart) => {
                // Check DMA completion, process received data
            }
            Self::Can(_can) => {
                // Poll CAN RX FIFO
            }
            Self::UsbUart { usb: _usb, uart: _uart } => {
                // Poll both
            }
        }
        // COV:EXCL_STOP
    }

    fn status(&self) -> TransportStatus {
        // COV:EXCL_START(STUB) - Hardware-only stub
        match self {
            Self::Usb(usb) => TransportStatus {
                rx_errors: usb.rx_errors,
                tx_errors: usb.tx_errors,
                connected: usb.connected,
            },
            Self::Uart(uart) => TransportStatus {
                rx_errors: uart.rx_errors,
                tx_errors: uart.tx_errors,
                connected: uart.connected,
            },
            Self::Can(can) => TransportStatus {
                rx_errors: can.rx_errors,
                tx_errors: can.tx_errors,
                connected: can.connected,
            },
            Self::UsbUart { usb, uart } => TransportStatus {
                rx_errors: usb.rx_errors + uart.rx_errors,
                tx_errors: usb.tx_errors + uart.tx_errors,
                // Connected if either is connected
                connected: usb.connected || uart.connected,
            },
        }
        // COV:EXCL_STOP
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Helper type alias for tests - use () as command type
    type TestTransport = Stm32h7Transport;

    #[test]
    fn test_usb_transport_creation() {
        let transport = TestTransport::usb();
        let status = <TestTransport as TransportHal<()>>::status(&transport);
        assert!(!status.connected);
        assert_eq!(status.rx_errors, 0);
        assert_eq!(status.tx_errors, 0);
    }

    #[test]
    fn test_uart_transport_creation() {
        let transport = TestTransport::uart(115200);
        let status = <TestTransport as TransportHal<()>>::status(&transport);
        // UART is always "connected" (no handshake)
        assert!(status.connected);
    }

    #[test]
    fn test_can_transport_creation() {
        let transport = TestTransport::can(42);
        let status = <TestTransport as TransportHal<()>>::status(&transport);
        assert!(!status.connected);
    }

    #[test]
    fn test_usb_uart_transport_creation() {
        let transport = TestTransport::usb_uart(57600);
        let status = <TestTransport as TransportHal<()>>::status(&transport);
        // UART is connected, USB is not
        assert!(status.connected);
    }

    #[test]
    fn test_set_system_state() {
        let mut transport = TestTransport::usb();
        <TestTransport as TransportHal<()>>::set_system_state(&mut transport, SystemState::Active);

        if let Stm32h7Transport::Usb(usb) = &transport {
            assert_eq!(usb.system_state, SystemState::Active);
        }
    }

    #[test]
    fn test_set_armed() {
        let mut transport = TestTransport::uart(115200);
        <TestTransport as TransportHal<()>>::set_armed(&mut transport, true);

        if let Stm32h7Transport::Uart(uart) = &transport {
            assert!(uart.armed);
        }
    }

    #[test]
    fn test_poll_does_not_panic() {
        let mut transport = TestTransport::usb_uart(57600);
        <TestTransport as TransportHal<()>>::poll(&mut transport); // Should not panic
    }

    #[test]
    fn test_try_recv_returns_none() {
        let mut transport = TestTransport::usb();
        let cmd: Option<()> = <TestTransport as TransportHal<()>>::try_recv_command(&mut transport);
        assert!(cmd.is_none());
    }

    #[test]
    fn test_try_send_returns_false() {
        let mut transport = TestTransport::uart(115200);
        let result = <TestTransport as TransportHal<()>>::try_send_telemetry(&mut transport, &[0xFE, 0x09]);
        assert!(!result);
    }
}
