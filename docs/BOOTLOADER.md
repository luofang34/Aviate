# Aviate Bootloader

The Aviate bootloader provides secure firmware update capability for STM32H7-based flight controllers. It supports USB DFU (Device Firmware Upgrade) protocol and includes crash recovery features.

## Overview

The bootloader implements a multi-MCU trait architecture, allowing the same bootloader logic to work across different MCU families. Currently, STM32H743 is fully supported.

### Memory Layout (STM32H743)

```
┌─────────────────────────────────────────────────────┐
│ 0x08000000  Bootloader (128KB)                     │
│             - DFU protocol handler                  │
│             - LED status indication                 │
│             - Crash recovery logic                  │
├─────────────────────────────────────────────────────┤
│ 0x08020000  Application (1920KB)                   │
│             - Flight controller firmware            │
│             - May include software-bootloader       │
│               feature for remote updates            │
└─────────────────────────────────────────────────────┘
```

## Features

- **USB DFU Protocol**: Standard DFU for flashing via `dfu-util`
- **Software DFU Entry**: No physical button required (via serial command)
- **Crash Recovery**: Auto-enters DFU after watchdog timeout with crash flag
- **LED Status Codes**: Visual feedback during boot and flashing
- **Secure Confirmation**: 4-digit code required for remote reboot

## LED Status Codes

| Pattern | Meaning |
|---------|---------|
| Blue slow blink (500ms) | Normal operation |
| Red fast blink (100ms) | Awaiting DFU confirmation |
| Green 3x flash | Watchdog initialized |
| Red 5x flash | Watchdog failed (no crash recovery) |
| Solid red | Crash state (watchdog pending) |

## Usage

### Flashing Firmware

```bash
# Auto-detect serial port and flash
cargo xtask flash firmware.bin

# Specify port explicitly
cargo xtask flash firmware.bin /dev/ttyACM0   # Linux
cargo xtask flash firmware.bin COM3           # Windows

# Build and flash in one step
cargo xtask run my-app
```

### Manual DFU Entry via Serial

1. Connect to USB serial at 115200 baud
2. Send: `dfu`
3. Device responds: `CONFIRM:xxxx` (random 4-digit code)
4. Send the code within 5 seconds: `xxxx`
5. Device reboots to DFU bootloader
6. Flash with dfu-util:
   ```bash
   dfu-util -a 0 -s 0x08020000:leave -D firmware.bin
   ```

### ROM DFU (BOOT Button)

For first-time bootloader installation or recovery:

1. Hold BOOT button
2. Power on or press RESET
3. Release BOOT button
4. Device enters ROM DFU mode
5. Flash bootloader:
   ```bash
   dfu-util -a 0 -s 0x08000000:leave -D aviate-bootloader.bin
   ```

## Crash Recovery

The bootloader implements automatic crash recovery:

1. Application sets crash flag in RTC backup register
2. Application enters infinite loop (stops feeding watchdog)
3. After 10 seconds, watchdog resets MCU
4. Bootloader checks crash flag and auto-enters DFU
5. User can flash new firmware without physical access

To test crash recovery:
```bash
# Connect to serial and send:
crash
# Red LED turns on, device resets after 10s and enters DFU
```

## Building the Bootloader

```bash
cd aviate-bootloader
cargo build --release
arm-none-eabi-objcopy -O binary \
  target/thumbv7em-none-eabihf/release/aviate-bootloader aviate-bootloader.bin
```

## Production Builds

To disable software DFU for production (requires physical BOOT button):

```bash
cargo build --release --no-default-features
```

## Supported Boards

| Board | MCU | Status |
|-------|-----|--------|
| MicoAir H743-V2 | STM32H743VIT6 | Fully supported |
| (future) | STM32H7xx | Trait stubs ready |
| (future) | STM32F4xx | Trait stubs ready |

## Architecture

The bootloader uses trait-based abstraction:

```rust
pub trait BootloaderMcu {
    fn init_system();
    fn jump_to_app(app_addr: u32);
    fn enter_dfu_mode();
    fn check_crash_flag() -> bool;
    fn clear_crash_flag();
}
```

Adding support for a new MCU requires implementing this trait.

## Security Considerations

- **DFU Confirmation**: 4-digit random code prevents accidental remote reboots
- **5-second Timeout**: Limits window for confirmation attacks
- **Production Mode**: Can disable software DFU entirely
- **No Network Access**: DFU only via USB, no remote exploitation

## Troubleshooting

### "dfu-util not found"
Install dfu-util:
```bash
# Ubuntu/Debian
sudo apt install dfu-util

# macOS
brew install dfu-util

# Windows
# Download from dfu-util.sourceforge.net
```

### "No USB CDC ACM device found"
- Check USB cable (some cables are charge-only)
- Verify device has firmware with USB CDC support
- Check udev rules on Linux:
  ```bash
  # /etc/udev/rules.d/50-aviate.rules
  SUBSYSTEM=="usb", ATTR{idVendor}=="0483", MODE="0666"
  ```

### "Timeout waiting for DFU device"
- Bootloader may not be installed
- Try ROM DFU with BOOT button
- Check USB connection

### "Wrong confirmation code"
- Code expires after 5 seconds
- Each `dfu` command generates a new code
- Retry the sequence
