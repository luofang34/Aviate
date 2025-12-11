# Aviate Bootloader

Custom USB DFU bootloader for STM32H743-based flight controllers.

## Memory Layout

| Region | Address | Size | Description |
|--------|---------|------|-------------|
| Bootloader | 0x08000000 | 128KB | This bootloader |
| Application | 0x08020000 | 1920KB | User application |

## LED Indicators

### Normal Operation (Application)
| LED | State | Meaning |
|-----|-------|---------|
| Blue | Slow heartbeat (1Hz) | Application running normally |

### Bootloader States
| LED | State | Meaning |
|-----|-------|---------|
| Purple | 3 quick blinks (red+blue, 200ms on/off) | Crash recovery detected, entering DFU mode |
| Green | Solid | DFU mode ready for firmware update |
| Red | Solid (before reset) | Application hang detected (waiting for watchdog) |

### Error States (Application)
| LED | State | Meaning |
|-----|-------|---------|
| Red | Rapid blink | Critical initialization failure (HSI48, watchdog not available) |
| Red | 5 blinks | Watchdog initialization failed (crash recovery unavailable) |

## Boot Sequence

1. **Check RTC backup register** for magic word `0xB007B007`
2. **If magic present**: Clear it, enter DFU mode
3. **If no magic**: Validate app at 0x08020000
4. **If app valid**: Jump to application
5. **If app invalid**: Enter DFU mode

## Usage

### Flashing via DFU

When the bootloader is in DFU mode (solid green LED):

```sh
# Flash application firmware
dfu-util -a 0 -s 0x08020000:leave -D firmware.bin

# List DFU devices
dfu-util -l
```

### Entering DFU Mode

#### Hardware Method (Always works)
1. Hold BOOT button
2. Press and release RESET button
3. Release BOOT button

#### Software Method (Development only)

Enable the `software-bootloader` feature in your app:

```toml
[dependencies]
aviate-board-micoair-h743-v2 = { path = "...", features = ["software-bootloader"] }
```

Then in your code:

```rust
#[cfg(feature = "software-bootloader")]
use aviate_board_micoair_h743_v2::bootloader;

// Reboot to bootloader (e.g., on MAVLink command)
#[cfg(feature = "software-bootloader")]
bootloader::reboot_to_bootloader();
```

## Development Workflow (No BOOT Button)

For rapid development iteration without the BOOT button, use `cargo xtask`:

### Quick Flash (Recommended)

```sh
# Build and flash in one command (auto-detects serial port)
cargo xtask flash firmware.bin

# Or specify port explicitly
cargo xtask flash firmware.bin /dev/ttyACM0   # Linux/macOS
cargo xtask flash firmware.bin COM3           # Windows
```

### Manual Serial Protocol

If your app implements a USB CDC interface with the protected reboot protocol:

1. **Send `dfu`** via serial terminal
2. **Device responds** with `CONFIRM:xxxx` (random 4-digit code)
3. **Send the code** within 5 seconds
4. **Device reboots** to DFU bootloader
5. **Flash** with `dfu-util -a 0 -s 0x08020000:leave -D firmware.bin`

Example terminal session:
```
> help
Commands:
  help  - Show this help
  info  - Board information
  dfu   - Reboot to bootloader

> dfu
CONFIRM:7392

> 7392
Rebooting to bootloader...
```

### Protected Reboot Protocol

The random confirmation code provides protection against:
- Accidental reboots from serial noise
- Replay attacks
- Automated scripts without explicit confirmation

Each `dfu` command generates a new random code. Wrong codes or timeouts
(>5 seconds) cancel the reboot request.

## Feature Flags

### Production Build
```sh
# Must explicitly specify board target
cargo build --release --features micoair-h743-v2
```
- No default features - board selection required
- `forbid(unsafe_code)` enforced in board crate
- Software bootloader entry disabled
- Firmware updates require physical BOOT button

### Development Build
```sh
# Explicit board + software-dfu feature
cargo build --release --features micoair-h743-v2,software-dfu
```
- Software-dfu feature explicitly enabled
- Unsafe code allowed (for RTC register access)
- Software bootloader entry enabled
- Firmware updates via USB without BOOT button

## Building the Bootloader

```sh
cd aviate-bootloader
# Production build (explicit board, no software DFU)
cargo build --release --target thumbv7em-none-eabihf --features micoair-h743-v2
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/aviate-bootloader aviate-bootloader.bin

# Development build (with software DFU)
cargo build --release --target thumbv7em-none-eabihf --features micoair-h743-v2,software-dfu
```

## Flashing the Bootloader

The bootloader must be flashed using ST's ROM bootloader (hardware DFU):

1. Hold BOOT button while pressing RESET
2. Flash at address 0x08000000:
   ```sh
   dfu-util -a 0 -s 0x08000000:leave -D aviate-bootloader.bin
   ```

## Application Requirements

Applications must:

1. **Link at 0x08020000** (see memory.x):
   ```
   FLASH : ORIGIN = 0x08020000, LENGTH = 2048K - 128K
   ```

2. **Have valid vector table**:
   - Stack pointer in RAM range (0x20000000-0x24080000)
   - Reset handler in flash range (0x08020000-0x08200000)

## Compatibility

- **Board**: MicoAir H743-V2
- **MCU**: STM32H743VIT6
- **USB**: USB2 OTG FS on PA11/PA12
- **DFU VID/PID**: 0x0483:0xDF11 (ST DFU)
