# imxrt-usbh Examples

This directory contains comprehensive examples demonstrating different aspects of USB host functionality with the `imxrt-usbh` library. Each example is designed to be educational and demonstrate real-world USB host implementation patterns.

NOTE: For simplicity, these education examples do not currently use RTIC, even though underlying `imxrt-usbh` is RTIC based.

## USB Port Configuration (Teensy 4.1 Only)

**Important:** These examples are configured for **Teensy 4.1** which has two USB ports:

- **USB1 (micro USB)** - USB Device port (programming/CDC serial) - **Available for CDC output**
- **USB2 (host pins)** - USB Host port - **Used for connecting USB devices**

The examples use **USB2** for USB host functionality, leaving **USB1 free** for:
- Programming the Teensy
- USB CDC serial output (future feature)
- Simultaneous computer communication

**Current Logging Setup:**

These examples currently use **UART serial output** on pins 0/1 for simplicity:
- Connect a USB-to-serial adapter to pins 0 (RX) and 1 (TX)
- Open serial monitor at 115200 baud
- See real-time initialization messages

**Future:** Examples will be updated to optionally use USB1 CDC for serial output, eliminating the need for an external serial adapter.

## Getting Started Examples

### `01_basic_host_init.rs` - USB PHY Initialization

Simple introduction to USB host hardware initialization:

- **USB PHY Setup**: Initialize USB PHY for host mode
- **Serial Output**: Real-time status messages at 115200 baud (pins 0/1)
- **LED Feedback**: Visual indication of success/failure
- **Error Handling**: Basic error reporting patterns

**Hardware Requirements:**
- Teensy 4.0 or 4.1
- Serial monitor at 115200 baud on pins 0/1

```bash
cargo build --release --example 01_basic_host_init
cargo objcopy --release --example 01_basic_host_init -- -O ihex 01_basic_host_init.hex
teensy_loader_cli --mcu=TEENSY41 -w 01_basic_host_init.hex
# Open serial monitor to see initialization output
```

### `02_device_enumeration.rs` - EHCI Controller Setup

Builds on example 01 by adding EHCI controller:

- **EHCI Controller**: Create Enhanced Host Controller Interface instance
- **Component Overview**: Understand USB host architecture
- **Serial Output**: Detailed initialization steps
- **LED Feedback**: Visual indication of success/failure

**Hardware Requirements:**
- Teensy 4.0 or 4.1
- Serial monitor at 115200 baud on pins 0/1

```bash
cargo build --release --example 02_device_enumeration
cargo objcopy --release --example 02_device_enumeration -- -O ihex 02_device_enumeration.hex
teensy_loader_cli --mcu=TEENSY41 -w 02_device_enumeration.hex
# Open serial monitor to see initialization output
```

## Full Working Examples

### `enumerate_device.rs` - Advanced USB Device Enumeration

Comprehensive USB device enumeration with advanced features:

- **Multi-device tracking**: Manages multiple connected USB devices simultaneously
- **Real-time monitoring**: Device connection/disconnection event handling
- **Device classification**: Automatic device class identification and routing
- **Performance monitoring**: Device enumeration statistics and health monitoring
- **Educational register setup**: Detailed comments explaining hardware configuration

**Key Learning Points:**
- Complete USB 2.0 enumeration sequence
- Device descriptor parsing and validation
- Device class identification (HID, Mass Storage, Audio, Hub)
- Multiple device management patterns
- Performance monitoring and statistics

```bash
cargo build --release --example enumerate_device
cargo objcopy --release --example enumerate_device -- -O ihex enumerate_device.hex
teensy_loader_cli --mcu=TEENSY41 -w enumerate_device.hex
```

### `hid_keyboard.rs` - Complete HID Keyboard Implementation

Full-featured QWERTY keyboard implementation with comprehensive functionality:

- **Boot Protocol**: HID Boot Protocol for maximum compatibility
- **Advanced Key Mapping**: Complete USB keycode to ASCII conversion
- **Modifier Support**: Shift, Ctrl, Alt, and other modifier keys
- **Key Repeat**: Configurable key repeat rates and delays
- **N-Key Rollover**: Support for multiple simultaneous key presses
- **Performance Stats**: Typing statistics and performance monitoring

**Features Demonstrated:**
- HID boot protocol vs report protocol selection
- Real-time key press/release event detection
- Comprehensive modifier key handling (Shift, Ctrl, Alt, GUI)
- Key repeat timing and rate control
- ASCII conversion with international layout considerations
- Multi-device keyboard management

```bash
cargo build --release --example hid_keyboard
cargo objcopy --release --example hid_keyboard -- -O ihex hid_keyboard.hex
teensy_loader_cli --mcu=TEENSY41 -w hid_keyboard.hex
```

### `hid_gamepad.rs` - HID Gamepad Support

Gamepad and joystick support implementation:

- **Gamepad Detection**: Automatic gamepad device identification
- **Button Mapping**: Complete button state tracking
- **Analog Inputs**: Joystick and trigger analog value processing
- **D-pad Support**: Digital directional pad handling
- **Multi-controller**: Support for multiple connected gamepads

**Key Features:**
- HID report descriptor parsing for gamepad devices
- Analog stick dead-zone handling
- Button debouncing and state management
- Real-time input processing

```bash
cargo build --release --example hid_gamepad
cargo objcopy --release --example hid_gamepad -- -O ihex hid_gamepad.hex
teensy_loader_cli --mcu=TEENSY41 -w hid_gamepad.hex
```

### `mass_storage.rs` - Complete Mass Storage Implementation

Full USB Mass Storage Class (MSC) implementation using Bulk-Only Transport:

- **SCSI Command Set**: Complete implementation of essential SCSI commands
- **BOT Protocol**: Full Command Block Wrapper (CBW) and Command Status Wrapper (CSW) handling
- **Block Operations**: High-performance block-level read/write operations
- **Error Recovery**: Comprehensive error handling and recovery mechanisms
- **Multi-LUN Support**: Multiple Logical Unit Number support for complex devices

**SCSI Commands Implemented:**
- `TEST_UNIT_READY`: Device readiness verification
- `INQUIRY`: Device identification and capabilities
- `READ_CAPACITY_10`: Storage capacity and block size detection
- `READ_10` / `WRITE_10`: High-performance block data operations
- `REQUEST_SENSE`: Detailed error condition analysis
- `MODE_SENSE`: Device operating mode information

**Educational Features:**
- Detailed SCSI protocol explanations
- USB bulk transfer optimization techniques
- Error recovery and retry mechanisms
- Performance monitoring and statistics

```bash
cargo build --release --example mass_storage
cargo objcopy --release --example mass_storage -- -O ihex mass_storage.hex
teensy_loader_cli --mcu=TEENSY41 -w mass_storage.hex
```

### `midi_keyboard.rs` - MIDI Device Implementation

Complete USB MIDI device implementation with real-time processing:

- **MIDI Protocol**: Full USB MIDI packet parsing and processing
- **Real-time Processing**: Interrupt-driven MIDI event handling
- **Multi-channel**: Support for all 16 MIDI channels
- **Message Types**: Note On/Off, Control Change, Program Change, Pitch Bend
- **Performance Monitoring**: MIDI statistics and throughput analysis

**MIDI Features:**
- USB MIDI packet format parsing
- MIDI message classification and routing
- Real-time MIDI event processing
- Channel and velocity handling
- MIDI device identification and capabilities

**Educational Content:**
- MIDI protocol explanation with examples
- USB Audio Class device enumeration
- Real-time audio processing patterns
- Low-latency interrupt transfer usage

```bash
cargo build --release --example midi_keyboard
cargo objcopy --release --example midi_keyboard -- -O ihex midi_keyboard.hex
teensy_loader_cli --mcu=TEENSY41 -w midi_keyboard.hex
```

## Educational Design

All examples are designed with education in mind:

### Hardware Register Documentation
Examples include detailed explanations of hardware setup:

```rust
// Initialize USB PHY
let phy_base = 0x400D_9000; // USBPHY1 base address
let ccm_base = 0x400F_C000;  // CCM base address
let _usb_phy = unsafe { UsbPhy::new(phy_base, ccm_base) };

// Initialize USB host controller
let usb1_base = 0x402E_0140; // USB1 base address
let controller = unsafe {
    EhciController::<8, Uninitialized>::new(usb1_base)
        .expect("Failed to create EHCI controller")
};
```

### Clock Configuration Education
Detailed explanations of USB clock setup:

```rust
/// Configure USB clocks for i.MX RT1062 (educational example showing register-level setup)
///
/// This function demonstrates the low-level clock configuration required for USB operation.
/// In a production system, you might use imxrt-hal's clock management instead.
fn configure_usb_clocks() {
    // Enable USB-related clocks in Clock Gating Register 6 (CCGR6)
    // Each CG field controls a specific clock gate (0b00=off, 0b11=always on)
    modify_reg!(ral::ccm, ccm, CCGR6,
        CG0: 0b11,  // usb_ctrl1_clk - USB controller 1 clock
        CG1: 0b11,  // usb_ctrl2_clk - USB controller 2 clock
        CG2: 0b11,  // usb_phy1_clk - USB PHY 1 clock
        CG3: 0b11   // usb_phy2_clk - USB PHY 2 clock
    );
}
```

## Building and Testing

### Prerequisites
- Rust toolchain with `thumbv7em-none-eabihf` target
- Teensy 4.0/4.1 development board
- USB devices for testing (keyboard, flash drive, etc.)
- `cargo-binutils` for creating hex files (install with `cargo install cargo-binutils`)
- `llvm-tools-preview` component (install with `rustup component add llvm-tools-preview`)

### Build Examples

Build examples and create hex files for flashing:

```bash
# Build specific example
cargo build --release --example enumerate_device

# Convert to Intel hex format for flashing
cargo objcopy --release --example enumerate_device -- -O ihex enumerate_device.hex

# Flash to Teensy using teensy_loader_cli
teensy_loader_cli --mcu=TEENSY41 -w enumerate_device.hex

# Or use the Teensy Loader GUI application
```

Build all examples at once:

```bash
cargo build --release --examples
```

### Hardware Setup

#### Teensy 4.x USB Host Setup
```
Teensy 4.x USB Host Connections:
├── USB1_DN  (Pin 30) → USB D-
├── USB1_DP  (Pin 31) → USB D+
├── USB1_ID  (Pin 32) → Ground (host mode)
└── 5V Power → VBUS (through switching circuit)
```

**Important**: Ensure proper VBUS power switching for device power control.

### Tested Devices

Examples have been validated with:

- **HID Keyboards**: Various USB keyboards (mechanical, wireless adapters, gaming)
- **HID Gamepads**: Xbox controllers, PlayStation controllers, generic USB gamepads
- **Mass Storage**: USB flash drives, external hard drives, SD card readers
- **MIDI Devices**: USB MIDI keyboards, controllers, audio interfaces

## Example Architecture Patterns

All examples follow consistent architectural patterns:

### Device Management Structure
```rust
/// Application structure managing USB devices
struct UsbApp {
    usb_controller: EhciController<8, Running>,
    memory_pool: UsbMemoryPool,
    devices: DeviceManager,
    stats: Statistics,
}

impl UsbApp {
    fn new() -> Result<Self> { /* initialization */ }
    fn run(&mut self) -> ! { /* main loop */ }
    fn detect_devices(&mut self) { /* device detection */ }
    fn process_events(&mut self) { /* event processing */ }
}
```

### Error Handling Patterns
```rust
// Comprehensive error handling with recovery
match device.process_data() {
    Ok(events) => self.handle_events(events),
    Err(UsbError::TransactionError) => {
        // Retry with backoff
        self.retry_with_backoff(&mut device)?;
    }
    Err(UsbError::DeviceDisconnected) => {
        // Clean up and remove device
        self.remove_device(device_id);
    }
    Err(e) => return Err(e), // Fatal error
}
```

## Protocol Documentation

### USB Protocol Compliance
Examples demonstrate proper USB 2.0 protocol compliance:

- Correct timing for reset sequences and state transitions
- Proper descriptor parsing with validation
- Standard and class-specific request handling
- Error recovery and retry mechanisms

### Class-Specific Implementations
Each device class example includes:

- Complete protocol state machines
- Proper endpoint configuration and usage
- Class-specific error handling
- Performance optimization techniques

## Troubleshooting

### Common Issues

1. **Device Not Detected**
   - Verify VBUS power supply (5V, sufficient current)
   - Check USB cable quality and connections
   - Ensure device is USB 2.0 compatible

2. **Enumeration Failures**
   - Check reset timing (at least 10ms pulse width)
   - Verify descriptor parsing logic handles edge cases
   - Enable debug logging for detailed protocol trace

3. **Data Transfer Issues**
   - Verify endpoint addresses match device descriptors
   - Check DMA buffer alignment (32-byte boundaries)
   - Validate transfer timing and size limits

### Debug Configuration
```toml
[dependencies]
imxrt-usbh = { version = "0.1", features = ["defmt"] }
defmt-rtt = "0.4"
```

## Performance Considerations

### Optimization Guidelines
- **Memory Management**: Reuse DMA buffers to minimize allocation overhead
- **Transfer Sizing**: Use maximum packet sizes for bulk transfers
- **Polling Intervals**: Optimize interrupt transfer polling for your application needs
- **Cache Coherency**: Ensure proper cache management for DMA operations

### Benchmarking
Examples include performance monitoring:
- Transfer throughput measurement
- Latency tracking for real-time applications
- Error rate monitoring
- Resource utilization statistics

## References

- [USB 2.0 Specification](https://www.usb.org/document-library/usb-20-specification)
- [HID Usage Tables](https://www.usb.org/sites/default/files/documents/hut1_12v2.pdf)
- [Mass Storage Class Specification](https://www.usb.org/sites/default/files/documents/usb_msc_overview_1.2.pdf)
- [SCSI Primary Commands](https://www.t10.org/lists/op-num.htm)
- [USB MIDI Specification](https://www.usb.org/sites/default/files/midi10.pdf)
- [i.MX RT1062 Reference Manual](https://www.nxp.com/docs/en/reference-manual/IMXRT1060RM.pdf)
