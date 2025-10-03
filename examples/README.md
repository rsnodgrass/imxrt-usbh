# imxrt-usbh Examples

## Getting Started (Start Here!)

Start with these examples if you're new to this library. They show basic USB host initialization.

### Example 01: USB PHY Initialization

The simplest example - just turn on the USB hardware.

**What it does**: Initializes the USB PHY (Physical Layer) hardware.

**Build and flash:**
```bash
cargo build --release --example 01_basic_host_init --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/01_basic_host_init target/thumbv7em-none-eabihf/release/examples/01_basic_host_init.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/01_basic_host_init.hex
```

**Success indicator**: LED blinks slowly (fast blink = error)

### Example 02: EHCI Controller Setup

Builds on Example 01 by adding the USB host controller.

**What it does**: Creates an EHCI controller instance on top of the initialized PHY.

**Build and flash:**
```bash
cargo build --release --example 02_device_enumeration --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/02_device_enumeration target/thumbv7em-none-eabihf/release/examples/02_device_enumeration.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/02_device_enumeration.hex
```

**Success indicator**: LED blinks slowly (fast blink = error)

### Optional: Serial Output

Want to see detailed initialization messages?

1. **Connect**: USB-to-serial adapter TX→Pin 0, RX→Pin 1, GND→GND
2. **Open**: Serial monitor at 115200 baud
3. **Reset**: Press Teensy reset button to see output

**Why optional?** The LED tells you success/failure, so serial is only needed for debugging.

---

## Hardware Setup

These examples use:
- **USB2** (5-pin header, pins 30-32) for USB Host functionality
- **USB1** (micro USB port) for programming the Teensy

---

## Advanced Examples

These examples demonstrate full device functionality and are more complex:

### `05_multi_device_manager.rs` - Multi-Device Management

Complete USB device enumeration with multi-device tracking:
- Real-time device connection/disconnection monitoring
- Device class identification (HID, Mass Storage, Audio, Hub)
- String descriptor retrieval (vendor, product, serial)
- Performance statistics and health monitoring

```bash
cargo build --release --example 05_multi_device_manager --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/05_multi_device_manager target/thumbv7em-none-eabihf/release/examples/05_multi_device_manager.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/05_multi_device_manager.hex
```

### `03_qwerty_keyboard.rs` - USB Keyboard Support

HID keyboard implementation showing real keypresses:
- Boot protocol initialization
- Interrupt transfer monitoring
- Real-time keypress detection
- Activity display via LED and logging

```bash
cargo build --release --example 03_qwerty_keyboard --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard.hex
```

### `hid_gamepad.rs` - Game Controller Support

Gamepad and joystick support showing real input:
- HID gamepad enumeration
- Interrupt transfer for input data
- Button press detection
- Analog stick movement tracking

```bash
cargo build --release --example hid_gamepad --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/hid_gamepad target/thumbv7em-none-eabihf/release/examples/hid_gamepad.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/hid_gamepad.hex
```

### `mass_storage.rs` - USB Flash Drive Support

USB Mass Storage Class implementation (SCSI/BOT protocol):
- Mass storage device enumeration
- SCSI INQUIRY and READ_CAPACITY commands
- Device information display (capacity, vendor, product)
- Bulk transfer setup

```bash
cargo build --release --example mass_storage --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/mass_storage target/thumbv7em-none-eabihf/release/examples/mass_storage.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/mass_storage.hex
```

### `04_midi_keyboard.rs` - MIDI Device Support

USB MIDI device implementation showing real MIDI events:
- USB MIDI packet parsing
- 16-channel support
- Note On/Off, Control Change, Program Change, Pitch Bend
- Real-time event processing and display

```bash
cargo build --release --example 04_midi_keyboard --target thumbv7em-none-eabihf
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/04_midi_keyboard target/thumbv7em-none-eabihf/release/examples/04_midi_keyboard.hex
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/04_midi_keyboard.hex
```

---

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
- Rust toolchain with `thumbv7em-none-eabihf` target (install with `rustup target add thumbv7em-none-eabihf`)
- Teensy 4.0/4.1 development board
- USB devices for testing (keyboard, flash drive, etc.)
- `llvm-tools-preview` component (install with `rustup component add llvm-tools-preview`)
- Teensy Loader (download from [PJRC](https://www.pjrc.com/teensy/loader.html) or install CLI: `brew install teensy_loader_cli`)

### Build Examples

Build examples and create hex files for flashing:

```bash
# Build specific example
cargo build --release --example 03_qwerty_keyboard --target thumbv7em-none-eabihf

# Convert to Intel hex format for flashing
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard.hex

# Flash to Teensy using teensy_loader_cli
teensy_loader_cli --mcu=TEENSY41 -w target/thumbv7em-none-eabihf/release/examples/03_qwerty_keyboard.hex

# Or use the Teensy Loader GUI application
```

Build all examples at once:

```bash
cargo build --release --examples --target thumbv7em-none-eabihf
```

### Hardware Setup

#### Teensy 4.x USB Host Setup
```
Teensy 4.x USB Host Connections:
├── USB2 (5-pin header, pins 30-32) → USB Host devices
│   ├── USB2_DN  (Pin 30) → USB D-
│   ├── USB2_DP  (Pin 31) → USB D+
│   ├── USB2_ID  (Pin 32) → Ground (host mode)
│   └── 5V Power → VBUS (through switching circuit)
└── USB1 (micro USB) → Programming/Debugging
```

**Important**:
- USB2 (pins 30-32) is used for USB Host functionality
- USB1 (micro USB port) is used for programming and serial debugging
- Ensure proper VBUS power switching for device power control

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
