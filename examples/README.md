# imxrt-usbh Examples

This directory contains comprehensive examples demonstrating different aspects of USB host functionality with the `imxrt-usbh` library.

## Core USB Host Examples

### `enumerate_device.rs` - Complete USB Device Enumeration

Demonstrates the complete USB device enumeration sequence from port reset to configuration:

- Port reset and device detection
- Standard descriptor requests (GET_DESCRIPTOR, SET_ADDRESS) 
- Configuration setup and device initialization
- String descriptor parsing for device identification
- Error handling throughout the enumeration process

**Key Learning Points:**
- USB 2.0 enumeration sequence compliance
- Control transfer implementation
- Descriptor parsing and validation
- Device address assignment

```bash
# Build and run (requires hardware)
cargo build --example enumerate_device --target thumbv7em-none-eabihf
```

## Device Class Reference Implementations

These examples serve as reference implementations for building separate class driver crates. They demonstrate how to build on top of the core `imxrt-usbh` functionality.

### `example_hid.rs` - HID (Human Interface Device) Support

Complete HID implementation focusing on keyboard support:

- **Boot Protocol**: Keyboard boot protocol implementation for maximum compatibility
- **Report Processing**: HID report descriptor parsing and key event detection
- **Multi-Device Support**: HidManager for handling multiple concurrent HID devices
- **Interrupt Transfers**: Demonstrates proper interrupt transfer usage for periodic data
- **Key Mapping**: USB keycode to ASCII conversion with modifier key support

**Features Demonstrated:**
- HID descriptor parsing and validation
- Keyboard boot protocol vs report protocol
- Key press/release event detection
- Modifier key handling (Shift, Ctrl, Alt, etc.)
- Multiple keyboard management

**Usage Example:**
```rust
use imxrt_usbh::examples::example_hid::{HidManager, HidKeyboard};

let mut hid_manager = HidManager::new();
let keyboard = HidKeyboard::new(device_addr, interface, endpoint, poll_interval);
let keyboard_id = hid_manager.add_keyboard(keyboard)?;

// Poll for events
loop {
    if let Some(events) = hid_manager.poll_keyboard(keyboard_id)? {
        for event in events {
            match event {
                KeyEvent::KeyPress(key) => println!("Key pressed: {}", key),
                KeyEvent::KeyRelease(key) => println!("Key released: {}", key),
            }
        }
    }
}
```

### `example_msc.rs` - MSC (Mass Storage Class) Support

Complete mass storage implementation using Bulk-Only Transport (BOT) protocol:

- **SCSI Commands**: Full implementation of essential SCSI commands (INQUIRY, READ_CAPACITY, READ_10, WRITE_10)
- **BOT Protocol**: Command Block Wrapper (CBW) and Command Status Wrapper (CSW) handling
- **Device Management**: Mass storage device enumeration, initialization, and management
- **Block Operations**: Block-level read/write operations with proper error handling
- **Bulk Transfers**: Demonstrates high-performance bulk transfer usage

**SCSI Commands Implemented:**
- `TEST_UNIT_READY`: Check device readiness
- `INQUIRY`: Get device information (vendor, product, version)
- `READ_CAPACITY_10`: Determine device capacity and block size
- `READ_10` / `WRITE_10`: Block data transfer operations
- `REQUEST_SENSE`: Error condition analysis

**Usage Example:**
```rust
use imxrt_usbh::examples::example_msc::{MscManager, MassStorageDevice};

let mut msc_manager = MscManager::new();

// Initialize device
let mut device = MassStorageDevice::new(
    device_addr, interface, bulk_in_ep, bulk_out_ep,
    max_packet_in, max_packet_out, max_lun, subclass, protocol
);

device.initialize()?;  // INQUIRY + READ_CAPACITY

// Read first sector
let mut buffer = [0u8; 512];
let bytes_read = device.read_blocks(0, 1, &mut buffer)?;
println!("Read {} bytes from LBA 0", bytes_read);

// Get device info
println!("Device: {} {}", device.get_vendor_id(), device.get_product_id());
println!("Capacity: {} bytes", device.get_capacity_bytes());
```

## RTIC Integration Example

### `midi_keyboard.rs` - Real-Time USB with RTIC 2.x

Demonstrates integration with RTIC (Real-Time Interrupt-driven Concurrency) v2.x:

- **RTIC Integration**: Proper resource sharing and interrupt handling patterns
- **Real-Time Processing**: Interrupt-driven USB event handling with deterministic timing
- **MIDI Protocol**: Basic MIDI device detection and communication
- **Resource Management**: Safe sharing of USB resources across RTIC tasks
- **Error Recovery**: Robust error handling in real-time context

**RTIC Patterns Demonstrated:**
- Shared resources with proper locking
- Interrupt-driven USB event processing
- Task prioritization for USB operations
- Safe hardware abstraction usage

```bash
# Build RTIC example (requires RTIC features)
cargo build --example midi_keyboard --target thumbv7em-none-eabihf --features rtic-support
```

## Building Your Own Class Drivers

These examples provide templates for building separate class driver crates:

### Recommended Crate Structure

Based on the examples, here's how to structure class driver crates:

#### `imxrt-usbh-hid` (based on `example_hid.rs`)
```
imxrt-usbh-hid/
├── src/
│   ├── lib.rs           # Public API and re-exports
│   ├── keyboard.rs      # Keyboard-specific functionality  
│   ├── mouse.rs         # Mouse support
│   ├── generic.rs       # Generic HID device support
│   ├── report.rs        # HID report descriptor parsing
│   └── usage_tables.rs  # HID Usage Tables implementation
└── examples/
    ├── keyboard_demo.rs
    └── mouse_demo.rs
```

**Key Features to Include:**
- Report descriptor parsing with full HID Usage Tables support
- Input, output, and feature report handling
- Multiple device type support (keyboard, mouse, gamepad, etc.)
- Async/await compatibility for RTIC and Embassy

#### `imxrt-usbh-msc` (based on `example_msc.rs`)
```
imxrt-usbh-msc/
├── src/
│   ├── lib.rs           # Public API
│   ├── scsi.rs          # SCSI command implementation
│   ├── bot.rs           # Bulk-Only Transport protocol
│   ├── device.rs        # Mass storage device abstraction
│   └── fs.rs            # File system integration (optional)
└── examples/
    ├── file_operations.rs
    └── disk_benchmark.rs
```

**Integration Options:**
- `embedded-sdmmc` for FAT32 file system support
- Raw block device interface for custom file systems
- Multiple LUN (Logical Unit Number) support

#### `imxrt-usbh-cdc` (Communications Device Class)
```
imxrt-usbh-cdc/
├── src/
│   ├── lib.rs           # Public API
│   ├── acm.rs           # Abstract Control Model (virtual serial)
│   ├── ecm.rs           # Ethernet Control Model
│   └── line_coding.rs   # Serial line parameters
```

### Development Guidelines

1. **Dependency Management**: Depend on `imxrt-usbh` for core functionality
2. **Error Handling**: Provide class-specific error types that wrap core USB errors
3. **Async Support**: Design APIs to work with both blocking and async patterns
4. **Testing**: Include unit tests and hardware-in-the-loop test examples
5. **Documentation**: Comprehensive docs with usage examples and protocol explanations

### Class Driver API Pattern

```rust
// Typical class driver structure
pub struct HidDevice {
    // Core USB device info
    device_addr: u8,
    interface: u8,
    
    // Class-specific endpoints and configuration
    interrupt_in: u8,
    poll_interval: u8,
    
    // Class-specific state
    report_descriptor: Vec<u8>,
    current_report: HidReport,
}

impl HidDevice {
    pub fn new(/* USB enumeration info */) -> Result<Self> { /* ... */ }
    
    pub fn poll(&mut self) -> Result<Option<HidEvent>> { /* ... */ }
    
    pub fn send_report(&mut self, report: &HidReport) -> Result<()> { /* ... */ }
}

// Manager for multiple devices
pub struct HidManager {
    devices: Vec<HidDevice>,
}

impl HidManager {
    pub fn add_device(&mut self, device: HidDevice) -> Result<DeviceId> { /* ... */ }
    
    pub fn poll_all(&mut self) -> Result<Vec<(DeviceId, HidEvent)>> { /* ... */ }
}
```

## Testing Examples

### Unit Tests
Examples include comprehensive unit tests demonstrating:
- Protocol message construction and parsing
- State machine validation
- Error condition handling
- Edge case behavior

### Hardware-in-the-Loop (HIL) Tests
See `../tests/hil.rs` for hardware testing patterns:
- Real device enumeration
- Data transfer validation
- Error recovery testing
- Performance benchmarking

### Running Tests
```bash
# Unit tests (no hardware required)
cargo test --features std

# Integration tests
cargo test --test integration --features std

# Hardware-in-the-loop tests (requires connected devices)
cargo test --test hil --features std -- --ignored
```

## Hardware Requirements

### Teensy 4.x Setup
- **Teensy 4.0 or 4.1** with USB host capability
- **USB Type-A connector** for connecting devices
- **5V power supply** for bus-powered devices
- **Proper VBUS control** (see hardware documentation)

### Wiring
```
Teensy 4.x USB Host Pins:
├── USB1_DN  (Pin 30) → USB D-
├── USB1_DP  (Pin 31) → USB D+  
├── USB1_ID  (Pin 32) → Ground (host mode)
└── 5V       → VBUS (through switching circuit)
```

### Supported Devices
Examples have been tested with:
- **HID**: Standard USB keyboards, mice, gamepads
- **MSC**: USB flash drives, external hard drives, SD card readers
- **MIDI**: USB MIDI keyboards and controllers

## Troubleshooting

### Common Issues

1. **Device Not Detected**
   - Check VBUS power supply and switching
   - Verify USB cable and connections
   - Ensure device is USB 2.0 compatible

2. **Enumeration Failures**
   - Check timing compliance (reset pulse width, delay periods)
   - Verify descriptor parsing logic
   - Enable debug logging for detailed trace

3. **Transfer Errors**
   - Verify endpoint configuration matches device descriptors
   - Check DMA buffer alignment (32-byte requirement)
   - Validate transfer sizes and timing

### Debug Logging
Enable detailed logging with:
```toml
[dependencies]
imxrt-usbh = { version = "0.1", features = ["defmt"] }
```

```rust
// In your application
use defmt_rtt as _; // Enable RTT logging
```

### Performance Tuning
- **Bulk Transfers**: Use largest possible packet sizes
- **Interrupt Transfers**: Optimize polling intervals
- **Memory Management**: Reuse DMA buffers when possible
- **Cache Management**: Ensure proper cache coherency

## Contributing

When adding new examples:
1. Follow existing code style and documentation patterns
2. Include comprehensive error handling
3. Add unit tests for protocol logic
4. Document hardware requirements and setup
5. Test with real hardware when possible

## References

- [USB 2.0 Specification](http://www.poweredusb.org/pdf/usb20.pdf)
- [HID Specification 1.11](https://www.usb.org/sites/default/files/documents/hid1_11.pdf)
- [Mass Storage Class Specification](https://www.usb.org/sites/default/files/documents/usb_msc_overview_1.2.pdf)
- [SCSI Commands Reference](https://www.t10.org/lists/op-num.htm)
- [RTIC Book](https://rtic.rs/)