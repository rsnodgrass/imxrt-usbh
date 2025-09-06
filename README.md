# imxrt-usbh

**Work in progress - not ready for use yet**

A USB host driver for i.MX RT processors. `imxrt-usbh` provides EHCI USB host functionality
for the i.MX RT1062 microcontroller family, targeting Teensy 4.0/4.1 boards with RTIC support.

Initially designed for MIDI keyboard integration and real-time applications.

## Target Hardware

- **Primary**: Teensy 4.0/4.1 (i.MX RT1062)
- **CPU**: ARM Cortex-M7 @ 600MHz
- **USB**: EHCI host controller
- **Memory**: TCM/OCRAM with D-cache management
- **RTIC**: v2.x for real-time guarantees

## Architecture

- **EHCI USB Host**: Full EHCI 1.0 compliant implementation
- **RTIC Integration**: Real-time interrupt handling with <10μs ISR latency
- **Cache Coherent DMA**: ARM Cortex-M7 cache management for USB transfers
- **Safety First**: `#![deny(unsafe_op_in_unsafe_fn)]` with comprehensive bounds checking

## Design Choices

* prefer simplification over feature completeness
- **Real-time friendly**: Predictable timing, no locks in hot paths
- **Stack overflow protection**: Runtime monitoring with canary patterns
- **Hardware timing compliance**: Proper PHY initialization per i.MX RT reference manual
- **Cache coherency**: Actual Cortex-M7 cache operations, not placeholders
- **Error recovery**: Actionable errors with automatic retry logic


## Quick Start

```bash
# Install LLD for faster compilation
brew install lld  # macOS
# or: sudo apt install lld  # Linux

# Add ARM Cortex-M7 target
rustup target add thumbv7em-none-eabihf
```

### Basic Usage

```rust
use imxrt_usbh::{
    enumeration::DeviceEnumerator,
    dma::memory::USB_MEMORY_POOL,
    ehci::controller::EhciController,
};

// Initialize USB host
let mut controller = EhciController::new(usb1_base, usb1_registers);
controller.init()?;

// Enumerate connected device
let mut memory_pool = unsafe { &mut USB_MEMORY_POOL };
let mut enumerator = DeviceEnumerator::new(&mut controller, &mut memory_pool);
let device = enumerator.enumerate_device()?;

// Check if it's a MIDI device
if device.is_midi {
    // Ready for MIDI communication
}
```


## Current Status

- ✅ **Core enumeration**: Complete with MIDI device detection
- ✅ **Memory management**: Simplified and production-ready
- ✅ **Error handling**: Actionable recovery strategies
- ✅ **Safety monitoring**: Stack overflow and bounds checking
- ✅ **RTIC integration**: Real-time interrupt handling
- ✅ **Cache coherency**: Proper Cortex-M7 cache operations
- 🔄 **MIDI data transfer**: Bulk endpoint setup (next priority)
- 🔄 **MIDI message parsing**: Note on/off, control change handling
- 🔄 **Multiple device support**: Hub and multi-device management





## Examples

- `examples/midi_keyboard.rs`: Complete RTIC application for MIDI keyboard enumeration with LED status indication.

### MIDI Keyboard Support

This driver was built with MIDI keyboards as the primary use case, providing an alternative to MIDI serial communication:

```rust
// Automatic MIDI keyboard detection and enumeration
if device.is_midi {
    println!("MIDI keyboard connected!");
    println!("VID: {:04x}, PID: {:04x}", device.device_desc.id_vendor, device.device_desc.id_product);
    // Ready for MIDI message parsing and real-time data transfer
}
```


## License

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
