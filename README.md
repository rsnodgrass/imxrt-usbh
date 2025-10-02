# imxrt-usbh

**Work in progress - not ready for use yet**

A production-ready, simplified USB host driver for i.MX RT processors with embedded systems best practices.

**Core USB host functionality only** - device class drivers (MIDI, HID, etc.) are separate libraries that build on top of this foundation.

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
    BulkTransferManager, InterruptTransferManager, Direction,
};

// Initialize USB host
let mut controller = EhciController::new(usb1_base, usb1_registers);
controller.init()?;

// Enumerate connected device
let mut memory_pool = unsafe { &mut USB_MEMORY_POOL };
let mut enumerator = DeviceEnumerator::new(&mut controller, &mut memory_pool);
let device = enumerator.enumerate_device()?;

// Perform bulk data transfer
let mut bulk_manager = BulkTransferManager::new();
let mut buffer_pool = memory_pool.dma_buffer_pool();
let data_buffer = buffer_pool.alloc(512)?;

// Submit bulk IN transfer
let transfer_id = bulk_manager.submit(
    Direction::In,        // Read from device
    device.address,       // Device address
    0x81,                 // Bulk IN endpoint
    512,                  // Max packet size
    data_buffer,          // DMA buffer
    1000,                 // Timeout (1 second)
)?;

// Start the transfer
bulk_manager.start_transfer(transfer_id, &mut allocator)?;

// Setup periodic interrupt transfer (for HID devices)
let mut interrupt_manager = InterruptTransferManager::new();
let hid_buffer = buffer_pool.alloc(64)?;

let int_transfer_id = interrupt_manager.submit(
    Direction::In,        // Read from device
    device.address,       // Device address
    0x81,                 // Interrupt IN endpoint
    64,                   // Max packet size
    hid_buffer,           // DMA buffer
    10,                   // Poll every 10ms
    true,                 // Periodic transfer
)?;

// Device class handling happens in separate crates:
// - imxrt-usbh-midi for MIDI keyboards  
// - imxrt-usbh-hid for mice/keyboards
// - imxrt-usbh-msc for mass storage
```

## Building Class Driver Crates

The examples demonstrate how to build separate class driver crates like `imxrt-usbh-hid`, `imxrt-usbh-msc`, and `imxrt-usbh-cdc`. Each depends on this core library while providing high-level, type-safe APIs for specific device classes.

## Examples

See the [`examples/`](examples/) directory for comprehensive examples and documentation:

### Getting Started Examples
- **`01_basic_host_init.rs`** - USB PHY initialization with serial output
- **`02_device_enumeration.rs`** - EHCI controller setup and basic host components

These introductory examples output detailed status messages to serial (115200 baud on pins 0/1).
Just flash to your Teensy 4.0/4.1 and open a serial monitor to see the initialization process.

### Full Working Examples
- **HID keyboards**: Reference implementation for HID class drivers
- **Mass storage**: SCSI/BOT protocol implementation for USB drives
- **MIDI devices**: USB MIDI keyboard handling
- **RTIC integration**: Real-time USB handling patterns

For detailed documentation and usage examples, see [examples/README.md](examples/README.md).

## References

* [i.MX RT1060 Reference Manual Rev. 3 (IMXRT1060RM)](https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf)
* [AN12042: "Using the i.MX RT L1 Cache"](https://www.nxp.com/docs/en/application-note/AN12042.pdf)
* [EHCI Specification 1.0](https://www.intel.com/content/dam/www/public/us/en/documents/technical-specifications/ehci-specification-for-usb.pdf)
* [USB 2.0 Specification](http://www.poweredusb.org/pdf/usb20.pdf)
* [ARM Cortex-M7 Technical Reference Manual](https://developer.arm.com/documentation/ddi0489/f/introduction/documentation)
* [cotton-usb-host architecture](https://docs.rs/cotton-usb-host/latest/cotton_usb_host/) (adaptation reference)

## License

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
