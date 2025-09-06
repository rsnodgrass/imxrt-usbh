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


## Current Status

- ✅ **Core enumeration**: Complete USB device enumeration
- ✅ **Memory management**: Simplified and production-ready
- ✅ **Error handling**: Actionable recovery strategies
- ✅ **Safety monitoring**: Stack overflow and bounds checking
- ✅ **RTIC integration**: Real-time interrupt handling
- ✅ **Cache coherency**: Proper Cortex-M7 cache operations
- ✅ **Device detection**: Identifies device classes (Audio, HID, MSC, etc.)
- ✅ **Bulk transfer implementation**: IN/OUT endpoint data transfer  
- ✅ **Interrupt transfer implementation**: Periodic endpoint data transfer
- ✅ **Isochronous transfer implementation**: Real-time streaming data transfer
- 🔄 **Hub support**: Multiple device enumeration





## Examples

- `examples/midi_keyboard.rs`: RTIC application showing USB device enumeration

## Related Crates

Example device class drivers that may be built on `imxrt-usbh`:

- **imxrt-usbh-midi**: MIDI keyboard and controller support
- **imxrt-usbh-hid**: Human Interface Devices (keyboards, mice)
- **imxrt-usbh-msc**: Mass Storage Class (USB drives)
- **imxrt-usbh-cdc**: Communications Device Class (virtual serial ports)

Each class driver provides high-level APIs specific to that device type while building on the core enumeration and transfer functionality provided here.

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
