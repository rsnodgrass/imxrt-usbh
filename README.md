# imxrt-usbh

**Work in progress - not ready for use yet**

A production-ready USB host driver for i.MX RT processors (Teensy 4.x).

**Core USB host functionality only** - device class drivers (MIDI, HID, etc.) are separate libraries that build on top of this foundation.

## Quick Start - Get Running in 5 Minutes

### Prerequisites

1. **Hardware**: Teensy 4.1 board
2. **Rust toolchain**:
   ```bash
   rustup target add thumbv7em-none-eabihf
   cargo install cargo-binutils
   rustup component add llvm-tools-preview
   ```
3. **Teensy Loader**: Download from [PJRC](https://www.pjrc.com/teensy/loader.html) or install CLI: `brew install teensy_loader_cli`

### Build and Flash Example 01

The simplest example - initializes USB hardware:

```bash
# Clone repository
git clone https://github.com/acertain/imxrt-usbh
cd imxrt-usbh

# Build and create hex file
cargo build --release --example 01_basic_host_init
cargo objcopy --release --example 01_basic_host_init -- -O ihex 01.hex

# Flash to Teensy
teensy_loader_cli --mcu=TEENSY41 -w 01.hex
```

**Success indicator**: LED blinks slowly. Fast blink means error.

**What it does**: Initializes the USB PHY (Physical Layer) hardware.

### Build and Flash Example 02

Adds the EHCI controller:

```bash
cargo build --release --example 02_device_enumeration
cargo objcopy --release --example 02_device_enumeration -- -O ihex 02.hex
teensy_loader_cli --mcu=TEENSY41 -w 02.hex
```

**Success indicator**: LED blinks slowly. Fast blink means error.

**What it does**: Creates the USB host controller on top of the PHY.

### Optional: View Serial Output

To see detailed initialization messages:
1. Connect USB-to-serial adapter: TX→Pin 0, RX→Pin 1, GND→GND
2. Open serial monitor at 115200 baud
3. Press Teensy reset button

**Note**: Serial output is optional - the LED indicates success/failure.

---

## Target Hardware

- **Primary**: Teensy 4.0/4.1 (i.MX RT1062)
- **CPU**: ARM Cortex-M7 @ 600MHz
- **USB**: EHCI host controller
- **Memory**: TCM/OCRAM with D-cache management

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

## Additional Examples

See the [`examples/`](examples/) directory for more advanced examples:
- **`enumerate_device.rs`** - Full device enumeration and management
- **`hid_keyboard.rs`** - USB keyboard support with key mapping
- **`hid_gamepad.rs`** - Game controller support
- **`mass_storage.rs`** - USB flash drive support (SCSI/BOT protocol)
- **`midi_keyboard.rs`** - MIDI device handling

For detailed documentation, see [examples/README.md](examples/README.md).

## Library Usage

For advanced usage and integration into your own projects:

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
