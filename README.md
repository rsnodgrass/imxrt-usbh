# imxrt-usbh

**Production-ready USB host driver for i.MX RT processors (Teensy 4.x)**

A comprehensive USB host library with two-tier API design: a simple API for common use cases and a low-level API for advanced control.

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

### Build and Flash Your First Example

The simplest example - read from a USB keyboard:

```bash
# Clone repository
git clone https://github.com/acertain/imxrt-usbh
cd imxrt-usbh

# Build and create hex file
cargo build --release --example 01_hello_keyboard --target thumbv7em-none-eabihf
rust-objcopy -O ihex \
  target/thumbv7em-none-eabihf/release/examples/01_hello_keyboard \
  target/thumbv7em-none-eabihf/release/examples/01_hello_keyboard.hex

# Flash to Teensy
teensy_loader_cli --mcu=TEENSY41 -w \
  target/thumbv7em-none-eabihf/release/examples/01_hello_keyboard.hex
```

**What it does**: Automatically detects a USB keyboard and prints keys as you type!

**View output**: Connect USB-to-serial adapter (TX→Pin 0, RX→Pin 1, GND→GND) and open serial monitor at 115200 baud.

---

## Two-Tier API Design

This library provides two APIs to serve different needs:

### Simple API (Recommended for Most Users)

High-level, batteries-included API for common use cases:

```rust
use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::{HidDevice, KeyboardReport};

// Initialize (handles PHY, DMA, controller setup)
let mut usb = SimpleUsbHost::new(0x402E_0200, 0x400DA000, 0x400F_C000)?;

// Wait for keyboard
let device = usb.wait_for_device()?;
let mut kbd = HidDevice::from_device(device)?;
kbd.enable_boot_protocol(&mut usb)?;

// Read keys
// ... (see examples for full code)
```

**Use the simple API when you:**
- Want to get started quickly
- Are working with HID devices (keyboard, mouse, gamepad)
- Don't need precise timing control
- Prefer synchronous, blocking operations

### Low-Level API (Advanced Users)

Direct hardware control for specialized needs:

```rust
use imxrt_usbh::{
    ehci::{EhciController, TransferExecutor},
    dma::UsbMemoryPool,
    transfer::{BulkTransferManager, Direction},
};

// Manual initialization
unsafe { imxrt_usbh::dma::init_dma_region()?; }
let mut controller = unsafe { EhciController::<8>::new(0x402E_0200)? };
let mut controller = unsafe { controller.initialize()?.start() };

// Direct transfer control
let mut bulk_manager = BulkTransferManager::<16>::new();
// ... (see examples/advanced/ for full code)
```

**Use the low-level API when you:**
- Need isochronous transfers (audio/video streaming)
- Require precise timing control
- Want custom transfer scheduling
- Need direct EHCI register access

---

## Examples

### Simple Examples (Start Here!)

1. **`01_hello_keyboard.rs`** - Minimal keyboard input (best first example)
2. **`02_device_info.rs`** - Display device information (VID/PID, endpoints)
3. **`03_hid_mouse.rs`** - Mouse input with cursor tracking
4. **`04_hid_gamepad.rs`** - Game controller support
5. **`05_typing_game.rs`** - Interactive typing speed game (30 seconds, WPM tracking)

### Advanced Examples

See [`examples/advanced/`](examples/advanced/) for low-level API usage:

1. **`01_phy_initialization.rs`** - Manual PHY setup
2. **`02_manual_enumeration.rs`** - Manual device enumeration
3. **`03_midi_keyboard.rs`** - MIDI device support
4. **`04_multi_device_manager.rs`** - Multi-device management
5. **`05_mass_storage.rs`** - USB flash drive support
6. **`06_hid_report_protocol.rs`** - Advanced HID descriptor parsing
7. **`07_qwerty_keyboard.rs`** - Low-level keyboard implementation

For detailed documentation, see [examples/README.md](examples/README.md).

---

## Target Hardware

- **Primary**: Teensy 4.0/4.1 (i.MX RT1062)
- **CPU**: ARM Cortex-M7 @ 600MHz
- **USB**: EHCI host controller
- **Memory**: TCM/OCRAM with D-cache management

## Architecture

- **Two-Tier API**: Simple wrapper + Low-level EHCI access
- **EHCI USB Host**: Full EHCI 1.0 compliant implementation
- **Cache Coherent DMA**: ARM Cortex-M7 cache management for USB transfers
- **Safety First**: `#![deny(unsafe_op_in_unsafe_fn)]` with comprehensive bounds checking
- **RTIC Integration**: Real-time interrupt handling with <10μs ISR latency

## Design Choices

- **Developer friendly**: Simple API for 90% of use cases, low-level API for the rest
- **Real-time friendly**: Predictable timing, no locks in hot paths
- **Stack overflow protection**: Runtime monitoring with canary patterns
- **Hardware timing compliance**: Proper PHY initialization per i.MX RT reference manual
- **Cache coherency**: Actual Cortex-M7 cache operations, not placeholders
- **Error recovery**: Actionable errors with automatic retry logic

## Library Usage

### Simple API (Recommended)

For HID devices (keyboard, mouse, gamepad):

```rust
use imxrt_usbh::simple::SimpleUsbHost;
use imxrt_usbh::hid::{HidDevice, KeyboardReport};

// 1. Initialize USB host (handles PHY, DMA, controller)
let mut usb = SimpleUsbHost::new(
    0x402E_0200,  // USB2 base
    0x400DA000,   // PHY2 base
    0x400F_C000,  // CCM base
)?;

// 2. Wait for device
let device = usb.wait_for_device()?;

// 3. Create HID wrapper and enable boot protocol
let mut kbd = HidDevice::from_device(device)?;
kbd.enable_boot_protocol(&mut usb)?;

// 4. Set up interrupt transfer (one line with factory helper!)
let (interrupt_mgr, transfer_id) = kbd.create_polling_manager::<4>(&mut usb)?;

// 5. Main loop - simplified with poll helper
loop {
    if let Some(data) = interrupt_mgr.poll_transfer_data(transfer_id) {
        let report = KeyboardReport::parse(data);
        // Process keys...
    }
}
```

### Low-Level API (Advanced)

For direct hardware control:

```rust
use imxrt_usbh::{
    enumeration::DeviceEnumerator,
    dma::UsbMemoryPool,
    ehci::{EhciController, TransferExecutor, Uninitialized},
    transfer::{BulkTransferManager, InterruptTransferManager, Direction},
    phy::UsbPhy,
};

// 1. Initialize DMA region (CRITICAL: must be called first)
unsafe { imxrt_usbh::dma::init_dma_region()?; }

// 2. Initialize USB PHY
let mut phy = unsafe { UsbPhy::new(0x400DA000, 0x400F_C000) };
phy.init_host_mode()?;

// 3. Initialize EHCI controller
let controller = unsafe { EhciController::<8, Uninitialized>::new(0x402E_0200)? };
let controller = unsafe { controller.initialize()? };
let mut controller = unsafe { controller.start() };

// 4. Initialize memory pool and transfer executor
let mut memory_pool = UsbMemoryPool::new();
let mut transfer_executor = unsafe { TransferExecutor::new(0x402E_0200) };

// 5. Enumerate device
let mut enumerator = DeviceEnumerator::new(
    &mut controller,
    &mut memory_pool,
    &mut transfer_executor,
);
let device = enumerator.enumerate_device()?;

// 6. Create transfer managers
let mut bulk_manager = BulkTransferManager::<16>::new();
let data_buffer = memory_pool.alloc_buffer(512)?;

let transfer_id = bulk_manager.submit(
    Direction::In,
    device.address,
    0x81,          // Bulk IN endpoint
    512,           // Max packet size
    data_buffer,
    1000,          // Timeout (1 second)
)?;

bulk_manager.start_transfer(transfer_id, &mut controller)?;

// 7. Process transfers
loop {
    bulk_manager.process_completed(&mut controller);
    // ... handle completed transfers
}
```

## Documentation

- **API Guides**: See [`docs/`](docs/) directory
- **Examples**: See [`examples/`](examples/) and [`examples/advanced/`](examples/advanced/)
- **Architecture Notes**: See [`CLAUDE.md`](CLAUDE.md)

## References

* [i.MX RT1060 Reference Manual Rev. 3 (IMXRT1060RM)](https://www.pjrc.com/teensy/IMXRT1060RM_rev3.pdf)
* [AN12042: "Using the i.MX RT L1 Cache"](https://www.nxp.com/docs/en/application-note/AN12042.pdf)
* [EHCI Specification 1.0](https://www.intel.com/content/dam/www/public/us/en/documents/technical-specifications/ehci-specification-for-usb.pdf)
* [USB 2.0 Specification](http://www.poweredusb.org/pdf/usb20.pdf)
* [USB HID Specification 1.11](https://www.usb.org/document-library/device-class-definition-hid-111)
* [HID Usage Tables 1.12](https://usb.org/document-library/hid-usage-tables-15)
* [ARM Cortex-M7 Technical Reference Manual](https://developer.arm.com/documentation/ddi0489/f/introduction/documentation)

## License

Licensed under either of

- [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0) ([LICENSE-APACHE](./LICENSE-APACHE))
- [MIT License](http://opensource.org/licenses/MIT) ([LICENSE-MIT](./LICENSE-MIT))

at your option.

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
