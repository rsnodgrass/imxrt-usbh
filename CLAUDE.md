# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Production-ready USB host driver for i.MX RT1062 (Teensy 4.0/4.1) with EHCI controller. Embedded systems focus with real-time guarantees, cache coherency, and safety-first design.

**Target**: ARM Cortex-M7 @ 600MHz with EHCI USB host controller

## Build Commands

### Prerequisites

```bash
# Install ARM Cortex-M7 target
rustup target add thumbv7em-none-eabihf

# Install cargo-binutils for creating hex files
cargo install cargo-binutils
rustup component add llvm-tools-preview

# Optional: Install LLD for faster linking
brew install lld  # macOS
# or: sudo apt install lld  # Linux
```

### Building

```bash
# Build library (default target: thumbv7em-none-eabihf)
cargo build

# Build with release optimizations (size-optimized)
cargo build --release

# Build specific example for Teensy
cargo build --release --example 01_basic_host_init

# Build all examples
cargo build --release --examples

# Run tests (requires std feature)
cargo test --features std

# Check without building (fast iteration)
cargo check

# Check specific feature combinations
cargo check --features "rtic-support,hub"
```

### Flashing to Teensy

```bash
# Build and convert to hex format
cargo build --release --example enumerate_device
cargo objcopy --release --example enumerate_device -- -O ihex enumerate_device.hex

# Flash using teensy_loader_cli
teensy_loader_cli --mcu=TEENSY41 -w enumerate_device.hex

# Or use Teensy Loader GUI application
```

## Architecture

### Core Module Structure

- **`ehci/`** - EHCI 1.0 controller implementation with register-level access
  - `controller.rs` - Main EHCI controller with type-state pattern (Uninitialized → Initialized → Running)
  - `qtd.rs` - Queue Transfer Descriptors for DMA transfers
  - `qh.rs` - Queue Heads for endpoint management
  - `register.rs` - Memory-mapped register abstractions

- **`dma/`** - Cache-coherent DMA buffer management
  - `memory.rs` - Static memory pool allocator (32 QH, 32 QTD, 32 buffers)
  - `buffer.rs` - DMA buffer with ARM Cortex-M7 cache operations
  - `descriptor.rs` - EHCI data structure allocators
  - All DMA structures are 32-byte aligned per EHCI spec

- **`transfer/`** - High-level transfer APIs
  - `control.rs` - Control transfers (device enumeration, configuration)
  - `bulk.rs` - Bulk transfers (mass storage, printers)
  - `interrupt.rs` - Interrupt transfers (HID keyboards, mice)
  - `isochronous.rs` - Isochronous transfers (audio, video)
  - `simple_control.rs` - Simplified control transfer executor

- **`enumeration.rs`** - USB device discovery and descriptor parsing
  - Device/Configuration/Interface/Endpoint descriptor handling
  - Device class identification (Audio, HID, Mass Storage, Hub)

- **`phy.rs`** - USB PHY initialization and calibration for i.MX RT1062
- **`vbus.rs`** - VBUS power management and control
- **`safety.rs`** - Stack monitoring, canary patterns, overflow detection
- **`error.rs`** - Comprehensive error types with recovery guidance
- **`perf.rs`** - Performance monitoring and statistics
- **`recovery.rs`** - Automatic error recovery mechanisms

### Memory Safety

- `#![deny(unsafe_op_in_unsafe_fn)]` enforced globally
- All EHCI data structures use proper alignment and repr(C)
- DMA buffers include explicit cache coherency operations (clean/invalidate)
- Static memory pools to avoid dynamic allocation in `no_std`

### Cache Coherency (Critical)

ARM Cortex-M7 has D-cache that must be managed for DMA:
```rust
// Before hardware reads (CPU wrote data)
buffer.clean_cache();

// After hardware writes (CPU will read data)
buffer.invalidate_cache();
```

### Type-State Pattern

EHCI controller uses compile-time state tracking:
```rust
EhciController<8, Uninitialized>  // Created but not initialized
  → EhciController<8, Initialized>  // PHY/clocks configured
    → EhciController<8, Running>     // Schedules active, ready for transfers
```

## Hardware-Specific Details

### i.MX RT1062 USB Configuration

- **USB1 Base**: 0x402E_0000 (typically USB Device on Teensy micro USB port)
- **USB2 Base**: 0x402E_0200 (typically USB Host on Teensy pins 30/31)
- **USBPHY1 Base**: 0x400D_9000
- **CCM Base**: 0x400F_C000 (clock control)

### USB Port Mapping (Teensy 4.1)

- USB1 (micro USB): Device mode for programming/CDC serial
- USB2 (pins 30/31): Host mode for connecting USB devices

Examples use USB2 for host functionality, leaving USB1 for CDC serial output.

## Testing Strategy

- Unit tests in `src/lib_test.rs` (requires `--features std`)
- Hardware tests via examples (examples/ directory)
- Examples serve dual purpose: educational + integration tests

## Common Patterns

### Device Enumeration Flow

```rust
// 1. Initialize controller
let mut controller = EhciController::new(USB2_BASE)?;
controller.init()?;

// 2. Wait for device connection
while !controller.is_port_connected(0)? {}

// 3. Reset port (required by USB spec)
controller.reset_port(0)?;

// 4. Get device descriptor
let mut executor = ControlExecutor::new(&mut controller, &mut memory_pool);
let descriptor = executor.get_device_descriptor(0)?;

// 5. Assign address
executor.set_address(0, new_address)?;

// 6. Get configuration
let config = executor.get_configuration_descriptor(new_address, 0)?;
```

### Transfer Submission

```rust
// Bulk transfer example
let mut bulk_manager = BulkTransferManager::new();
let buffer = memory_pool.alloc_buffer(512)?;

let transfer_id = bulk_manager.submit(
    Direction::In,
    device_addr,
    endpoint_num,
    max_packet_size,
    buffer,
    timeout_ms,
)?;

bulk_manager.start_transfer(transfer_id, &mut controller)?;
```

## Critical Safety Considerations

1. **No `unsafe` without explicit safety comment**: Every unsafe block requires detailed safety justification
2. **DMA buffer lifetimes**: Ensure buffers outlive hardware access
3. **Cache coherency**: Always clean before HW reads, invalidate before CPU reads
4. **Interrupt safety**: Use critical sections when accessing shared state
5. **Stack monitoring**: Built-in stack overflow detection with canaries

## USB Timing Requirements

Per USB 2.0 and i.MX RT reference manual:
- Port reset: minimum 20ms assertion, 50ms timeout
- Device response: 100ms for SET_ADDRESS
- Hub power-on: 100ms minimum
- Controller reset: 250ms timeout

## Dependencies

- `cortex-m` - ARM Cortex-M register access
- `imxrt-ral` - i.MX RT register access layer
- `heapless` - Static collections without heap allocation
- `bitflags` - Type-safe register bit flags
- `critical-section` - Interrupt-safe critical sections

Optional:
- `rtic` - Real-time interrupt-driven concurrency (v2.x)
- `imxrt-hal` - Higher-level HAL abstractions
- `defmt` - Efficient embedded logging

## Feature Flags

- `default = ["defmt"]` - Embedded logging
- `rt` - Include runtime support (cortex-m-rt)
- `hal` - Enable imxrt-hal integration
- `hub` - USB hub and Transaction Translator support
- `rtic-support` - RTIC v2.x integration
- `std` - Enable std for unit testing

## Performance Characteristics

- High-speed USB: up to 400 Mbps practical throughput
- CPU overhead: <5% for bulk transfers at full utilization
- Interrupt latency: <125 μs for scheduled transfers
- ISR execution time: <10 μs target for RTIC integration

## Code Style

- Descriptive variable names (avoid abbreviations unless standard USB terms)
- Extensive inline documentation for register operations
- Safety comments required for all `unsafe` blocks
- Hardware register operations include timing comments
- Error types include recovery guidance in documentation
