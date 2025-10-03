# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Production-ready USB host driver for i.MX RT1062 (Teensy 4.0/4.1) with EHCI controller. Embedded systems focus with real-time guarantees, cache coherency, and safety-first design.

**Target**: ARM Cortex-M7 @ 600MHz with EHCI USB host controller. Specifically Teensy 4.1

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

### Debugging

```bash
# Decode crash addresses from CrashReport (if using Teensy recovery features)
arm-none-eabi-addr2line -e target/thumbv7em-none-eabihf/release/examples/enumerate_device <address>

# View binary size breakdown
cargo size --release --example enumerate_device -- -A

# Examine memory sections and placement
cargo readobj --release --example enumerate_device -- --section-headers
```

**Debugging Output:**
- USB1 (micro USB) remains available for CDC serial even while USB2 runs host mode
- Use `defmt` with RTT for zero-overhead logging without USB interference

## Architecture

### Core Module Structure

- **`ehci/`** - EHCI 1.0 controller implementation with register-level access
  - `controller.rs` - Main EHCI controller with type-state pattern (Uninitialized → Initialized → Running)
  - `qtd.rs` - Queue Transfer Descriptors for DMA transfers
  - `qh.rs` - Queue Heads for endpoint management
  - `register.rs` - Memory-mapped register abstractions

- **`dma/`** - Cache-coherent DMA buffer management
  - **ALWAYS USE**: `dma::DmaBuffer` (defined in `dma.rs`) - Production DMA buffer type
    - Uses `NonNull<u8>` for safety
    - NOT Copy/Clone to prevent aliasing
    - Methods: `as_slice()`, `as_mut_slice()`, `dma_addr()`, `prepare_for_device()`, `prepare_for_cpu()`
  - **ALWAYS USE**: `UsbMemoryPool` (in `memory.rs`) - Allocator for DmaBuffer
    - Simple bit-mask allocator (32 buffers max, 512 bytes each)
    - `alloc_buffer(size)` returns `DmaBuffer`
    - `free_buffer(&buffer)` returns buffer to pool
  - `descriptor.rs` - EHCI QH/QTD allocators (QhHandle, QtdHandle)
  - `pools.rs` - Alternative pool systems (NOT currently used for buffers)
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

**Cache Line Alignment Requirements:**
- DMA buffers must be 32-byte aligned (cache line size)
- Buffer size must be multiple of 32 bytes
- **Warning**: Invalidating unaligned buffers corrupts adjacent memory

**DMA Buffer Placement Strategies (in order of preference):**

1. **DTCM** (Best): Non-cacheable, no maintenance needed, fast CPU access
2. **MPU-configured non-cacheable OCRAM**: Configure via MPU, good for large buffers
3. **Cache-managed OCRAM**: Use clean/invalidate APIs, requires strict alignment

**i.MX RT1062 Errata Considerations:**
- Refer to NXP i.MX RT1060 Chip Errata document for silicon issues
- Cache maintenance on Write-Through memory has known errata (avoid if possible)
- Use DTCM or non-cacheable OCRAM for USB DMA operations

### Type-State Pattern

EHCI controller uses compile-time state tracking:
```rust
EhciController<8, Uninitialized>  // Created but not initialized
  → EhciController<8, Initialized>  // PHY/clocks configured
    → EhciController<8, Running>     // Schedules active, ready for transfers
```

## Hardware-Specific Details

### i.MX RT1062 Memory Layout

**FlexRAM (512 KB)** - Configurable as ITCM/DTCM/OCRAM:
- **ITCM** (Instruction Tightly Coupled Memory): Ultra-fast code execution, non-cacheable
- **DTCM** (Data Tightly Coupled Memory): Ultra-fast data access, non-cacheable, **preferred for DMA buffers**
- Default allocation: 128KB ITCM, 128KB DTCM, 256KB OCRAM (configurable via linker)

**OCRAM (512 KB)** - On-Chip RAM:
- 64-bit data bus optimized for DMA access
- **Cacheable by default** - requires MPU configuration or cache maintenance for DMA

**Flash**: 1.9 MB program storage

**Critical**: DMA buffers should be placed in DTCM (non-cacheable) or properly cache-managed OCRAM regions to avoid coherency issues.

### i.MX RT1062 USB Configuration

- **USB1 Base**: 0x402E_0000 (typically USB Device on Teensy micro USB port)
- **USB2 Base**: 0x402E_0200 (typically USB Host on Teensy pins 30/31)
- **USBPHY1 Base**: 0x400D_9000
- **USBPHY2 Base**: 0x400DA_000
- **CCM Base**: 0x400F_C000 (clock control)

### USB Port Mapping (Teensy 4.1)

- USB1 (micro USB): Device mode for programming/CDC serial
- USB2 (pins 30/31): Host mode for connecting USB devices

Examples use USB2 for host functionality, leaving USB1 for CDC serial output.

### Teensy-Specific Hardware Constraints

**Power:**
- **3.3V logic only** - I/O pins are **NOT 5V tolerant**. Applying >3.3V to any pin causes permanent damage.
- VIN accepts 3.6-6.0V external power (stay at 5V or below for safety)
- **Critical**: If using VIN for external power, cut the VUSB-VIN trace on board bottom to prevent back-feeding USB port

**Pin Limitations:**
- All GPIO pulled down at reset for safe startup
- Maximum pin current: 6.8 mA continuous per pin (per i.MX RT1062 spec)

**Clock Domains:**
- Base clock: 600 MHz (overclockable but not recommended for production)
- LPSPI maximum reliable speed: 30 MHz (higher speeds risk setup/hold violations)

## Testing Strategy

### Test Organization (2025 Rust Best Practices)

```
tests/
├── common/
│   ├── mod.rs              # Test utility exports
│   └── mock_hardware.rs    # Mock DMA buffers & USB descriptors
├── integration.rs          # Multi-transfer scenarios (8 tests)
├── error_handling.rs       # Error recovery (11 tests)
├── resource_management.rs  # Pool lifecycle (12 tests)
├── hil.rs                  # Hardware-in-the-loop tests
├── TESTS_IMPLEMENTED.md    # Test documentation
└── TESTING_MODEL.md        # Test architecture plan
```

**Test Types:**
- **Inline unit tests** (68 tests): In `src/` modules under `#[cfg(test)] mod tests`
  - Transfer managers (bulk, interrupt, isochronous)
  - DMA buffer safety and alignment
  - Descriptor parsing and enumeration

- **Integration tests** (31+ tests): In `tests/` directory
  - Multi-transfer scenarios and coexistence
  - Error handling and recovery
  - Resource pool management

- **Hardware tests**: `tests/hil.rs` with `#[ignore = "requires hardware"]`
  - Requires actual Teensy 4.1 with connected USB devices
  - Run with: `cargo test --test hil --features std -- --ignored`

**Critical**: All tests compile for ARM Cortex-M7 (`thumbv7em-none-eabihf`). This library directly accesses i.MX RT1062 hardware registers and cannot run on x86 platforms.

**Running Tests:**
```bash
# Build and verify tests compile for ARM target
cargo test --lib --no-run --target thumbv7em-none-eabihf

# Compile integration tests
cargo test --test integration --no-run --target thumbv7em-none-eabihf
cargo test --test error_handling --no-run --target thumbv7em-none-eabihf
cargo test --test resource_management --no-run --target thumbv7em-none-eabihf

# Run hardware tests on device (requires test runner on Teensy)
cargo test --test hil --features std --target thumbv7em-none-eabihf -- --ignored
```

**Test Utilities:**
- Mock DMA buffers using static arrays with `NonNull<u8>`
- USB descriptor builders (device, config, HID, MIDI)
- Alignment and overlap assertion helpers
- All tests use `#![no_std]` and `#![no_main]` with panic handlers

## Common Patterns

### DMA Buffer Usage (CRITICAL)

**ALWAYS use `dma::DmaBuffer` + `UsbMemoryPool` for all buffer allocations:**

```rust
use imxrt_usbh::dma::UsbMemoryPool;

// Initialize pool
let mut memory_pool = unsafe { &mut imxrt_usbh::dma::memory::USB_MEMORY_POOL };

// Allocate buffer
let buffer = memory_pool.alloc_buffer(512)
    .ok_or(UsbError::NoResources)?;

// Use buffer (automatically has proper type)
transfer_manager.submit(Direction::In, addr, ep, max_pkt, buffer, interval, true)?;

// Buffer is returned via free_buffer() or consumed by transfer manager
let _ = memory_pool.free_buffer(&buffer);
```

**DO NOT**:
- Create custom DmaBuffer types
- Use raw pointers for DMA buffers
- Copy or clone DmaBuffer (it's not Copy/Clone)
- Allocate buffers outside of UsbMemoryPool

**Buffer Lifecycle**:
1. Allocate: `memory_pool.alloc_buffer(size)` → Returns `Option<DmaBuffer>`
2. Use: Pass to transfer managers or access via `as_slice()` / `as_mut_slice()`
3. Free: Either consumed by transfer or manually freed with `free_buffer(&buffer)`

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
6. **Voltage protection**: Never apply >3.3V to any Teensy I/O pin (not 5V tolerant)
7. **Power inrush**: Large peripheral capacitors can delay startup; use powered USB hub or load switch for high-capacitance devices
8. **Buffer alignment**: All DMA buffers must be 32-byte aligned with size as multiple of 32 bytes

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

## Teensy 4.x Linker Configuration

**Memory Regions (typical layout):**
```
FLASH:  0x6000_0000 - 0x601F_0000 (1.9 MB)
ITCM:   0x0000_0000 - 0x0001_FFFF (128 KB)
DTCM:   0x2000_0000 - 0x2001_FFFF (128 KB)
OCRAM:  0x2020_0000 - 0x2023_FFFF (256 KB)
```

**Section Placement Best Practices:**
- Critical interrupt handlers → ITCM (fastest execution)
- DMA buffers → DTCM (non-cacheable, fast CPU access)
- Large buffers → OCRAM with MPU configuration
- Const data → Flash with PROGMEM attribute when space-constrained

## Common Teensy 4.x Gotchas

1. **FlexSPI**: Specialized for Flash/PSRAM with LUT configuration, not general-purpose SPI
2. **EEPROM**: Emulated with wear-leveling; suitable for settings, not high-frequency logging
3. **Clock configuration**: Changing clocks during USB operations causes instability

## Critical Software Development Patterns

### Memory Barriers for MMIO (Cortex-M7 Weak Ordering)

ARM Cortex-M7 has weakly-ordered memory. **All** MMIO register access requires memory barriers:

```rust
// CORRECT: Register write with barriers
pub fn write_register(addr: *mut u32, value: u32) {
    unsafe {
        cortex_m::asm::dmb();  // Data Memory Barrier before write
        core::ptr::write_volatile(addr, value);
        cortex_m::asm::dsb();  // Data Synchronization Barrier ensures completion
    }
}

// CORRECT: Register read with barriers
pub fn read_register(addr: *const u32) -> u32 {
    unsafe {
        cortex_m::asm::dmb();
        let value = core::ptr::read_volatile(addr);
        cortex_m::asm::dmb();
        value
    }
}
```

**When to use barriers:**
- Control register writes: `dmb()` before, `dsb()` after
- Status register reads affecting control flow: `dmb()` before and after
- Read-only capability registers: can omit barriers
- After MPU/cache config changes: `dsb()` + `isb()`

**Critical**: Missing `dsb()` after register write causes next instruction to execute before hardware sees write. Bug symptom: "works with debug prints, fails without them" (print adds delay).

### Write-1-to-Clear Register Pattern

Many i.MX RT1062 status registers use write-1-to-clear semantics:

```rust
// CORRECT: Clearing EHCI interrupt status
let status = core::ptr::read_volatile(usbsts_addr);
core::ptr::write_volatile(usbsts_addr, status);  // Write value back to clear

// WRONG: Read-modify-write doesn't work for W1C registers
let mut status = core::ptr::read_volatile(usbsts_addr);
status &= !bit_to_clear;  // This has no effect on W1C registers!
core::ptr::write_volatile(usbsts_addr, status);
```

**W1C registers:** USBSTS, PORTSC change bits, many CCM status registers

### DMA Buffer Alignment (Hardware Requirement)

```rust
// CORRECT: 32-byte aligned DMA buffer
#[repr(C, align(32))]
pub struct DmaBuffer {
    data: [u8; 512],  // Size must be multiple of 32
}

// CORRECT: Compile-time alignment validation
const _: () = {
    assert!(core::mem::align_of::<QueueTD>() == 32);
    assert!(512 % 32 == 0);
};
```

**Critical**: Invalidating unaligned cache buffers corrupts adjacent memory. All DMA buffers must be 32-byte aligned with size as multiple of 32 bytes.

### Static Mut Alternatives

Modern embedded Rust **never** uses `static mut`:

```rust
// CORRECT: AtomicBool for simple flags
static INITIALIZED: AtomicBool = AtomicBool::new(false);

// CORRECT: Mutex<RefCell<Option<T>>> for complex state
use critical_section::Mutex;
use core::cell::RefCell;

static STATE: Mutex<RefCell<Option<ControllerState>>> =
    Mutex::new(RefCell::new(None));

pub fn access_state() {
    critical_section::with(|cs| {
        let mut state = STATE.borrow_ref_mut(cs);
        // Access state safely
    });
}

// WRONG: Never use static mut
static mut COUNTER: u32 = 0;  // Data race! Undefined behavior!
```

### Atomic Ordering Guidelines

```rust
// CORRECT: Relaxed for CPU-only statistics
self.stats.allocs.fetch_add(1, Ordering::Relaxed);

// CORRECT: AcqRel for buffer allocation (shared state)
self.allocated.compare_exchange(
    false, true,
    Ordering::AcqRel,   // Success ordering
    Ordering::Acquire   // Failure ordering
);

// CORRECT: Release when freeing resource
self.allocated.store(false, Ordering::Release);

// WRONG: SeqCst everywhere wastes cycles
self.stats.store(x, Ordering::SeqCst);  // Too strong for stats
```

**Guidelines:**
- `Relaxed`: CPU-only counters, statistics
- `Acquire/Release`: Most shared state (producer-consumer patterns)
- `SeqCst`: Rarely needed, has performance cost

### Precise Timing with DWT Cycle Counter

```rust
// CORRECT: DWT-based microsecond delay
#[inline(always)]
fn delay_us(us: u32) {
    let start = cortex_m::peripheral::DWT::cycle_count();
    let target = us * 600;  // CPU_FREQ_MHZ = 600

    while cortex_m::peripheral::DWT::cycle_count()
        .wrapping_sub(start) < target
    {
        cortex_m::asm::nop();
    }
}

// WRONG: Loop-based delay - changes with optimization level
for _ in 0..delay_count {  // Unreliable!
    cortex_m::asm::nop();
}
```

**Critical timings (from hardware spec):**
- USB port reset: 20ms minimum assertion
- PLL lock wait: 10ms timeout
- Port power stabilization: 100ms

### DMA Lifetime Safety

```rust
// CORRECT: Transfer guard ensures buffer outlives DMA
pub struct TransferGuard<'buf> {
    qtd: &'buf QueueTD,
    _buffer: &'buf [u8],  // Lifetime ensures buffer not freed
}

impl Drop for TransferGuard<'_> {
    fn drop(&mut self) {
        // Wait for DMA completion before allowing buffer drop
        while self.qtd.is_active() {
            core::hint::spin_loop();
        }
    }
}

// CORRECT: Non-Copy buffer handle prevents aliasing
pub struct DmaBuffer {
    ptr: NonNull<u8>,
    // NOT Copy or Clone
}

impl DmaBuffer {
    pub fn free(self, pool: &mut BufferPool) {
        pool.mark_free(self.pool_index);
        // `self` consumed, can't be used again
    }
}
```

### Unsafe Block Documentation

**All** unsafe blocks require detailed safety comments:

```rust
/// # Safety
///
/// Caller must ensure:
/// - Buffer remains valid until transfer completes (check is_active())
/// - Buffer in DMA-accessible memory (DTCM/OCRAM)
/// - Buffer is 32-byte aligned
/// - Buffer lifetime exceeds transfer duration
pub unsafe fn init_transfer(&self, buffer: *const u8, len: usize) {
    // Safety: Alignment validated by caller per contract above
    unsafe {
        core::ptr::write_volatile(self.buffer_addr.get(), buffer as u32);
    }
}
```

**Enforce globally:**
```rust
#![deny(unsafe_op_in_unsafe_fn)]
```

This forces explicit `unsafe {}` blocks inside unsafe functions, each with its own safety comment.
- Make sure examples can be linked into a hex file using rust-objcopy that produces file which can be directly flashed to the Teensy.
