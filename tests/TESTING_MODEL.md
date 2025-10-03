# Comprehensive Test Suite Plan for imxrt-usbh

## Current Test Status

### Existing Tests

- **Inline unit tests** (68 tests): In `src/` modules under `#[cfg(test)] mod tests`
  - `src/transfer/bulk.rs` (12 tests)
  - `src/transfer/interrupt.rs` (13 tests)
  - `src/transfer/isochronous.rs` (16 tests)
  - `src/enumeration.rs` (10 tests)
  - `src/dma.rs` (17 tests)
  - Plus: `ehci/qh.rs`, `ehci/qtd.rs`, `ehci/controller.rs`, `hub.rs`, `perf.rs`

- **Integration tests** (31+ tests): In `tests/` directory
  - `tests/integration.rs` - Multi-transfer scenarios (8 tests)
  - `tests/error_handling.rs` - Error recovery (11 tests)
  - `tests/resource_management.rs` - Pool lifecycle (12 tests)

- **Hardware tests**: `tests/hil.rs` - Hardware-in-the-loop (requires Teensy 4.1)

- **Test utilities**: `tests/common/` - Mock DMA buffers, USB descriptors

### Current Gaps

- ✅ **FIXED**: DMA buffer tests implemented (17 tests)
- ✅ **FIXED**: Transfer manager tests implemented (41 tests for all 3 types)
- ✅ **FIXED**: Safety verification tests implemented (DMA alignment, no aliasing)
- ✅ **FIXED**: Descriptor parsing tests implemented (10 tests)
- ✅ **FIXED**: Integration tests implemented (31+ tests across 3 files)
- ✅ **FIXED**: Error handling tests implemented (11 tests)
- ✅ **FIXED**: Resource management tests implemented (12 tests)
- ❌ No benchmarks or performance regression tests

---

## Recommended Test Suite Architecture

### 1. Unit Tests (no_std, no hardware required)

**Location:** `src/<module>/tests.rs` or `#[cfg(test)] mod tests` blocks

#### Core Data Structures

**✅ EHCI Structures** (`src/ehci/`) - EXISTING TESTS
- QueueHead creation, initialization, field manipulation
- QueueTD creation, token configuration, buffer setup
- Structure size and alignment verification
- Atomic field access patterns

**✅ DMA Buffers** (`src/dma/`) - **IMPLEMENTED** (17 tests)
- Buffer allocation/deallocation ✅
- Alignment verification (32-byte cache line) ✅
- Pool exhaustion handling ✅
- Buffer lifecycle (no leaks, no double-free) ✅
- Cache coherency operation verification ✅
- No aliasing verification ✅

**✅ Transfer Managers** - **IMPLEMENTED** (41 tests total)
- `BulkTransferManager` state transitions ✅ (12 tests)
- `InterruptTransferManager` periodic scheduling ✅ (13 tests)
- `IsochronousTransferManager` microframe timing ✅ (16 tests)
- Transfer submission and completion ✅
- Error handling and retry logic ✅
- Buffer lifecycle in transfer context ✅

**✅ Enumeration** - **IMPLEMENTED** (10 tests)
- Descriptor parsing (device, configuration, interface, endpoint) ✅
- Device class identification ✅
- String descriptor handling ✅
- Invalid descriptor rejection ✅

**✅ Error Handling** (`src/error.rs`)
- Error type creation and matching
- Result propagation
- Recovery action determination

#### Safety Verification

**✅ Bounds Checking** - **IMPLEMENTED**
- DMA address validation ✅
- Buffer size limits ✅
- Descriptor pool bounds ✅
- Array index validation ✅

**✅ Lifetime Safety** - **VERIFIED**
- Buffer not used after free ✅ (enforced by !Copy trait)
- Transfer not used after completion ✅ (state machine)
- No aliasing of DMA buffers ✅ (test_dma_buffer_no_aliasing)

---

### 2. Integration Tests (no_std, no hardware)

**Location:** `tests/` directory

#### ✅ Multi-Transfer Scenarios (`tests/integration.rs`) - **IMPLEMENTED**
- ✅ Bulk and interrupt transfer coexistence
- ✅ Bulk transfer with multiple packets
- ✅ Interrupt transfer periodic scheduling
- ✅ Isochronous transfer microframe alignment
- ✅ Transfer state coordination

#### ✅ Resource Management (`tests/resource_management.rs`) - **IMPLEMENTED**
- ✅ DMA buffer pool allocation/exhaustion
- ✅ Transfer pool management
- ✅ Concurrent transfer resource contention
- ✅ Memory leak detection
- ✅ Buffer reuse verification

#### ✅ Error Scenarios (`tests/error_handling.rs`) - **IMPLEMENTED**
- ✅ NAK handling and retry
- ✅ STALL condition recovery
- ✅ Timeout handling
- ✅ Buffer overflow detection
- ✅ Transaction error recovery
- ✅ Independent error handling across transfers

---

### 3. Property-Based Tests (Optional, with `proptest`)

**Location:** `tests/property/`

- ❌ DMA buffer allocation always returns aligned addresses
- ❌ Transfer state transitions follow valid state machine
- ❌ Pool allocation/free balance (no leaks)
- ❌ Descriptor parsing handles malformed input safely

---

### 4. Hardware-in-the-Loop (HIL) Tests

**Location:** `tests/hil/` (requires hardware, `std` feature)

#### Device Enumeration
- ❌ Basic device detection and enumeration
- ❌ Multi-port enumeration
- ❌ Hub enumeration with downstream devices
- ❌ Hot-plug detection
- ❌ Disconnect detection

#### Real Transfer Tests
- ❌ Control transfers with real device (GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION)
- ❌ Bulk transfers with USB flash drive (read/write blocks)
- ❌ Interrupt transfers with HID keyboard (read reports)
- ❌ Isochronous transfers with webcam/audio device

#### Device Class Specific
- ❌ HID keyboard (boot protocol)
- ❌ HID mouse
- ❌ Mass storage (BOT protocol)
- ❌ MIDI device
- ❌ CDC serial device

#### Stress Tests
- ❌ Continuous bulk transfers (sustained throughput)
- ❌ Rapid connect/disconnect cycles
- ❌ Multiple concurrent transfers
- ❌ Transfer cancellation
- ❌ Buffer pool exhaustion recovery

#### Compliance Tests
- ❌ USB 2.0 specification timing requirements
- ❌ Port reset timing (10-20ms)
- ❌ Device address assignment
- ❌ Configuration switching
- ❌ Endpoint halt/clear

---

### 5. Mock/Simulator Tests (with mock EHCI hardware)

**Location:** `tests/mock/`

- ❌ Mock EHCI controller for software testing
- ❌ Simulated USB device responses
- ❌ Controlled error injection (timeouts, NAKs, STALLs)
- ❌ Deterministic timing tests
- ❌ Edge case scenarios without real hardware

---

### 6. Documentation Tests

**Location:** Doc comments in `src/**/*.rs`

- ❌ Example code in documentation compiles
- ❌ Usage examples are correct
- ❌ API examples demonstrate best practices

---

### 7. Performance/Benchmark Tests

**Location:** `benches/` (requires `criterion` or custom framework)

- ❌ Transfer submission latency
- ❌ Bulk transfer throughput
- ❌ Interrupt transfer latency
- ❌ Enumeration time
- ❌ Memory allocation overhead
- ❌ Cache operation performance

---

### 8. Safety Audit Tests

**Location:** `tests/safety/`

- ❌ All unsafe blocks have safety documentation
- ❌ No undefined behavior in unsafe code
- ❌ No data races (verify atomic ordering)
- ❌ No aliasing violations
- ❌ Miri compatibility (detect UB)

---

## Specific Missing Test Cases

### Critical DMA Safety Tests

```rust
#[test]
fn test_dma_buffer_no_aliasing() {
    // Verify two allocations don't overlap
    let mut pool = DmaBufferPool::new();
    let buf1 = pool.alloc(512).unwrap();
    let buf2 = pool.alloc(512).unwrap();

    // Ensure buffers don't overlap
    let addr1 = buf1.as_ptr() as usize;
    let addr2 = buf2.as_ptr() as usize;
    assert!(addr1 + 512 <= addr2 || addr2 + 512 <= addr1);
}

#[test]
fn test_dma_buffer_freed_not_accessible() {
    // Verify use-after-free prevented by type system
    let mut pool = DmaBufferPool::new();
    let buf = pool.alloc(512).unwrap();
    pool.free(buf);
    // buf is now moved, cannot access it
    // This should not compile: buf.as_slice();
}

#[test]
fn test_cache_coherency_operations() {
    // Verify prepare_for_device/prepare_for_cpu work correctly
    let mut pool = DmaBufferPool::new();
    let mut buf = pool.alloc(512).unwrap();

    // Write data
    buf.as_mut_slice()[0] = 0xAA;

    // Prepare for DMA write
    buf.prepare_for_device();

    // Simulate DMA read
    buf.prepare_for_cpu();

    // Verify data integrity
    assert_eq!(buf.as_slice()[0], 0xAA);
}
```

### Transfer Manager Tests

```rust
#[test]
fn test_bulk_transfer_lifecycle() {
    // Submit → Active → Complete → Buffer returned
    let mut mgr = BulkTransferManager::new();
    let mut pool = DmaBufferPool::new();

    let buffer = pool.alloc(512).unwrap();
    let transfer_id = mgr.submit(
        Direction::In,
        1, // device address
        0x81, // endpoint
        64, // max packet size
        buffer,
        1000 // timeout
    ).unwrap();

    // Verify transfer is active
    let transfer = mgr.get_transfer(transfer_id).unwrap();
    assert_eq!(transfer.state(), BulkState::Active);

    // Simulate completion...
    // Verify buffer can be retrieved
}

#[test]
fn test_interrupt_transfer_periodic() {
    // Verify periodic scheduling at correct interval
    let mut mgr = InterruptTransferManager::new();
    // Test that transfers are scheduled at poll_interval
}

#[test]
fn test_transfer_error_recovery() {
    // NAK → retry, STALL → error, timeout → retry
    let mut mgr = BulkTransferManager::new();
    // Inject NAK error, verify retry
    // Inject STALL error, verify failure
    // Inject timeout, verify retry with backoff
}
```

### Enumeration Tests

```rust
#[test]
fn test_parse_device_descriptor() {
    // Valid descriptor → correct fields
    let desc_bytes = [
        0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
        0x83, 0x04, 0x40, 0x00, 0x00, 0x01, 0x01, 0x02,
        0x03, 0x01,
    ];

    let desc = DeviceDescriptor::parse(&desc_bytes).unwrap();
    assert_eq!(desc.b_length, 18);
    assert_eq!(desc.b_descriptor_type, 1);
    assert_eq!(desc.bcd_usb, 0x0200);
}

#[test]
fn test_parse_invalid_descriptor() {
    // Malformed data → error (no panic)
    let invalid_bytes = [0x00, 0x00, 0x00];
    let result = DeviceDescriptor::parse(&invalid_bytes);
    assert!(result.is_err());
}

#[test]
fn test_enumerate_sequence() {
    // Reset → GET_DESCRIPTOR(8) → SET_ADDRESS → GET_DESCRIPTOR(18)
    // Mock controller to verify correct sequence
}
```

---

## Test Infrastructure Needed

### Test Utilities (`tests/common/`)
- Mock EHCI register interface
- USB descriptor builders
- Test device simulators
- Timing helpers
- Assertion helpers for async operations

### CI/CD Integration
- Unit tests run on every commit
- HIL tests run on dedicated hardware
- Coverage reporting
- Performance regression detection

### Test Documentation
- README explaining how to run different test types
- Hardware setup guide for HIL tests
- Troubleshooting guide

---

## Priority Recommendations

### High Priority (Implement First)
1. **DMA buffer safety tests** (prevent memory corruption)
2. **Transfer manager state machine tests**
3. **Descriptor parsing tests** (security critical)
4. **Resource leak detection tests**

### Medium Priority
5. Mock EHCI controller for software-only testing
6. Property-based tests for invariants
7. Basic HIL tests with common devices

### Low Priority
8. Performance benchmarks
9. Compliance test suite
10. Stress tests

---

## Running Tests

### ARM Target Only

All tests compile for the ARM Cortex-M7 target (`thumbv7em-none-eabihf`) as this library directly accesses i.MX RT1062 hardware registers.

### Running Unit Tests

```bash
# Build and verify inline unit tests compile for ARM
cargo test --lib --no-run --target thumbv7em-none-eabihf

# Run integration tests that don't require hardware (compiles only)
cargo test --test integration --no-run --target thumbv7em-none-eabihf
cargo test --test error_handling --no-run --target thumbv7em-none-eabihf
cargo test --test resource_management --no-run --target thumbv7em-none-eabihf
```

### Running Hardware-in-the-Loop Tests

Hardware tests require actual Teensy 4.1 hardware and are marked with `#[ignore]`:

```bash
# Run HIL tests on device (requires test runner on Teensy 4.1)
cargo test --test hil --features std --target thumbv7em-none-eabihf -- --ignored
```

### Implementation Summary

**99+ tests implemented across:**
- 68 inline unit tests in `src/` modules
- 31+ integration tests in `tests/` directory
- Hardware tests in `tests/hil.rs`

See `tests/TESTS_IMPLEMENTED.md` for complete details.

### Future Plan
```bash
# Performance benchmarks
cargo bench

# Safety audit with Miri (may not support embedded targets)
cargo +nightly miri test
```

---

## Test Coverage Goals

- **Unit Tests:** 80%+ code coverage
- **Integration Tests:** All public APIs tested
- **HIL Tests:** All device classes tested with real hardware
- **Safety Tests:** 100% of unsafe blocks verified
- **Documentation Tests:** All examples compile and run

---

## Notes

- Tests should be fast (unit tests < 100ms each)
- Tests should be deterministic (no flaky tests)
- Tests should be isolated (no shared state)
- Tests should be documented (clear purpose and expectations)
- Tests should fail fast (clear error messages)
