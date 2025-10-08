# Dead Code Analysis Report
**Project:** imxrt-usbh - USB Host Driver for i.MX RT1062 (Teensy 4.x)
**Date:** 2025-10-08
**Branch:** cleanup
**Total LOC:** 12,847 (Rust source)

---

## Executive Summary

This report consolidates findings from three specialized reviews:
- **@refactoring-specialist** - Code duplication and refactoring opportunities
- **@simplify** - Over-engineering and complexity analysis
- **@teensy-expert** - Hardware-specific correctness and dead code

### Key Findings
- **0 Critical Safety Issues** - Hardware implementation is excellent
- **11 Dead Code Items** - Can safely remove 100-150 LOC
- **3 Missing Memory Barriers** - High priority fixes needed
- **800-1500 LOC Duplication** - Refactoring opportunities identified
- **4 Compilation Errors** - Hub feature needs fixes

---

## 1. Truly Dead Code (REMOVE)

### 1.1 Unused Method: `get_qh()`
**File:** `src/ehci/transfer_executor.rs:176-181`
**Status:** ❌ Remove
**LOC:** 6 lines

```rust
fn get_qh(&self, handle: &QhHandle) -> Result<&QueueHead> {
    if handle.index >= 32 {
        return Err(UsbError::InvalidParameter);
    }
    Ok(unsafe { &QH_POOL[handle.index] })
}
```

**Justification:**
- Method is never called in codebase
- Mutable version `get_qh_mut()` exists and IS used
- Not referenced in examples or tests
- Originally intended for debugging but superseded by other inspection methods

**Action:** Delete method entirely

---

### 1.2 Unused Field: `device_address`
**File:** `src/transfer/isochronous.rs:86`
**Status:** ❌ Remove
**LOC:** 1 line

```rust
pub struct IsochronousTransfer {
    state: AtomicU8,
    direction: Direction,
    device_address: u8,  // ← Never read
    endpoint: u8,
    // ...
}
```

**Justification:**
- Field is set during construction but never read
- Device address is tracked by QueueHead, not transfer
- Adds 1 byte to struct size unnecessarily

**Action:** Remove field and constructor parameter

---

### 1.3 Dead Function: `enable_interrupts()`
**File:** `src/phy.rs:355-362`
**Status:** ❌ Remove (BUGGY)
**LOC:** 8 lines

```rust
pub fn enable_interrupts(&mut self) {
    unsafe {
        let ctrl_reg = (self.phy_base + 0x00) as *mut u32;  // WRONG OFFSET!
        let mut ctrl = core::ptr::read_volatile(ctrl_reg);  // MISSING BARRIERS!
        ctrl |= USBPHY_CTRL_HOSTDISCONDETECT_IRQ;
        core::ptr::write_volatile(ctrl_reg, ctrl);
    }
}
```

**Justification:**
- **Function is buggy**: Uses `0x00` (PWD register) instead of `USBPHY_CTRL_OFFSET` (0x30)
- Missing memory barriers around MMIO access
- Never called - interrupt configuration already done in `configure_host_mode()`
- @teensy-expert flagged as dead code with hardware bugs

**Action:** Delete function entirely

---

### 1.4 Unused PHY Register Offsets
**File:** `src/phy.rs:37-39`
**Status:** ❌ Remove
**LOC:** 4 lines (including annotations)

```rust
#[allow(dead_code)]
const USBPHY_TX_OFFSET: usize = 0x10; // TX register
#[allow(dead_code)]
const USBPHY_RX_OFFSET: usize = 0x20; // RX register
```

**Justification:**
- These registers are for PHY tuning/calibration
- Current driver doesn't implement PHY tuning
- Not needed for standard USB operation on Teensy 4.x
- If tuning is needed later, they can be re-added

**Action:** Remove both constants

---

### 1.5 Unused Variables in Transfer Modules
**Files:** Multiple
**Status:** ❌ Remove
**LOC:** 9 unused variable declarations

| File | Line | Variable | Context |
|------|------|----------|---------|
| `src/transfer/bulk.rs` | 197 | `transfer_size` | Calculated but not used |
| `src/transfer/bulk.rs` | 202 | `toggle` | Assigned but not used |
| `src/transfer/interrupt.rs` | 217 | `transfer_size` | Calculated but not used |
| `src/transfer/interrupt.rs` | 218 | `toggle` | Assigned but not used |
| `src/transfer/interrupt.rs` | 536 | `e` | Error value ignored |
| `src/transfer/interrupt.rs` | 551 | `e` | Error value ignored |
| `src/transfer/isochronous.rs` | 333 | `transaction_buffer_addr` | Calculated but not used |
| `src/transfer/isochronous.rs` | 654 | `e` | Error value ignored |
| `src/transfer/isochronous.rs` | 669 | `e` | Error value ignored |

**Action:** Remove all unused variables or prefix with `_` if intentional placeholders

---

## 2. Keep (False Positives / Future-Ready)

### 2.1 Periodic Schedule Type Constants
**File:** `src/ehci/periodic.rs:13-19`
**Status:** ✅ KEEP
**LOC:** 4 lines

```rust
#[allow(dead_code)]
const TYPE_ITD: u32 = 0 << 1;      // Isochronous Transfer Descriptor
#[allow(dead_code)]
const TYPE_SITD: u32 = 2 << 1;     // Split Isochronous Transfer Descriptor
#[allow(dead_code)]
const TYPE_FSTN: u32 = 3 << 1;     // Frame Span Traversal Node
```

**Justification (from @teensy-expert):**
- **TYPE_ITD**: Required for USB audio/video (high-speed isochronous)
- **TYPE_SITD**: Required for USB audio/video through full-speed hubs
- **TYPE_FSTN**: For periodic schedule optimization
- Driver already has `src/transfer/isochronous.rs` (1,050 LOC) for future implementation
- Removing would require re-adding when isochronous support is implemented

**Action:** Keep with `#[allow(dead_code)]`

---

### 2.2 PHY Register Offset: USBPHY_PWD_OFFSET
**File:** `src/phy.rs:35-36`
**Status:** ✅ KEEP (and fix usage)
**LOC:** 2 lines

```rust
#[allow(dead_code)]
const USBPHY_PWD_OFFSET: usize = 0x00; // Power-Down register
```

**Justification:**
- Currently used via hardcoded `0x00` at line 201
- Should use this constant instead for maintainability

**Action:** Keep constant, replace hardcoded `0x00` with `USBPHY_PWD_OFFSET`

---

### 2.3 PHY CTRL SET/CLR Offsets
**File:** `src/phy.rs:43-45`
**Status:** ✅ KEEP
**LOC:** 4 lines

```rust
#[allow(dead_code)]
const USBPHY_CTRL_SET_OFFSET: usize = 0x34;
#[allow(dead_code)]
const USBPHY_CTRL_CLR_OFFSET: usize = 0x38;
```

**Justification (from @teensy-expert):**
- SET/CLR registers enable atomic bit operations (no read-modify-write)
- Future optimization: replace current RMW pattern with atomic SET/CLR writes
- Eliminates race conditions and reduces bus transactions from 3 to 1

**Action:** Keep for future optimization

---

### 2.4 Control Transfer State Variants
**File:** `src/transfer/control.rs:102,117,120`
**Status:** ✅ KEEP
**LOC:** 3 lines

```rust
#[allow(dead_code)]
const STATE_SETUP: u8 = 1;
#[allow(dead_code)]
const STATE_DATA: u8 = 2;
#[allow(dead_code)]
const STATE_STATUS: u8 = 3;
```

**Justification:**
- Control transfers have 3 phases: Setup, Data (optional), Status
- Currently simplified implementation doesn't track phases separately
- Needed if implementing phase-aware error recovery or debugging

**Action:** Keep with `#[allow(dead_code)]`

---

## 3. Compilation Errors (MUST FIX)

### 3.1 Hub Feature: Arithmetic Overflow
**File:** `src/hub.rs:491, 543, 580`
**Status:** 🚨 CRITICAL FIX
**Severity:** Compilation Error

```rust
// Line 491 (and 543, 580):
(port >> 8) as u8, // wIndex - OVERFLOW! port is u8, can't shift by 8
```

**Problem:**
- `port` parameter is `u8` (0-255)
- Attempting to shift right by 8 bits causes arithmetic overflow
- Code expects `port` to be `u16` for multi-byte port index

**Fix:**
```rust
// Change parameter type from u8 to u16:
pub fn get_port_status(
    &mut self,
    port: u16,  // Changed from u8
    executor: &mut TransferExecutor,
    memory_pool: &mut UsbMemoryPool,
) -> Result<(PortStatus, PortChange)> {
    // Now shift works:
    (port & 0xFF) as u8,     // Low byte
    (port >> 8) as u8,       // High byte
}
```

**Applies to 3 functions:**
- `get_port_status()` at line 475
- `set_port_feature()` at line 521
- `clear_port_feature()` at line 558

---

### 3.2 Hub Feature: Borrow Checker Error
**File:** `src/hub.rs:932-943`
**Status:** 🚨 CRITICAL FIX
**Severity:** Compilation Error

```rust
self.active_splits.retain(|split| {  // split is &SplitTransaction
    match split.state {
        SplitState::CompleteSplit => {
            if split.cs_retries >= 3 {
                split.cs_retries += 1;  // ERROR: Can't mutate through & reference
                false
            }
        }
    }
});
```

**Fix:**
```rust
// Option 1: Use retain_mut (requires heapless 0.8.1+)
self.active_splits.retain_mut(|split| {  // split is &mut SplitTransaction
    match split.state {
        SplitState::CompleteSplit => {
            if split.cs_retries >= 3 {
                split.cs_retries += 1;  // OK: mutable reference
                false
            } else {
                true
            }
        }
        _ => true,
    }
});

// Option 2: Iterate and build new Vec (more explicit)
let mut new_splits = heapless::Vec::new();
for mut split in self.active_splits.drain(..) {
    match split.state {
        SplitState::CompleteSplit if split.cs_retries >= 3 => {
            split.cs_retries += 1;
            // Don't add to new_splits (filtered out)
        }
        _ => {
            let _ = new_splits.push(split);
        }
    }
}
self.active_splits = new_splits;
```

---

### 3.3 Hub Feature: Unused Parameters
**File:** `src/hub.rs`
**Status:** ⚠️ Warning
**Severity:** Compiler Warning (not error)

```rust
// Line 329:
pub fn interrupt(interval: u8) -> Self {  // interval unused

// Lines 530, 567:
memory_pool: &mut UsbMemoryPool,  // parameter unused
```

**Fix:**
```rust
// Prefix with underscore to indicate intentionally unused:
pub fn interrupt(_interval: u8) -> Self {

fn set_port_feature(..., _memory_pool: &mut UsbMemoryPool) -> Result<()> {
```

---

## 4. Missing Memory Barriers (HIGH PRIORITY)

### 4.1 Recovery Module: Port Reset
**File:** `src/recovery.rs:218-231`
**Status:** 🔴 HIGH PRIORITY
**Severity:** Hardware Correctness Issue

**Current Code (INCORRECT):**
```rust
let mut portsc = core::ptr::read_volatile(portsc_addr);  // Missing dmb()
portsc |= 1 << 8; // Port Reset
core::ptr::write_volatile(portsc_addr, portsc);  // Missing dsb()
```

**Fixed Code:**
```rust
cortex_m::asm::dmb();
let mut portsc = core::ptr::read_volatile(portsc_addr);
cortex_m::asm::dmb();
portsc |= 1 << 8; // Port Reset
cortex_m::asm::dmb();
core::ptr::write_volatile(portsc_addr, portsc);
cortex_m::asm::dsb();  // Critical: ensure write completes before continuing
```

**Why Critical:**
- ARM Cortex-M7 has weakly-ordered memory
- Without barriers, next instruction may execute before hardware sees write
- Can cause "works with debug prints, fails without" bugs

---

### 4.2 RTIC Interrupt: USBSTS Reads
**File:** `src/rtic/interrupt.rs:86-89`
**Status:** 🔴 HIGH PRIORITY

**Current Code:**
```rust
let status = core::ptr::read_volatile(usbsts_addr);  // Missing barriers
```

**Fixed Code:**
```rust
cortex_m::asm::dmb();
let status = core::ptr::read_volatile(usbsts_addr);
cortex_m::asm::dmb();
```

**Also applies to:**
- Lines 198-202 (second USBSTS read)
- Lines 384-388 (PORTSC read)

---

## 5. Refactoring Opportunities

### 5.1 Transfer Module Duplication (800-1000 LOC)
**Priority:** HIGH
**Effort:** 4-6 hours
**LOC Saved:** 800-1000

All four transfer types share:
- 9 identical methods (`get_bytes_from_qtd`, `cleanup`, `is_complete`, etc.)
- Common fields (`state`, `data_toggle`, `bytes_transferred`, `qh_handle`, `qtd_handle`)
- Similar state machines

**Recommendation:** Extract `TransferBase` struct in `src/transfer/common.rs`

---

### 5.2 Type-State Pattern Boilerplate (150 LOC)
**Priority:** MEDIUM
**Effort:** 1-2 hours
**LOC Saved:** 150

EHCI controller type-state pattern forces manual field copying across states:
- `Uninitialized → Initialized → Running`
- Each transition copies 5-6 fields manually
- Examples don't leverage compile-time guarantees (use runtime checks anyway)

**Recommendation:** Replace with runtime `ControllerState` enum

---

### 5.3 Recovery Module Over-Engineering (300 LOC)
**Priority:** LOW
**Effort:** 2 hours
**LOC Saved:** 300

`RecoveryCoordinator` is unused by examples:
- Complex state machine
- Recovery logic already in `UsbError::retry_delay_ms()`
- 496 LOC for functionality already available

**Recommendation:** Delete module, keep 20-line retry helper

---

## 6. Summary & Action Plan

### Immediate Actions (Before Merge)

**Critical Fixes:**
1. ✅ Fix hub arithmetic overflow (3 locations) - 10 minutes
2. ✅ Fix hub borrow checker error - 5 minutes
3. ✅ Add memory barriers to recovery.rs - 5 minutes
4. ✅ Add memory barriers to rtic/interrupt.rs - 5 minutes

**Dead Code Removal:**
5. ✅ Remove `get_qh()` method - 1 minute
6. ✅ Remove `device_address` field from IsochronousTransfer - 2 minutes
7. ✅ Remove `enable_interrupts()` function - 1 minute
8. ✅ Remove USBPHY_TX/RX_OFFSET constants - 1 minute
9. ✅ Fix unused variables (9 locations) - 10 minutes

**Total Time:** ~40 minutes

### Future Refactoring (Separate PRs)

1. **Transfer Base Extraction** - 4-6 hours, -800 LOC
2. **Type-State Simplification** - 1-2 hours, -150 LOC
3. **Recovery Module Removal** - 2 hours, -300 LOC

**Potential Total Reduction:** -1,250 to -1,450 LOC (9-10% of codebase)

---

## 7. Metrics

### Code Quality Before/After

| Metric | Before | After (Immediate) | After (Full Refactor) |
|--------|--------|-------------------|----------------------|
| Total LOC | 12,847 | 12,697 (-150) | 11,400-11,600 (-1,250) |
| Dead Code Items | 11 | 0 | 0 |
| Compilation Errors | 4 | 0 | 0 |
| Memory Barrier Issues | 3 | 0 | 0 |
| Code Duplication | High | High | Low |
| `#[allow(dead_code)]` | 11 items | 7 items (justified) | 7 items |

### Safety Assessment

**Before:**
- ✅ Cache coherency: Excellent
- ⚠️ Memory barriers: 3 missing
- ✅ DMA lifetime safety: Excellent
- ✅ Interrupt safety: Proper

**After:**
- ✅ All critical hardware patterns correct
- ✅ Production-ready for Teensy 4.x
- ✅ Zero safety compromises

---

## 8. References

**Agent Reviews:**
- @refactoring-specialist - Duplication analysis
- @simplify - Complexity assessment
- @teensy-expert - Hardware correctness review

**Related Files:**
- `CLAUDE.md` - Project coding standards
- `tests/TESTS_IMPLEMENTED.md` - Test coverage
- `examples/README.md` - Example usage patterns

---

**Report Generated:** 2025-10-08
**Next Review:** After major refactoring (Transfer Base Extraction)
