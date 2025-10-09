# Dead Code Analysis - Remaining Optional Items

**Branch:** cleanup
**Status:** All critical issues resolved ✅

---

## Optional Cleanup (Non-Critical)

### 1. Unused Method: `get_qh()`
**File:** `src/ehci/transfer_executor.rs:176-181`

Immutable version never called; mutable `get_qh_mut()` is used instead. Safe to remove (6 LOC).

---

### 2. Unused Field: `device_address`
**File:** `src/transfer/isochronous.rs:86`

Currently has `#[allow(dead_code)]` annotation. Kept for defmt logging but could be removed if not actually logged.

---

### 3. Unused PHY Register Offsets
**File:** `src/phy.rs`

```rust
#[allow(dead_code)]
const USBPHY_TX_OFFSET: usize = 0x10;
#[allow(dead_code)]
const USBPHY_RX_OFFSET: usize = 0x20;
```

Reserved for future PHY calibration features. Can remove if never needed.

---

**Total impact if removed:** ~20 LOC
**Recommendation:** Keep as-is or clean up incrementally. No functional issues.
