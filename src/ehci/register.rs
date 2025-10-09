//! Optimized register access patterns for EHCI
//!
//! Implements efficient, cache-friendly register access with proper memory ordering
//! for ARM Cortex-M7 weakly-ordered memory model.

use core::cell::UnsafeCell;
use core::ptr::{read_volatile, write_volatile};

/// Optimized register access wrapper with proper memory ordering
///
/// This provides more efficient register access than AtomicU32 for MMIO operations
/// while maintaining memory safety through proper barriers.
#[repr(transparent)]
pub struct Register<T> {
    value: UnsafeCell<T>,
}

unsafe impl<T> Send for Register<T> where T: Send {}
unsafe impl<T> Sync for Register<T> where T: Sync {}

impl Register<u32> {
    /// Create a new register wrapper
    pub const fn new(value: u32) -> Self {
        Self {
            value: UnsafeCell::new(value),
        }
    }

    /// Read register with acquire semantics for status/data reads
    #[inline(always)]
    pub fn read(&self) -> u32 {
        unsafe {
            cortex_m::asm::dmb(); // Data Memory Barrier before read
            let value = read_volatile(self.value.get());
            cortex_m::asm::dmb(); // Data Memory Barrier after read
            value
        }
    }

    /// Read register without barriers for capability registers (read-only, no side effects)
    #[inline(always)]
    pub fn read_relaxed(&self) -> u32 {
        unsafe { read_volatile(self.value.get()) }
    }

    /// Write register with release semantics for control writes
    #[inline(always)]
    pub fn write(&self, value: u32) {
        unsafe {
            cortex_m::asm::dmb(); // Data Memory Barrier before write
            write_volatile(self.value.get(), value);
            cortex_m::asm::dsb(); // Data Synchronization Barrier ensures write completes
        }
    }

    /// Read-modify-write operation with full memory barriers
    ///
    /// Uses strict barrier ordering for ARM Cortex-M7 weakly-ordered memory:
    /// - DMB before read: ensures prior operations complete
    /// - DMB after read: ensures read completes before using value
    /// - DMB before write: ensures computation completes before write
    /// - DSB after write: ensures write completes before continuing
    #[inline(always)]
    pub fn modify<F>(&self, f: F)
    where
        F: FnOnce(u32) -> u32,
    {
        unsafe {
            cortex_m::asm::dmb();
            let current = read_volatile(self.value.get());
            cortex_m::asm::dmb(); // ensure read completes before using value
            let new_value = f(current);
            cortex_m::asm::dmb(); // ensure computation completes before write
            write_volatile(self.value.get(), new_value);
            cortex_m::asm::dsb();
        }
    }

    /// Atomic set bits operation
    #[inline(always)]
    pub fn set_bits(&self, mask: u32) {
        self.modify(|v| v | mask);
    }

    /// Atomic clear bits operation
    #[inline(always)]
    pub fn clear_bits(&self, mask: u32) {
        self.modify(|v| v & !mask);
    }

    /// Write-1-to-clear operation for status registers
    #[inline(always)]
    pub fn write_1_to_clear(&self, mask: u32) {
        self.write(mask);
    }
}

// === Raw Register Operations for Dynamic Addresses ===

/// Validate that an address is within known i.MX RT1062 MMIO regions
///
/// # Returns
/// `true` if address is in a known peripheral region, `false` otherwise
///
/// # Known MMIO Regions (from i.MX RT1060 Reference Manual Ch. 2):
/// - 0x4000_0000 - 0x400F_FFFF: AIPS-1 (GPIO, Timers, UARTs, etc.)
/// - 0x4010_0000 - 0x401F_FFFF: AIPS-2 (USB, USDHC, ENET, etc.)
/// - 0x4020_0000 - 0x403F_FFFF: AIPS-3 (FlexSPI, SEMC config)
/// - 0xE000_0000 - 0xE00F_FFFF: ARM Cortex-M7 system peripherals
#[inline]
const fn is_valid_mmio_address(addr: usize) -> bool {
    matches!(addr,
        0x4000_0000..=0x400F_FFFF  // AIPS-1
        | 0x4010_0000..=0x401F_FFFF  // AIPS-2 (includes USB at 0x402E_xxxx)
        | 0x4020_0000..=0x403F_FFFF  // AIPS-3
        | 0xE000_0000..=0xE00F_FFFF  // ARM Cortex-M7 peripherals
    )
}

/// Safely read a register at a raw address with memory barriers
///
/// # Safety
///
/// Caller must ensure address points to a valid MMIO register.
/// This function validates the address is in a known MMIO region in debug builds.
///
/// # Panics
///
/// In debug builds, panics if address is not in a known MMIO region
#[inline(always)]
pub unsafe fn read_register_at(addr: *const u32) -> u32 {
    debug_assert!(
        is_valid_mmio_address(addr as usize),
        "Invalid MMIO address: {:#x}",
        addr as usize
    );
    unsafe {
        cortex_m::asm::dmb();
        let value = core::ptr::read_volatile(addr);
        cortex_m::asm::dmb();
        value
    }
}

/// Safely write a register at a raw address with memory barriers
///
/// # Safety
///
/// Caller must ensure address points to a valid MMIO register.
/// This function validates the address is in a known MMIO region in debug builds.
///
/// # Panics
///
/// In debug builds, panics if address is not in a known MMIO region
#[inline(always)]
pub unsafe fn write_register_at(addr: *mut u32, value: u32) {
    debug_assert!(
        is_valid_mmio_address(addr as usize),
        "Invalid MMIO address: {:#x}",
        addr as usize
    );
    unsafe {
        cortex_m::asm::dmb();
        core::ptr::write_volatile(addr, value);
        cortex_m::asm::dsb();
    }
}

/// Safely modify a register at a raw address with full memory barriers
///
/// Uses strict barrier ordering for ARM Cortex-M7 weakly-ordered memory:
/// - DMB before read: ensures prior operations complete
/// - DMB after read: ensures read completes before using value
/// - DMB before write: ensures computation completes before write
/// - DSB after write: ensures write completes before continuing
///
/// # Safety
///
/// Caller must ensure address points to a valid MMIO register.
/// This function validates the address is in a known MMIO region in debug builds.
///
/// # Panics
///
/// In debug builds, panics if address is not in a known MMIO region
#[inline(always)]
pub unsafe fn modify_register_at<F>(addr: *mut u32, f: F)
where
    F: FnOnce(u32) -> u32,
{
    debug_assert!(
        is_valid_mmio_address(addr as usize),
        "Invalid MMIO address: {:#x}",
        addr as usize
    );
    unsafe {
        cortex_m::asm::dmb();
        let current = core::ptr::read_volatile(addr);
        cortex_m::asm::dmb();
        let new_value = f(current);
        cortex_m::asm::dmb();
        core::ptr::write_volatile(addr, new_value);
        cortex_m::asm::dsb();
    }
}

/// Safely set bits in a register at a raw address
///
/// # Safety
///
/// Caller must ensure address points to a valid MMIO register.
/// This function validates the address is in a known MMIO region in debug builds.
///
/// # Panics
///
/// In debug builds, panics if address is not in a known MMIO region
#[inline(always)]
pub unsafe fn set_bits_at(addr: *mut u32, mask: u32) {
    unsafe {
        modify_register_at(addr, |v| v | mask);
    }
}

/// Safely clear bits in a register at a raw address
///
/// # Safety
///
/// Caller must ensure address points to a valid MMIO register.
/// This function validates the address is in a known MMIO region in debug builds.
///
/// # Panics
///
/// In debug builds, panics if address is not in a known MMIO region
#[inline(always)]
pub unsafe fn clear_bits_at(addr: *mut u32, mask: u32) {
    unsafe {
        modify_register_at(addr, |v| v & !mask);
    }
}

/// Timeout handling for register operations
pub struct RegisterTimeout {
    start_cycles: u32,
    timeout_cycles: u32,
}

impl RegisterTimeout {
    /// Create new timeout with duration in microseconds
    pub fn new_us(timeout_us: u32) -> Self {
        Self {
            start_cycles: cortex_m::peripheral::DWT::cycle_count(),
            timeout_cycles: crate::timing::us_to_cycles(timeout_us),
        }
    }

    /// Create new timeout with duration in milliseconds
    pub fn new_ms(timeout_ms: u32) -> Self {
        Self::new_us(timeout_ms * 1000)
    }

    /// Check if timeout has elapsed
    #[inline(always)]
    pub fn is_expired(&self) -> bool {
        let current = cortex_m::peripheral::DWT::cycle_count();
        current.wrapping_sub(self.start_cycles) >= self.timeout_cycles
    }

    /// Wait for condition with timeout
    pub fn wait_for<F>(&self, mut condition: F) -> Result<(), crate::error::UsbError>
    where
        F: FnMut() -> bool,
    {
        while !condition() {
            if self.is_expired() {
                return Err(crate::error::UsbError::Timeout);
            }
            // Small delay to reduce bus contention
            cortex_m::asm::delay(10);
        }
        Ok(())
    }

    /// Wait for a register bit to clear (become 0)
    ///
    /// # Safety
    ///
    /// Caller must ensure reg points to a valid MMIO register.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::ehci::RegisterTimeout;
    /// # let usbcmd_ptr = 0x402E_0140 as *const u32;
    /// // Wait for HC Reset bit to clear
    /// unsafe {
    ///     RegisterTimeout::new_us(10_000).wait_for_bit_clear(usbcmd_ptr, 0x02)?;
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub unsafe fn wait_for_bit_clear(
        &self,
        reg: *const u32,
        mask: u32,
    ) -> Result<(), crate::error::UsbError> {
        self.wait_for(|| unsafe {
            cortex_m::asm::dmb();
            let val = core::ptr::read_volatile(reg);
            cortex_m::asm::dmb();
            (val & mask) == 0
        })
    }

    /// Wait for a register bit to be set (become 1)
    ///
    /// # Safety
    ///
    /// Caller must ensure reg points to a valid MMIO register.
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::ehci::RegisterTimeout;
    /// # let usbsts_ptr = 0x402E_0144 as *const u32;
    /// // Wait for HC Halted bit to be set
    /// unsafe {
    ///     RegisterTimeout::new_us(2_000).wait_for_bit_set(usbsts_ptr, 0x1000)?;
    /// }
    /// # Ok::<(), imxrt_usbh::UsbError>(())
    /// ```
    pub unsafe fn wait_for_bit_set(
        &self,
        reg: *const u32,
        mask: u32,
    ) -> Result<(), crate::error::UsbError> {
        self.wait_for(|| unsafe {
            cortex_m::asm::dmb();
            let val = core::ptr::read_volatile(reg);
            cortex_m::asm::dmb();
            (val & mask) != 0
        })
    }
}

// === RAL Wrapper Macros with ARM Cortex-M7 Memory Barriers ===

/// Safe register read with ARM Cortex-M7 memory barriers
///
/// Wraps `imxrt_ral::read_reg!` with proper memory barriers for weakly-ordered
/// ARM Cortex-M7 memory. Ensures hardware register reads are ordered correctly
/// relative to other memory operations.
///
/// # Memory Ordering
///
/// - **DMB before read**: Ensures all prior memory operations complete before reading
/// - **DMB after read**: Ensures register read completes before using the value
///
/// # Why This Is Needed
///
/// ARM Cortex-M7 has weakly-ordered memory. Without barriers, the CPU or hardware
/// can reorder register reads relative to other operations, causing:
/// - Reading stale values
/// - Race conditions with DMA
/// - Incorrect hardware state observations
///
/// # Example
///
/// ```no_run
/// # use imxrt_ral as ral;
/// let analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
/// let pll_value = safe_read_reg!(ral::ccm_analog, analog, PLL_USB1);
/// ```
///
/// # See Also
///
/// - ARM Cortex-M7 Technical Reference Manual, Section 8.3 (Memory Ordering)
/// - i.MX RT1060 Reference Manual, Chapter 2 (Memory Map)
#[macro_export]
macro_rules! safe_read_reg {
    ($peripheral:path, $instance:expr, $register:ident $(, $field:ident)* ) => {{
        cortex_m::asm::dmb(); // Data Memory Barrier before read
        let val = imxrt_ral::read_reg!($peripheral, $instance, $register $(, $field)*);
        cortex_m::asm::dmb(); // Data Memory Barrier after read
        val
    }};
}

/// Safe register modify with ARM Cortex-M7 memory barriers
///
/// Wraps `imxrt_ral::modify_reg!` with proper memory barriers for weakly-ordered
/// ARM Cortex-M7 memory. Ensures read-modify-write operations are atomic and
/// ordered correctly relative to other memory operations.
///
/// # Memory Ordering
///
/// - **DMB before**: Ensures all prior operations complete before read-modify-write
/// - **DSB after**: Ensures write completes before continuing (stronger than DMB)
///
/// # Why DSB After Write
///
/// DSB (Data Synchronization Barrier) is stronger than DMB. It ensures the register
/// write has fully completed and is visible to hardware before the next instruction
/// executes. This prevents bugs like "works with debug prints, fails without them"
/// where prints add implicit delays.
///
/// # Example
///
/// ```no_run
/// # use imxrt_ral as ral;
/// let analog = unsafe { ral::ccm_analog::CCM_ANALOG::instance() };
/// safe_modify_reg!(ral::ccm_analog, analog, PLL_USB1,
///     DIV_SELECT: 20,
///     ENABLE: 1
/// );
/// ```
///
/// # See Also
///
/// - ARM Cortex-M7 Technical Reference Manual, Section 8.3 (Memory Ordering)
/// - ARM Architecture Reference Manual, Section B2.2.2 (Memory Barriers)
#[macro_export]
macro_rules! safe_modify_reg {
    ($peripheral:path, $instance:expr, $register:ident, $($field:ident: $value:expr),+ $(,)? ) => {{
        cortex_m::asm::dmb(); // Data Memory Barrier before modify
        imxrt_ral::modify_reg!($peripheral, $instance, $register, $($field: $value),+);
        cortex_m::asm::dsb(); // Data Synchronization Barrier ensures write completes
    }};
}

/// Safe register write with ARM Cortex-M7 memory barriers
///
/// Wraps `imxrt_ral::write_reg!` with proper memory barriers for weakly-ordered
/// ARM Cortex-M7 memory. Ensures register writes are ordered correctly and
/// complete before continuing execution.
///
/// # Memory Ordering
///
/// - **DMB before write**: Ensures all prior operations complete before writing
/// - **DSB after write**: Ensures write completes before continuing
///
/// # When to Use
///
/// Use this for control registers where you need to ensure the write takes effect
/// before the next operation (e.g., starting a DMA transfer, enabling interrupts).
///
/// # Example
///
/// ```no_run
/// # use imxrt_ral as ral;
/// let ccm = unsafe { ral::ccm::CCM::instance() };
/// safe_write_reg!(ral::ccm, ccm, CCGR6, 0xFFFFFFFF); // Enable all clock gates
/// ```
///
/// # See Also
///
/// - ARM Cortex-M7 Technical Reference Manual, Section 8.3 (Memory Ordering)
#[macro_export]
macro_rules! safe_write_reg {
    ($peripheral:path, $instance:expr, $register:ident, $value:expr) => {{
        cortex_m::asm::dmb(); // Data Memory Barrier before write
        imxrt_ral::write_reg!($peripheral, $instance, $register, $value);
        cortex_m::asm::dsb(); // Data Synchronization Barrier ensures write completes
    }};
}
