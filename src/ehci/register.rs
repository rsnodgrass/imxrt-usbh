//! Optimized register access patterns for EHCI
//!
//! Implements efficient, cache-friendly register access with proper memory ordering
//! for ARM Cortex-M7 weakly-ordered memory model.

use core::ptr::{read_volatile, write_volatile};
use core::cell::UnsafeCell;

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
    #[inline(always)]
    pub fn modify<F>(&self, f: F) 
    where 
        F: FnOnce(u32) -> u32,
    {
        unsafe {
            cortex_m::asm::dmb();
            let current = read_volatile(self.value.get());
            let new_value = f(current);
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

/// Timeout handling for register operations
pub struct RegisterTimeout {
    start_cycles: u32,
    timeout_cycles: u32,
}

impl RegisterTimeout {
    /// Create new timeout with duration in microseconds
    pub fn new_us(timeout_us: u32) -> Self {
        // Use shared CPU frequency constant
        let cycles_per_us = crate::CPU_FREQ_MHZ;
        Self {
            start_cycles: cortex_m::peripheral::DWT::cycle_count(),
            timeout_cycles: timeout_us * cycles_per_us,
        }
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
}