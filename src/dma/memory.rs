//! Simplified USB memory management
//!
//! Simple bit-mask based allocation for USB descriptors and buffers

use crate::error::{Result, UsbError};
use core::ptr::NonNull;

/// Simple USB memory pool with static allocation
pub struct UsbMemoryPool {
    qh_allocated: u32,      // 32 queue heads max
    qtd_allocated: u32,     // 32 queue TDs max  
    buffer_allocated: u32,  // 32 buffers max
}

impl UsbMemoryPool {
    /// Create new memory pool
    pub const fn new() -> Self {
        Self {
            qh_allocated: 0,
            qtd_allocated: 0,
            buffer_allocated: 0,
        }
    }
    
    /// Allocate a queue head slot
    pub fn alloc_qh(&mut self) -> Option<u8> {
        let free_bit = (!self.qh_allocated).trailing_zeros();
        if free_bit < 32 {
            self.qh_allocated |= 1 << free_bit;
            Some(free_bit as u8)
        } else {
            None
        }
    }
    
    /// Free queue head slot
    pub fn free_qh(&mut self, index: u8) -> Result<()> {
        if index >= 32 {
            return Err(UsbError::InvalidParameter);
        }
        self.qh_allocated &= !(1 << index);
        Ok(())
    }
    
    /// Allocate a queue TD slot
    pub fn alloc_qtd(&mut self) -> Option<u8> {
        let free_bit = (!self.qtd_allocated).trailing_zeros();
        if free_bit < 32 {
            self.qtd_allocated |= 1 << free_bit;
            Some(free_bit as u8)
        } else {
            None
        }
    }
    
    /// Free queue TD slot
    pub fn free_qtd(&mut self, index: u8) -> Result<()> {
        if index >= 32 {
            return Err(UsbError::InvalidParameter);
        }
        self.qtd_allocated &= !(1 << index);
        Ok(())
    }
    
    /// Allocate DMA buffer slot
    pub fn alloc_buffer(&mut self, size: usize) -> Option<super::DmaBuffer> {
        // Fixed 512-byte buffers, reject requests that are too large
        if size > 512 {
            return None;
        }

        let free_bit = (!self.buffer_allocated).trailing_zeros();
        if free_bit < 32 {
            self.buffer_allocated |= 1 << free_bit;
            let index = free_bit as usize;

            // Safety: BUFFER_POOL is static memory, always valid
            // Index is guaranteed < 32 by the trailing_zeros check above
            let ptr = unsafe {
                NonNull::new_unchecked(BUFFER_POOL[index].as_mut_ptr())
            };

            Some(super::DmaBuffer::new(ptr, size.max(1), index))
        } else {
            None
        }
    }

    /// Free DMA buffer
    pub fn free_buffer(&mut self, buffer: &super::DmaBuffer) -> Result<()> {
        let index = buffer.pool_index();

        if index >= 32 {
            return Err(UsbError::InvalidParameter);
        }

        self.buffer_allocated &= !(1 << index);
        Ok(())
    }
    
    /// Get available slots count
    pub fn available_slots(&self) -> (u8, u8, u8) {
        (
            (!self.qh_allocated).count_ones() as u8,
            (!self.qtd_allocated).count_ones() as u8, 
            (!self.buffer_allocated).count_ones() as u8,
        )
    }
}

/// Static buffer pool - 32 buffers of 512 bytes each
static mut BUFFER_POOL: [[u8; 512]; 32] = [[0; 512]; 32];

/// Global memory pool instance
pub static mut USB_MEMORY_POOL: UsbMemoryPool = UsbMemoryPool::new();