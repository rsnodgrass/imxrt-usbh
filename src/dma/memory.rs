//! Simplified USB memory management
//! 
//! Simple bit-mask based allocation for USB descriptors and buffers

use crate::error::{Result, UsbError};

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
    pub fn alloc_buffer(&mut self, _size: usize) -> Option<DmaBuffer> {
        let free_bit = (!self.buffer_allocated).trailing_zeros();
        if free_bit < 32 {
            self.buffer_allocated |= 1 << free_bit;
            Some(DmaBuffer {
                ptr: unsafe { BUFFER_POOL[free_bit as usize].as_mut_ptr() },
                len: 512, // Fixed size
                actual_size: 512,
            })
        } else {
            None
        }
    }
    
    /// Free DMA buffer
    pub fn free_buffer(&mut self, buffer: DmaBuffer) -> Result<()> {
        let buffer_addr = buffer.ptr as usize;
        let pool_start = unsafe { BUFFER_POOL.as_ptr() as usize };
        let index = (buffer_addr - pool_start) / 512;
        
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

/// DMA buffer handle
#[derive(Debug, Clone, Copy)]
pub struct DmaBuffer {
    /// Pointer to buffer data
    pub ptr: *mut u8,
    /// Requested length
    pub len: usize,
    /// Actual allocated size
    pub actual_size: usize,
}

impl DmaBuffer {
    /// Get buffer as slice
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.ptr, self.len) }
    }
    
    /// Get buffer as mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.ptr, self.len) }
    }
    
    /// Get physical address for DMA
    pub fn physical_address(&self) -> u32 {
        self.ptr as u32
    }
    
    /// Prepare buffer for DMA write (CPU -> Device)
    pub fn prepare_for_device(&self) {
        crate::dma::cache_ops::prepare_for_dma_write(self.as_slice());
    }
    
    /// Prepare buffer for CPU read (Device -> CPU)
    pub fn prepare_for_cpu(&mut self) {
        crate::dma::cache_ops::prepare_for_dma_read(self.as_mut_slice());
    }
}

/// Global memory pool instance
pub static mut USB_MEMORY_POOL: UsbMemoryPool = UsbMemoryPool::new();