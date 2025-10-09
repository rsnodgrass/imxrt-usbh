//! Simplified USB memory management
//!
//! Simple bit-mask based allocation for USB descriptors and buffers

use crate::error::{Result, UsbError};
use core::cell::UnsafeCell;
use core::ptr::NonNull;

/// Simple USB memory pool with static allocation
pub struct UsbMemoryPool {
    qh_allocated: u32,     // 32 queue heads max
    qtd_allocated: u32,    // 32 queue TDs max
    buffer_allocated: u32, // 32 buffers max
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
    ///
    /// # Errors
    ///
    /// Returns `UsbError::InvalidState` if DMA region not initialized (critical for cache coherency).
    /// Returns `UsbError::InvalidParameter` if size exceeds 512 bytes.
    /// Returns `UsbError::NoResources` if all buffer slots are allocated.
    pub fn alloc_buffer(&mut self, size: usize) -> Result<super::DmaBuffer> {
        // Check DMA initialization in ALL builds (not just debug)
        if !super::is_dma_initialized() {
            return Err(UsbError::InvalidState);
        }

        // Fixed 512-byte buffers, reject requests that are too large
        if size > 512 {
            return Err(UsbError::InvalidParameter);
        }

        let free_bit = (!self.buffer_allocated).trailing_zeros();
        if free_bit < 32 {
            self.buffer_allocated |= 1 << free_bit;
            let index = free_bit as usize;

            // Safety: BUFFER_POOL is static memory, always valid
            // Index is guaranteed < 32 by the trailing_zeros check above
            // Access is controlled by allocation bitmap ensuring exclusivity
            let ptr = unsafe {
                let buffers = BUFFER_POOL.get_mut();
                NonNull::new_unchecked(buffers[index].as_mut_ptr())
            };

            Ok(super::DmaBuffer::new(ptr, size.max(1), index))
        } else {
            Err(UsbError::NoResources)
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
///
/// Uses UnsafeCell for interior mutability with controlled access through UsbMemoryPool.
/// This is safer than `static mut` because:
/// - UnsafeCell explicitly marks interior mutability
/// - Access requires unsafe block, making mutation sites visible
/// - UsbMemoryPool allocation bitmask prevents concurrent access to same buffer
/// - Buffers are only accessed through DmaBuffer which has non-Copy semantics
struct BufferPool {
    buffers: UnsafeCell<[[u8; 512]; 32]>,
}

unsafe impl Sync for BufferPool {}

impl BufferPool {
    const fn new() -> Self {
        Self {
            buffers: UnsafeCell::new([[0; 512]; 32]),
        }
    }

    /// Get mutable access to buffer array
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - Only one mutable reference exists at a time
    /// - Access is controlled through UsbMemoryPool allocation bitmap
    unsafe fn get_mut(&self) -> &mut [[u8; 512]; 32] {
        // Safety: Caller guarantees exclusive access via allocation bitmap
        unsafe { &mut *self.buffers.get() }
    }
}

static BUFFER_POOL: BufferPool = BufferPool::new();
