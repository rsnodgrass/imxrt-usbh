//! Ultra-simple DMA memory management for embedded USB host
//!
//! Single-file, minimal-complexity memory pool implementation

use crate::error::{Result, UsbError};
use crate::ehci::{QueueHead, QueueTD};
use core::mem::MaybeUninit;

/// Simple bit-mask based allocator (single u32 = 32 items max)
pub struct SimplePool<T, const N: usize> {
    memory: [MaybeUninit<T>; N],
    allocated: u32, // bit mask: 1 = allocated, 0 = free
}

impl<T, const N: usize> SimplePool<T, N> {
    /// Create new pool
    pub const fn new() -> Self {
        assert!(N <= 32, "Pool size must be <= 32");
        
        Self {
            memory: unsafe { MaybeUninit::uninit().assume_init() },
            allocated: 0,
        }
    }
    
    /// Allocate item from pool
    pub fn alloc(&mut self) -> Option<&mut T> 
    where
        T: Default,
    {
        // Find first free slot
        let free_mask = !self.allocated;
        if free_mask == 0 {
            return None;
        }
        
        let slot = free_mask.trailing_zeros() as usize;
        if slot >= N {
            return None;
        }
        
        // Mark as allocated
        self.allocated |= 1 << slot;
        
        // Initialize and return
        unsafe {
            let ptr = self.memory[slot].as_mut_ptr();
            ptr.write(T::default());
            Some(&mut *ptr)
        }
    }
    
    /// Free item back to pool
    pub fn free(&mut self, item: &T) -> Result<()> {
        let item_addr = item as *const T as usize;
        let pool_start = self.memory.as_ptr() as usize;
        let item_size = core::mem::size_of::<T>();
        
        if item_addr < pool_start {
            return Err(UsbError::InvalidParameter);
        }
        
        let offset = item_addr - pool_start;
        if offset % item_size != 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        let slot = offset / item_size;
        if slot >= N {
            return Err(UsbError::InvalidParameter);
        }
        
        // Mark as free
        self.allocated &= !(1 << slot);
        Ok(())
    }
    
    /// Get number of free slots
    pub fn free_count(&self) -> usize {
        (N as u32 - self.allocated.count_ones()) as usize
    }
}

/// DMA buffer with fixed size
pub struct DmaBuffer {
    data: [u8; 512], // Single size fits most USB transfers
    allocated: bool,
}

impl DmaBuffer {
    const fn new() -> Self {
        Self {
            data: [0; 512],
            allocated: false,
        }
    }
    
    /// Get buffer data
    pub fn data_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
    
    /// Get buffer data (read-only)
    pub fn data(&self) -> &[u8] {
        &self.data
    }
    
    /// Get physical address (assumes identity mapping)
    pub fn physical_addr(&self) -> u32 {
        self.data.as_ptr() as u32
    }
}

/// Simple buffer pool
pub struct BufferPool<const N: usize> {
    buffers: [DmaBuffer; N],
}

impl<const N: usize> BufferPool<N> {
    pub const fn new() -> Self {
        Self {
            buffers: [DmaBuffer::new(); N],
        }
    }
    
    /// Allocate buffer
    pub fn alloc(&mut self) -> Option<&mut DmaBuffer> {
        for buffer in &mut self.buffers {
            if !buffer.allocated {
                buffer.allocated = true;
                return Some(buffer);
            }
        }
        None
    }
    
    /// Free buffer
    pub fn free(&mut self, buffer: &mut DmaBuffer) {
        buffer.allocated = false;
        // Clear buffer data
        buffer.data.fill(0);
    }
    
    /// Get free buffer count
    pub fn free_count(&self) -> usize {
        self.buffers.iter().filter(|b| !b.allocated).count()
    }
}

/// Global USB memory pools
pub struct UsbMemory {
    pub qh_pool: SimplePool<QueueHead, 16>,
    pub qtd_pool: SimplePool<QueueTD, 32>, 
    pub buffer_pool: BufferPool<16>,
}

impl UsbMemory {
    pub const fn new() -> Self {
        Self {
            qh_pool: SimplePool::new(),
            qtd_pool: SimplePool::new(),
            buffer_pool: BufferPool::new(),
        }
    }
}

/// Global memory instance
pub static mut USB_MEMORY: UsbMemory = UsbMemory::new();

/// Get mutable reference to global memory
/// 
/// # Safety
/// 
/// Must ensure single-threaded access or proper synchronization
pub unsafe fn get_usb_memory() -> &'static mut UsbMemory {
    &mut USB_MEMORY
}