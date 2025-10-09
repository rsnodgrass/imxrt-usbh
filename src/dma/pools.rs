//! Zero-allocation DMA buffer pools with compile-time sizing
//!
//! Implements the memory pool improvements from the expert review

use crate::ehci::{QueueHead, QueueTD};
use crate::error::{Result, UsbError};
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicBool, Ordering};

/// Memory pool for USB descriptors with compile-time sizing
pub struct UsbDescriptorPool<const N_QH: usize, const N_QTD: usize> {
    qh_memory: [MaybeUninit<QueueHead>; N_QH],
    qtd_memory: [MaybeUninit<QueueTD>; N_QTD],
    qh_allocated: [AtomicBool; N_QH],
    qtd_allocated: [AtomicBool; N_QTD],
}

impl<const N_QH: usize, const N_QTD: usize> UsbDescriptorPool<N_QH, N_QTD> {
    /// Create new descriptor pool (const-compatible)
    pub const fn new() -> Self {
        const UNINIT_QH: MaybeUninit<QueueHead> = MaybeUninit::uninit();
        const UNINIT_QTD: MaybeUninit<QueueTD> = MaybeUninit::uninit();
        const ATOMIC_FALSE: AtomicBool = AtomicBool::new(false);

        Self {
            qh_memory: [UNINIT_QH; N_QH],
            qtd_memory: [UNINIT_QTD; N_QTD],
            qh_allocated: [ATOMIC_FALSE; N_QH],
            qtd_allocated: [ATOMIC_FALSE; N_QTD],
        }
    }

    /// Allocate queue head from pool
    pub fn alloc_qh(&mut self) -> Option<&mut QueueHead> {
        for (i, allocated) in self.qh_allocated.iter().enumerate() {
            if !allocated.swap(true, Ordering::Acquire) {
                let qh_ptr = self.qh_memory[i].as_mut_ptr();
                // Safety: We atomically claimed this slot, qh_ptr is valid and properly aligned,
                // write initializes the memory, and returned reference has lifetime of &mut self
                unsafe {
                    qh_ptr.write(QueueHead::new());
                    return Some(&mut *qh_ptr);
                }
            }
        }
        None
    }

    /// Allocate queue transfer descriptor from pool
    pub fn alloc_qtd(&mut self) -> Option<&mut QueueTD> {
        for (i, allocated) in self.qtd_allocated.iter().enumerate() {
            if !allocated.swap(true, Ordering::Acquire) {
                let qtd_ptr = self.qtd_memory[i].as_mut_ptr();
                // Safety: We atomically claimed this slot, qtd_ptr is valid and properly aligned,
                // write initializes the memory, and returned reference has lifetime of &mut self
                unsafe {
                    qtd_ptr.write(QueueTD::new());
                    return Some(&mut *qtd_ptr);
                }
            }
        }
        None
    }

    /// Free queue head back to pool
    pub unsafe fn free_qh(&mut self, qh: &mut QueueHead) {
        let qh_addr = qh as *mut QueueHead as usize;
        let pool_start = self.qh_memory.as_ptr() as usize;
        let index = (qh_addr - pool_start) / core::mem::size_of::<QueueHead>();
        if index < N_QH {
            self.qh_allocated[index].store(false, Ordering::Release);
        }
    }

    /// Free queue transfer descriptor back to pool
    pub unsafe fn free_qtd(&mut self, qtd: &mut QueueTD) {
        let qtd_addr = qtd as *mut QueueTD as usize;
        let pool_start = self.qtd_memory.as_ptr() as usize;
        let index = (qtd_addr - pool_start) / core::mem::size_of::<QueueTD>();
        if index < N_QTD {
            self.qtd_allocated[index].store(false, Ordering::Release);
        }
    }

    /// Get pool utilization statistics
    pub fn stats(&self) -> PoolStats {
        let qh_used = self
            .qh_allocated
            .iter()
            .filter(|a| a.load(Ordering::Relaxed))
            .count();
        let qtd_used = self
            .qtd_allocated
            .iter()
            .filter(|a| a.load(Ordering::Relaxed))
            .count();

        PoolStats {
            qh_total: N_QH,
            qh_available: N_QH - qh_used,
            qtd_total: N_QTD,
            qtd_available: N_QTD - qtd_used,
        }
    }
}

/// Pool utilization statistics
#[derive(Debug, Clone, Copy)]
pub struct PoolStats {
    /// Total number of Queue Head descriptors in pool
    pub qh_total: usize,
    /// Number of available (unallocated) Queue Heads
    pub qh_available: usize,
    /// Total number of Queue Transfer Descriptors in pool
    pub qtd_total: usize,
    /// Number of available (unallocated) qTDs
    pub qtd_available: usize,
}

impl PoolStats {
    /// Check if pools are getting low on resources
    pub fn is_low(&self) -> bool {
        let qh_usage = (self.qh_total - self.qh_available) as f32 / self.qh_total as f32;
        let qtd_usage = (self.qtd_total - self.qtd_available) as f32 / self.qtd_total as f32;
        qh_usage > 0.8 || qtd_usage > 0.8
    }
}

/// Static pools for different use cases
pub mod static_pools {
    use super::*;

    /// Small pool for basic enumeration (8 QH, 32 qTD)
    pub static SMALL_POOL: UsbDescriptorPool<8, 32> = UsbDescriptorPool::new();

    /// Medium pool for typical applications (16 QH, 64 qTD)
    pub static MEDIUM_POOL: UsbDescriptorPool<16, 64> = UsbDescriptorPool::new();

    /// Large pool for high-throughput applications (32 QH, 128 qTD)
    pub static LARGE_POOL: UsbDescriptorPool<32, 128> = UsbDescriptorPool::new();

    /// Initialize all static pools (call once at startup)
    ///
    /// # Safety
    ///
    /// Must be called exactly once during system initialization
    pub unsafe fn init_all() {
        // Pools are self-contained now, no external initialization needed
    }
}

/// Buffer pool for data transfers with different size classes
pub struct DataBufferPool<const N_SMALL: usize, const N_LARGE: usize> {
    small_buffers: [[u8; 64]; N_SMALL],
    large_buffers: [[u8; 512]; N_LARGE],
    small_allocated: [AtomicBool; N_SMALL],
    large_allocated: [AtomicBool; N_LARGE],
}

impl<const N_SMALL: usize, const N_LARGE: usize> DataBufferPool<N_SMALL, N_LARGE> {
    /// Create a new data buffer pool with specified capacities
    ///
    /// Creates a pool with N_SMALL small buffers (64 bytes each) and
    /// N_LARGE large buffers (512 bytes each). All buffers are initially
    /// unallocated and zero-initialized.
    pub const fn new() -> Self {
        const ATOMIC_FALSE: AtomicBool = AtomicBool::new(false);

        Self {
            small_buffers: [[0; 64]; N_SMALL],
            large_buffers: [[0; 512]; N_LARGE],
            small_allocated: [ATOMIC_FALSE; N_SMALL],
            large_allocated: [ATOMIC_FALSE; N_LARGE],
        }
    }

    /// Allocate appropriate buffer size based on request
    ///
    /// # Safety Requirements
    ///
    /// DMA region must be initialized via `init_dma_region()` before calling this function.
    /// Failure to do so may result in cache coherency issues and data corruption.
    pub fn alloc_buffer(&mut self, size: usize) -> Result<BufferHandle> {
        // Verify DMA region is initialized
        if !crate::dma::is_dma_initialized() {
            #[cfg(feature = "defmt")]
            defmt::error!("DMA buffer allocation attempted before init_dma_region() called");
            return Err(UsbError::InvalidState);
        }

        if size <= 64 {
            for (i, allocated) in self.small_allocated.iter().enumerate() {
                if !allocated.swap(true, Ordering::Acquire) {
                    return Ok(BufferHandle::Small(i));
                }
            }
            Err(UsbError::NoResources)
        } else if size <= 512 {
            for (i, allocated) in self.large_allocated.iter().enumerate() {
                if !allocated.swap(true, Ordering::Acquire) {
                    return Ok(BufferHandle::Large(i));
                }
            }
            Err(UsbError::NoResources)
        } else {
            Err(UsbError::BufferOverflow)
        }
    }

    /// Get buffer slice from handle
    pub fn get_buffer_slice(&self, handle: &BufferHandle) -> &[u8] {
        match handle {
            BufferHandle::Small(i) => &self.small_buffers[*i],
            BufferHandle::Large(i) => &self.large_buffers[*i],
        }
    }

    /// Get mutable buffer slice from handle
    pub fn get_buffer_slice_mut(&mut self, handle: &BufferHandle) -> &mut [u8] {
        match handle {
            BufferHandle::Small(i) => &mut self.small_buffers[*i],
            BufferHandle::Large(i) => &mut self.large_buffers[*i],
        }
    }

    /// Free buffer back to pool
    pub fn free_buffer(&mut self, handle: BufferHandle) {
        match handle {
            BufferHandle::Small(i) => {
                if i < N_SMALL {
                    self.small_allocated[i].store(false, Ordering::Release);
                }
            }
            BufferHandle::Large(i) => {
                if i < N_LARGE {
                    self.large_allocated[i].store(false, Ordering::Release);
                }
            }
        }
    }

    /// Get buffer pool statistics
    pub fn pool_stats(&self) -> DataBufferStats {
        let small_used = self
            .small_allocated
            .iter()
            .filter(|a| a.load(Ordering::Relaxed))
            .count();
        let large_used = self
            .large_allocated
            .iter()
            .filter(|a| a.load(Ordering::Relaxed))
            .count();

        DataBufferStats {
            small_total: N_SMALL,
            small_allocated: small_used,
            large_total: N_LARGE,
            large_allocated: large_used,
        }
    }
}

/// Handle to allocated buffer with automatic size tracking
pub enum BufferHandle {
    /// Small buffer (64 bytes)
    Small(usize),
    /// Large buffer (512 bytes)
    Large(usize),
}

/// Data buffer pool statistics
#[derive(Debug, Clone, Copy)]
pub struct DataBufferStats {
    /// Total number of small buffers (64 bytes) in pool
    pub small_total: usize,
    /// Number of allocated small buffers
    pub small_allocated: usize,
    /// Total number of large buffers (512 bytes) in pool
    pub large_total: usize,
    /// Number of allocated large buffers
    pub large_allocated: usize,
}

impl BufferHandle {
    pub fn index(&self) -> usize {
        match self {
            BufferHandle::Small(index) => *index,
            BufferHandle::Large(index) => *index,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pool_creation() {
        let pool: UsbDescriptorPool<4, 8> = UsbDescriptorPool::new();
        let stats = pool.stats();

        assert_eq!(stats.qh_total, 4);
        assert_eq!(stats.qtd_total, 8);
    }
}
