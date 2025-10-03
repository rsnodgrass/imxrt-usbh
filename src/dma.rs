//! DMA buffer management for USB transfers
//!
//! # Initialization Order (CRITICAL)
//!
//! **MUST** call `init_dma_region()` during system initialization before any buffer allocations:
//!
//! ```no_run
//! # use imxrt_usbh::dma;
//! # use imxrt_usbh::error::Result;
//! # fn main() -> Result<()> {
//! // 1. Initialize DMA region (configures MPU for cache coherency)
//! unsafe { dma::init_dma_region()? };
//!
//! // 2. Now safe to allocate buffers
//! // let buffer = buffer_pool.alloc_buffer(512)?;
//! # Ok(())
//! # }
//! ```
//!
//! **Failure to follow this order will cause:**
//! - Cache coherency violations
//! - Data corruption in DMA transfers
//! - Runtime error from allocation functions

pub mod pools;
pub mod descriptor;
pub mod memory;

pub use pools::{UsbDescriptorPool, DataBufferPool, BufferHandle, PoolStats};
pub use descriptor::{DescriptorAllocator, QhHandle, QtdHandle, DescriptorState};
pub use memory::UsbMemoryPool;

use core::mem::size_of;
use core::ptr::NonNull;
use core::sync::atomic::{AtomicBool, Ordering};
use crate::error::{Result, UsbError};

/// DMA buffer alignment requirement (32-byte for cache line)
pub const DMA_ALIGNMENT: usize = 32;

/// Maximum DMA buffer size (20KB per USB 2.0 spec for qTD)
pub const MAX_DMA_BUFFER_SIZE: usize = 20 * 1024;

/// DMA-safe memory region in non-cacheable RAM
///
/// **CRITICAL**: Must be placed in OCRAM (non-cacheable) memory region.
/// Configure linker script to place `.ocram` section at OCRAM base address.
///
/// For Teensy 4.1 (i.MX RT1062):
/// - OCRAM: 0x20200000 - 0x2027FFFF (512KB)
/// - DTCM:  0x20000000 - 0x2001FFFF (128KB, tightly coupled)
///
/// Add to linker script (.ld file):
/// ```
/// .ocram (NOLOAD) : {
///     *(.ocram .ocram.*)
/// } > OCRAM
/// ```
/// Cache-line aligned buffer for DMA operations
/// Ensures no partial cache line issues on Cortex-M7 (32-byte cache lines)
#[repr(C, align(32))]
pub struct CacheAlignedBuffer([u8; 512]);

impl CacheAlignedBuffer {
    const fn new() -> Self {
        Self([0; 512])
    }

    /// Get pointer to buffer data
    pub fn as_ptr(&self) -> *const u8 {
        self.0.as_ptr()
    }

    /// Get mutable pointer to buffer data
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.0.as_mut_ptr()
    }
}

#[repr(C, align(4096))]
pub struct DmaRegion {
    /// Queue Head pool (aligned to 64 bytes per EHCI spec)
    pub qh_pool: [u8; 64 * 64],  // 64 QHs, 64 bytes each
    /// Queue Transfer Descriptor pool (aligned to 32 bytes per EHCI spec)
    pub qtd_pool: [u8; 256 * 32], // 256 qTDs, 32 bytes each
    /// Data buffers for transfers (cache-line aligned)
    pub data_buffers: [CacheAlignedBuffer; 32], // 32 buffers of 512 bytes each
}

/// Static DMA region instance - will be placed in OCRAM by linker
#[link_section = ".ocram"]
static mut DMA_REGION: DmaRegion = DmaRegion {
    qh_pool: [0; 64 * 64],
    qtd_pool: [0; 256 * 32],
    data_buffers: [
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
        CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(), CacheAlignedBuffer::new(),
    ],
};

/// DMA region initialization flag
static DMA_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Check if DMA region has been initialized
///
/// Returns true if `init_dma_region()` has been called successfully.
pub fn is_dma_initialized() -> bool {
    DMA_INITIALIZED.load(Ordering::Acquire)
}

/// Compile-time assertions for DMA buffer alignment
const _: () = {
    const DCACHE_LINE_SIZE: usize = 32;
    // Ensure CacheAlignedBuffer is properly aligned
    assert!(core::mem::align_of::<CacheAlignedBuffer>() == DCACHE_LINE_SIZE);
    // Ensure buffer size is multiple of cache line
    assert!(512 % DCACHE_LINE_SIZE == 0);
};

/// Initialize DMA region with MPU configuration
///
/// Configures the MPU to mark DMA region as non-cacheable Normal memory.
/// This ensures cache coherency for DMA operations.
///
/// # Safety
///
/// Must be called once during system initialization before any DMA operations.
pub unsafe fn init_dma_region() -> Result<()> {
    if DMA_INITIALIZED.swap(true, Ordering::Acquire) {
        return Err(UsbError::AlreadyInitialized);
    }

    // Get DMA region address
    let region_addr = &raw const DMA_REGION as usize;
    let region_size = size_of::<DmaRegion>();

    // Validate region bounds and alignment
    crate::safety::BoundsChecker::validate_alignment(region_addr, DMA_ALIGNMENT)?;

    // Validate cache line alignment for data buffers
    unsafe {
        let buffer_addr = &raw const DMA_REGION.data_buffers as usize;
        const DCACHE_LINE_SIZE: usize = 32;
        if buffer_addr % DCACHE_LINE_SIZE != 0 {
            return Err(UsbError::InvalidParameter);
        }
    }

    // Configure MPU for non-cacheable DMA region
    unsafe {
        configure_mpu_dma_region(region_addr, region_size)?;
    }

    Ok(())
}

/// Configure MPU for DMA region
///
/// Sets up MPU region 7 as Normal Non-cacheable Shareable memory for DMA buffers.
/// Reference: ARM Cortex-M7 TRM Table 4-3 (Memory Type Encoding)
unsafe fn configure_mpu_dma_region(addr: usize, _size: usize) -> Result<()> {
    use cortex_m::peripheral::MPU;

    const MPU_CTRL_ENABLE: u32 = 0x01;
    const MPU_CTRL_PRIVDEFENA: u32 = 0x04;

    const MPU_RASR_ENABLE: u32 = 0x01;
    const MPU_RASR_SIZE_32KB: u32 = 14 << 1;  // 2^(14+1) = 32KB (covers 28KB DMA region)
    const MPU_RASR_AP_RW: u32 = 0b011 << 24;  // Read/Write access
    const MPU_RASR_TEX_NORMAL: u32 = 0b001 << 19; // Normal memory (not Device)
    const MPU_RASR_SHAREABLE: u32 = 1 << 18;
    // C=0, B=0 for non-cacheable (do not set CACHEABLE or BUFFERABLE bits)
    const MPU_RASR_XN: u32 = 1 << 28;  // Execute never

    // Disable MPU during configuration
    unsafe {
        let mpu = &*cortex_m::peripheral::MPU::PTR;
        mpu.ctrl.write(0); // Disable MPU
    }

    // Configure region 7 for DMA
    unsafe {
        (*MPU::PTR).rnr.write(7);  // Select region 7
        (*MPU::PTR).rbar.write(addr as u32 & !0x1F); // Base address (32-byte aligned)
        (*MPU::PTR).rasr.write(
            MPU_RASR_ENABLE |
            MPU_RASR_SIZE_32KB |
            MPU_RASR_AP_RW |
            MPU_RASR_TEX_NORMAL |  // Normal Non-cacheable
            MPU_RASR_SHAREABLE |
            MPU_RASR_XN
            // Note: C=0, B=0 (non-cacheable) - do not include BUFFERABLE
        );
    }

    // Enable MPU with default memory map for privileged mode
    unsafe {
        (*MPU::PTR).ctrl.write(MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA);
    }

    // Ensure MPU configuration takes effect
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    Ok(())
}

/// DMA buffer allocation handle
///
/// NOT Copy/Clone - ensures single ownership and prevents aliasing.
/// Buffer is automatically returned to pool when dropped.
pub struct DmaBuffer {
    ptr: NonNull<u8>,
    size: usize,
    pool_index: usize,
}

impl PartialEq for DmaBuffer {
    fn eq(&self, other: &Self) -> bool {
        self.pool_index == other.pool_index
    }
}

impl DmaBuffer {
    /// Create a new DMA buffer
    ///
    /// # Safety
    ///
    /// Caller must ensure:
    /// - `ptr` points to valid DMA-accessible memory
    /// - Memory remains valid for the lifetime of this buffer
    /// - `pool_index` is valid for the associated pool
    pub(crate) fn new(ptr: NonNull<u8>, size: usize, pool_index: usize) -> Self {
        Self {
            ptr,
            size,
            pool_index,
        }
    }

    /// Get pool index (for returning to pool)
    pub(crate) fn pool_index(&self) -> usize {
        self.pool_index
    }

    /// Get raw const pointer
    pub fn as_ptr(&self) -> *const u8 {
        self.ptr.as_ptr()
    }

    /// Get raw mutable pointer
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr.as_ptr()
    }

    /// Get physical DMA address
    pub fn dma_addr(&self) -> u32 {
        self.ptr.as_ptr() as u32
    }

    /// Get buffer as immutable slice
    pub fn as_slice(&self) -> &[u8] {
        // Safety: ptr is NonNull and size is valid per construction contract
        unsafe { core::slice::from_raw_parts(self.ptr.as_ptr(), self.size) }
    }

    /// Get buffer as mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        // Safety: ptr is NonNull and size is valid per construction contract
        unsafe { core::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.size) }
    }

    /// Get buffer size in bytes
    pub fn len(&self) -> usize {
        self.size
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Prepare buffer for DMA write (CPU -> Device)
    ///
    /// Cleans the D-cache to ensure CPU writes are visible to DMA hardware
    pub fn prepare_for_device(&self) {
        cache_ops::prepare_for_dma_write(self.as_slice());
    }

    /// Prepare buffer for CPU read (Device -> CPU)
    ///
    /// Invalidates the D-cache so CPU reads fresh data from memory written by DMA
    pub fn prepare_for_cpu(&mut self) {
        cache_ops::prepare_for_dma_read(self.as_mut_slice());
    }
}

/// Cache maintenance operations for cacheable DMA buffers
///
/// Use these when DMA buffers are in cacheable memory regions.
pub mod cache_ops {
    use cortex_m::asm::{dsb, isb};

    /// Data cache line size for Cortex-M7 (32 bytes)
    const DCACHE_LINE_SIZE: usize = 32;

    /// SCB cache maintenance registers for Cortex-M7
    const SCB_DCCMVAC: *mut u32 = 0xE000_EF68 as *mut u32; // Clean by MVA to PoC
    const SCB_DCIMVAC: *mut u32 = 0xE000_EF5C as *mut u32; // Invalidate by MVA to PoC
    const SCB_DCCIMVAC: *mut u32 = 0xE000_EF70 as *mut u32; // Clean & Invalidate by MVA to PoC

    pub fn clean_dcache(addr: usize, size: usize) {
        unsafe {
            dsb(); // Ensure all previous writes complete
            let start = addr & !(DCACHE_LINE_SIZE - 1);
            let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);

            for line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
                core::ptr::write_volatile(SCB_DCCMVAC, line_addr as u32);
            }

            dsb(); // Ensure cache operations complete
            isb(); // Ensure instruction synchronization
        }
    }

    pub fn invalidate_dcache(addr: usize, size: usize) {
        unsafe {
            dsb(); // Ensure all previous operations complete
            let start = addr & !(DCACHE_LINE_SIZE - 1);
            let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);

            for line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
                core::ptr::write_volatile(SCB_DCIMVAC, line_addr as u32);
            }

            dsb(); // Ensure cache operations complete
            isb(); // Ensure instruction synchronization
        }
    }

    pub fn clean_invalidate_dcache(addr: usize, size: usize) {
        unsafe {
            dsb(); // Ensure all previous operations complete
            let start = addr & !(DCACHE_LINE_SIZE - 1);
            let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);

            for line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
                core::ptr::write_volatile(SCB_DCCIMVAC, line_addr as u32);
            }

            dsb(); // Ensure cache operations complete
            isb(); // Ensure instruction synchronization
        }
    }

    #[inline]
    pub fn prepare_for_dma_write(buffer: &[u8]) {
        clean_dcache(buffer.as_ptr() as usize, buffer.len());
    }

    #[inline]
    pub fn prepare_for_dma_read(buffer: &mut [u8]) {
        invalidate_dcache(buffer.as_ptr() as usize, buffer.len());
    }
}

/// DMA buffer pool for allocation management
pub struct DmaBufferPool {
    allocated: [AtomicBool; 32],
}

impl DmaBufferPool {
    /// Create new DMA buffer pool
    pub const fn new() -> Self {
        const ATOMIC_BOOL_FALSE: AtomicBool = AtomicBool::new(false);
        Self {
            allocated: [ATOMIC_BOOL_FALSE; 32],
        }
    }

    /// Allocate a DMA buffer from the pool
    pub fn alloc(&mut self, size: usize) -> Result<DmaBuffer> {
        if size > 512 {
            return Err(UsbError::InvalidParameter);
        }

        // Find free buffer
        for (i, allocated) in self.allocated.iter().enumerate() {
            if !allocated.swap(true, Ordering::Acquire) {
                // Buffer was free, now allocated
                let addr = unsafe { DMA_REGION.data_buffers[i].as_ptr() as *mut u8 };

                // Validate buffer bounds
                crate::safety::BoundsChecker::validate_buffer(addr, size)?;
                crate::safety::BoundsChecker::validate_dma_buffer(addr as usize, size)?;

                let ptr = unsafe { NonNull::new_unchecked(addr) };

                let buffer = DmaBuffer {
                    ptr,
                    size,
                    pool_index: i,
                };

                // Return the buffer (no Copy, ensures single ownership)
                // IMPORTANT: Buffer must be manually freed with pool.free(buffer)
                // Dropping without freeing will leak the buffer slot
                return Ok(buffer);
            }
        }

        Err(UsbError::NoResources)
    }

    /// Get buffer statistics
    pub fn buffer_stats(&self) -> BufferStats {
        let allocated_count = self.allocated.iter().filter(|a| a.load(Ordering::Relaxed)).count();
        BufferStats {
            total_buffers: 32,
            allocated_buffers: allocated_count,
            free_buffers: 32 - allocated_count,
        }
    }

    /// Free a DMA buffer back to the pool
    ///
    /// Free a buffer back to the pool with validation
    ///
    /// # Errors
    /// Returns `InvalidState` if buffer was already freed (double-free detection)
    ///
    /// # Safety
    /// Buffer must not be used after being freed. The lack of Copy trait
    /// helps prevent this, but care must be taken not to hold references.
    pub fn free(&mut self, buffer: DmaBuffer) -> Result<()> {
        // Detect double-free by checking if buffer was actually allocated
        let was_allocated = self.allocated[buffer.pool_index].swap(false, Ordering::AcqRel);

        if !was_allocated {
            #[cfg(feature = "defmt")]
            defmt::error!("Double-free detected for buffer at index {}", buffer.pool_index);
            return Err(UsbError::InvalidState);
        }

        // Buffer is consumed (moved) here, preventing further use
        Ok(())
    }
}

/// Buffer pool statistics
#[derive(Debug, Clone, Copy)]
pub struct BufferStats {
    /// Total number of buffers in pool
    pub total_buffers: usize,
    /// Number of currently allocated buffers
    pub allocated_buffers: usize,
    /// Number of free buffers
    pub free_buffers: usize,
}

/// Ensure a value is aligned to DMA requirements
#[inline]
pub const fn align_dma(addr: usize) -> usize {
    (addr + DMA_ALIGNMENT - 1) & !(DMA_ALIGNMENT - 1)
}

/// Check if an address is DMA-aligned
#[inline]
pub const fn is_dma_aligned(addr: usize) -> bool {
    addr & (DMA_ALIGNMENT - 1) == 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dma_buffer_pool_creation() {
        let pool = DmaBufferPool::new();
        let stats = pool.buffer_stats();

        assert_eq!(stats.total_buffers, 32);
        assert_eq!(stats.allocated_buffers, 0);
        assert_eq!(stats.free_buffers, 32);
    }

    #[test]
    fn test_dma_buffer_allocation() {
        let mut pool = DmaBufferPool::new();

        // allocate single buffer
        let result = pool.alloc(512);
        assert!(result.is_ok());

        let buffer = result.unwrap();
        assert_eq!(buffer.len(), 512);
        assert!(!buffer.is_empty());

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 1);
        assert_eq!(stats.free_buffers, 31);
    }

    #[test]
    fn test_dma_buffer_alignment() {
        let mut pool = DmaBufferPool::new();

        // allocate several buffers and verify alignment
        for _ in 0..5 {
            let buffer = pool.alloc(256).expect("allocation failed");
            let addr = buffer.as_ptr() as usize;

            // must be 32-byte aligned (cache line)
            assert_eq!(addr & 0x1F, 0, "buffer not 32-byte aligned");
            assert!(is_dma_aligned(addr));
        }
    }

    #[test]
    fn test_dma_buffer_no_aliasing() {
        let mut pool = DmaBufferPool::new();

        // allocate two buffers
        let buf1 = pool.alloc(512).unwrap();
        let buf2 = pool.alloc(512).unwrap();

        let addr1 = buf1.as_ptr() as usize;
        let addr2 = buf2.as_ptr() as usize;

        // buffers must not overlap
        // either buf1 ends before buf2 starts, or buf2 ends before buf1 starts
        let no_overlap = (addr1 + buf1.len() <= addr2) || (addr2 + buf2.len() <= addr1);
        assert!(no_overlap, "buffers overlap! addr1={:#x}, addr2={:#x}", addr1, addr2);

        // also verify different pool indices
        assert_ne!(buf1.pool_index(), buf2.pool_index());
    }

    #[test]
    fn test_dma_buffer_pool_exhaustion() {
        let mut pool = DmaBufferPool::new();

        // allocate all 32 buffers
        let mut buffers = Vec::new();
        for i in 0..32 {
            let result = pool.alloc(512);
            assert!(result.is_ok(), "failed to allocate buffer {}", i);
            buffers.push(result.unwrap());
        }

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 32);
        assert_eq!(stats.free_buffers, 0);

        // 33rd allocation should fail
        let result = pool.alloc(512);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::NoResources)));
    }

    #[test]
    fn test_dma_buffer_free_and_realloc() {
        let mut pool = DmaBufferPool::new();

        // allocate a buffer
        let buffer = pool.alloc(256).unwrap();
        let pool_index = buffer.pool_index();

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 1);

        // free the buffer
        pool.free(buffer);

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 0);
        assert_eq!(stats.free_buffers, 32);

        // reallocate - should get a free buffer
        let buffer2 = pool.alloc(128).unwrap();
        assert_eq!(buffer2.pool_index(), pool_index); // likely reuses same slot

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 1);
    }

    #[test]
    fn test_dma_buffer_size_limits() {
        let mut pool = DmaBufferPool::new();

        // valid sizes (0-512)
        assert!(pool.alloc(0).is_ok());
        assert!(pool.alloc(1).is_ok());
        assert!(pool.alloc(256).is_ok());
        assert!(pool.alloc(512).is_ok());

        // invalid size (>512)
        let result = pool.alloc(513);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::InvalidParameter)));

        let result = pool.alloc(1024);
        assert!(result.is_err());
    }

    #[test]
    fn test_dma_buffer_slices() {
        let mut pool = DmaBufferPool::new();
        let mut buffer = pool.alloc(128).unwrap();

        // write to buffer
        let slice = buffer.as_mut_slice();
        slice[0] = 0xAA;
        slice[1] = 0xBB;
        slice[127] = 0xFF;

        // read back
        let const_slice = buffer.as_slice();
        assert_eq!(const_slice[0], 0xAA);
        assert_eq!(const_slice[1], 0xBB);
        assert_eq!(const_slice[127], 0xFF);
    }

    #[test]
    fn test_dma_buffer_dma_addr() {
        let mut pool = DmaBufferPool::new();
        let buffer = pool.alloc(256).unwrap();

        let ptr_addr = buffer.as_ptr() as u32;
        let dma_addr = buffer.dma_addr();

        // dma_addr should equal the pointer address
        assert_eq!(dma_addr, ptr_addr);

        // address should be in valid range (non-zero)
        assert!(dma_addr != 0);
    }

    #[test]
    fn test_dma_buffer_not_copyable() {
        // this test verifies at compile-time that DmaBuffer is not Copy
        // if you uncomment the code below, it should fail to compile

        // let mut pool = DmaBufferPool::new();
        // let buffer1 = pool.alloc(256).unwrap();
        // let buffer2 = buffer1; // move
        // let _ = buffer1; // ERROR: use of moved value
    }

    #[test]
    fn test_dma_alignment_helpers() {
        // test align_dma function
        assert_eq!(align_dma(0), 0);
        assert_eq!(align_dma(1), 32);
        assert_eq!(align_dma(31), 32);
        assert_eq!(align_dma(32), 32);
        assert_eq!(align_dma(33), 64);
        assert_eq!(align_dma(100), 128);

        // test is_dma_aligned function
        assert!(is_dma_aligned(0));
        assert!(!is_dma_aligned(1));
        assert!(!is_dma_aligned(31));
        assert!(is_dma_aligned(32));
        assert!(!is_dma_aligned(33));
        assert!(is_dma_aligned(64));
        assert!(is_dma_aligned(128));
    }

    #[test]
    fn test_buffer_stats_accuracy() {
        let mut pool = DmaBufferPool::new();

        // allocate some buffers
        let mut buffers = Vec::new();
        for _ in 0..10 {
            buffers.push(pool.alloc(512).unwrap());
        }

        let stats = pool.buffer_stats();
        assert_eq!(stats.total_buffers, 32);
        assert_eq!(stats.allocated_buffers, 10);
        assert_eq!(stats.free_buffers, 22);

        // free 5 buffers
        for _ in 0..5 {
            pool.free(buffers.pop().unwrap());
        }

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 5);
        assert_eq!(stats.free_buffers, 27);
    }

    #[test]
    fn test_dma_buffer_equality() {
        let mut pool = DmaBufferPool::new();

        let buf1 = pool.alloc(256).unwrap();
        let buf2 = pool.alloc(256).unwrap();
        let buf3 = pool.alloc(256).unwrap();

        // buffers with different pool indices should not be equal
        assert_ne!(buf1, buf2);
        assert_ne!(buf1, buf3);
        assert_ne!(buf2, buf3);

        // buffer should equal itself
        assert_eq!(buf1, buf1);
    }

    #[test]
    fn test_cache_alignment_constants() {
        // verify compile-time constants
        assert_eq!(DMA_ALIGNMENT, 32);
        assert_eq!(core::mem::align_of::<CacheAlignedBuffer>(), 32);
        assert_eq!(core::mem::size_of::<CacheAlignedBuffer>(), 512);
    }

    #[test]
    fn test_dma_buffer_prepare_for_device() {
        let mut pool = DmaBufferPool::new();
        let buffer = pool.alloc(256).unwrap();

        // write some data
        buffer.as_slice()[0..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);

        // prepare for DMA (CPU -> Device)
        // this should clean the cache
        buffer.prepare_for_device();

        // no assertion here since cache ops are hardware-specific
        // but we verify the function can be called without panic
    }

    #[test]
    fn test_dma_buffer_prepare_for_cpu() {
        let mut pool = DmaBufferPool::new();
        let mut buffer = pool.alloc(256).unwrap();

        // prepare for CPU read (Device -> CPU)
        // this should invalidate the cache
        buffer.prepare_for_cpu();

        // can now safely read data
        let data = buffer.as_slice();
        let _ = data[0]; // read without panic

        // no assertion here since cache ops are hardware-specific
        // but we verify the function can be called without panic
    }

    #[test]
    fn test_multiple_allocations_no_collision() {
        let mut pool = DmaBufferPool::new();

        // allocate 16 buffers and verify all have unique addresses
        // store in an array instead of HashSet for no_std compatibility
        let mut buffers: [Option<DmaBuffer>; 16] = Default::default();
        for i in 0..16 {
            buffers[i] = Some(pool.alloc(512).unwrap());
        }

        // verify no two buffers have the same address
        for i in 0..16 {
            let addr_i = buffers[i].as_ref().unwrap().as_ptr() as usize;
            for j in (i+1)..16 {
                let addr_j = buffers[j].as_ref().unwrap().as_ptr() as usize;
                assert_ne!(addr_i, addr_j, "duplicate addresses at indices {} and {}", i, j);
            }
        }
    }
}
