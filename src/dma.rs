//! DMA buffer management for USB transfers

pub mod pools;
pub mod descriptor;
pub mod buffer;
pub mod memory;

pub use pools::{UsbDescriptorPool, DataBufferPool, BufferHandle, PoolStats};
pub use descriptor::{DescriptorAllocator, QhHandle, QtdHandle, DescriptorState};
pub use buffer::{BufferPool, BufferHandle as DmaBufferHandle, BufferSize};
pub use memory::{UsbMemoryPool, DmaBuffer as MemoryDmaBuffer};

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
#[derive(Clone, Copy)]
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
    pub fn as_ptr(&self) -> *const u8 {
        self.ptr.as_ptr()
    }
    
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr.as_ptr()
    }
    
    pub fn dma_addr(&self) -> u32 {
        self.ptr.as_ptr() as u32
    }
    
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.ptr.as_ptr(), self.size) }
    }
    
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.size) }
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
    buffers: heapless::Vec<DmaBuffer, 32>,
    allocated: [AtomicBool; 32],
}

impl DmaBufferPool {
    /// Create new DMA buffer pool
    pub const fn new() -> Self {
        const ATOMIC_BOOL_FALSE: AtomicBool = AtomicBool::new(false);
        Self {
            buffers: heapless::Vec::new(),
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
                
                // Store buffer in tracking vec
                if self.buffers.push(buffer).is_err() {
                    // Reset allocation on failure
                    allocated.store(false, Ordering::Release);
                    return Err(UsbError::NoResources);
                }
                
                // Return copy of the buffer
                return Ok(buffer);
            }
        }
        
        Err(UsbError::NoResources)
    }
    
    /// Get all allocated buffers for inspection
    pub fn get_allocated_buffers(&self) -> &[DmaBuffer] {
        &self.buffers
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
    pub fn free(&mut self, buffer: DmaBuffer) {
        // Find and remove buffer from tracking vec
        if let Some(pos) = self.buffers.iter().position(|b| b.pool_index == buffer.pool_index) {
            self.buffers.swap_remove(pos);
        }
        
        // Mark buffer as free
        self.allocated[buffer.pool_index].store(false, Ordering::Release);
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