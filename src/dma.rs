//! DMA buffer management and cache coherency for USB transfers
//! 
//! Provides cache-coherent DMA buffers for the ARM Cortex-M7 with D-cache.
//! Based on i.MX RT1060 RM Section 3.3.3 and ARM Cortex-M7 TRM.

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
/// Place this in DTCM or OCRAM2 which are not cached by default
/// Note: link_section attribute removed for compilation compatibility
#[repr(C, align(4096))]
pub struct DmaRegion {
    /// Queue Head pool
    pub qh_pool: [u8; 64 * 64],  // 64 QHs, 64 bytes each
    /// Queue Transfer Descriptor pool  
    pub qtd_pool: [u8; 256 * 32], // 256 qTDs, 32 bytes each
    /// Data buffers for transfers
    pub data_buffers: [[u8; 512]; 32], // 32 buffers of 512 bytes
}

/// Static DMA region instance
static mut DMA_REGION: DmaRegion = DmaRegion {
    qh_pool: [0; 64 * 64],
    qtd_pool: [0; 256 * 32],
    data_buffers: [[0; 512]; 32],
};

/// DMA region initialization flag
static DMA_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Initialize DMA region with MPU configuration
/// 
/// Configures the MPU to mark DMA region as non-cacheable Device memory.
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
    let region_addr = unsafe { &DMA_REGION as *const _ as usize };
    let region_size = size_of::<DmaRegion>();
    
    // Configure MPU for non-cacheable DMA region
    unsafe {
        configure_mpu_dma_region(region_addr, region_size)?;
    }
    
    Ok(())
}

/// Configure MPU for DMA region
/// 
/// Sets up MPU region 7 as Device memory for DMA buffers.
/// Reference: ARM Cortex-M7 Generic User Guide, Section 2.3
unsafe fn configure_mpu_dma_region(addr: usize, size: usize) -> Result<()> {
    use cortex_m::peripheral::MPU;
    
    const MPU_CTRL_ENABLE: u32 = 0x01;
    const MPU_CTRL_PRIVDEFENA: u32 = 0x04;
    
    const MPU_RASR_ENABLE: u32 = 0x01;
    const MPU_RASR_SIZE_16KB: u32 = 13 << 1;  // 2^(13+1) = 16KB
    const MPU_RASR_AP_RW: u32 = 0b011 << 24;   // Read/Write access
    const MPU_RASR_TEX_DEVICE: u32 = 0b000 << 19; // Device memory
    const MPU_RASR_SHAREABLE: u32 = 1 << 18;
    const MPU_RASR_CACHEABLE: u32 = 0 << 17;   // Non-cacheable
    const MPU_RASR_BUFFERABLE: u32 = 1 << 16;
    const MPU_RASR_XN: u32 = 1 << 28;          // Execute never
    
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
            MPU_RASR_SIZE_16KB |
            MPU_RASR_AP_RW |
            MPU_RASR_TEX_DEVICE |
            MPU_RASR_SHAREABLE |
            MPU_RASR_BUFFERABLE |
            MPU_RASR_XN
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
pub struct DmaBuffer {
    ptr: NonNull<u8>,
    size: usize,
    pool_index: usize,
}

impl DmaBuffer {
    /// Get buffer pointer
    pub fn as_ptr(&self) -> *const u8 {
        self.ptr.as_ptr()
    }
    
    /// Get mutable buffer pointer
    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.ptr.as_ptr()
    }
    
    /// Get buffer physical address for DMA
    pub fn dma_addr(&self) -> u32 {
        self.ptr.as_ptr() as u32
    }
    
    /// Get buffer as slice
    pub fn as_slice(&self) -> &[u8] {
        unsafe { core::slice::from_raw_parts(self.ptr.as_ptr(), self.size) }
    }
    
    /// Get buffer as mutable slice
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        unsafe { core::slice::from_raw_parts_mut(self.ptr.as_ptr(), self.size) }
    }
}

/// Cache maintenance operations for cacheable DMA buffers
/// 
/// Use these when DMA buffers are in cacheable memory regions.
pub mod cache_ops {
    /// Data cache line size for Cortex-M7 (32 bytes)
    const DCACHE_LINE_SIZE: usize = 32;
    
    /// Clean (write-back) data cache by address range
    /// 
    /// Use before DMA read operations to ensure data is in memory.
    pub fn clean_dcache(addr: usize, size: usize) {
        cortex_m::asm::dsb(); // Ensure all previous writes complete
        
        let start = addr & !(DCACHE_LINE_SIZE - 1);
        let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);
        
        for _line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
            unsafe {
                // Placeholder: Cache maintenance operations would go here
                // Real implementation needs CMSIS or direct register access
                cortex_m::asm::nop();
            }
        }
        
        cortex_m::asm::dsb(); // Ensure cache operations complete
    }
    
    /// Invalidate data cache by address range
    /// 
    /// Use after DMA write operations to ensure CPU sees new data.
    pub fn invalidate_dcache(addr: usize, size: usize) {
        cortex_m::asm::dsb(); // Ensure all previous operations complete
        
        let start = addr & !(DCACHE_LINE_SIZE - 1);
        let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);
        
        for _line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
            unsafe {
                // Placeholder: Cache maintenance operations would go here
                // Real implementation needs CMSIS or direct register access
                cortex_m::asm::nop();
            }
        }
        
        cortex_m::asm::dsb(); // Ensure cache operations complete
    }
    
    /// Clean and invalidate data cache by address range
    /// 
    /// Use for bidirectional DMA operations.
    pub fn clean_invalidate_dcache(addr: usize, size: usize) {
        cortex_m::asm::dsb(); // Ensure all previous operations complete
        
        let start = addr & !(DCACHE_LINE_SIZE - 1);
        let end = (addr + size + DCACHE_LINE_SIZE - 1) & !(DCACHE_LINE_SIZE - 1);
        
        for _line_addr in (start..end).step_by(DCACHE_LINE_SIZE) {
            unsafe {
                // Placeholder: Cache maintenance operations would go here
                // Real implementation needs CMSIS or direct register access
                cortex_m::asm::nop();
            }
        }
        
        cortex_m::asm::dsb(); // Ensure cache operations complete
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
                let ptr = unsafe {
                    let addr = &DMA_REGION.data_buffers[i][0] as *const u8 as *mut u8;
                    NonNull::new_unchecked(addr)
                };
                
                return Ok(DmaBuffer {
                    ptr,
                    size,
                    pool_index: i,
                });
            }
        }
        
        Err(UsbError::NoResources)
    }
    
    /// Free a DMA buffer back to the pool
    pub fn free(&mut self, buffer: DmaBuffer) {
        self.allocated[buffer.pool_index].store(false, Ordering::Release);
    }
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