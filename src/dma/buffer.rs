//! DMA buffer management with size classes and alignment
//! 
//! Provides efficient buffer allocation for USB transfers

use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::ptr::NonNull;

/// Buffer size classes for efficient allocation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BufferSize {
    /// 8 bytes - minimal control transfers
    Tiny = 0,
    /// 64 bytes - control/interrupt transfers
    Small = 1,
    /// 512 bytes - bulk transfers (FS)
    Medium = 2,
    /// 1024 bytes - bulk transfers (HS)
    Large = 3,
    /// 4096 bytes - large bulk transfers
    Huge = 4,
}

impl BufferSize {
    /// Get actual size in bytes
    pub const fn bytes(&self) -> usize {
        match self {
            BufferSize::Tiny => 8,
            BufferSize::Small => 64,
            BufferSize::Medium => 512,
            BufferSize::Large => 1024,
            BufferSize::Huge => 4096,
        }
    }
    
    /// Select appropriate size class for requested bytes
    pub const fn for_bytes(bytes: usize) -> Option<Self> {
        if bytes <= 8 {
            Some(BufferSize::Tiny)
        } else if bytes <= 64 {
            Some(BufferSize::Small)
        } else if bytes <= 512 {
            Some(BufferSize::Medium)
        } else if bytes <= 1024 {
            Some(BufferSize::Large)
        } else if bytes <= 4096 {
            Some(BufferSize::Huge)
        } else {
            None
        }
    }
}

/// DMA buffer with metadata
pub struct DmaBuffer {
    /// Pointer to buffer data
    data: NonNull<u8>,
    /// Buffer size class
    size_class: BufferSize,
    /// Pool index for recycling
    #[allow(dead_code)]
    pool_index: u16,
    /// Currently allocated
    allocated: AtomicBool,
    /// Physical address for DMA
    phys_addr: u32,
}

impl DmaBuffer {
    /// Create new DMA buffer
    /// 
    /// # Safety
    /// 
    /// Caller must ensure memory is DMA-capable and properly aligned
    pub unsafe fn new(data: NonNull<u8>, size_class: BufferSize, pool_index: usize, phys_addr: u32) -> Self {
        Self {
            data,
            size_class,
            pool_index: pool_index as u16,
            allocated: AtomicBool::new(false),
            phys_addr,
        }
    }
    
    /// Try to allocate this buffer
    pub fn try_allocate(&self) -> bool {
        self.allocated.compare_exchange(
            false,
            true,
            Ordering::AcqRel,
            Ordering::Acquire
        ).is_ok()
    }
    
    /// Free this buffer
    pub fn free(&self) {
        self.allocated.store(false, Ordering::Release);
    }
    
    /// Get buffer data pointer
    pub fn data(&self) -> NonNull<u8> {
        self.data
    }
    
    /// Get physical address for DMA
    pub fn physical_address(&self) -> u32 {
        self.phys_addr
    }
    
    /// Get buffer size
    pub fn size(&self) -> usize {
        self.size_class.bytes()
    }
    
    /// Check if allocated
    pub fn is_allocated(&self) -> bool {
        self.allocated.load(Ordering::Acquire)
    }
}

/// Buffer pool with multiple size classes
pub struct BufferPool<
    const N_TINY: usize,
    const N_SMALL: usize,
    const N_MEDIUM: usize,
    const N_LARGE: usize,
    const N_HUGE: usize,
> {
    /// Tiny buffers (8 bytes)
    tiny_buffers: [DmaBuffer; N_TINY],
    tiny_storage: [[u8; 8]; N_TINY],
    
    /// Small buffers (64 bytes)
    small_buffers: [DmaBuffer; N_SMALL],
    small_storage: [[u8; 64]; N_SMALL],
    
    /// Medium buffers (512 bytes)
    medium_buffers: [DmaBuffer; N_MEDIUM],
    medium_storage: [[u8; 512]; N_MEDIUM],
    
    /// Large buffers (1024 bytes)
    large_buffers: [DmaBuffer; N_LARGE],
    large_storage: [[u8; 1024]; N_LARGE],
    
    /// Huge buffers (4096 bytes)
    huge_buffers: [DmaBuffer; N_HUGE],
    huge_storage: [[u8; 4096]; N_HUGE],
    
    /// Allocation statistics
    stats: BufferPoolStats,
}

impl<
    const N_TINY: usize,
    const N_SMALL: usize,
    const N_MEDIUM: usize,
    const N_LARGE: usize,
    const N_HUGE: usize,
> BufferPool<N_TINY, N_SMALL, N_MEDIUM, N_LARGE, N_HUGE> {
    
    /// Create new buffer pool
    /// 
    /// # Safety
    /// 
    /// Caller must ensure this is created in DMA-capable memory region
    pub unsafe fn new(base_phys_addr: u32) -> Self {
        let mut phys_addr = base_phys_addr;
        
        // Initialize tiny buffers
        let mut tiny_buffers = core::mem::MaybeUninit::<[DmaBuffer; N_TINY]>::uninit();
        let mut tiny_storage = [[0u8; 8]; N_TINY];
        let tiny_ptr = tiny_buffers.as_mut_ptr() as *mut DmaBuffer;
        
        for i in 0..N_TINY {
            let data = unsafe { NonNull::new_unchecked(tiny_storage[i].as_mut_ptr()) };
            let buffer = unsafe { DmaBuffer::new(data, BufferSize::Tiny, i, phys_addr) };
            unsafe {
                tiny_ptr.add(i).write(buffer);
            }
            phys_addr += 8;
        }
        
        // Initialize small buffers
        let mut small_buffers = core::mem::MaybeUninit::<[DmaBuffer; N_SMALL]>::uninit();
        let mut small_storage = [[0u8; 64]; N_SMALL];
        let small_ptr = small_buffers.as_mut_ptr() as *mut DmaBuffer;
        
        for i in 0..N_SMALL {
            let data = unsafe { NonNull::new_unchecked(small_storage[i].as_mut_ptr()) };
            let buffer = unsafe { DmaBuffer::new(data, BufferSize::Small, i, phys_addr) };
            unsafe {
                small_ptr.add(i).write(buffer);
            }
            phys_addr += 64;
        }
        
        // Initialize medium buffers
        let mut medium_buffers = core::mem::MaybeUninit::<[DmaBuffer; N_MEDIUM]>::uninit();
        let mut medium_storage = [[0u8; 512]; N_MEDIUM];
        let medium_ptr = medium_buffers.as_mut_ptr() as *mut DmaBuffer;
        
        for i in 0..N_MEDIUM {
            let data = unsafe { NonNull::new_unchecked(medium_storage[i].as_mut_ptr()) };
            let buffer = unsafe { DmaBuffer::new(data, BufferSize::Medium, i, phys_addr) };
            unsafe {
                medium_ptr.add(i).write(buffer);
            }
            phys_addr += 512;
        }
        
        // Initialize large buffers
        let mut large_buffers = core::mem::MaybeUninit::<[DmaBuffer; N_LARGE]>::uninit();
        let mut large_storage = [[0u8; 1024]; N_LARGE];
        let large_ptr = large_buffers.as_mut_ptr() as *mut DmaBuffer;
        
        for i in 0..N_LARGE {
            let data = unsafe { NonNull::new_unchecked(large_storage[i].as_mut_ptr()) };
            let buffer = unsafe { DmaBuffer::new(data, BufferSize::Large, i, phys_addr) };
            unsafe {
                large_ptr.add(i).write(buffer);
            }
            phys_addr += 1024;
        }
        
        // Initialize huge buffers
        let mut huge_buffers = core::mem::MaybeUninit::<[DmaBuffer; N_HUGE]>::uninit();
        let mut huge_storage = [[0u8; 4096]; N_HUGE];
        let huge_ptr = huge_buffers.as_mut_ptr() as *mut DmaBuffer;
        
        for i in 0..N_HUGE {
            let data = unsafe { NonNull::new_unchecked(huge_storage[i].as_mut_ptr()) };
            let buffer = unsafe { DmaBuffer::new(data, BufferSize::Huge, i, phys_addr) };
            unsafe {
                huge_ptr.add(i).write(buffer);
            }
            phys_addr += 4096;
        }
        
        Self {
            tiny_buffers: unsafe { tiny_buffers.assume_init() },
            tiny_storage,
            small_buffers: unsafe { small_buffers.assume_init() },
            small_storage,
            medium_buffers: unsafe { medium_buffers.assume_init() },
            medium_storage,
            large_buffers: unsafe { large_buffers.assume_init() },
            large_storage,
            huge_buffers: unsafe { huge_buffers.assume_init() },
            huge_storage,
            stats: BufferPoolStats::new(),
        }
    }
    
    /// Allocate buffer of requested size
    pub fn alloc(&mut self, size: usize) -> Result<BufferHandle> {
        let size_class = BufferSize::for_bytes(size)
            .ok_or(UsbError::BufferOverflow)?;
        
        match size_class {
            BufferSize::Tiny => {
                for (i, buffer) in self.tiny_buffers.iter().enumerate() {
                    if buffer.try_allocate() {
                        self.stats.record_alloc(size_class);
                        return Ok(BufferHandle {
                            size_class,
                            index: i,
                            phys_addr: buffer.physical_address(),
                        });
                    }
                }
                self.stats.record_exhaustion(size_class);
                Err(UsbError::NoResources)
            }
            BufferSize::Small => {
                for (i, buffer) in self.small_buffers.iter().enumerate() {
                    if buffer.try_allocate() {
                        self.stats.record_alloc(size_class);
                        return Ok(BufferHandle {
                            size_class,
                            index: i,
                            phys_addr: buffer.physical_address(),
                        });
                    }
                }
                self.stats.record_exhaustion(size_class);
                Err(UsbError::NoResources)
            }
            BufferSize::Medium => {
                for (i, buffer) in self.medium_buffers.iter().enumerate() {
                    if buffer.try_allocate() {
                        self.stats.record_alloc(size_class);
                        return Ok(BufferHandle {
                            size_class,
                            index: i,
                            phys_addr: buffer.physical_address(),
                        });
                    }
                }
                self.stats.record_exhaustion(size_class);
                Err(UsbError::NoResources)
            }
            BufferSize::Large => {
                for (i, buffer) in self.large_buffers.iter().enumerate() {
                    if buffer.try_allocate() {
                        self.stats.record_alloc(size_class);
                        return Ok(BufferHandle {
                            size_class,
                            index: i,
                            phys_addr: buffer.physical_address(),
                        });
                    }
                }
                self.stats.record_exhaustion(size_class);
                Err(UsbError::NoResources)
            }
            BufferSize::Huge => {
                for (i, buffer) in self.huge_buffers.iter().enumerate() {
                    if buffer.try_allocate() {
                        self.stats.record_alloc(size_class);
                        return Ok(BufferHandle {
                            size_class,
                            index: i,
                            phys_addr: buffer.physical_address(),
                        });
                    }
                }
                self.stats.record_exhaustion(size_class);
                Err(UsbError::NoResources)
            }
        }
    }
    
    /// Free buffer
    pub fn free(&mut self, handle: BufferHandle) -> Result<()> {
        match handle.size_class {
            BufferSize::Tiny => {
                if handle.index >= N_TINY {
                    return Err(UsbError::InvalidParameter);
                }
                self.tiny_buffers[handle.index].free();
            }
            BufferSize::Small => {
                if handle.index >= N_SMALL {
                    return Err(UsbError::InvalidParameter);
                }
                self.small_buffers[handle.index].free();
            }
            BufferSize::Medium => {
                if handle.index >= N_MEDIUM {
                    return Err(UsbError::InvalidParameter);
                }
                self.medium_buffers[handle.index].free();
            }
            BufferSize::Large => {
                if handle.index >= N_LARGE {
                    return Err(UsbError::InvalidParameter);
                }
                self.large_buffers[handle.index].free();
            }
            BufferSize::Huge => {
                if handle.index >= N_HUGE {
                    return Err(UsbError::InvalidParameter);
                }
                self.huge_buffers[handle.index].free();
            }
        }
        
        self.stats.record_free(handle.size_class);
        Ok(())
    }
    
    /// Get buffer data for writing
    pub fn get_buffer_mut(&mut self, handle: &BufferHandle) -> Result<&mut [u8]> {
        match handle.size_class {
            BufferSize::Tiny => {
                if handle.index >= N_TINY {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&mut self.tiny_storage[handle.index])
            }
            BufferSize::Small => {
                if handle.index >= N_SMALL {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&mut self.small_storage[handle.index])
            }
            BufferSize::Medium => {
                if handle.index >= N_MEDIUM {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&mut self.medium_storage[handle.index])
            }
            BufferSize::Large => {
                if handle.index >= N_LARGE {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&mut self.large_storage[handle.index])
            }
            BufferSize::Huge => {
                if handle.index >= N_HUGE {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&mut self.huge_storage[handle.index])
            }
        }
    }
    
    /// Get buffer data for reading
    pub fn get_buffer(&self, handle: &BufferHandle) -> Result<&[u8]> {
        match handle.size_class {
            BufferSize::Tiny => {
                if handle.index >= N_TINY {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&self.tiny_storage[handle.index])
            }
            BufferSize::Small => {
                if handle.index >= N_SMALL {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&self.small_storage[handle.index])
            }
            BufferSize::Medium => {
                if handle.index >= N_MEDIUM {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&self.medium_storage[handle.index])
            }
            BufferSize::Large => {
                if handle.index >= N_LARGE {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&self.large_storage[handle.index])
            }
            BufferSize::Huge => {
                if handle.index >= N_HUGE {
                    return Err(UsbError::InvalidParameter);
                }
                Ok(&self.huge_storage[handle.index])
            }
        }
    }
    
    /// Get pool statistics
    pub fn statistics(&self) -> BufferPoolStatsSnapshot {
        self.stats.snapshot()
    }
}

/// Handle to allocated buffer
#[derive(Debug, Clone, Copy)]
pub struct BufferHandle {
    size_class: BufferSize,
    index: usize,
    phys_addr: u32,
}

impl BufferHandle {
    /// Get physical address for DMA
    pub fn physical_address(&self) -> u32 {
        self.phys_addr
    }
    
    /// Get buffer size
    pub fn size(&self) -> usize {
        self.size_class.bytes()
    }
}

/// Buffer pool statistics
struct BufferPoolStats {
    tiny_allocs: AtomicU32,
    tiny_frees: AtomicU32,
    tiny_exhaustions: AtomicU32,
    
    small_allocs: AtomicU32,
    small_frees: AtomicU32,
    small_exhaustions: AtomicU32,
    
    medium_allocs: AtomicU32,
    medium_frees: AtomicU32,
    medium_exhaustions: AtomicU32,
    
    large_allocs: AtomicU32,
    large_frees: AtomicU32,
    large_exhaustions: AtomicU32,
    
    huge_allocs: AtomicU32,
    huge_frees: AtomicU32,
    huge_exhaustions: AtomicU32,
}

impl BufferPoolStats {
    const fn new() -> Self {
        Self {
            tiny_allocs: AtomicU32::new(0),
            tiny_frees: AtomicU32::new(0),
            tiny_exhaustions: AtomicU32::new(0),
            
            small_allocs: AtomicU32::new(0),
            small_frees: AtomicU32::new(0),
            small_exhaustions: AtomicU32::new(0),
            
            medium_allocs: AtomicU32::new(0),
            medium_frees: AtomicU32::new(0),
            medium_exhaustions: AtomicU32::new(0),
            
            large_allocs: AtomicU32::new(0),
            large_frees: AtomicU32::new(0),
            large_exhaustions: AtomicU32::new(0),
            
            huge_allocs: AtomicU32::new(0),
            huge_frees: AtomicU32::new(0),
            huge_exhaustions: AtomicU32::new(0),
        }
    }
    
    fn record_alloc(&self, size_class: BufferSize) {
        match size_class {
            BufferSize::Tiny => self.tiny_allocs.fetch_add(1, Ordering::Relaxed),
            BufferSize::Small => self.small_allocs.fetch_add(1, Ordering::Relaxed),
            BufferSize::Medium => self.medium_allocs.fetch_add(1, Ordering::Relaxed),
            BufferSize::Large => self.large_allocs.fetch_add(1, Ordering::Relaxed),
            BufferSize::Huge => self.huge_allocs.fetch_add(1, Ordering::Relaxed),
        };
    }
    
    fn record_free(&self, size_class: BufferSize) {
        match size_class {
            BufferSize::Tiny => self.tiny_frees.fetch_add(1, Ordering::Relaxed),
            BufferSize::Small => self.small_frees.fetch_add(1, Ordering::Relaxed),
            BufferSize::Medium => self.medium_frees.fetch_add(1, Ordering::Relaxed),
            BufferSize::Large => self.large_frees.fetch_add(1, Ordering::Relaxed),
            BufferSize::Huge => self.huge_frees.fetch_add(1, Ordering::Relaxed),
        };
    }
    
    fn record_exhaustion(&self, size_class: BufferSize) {
        match size_class {
            BufferSize::Tiny => self.tiny_exhaustions.fetch_add(1, Ordering::Relaxed),
            BufferSize::Small => self.small_exhaustions.fetch_add(1, Ordering::Relaxed),
            BufferSize::Medium => self.medium_exhaustions.fetch_add(1, Ordering::Relaxed),
            BufferSize::Large => self.large_exhaustions.fetch_add(1, Ordering::Relaxed),
            BufferSize::Huge => self.huge_exhaustions.fetch_add(1, Ordering::Relaxed),
        };
    }
    
    fn snapshot(&self) -> BufferPoolStatsSnapshot {
        BufferPoolStatsSnapshot {
            tiny: SizeClassStats {
                allocations: self.tiny_allocs.load(Ordering::Relaxed),
                frees: self.tiny_frees.load(Ordering::Relaxed),
                exhaustions: self.tiny_exhaustions.load(Ordering::Relaxed),
            },
            small: SizeClassStats {
                allocations: self.small_allocs.load(Ordering::Relaxed),
                frees: self.small_frees.load(Ordering::Relaxed),
                exhaustions: self.small_exhaustions.load(Ordering::Relaxed),
            },
            medium: SizeClassStats {
                allocations: self.medium_allocs.load(Ordering::Relaxed),
                frees: self.medium_frees.load(Ordering::Relaxed),
                exhaustions: self.medium_exhaustions.load(Ordering::Relaxed),
            },
            large: SizeClassStats {
                allocations: self.large_allocs.load(Ordering::Relaxed),
                frees: self.large_frees.load(Ordering::Relaxed),
                exhaustions: self.large_exhaustions.load(Ordering::Relaxed),
            },
            huge: SizeClassStats {
                allocations: self.huge_allocs.load(Ordering::Relaxed),
                frees: self.huge_frees.load(Ordering::Relaxed),
                exhaustions: self.huge_exhaustions.load(Ordering::Relaxed),
            },
        }
    }
}

/// Statistics snapshot for buffer pool
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct BufferPoolStatsSnapshot {
    pub tiny: SizeClassStats,
    pub small: SizeClassStats,
    pub medium: SizeClassStats,
    pub large: SizeClassStats,
    pub huge: SizeClassStats,
}

/// Statistics for a single size class
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct SizeClassStats {
    pub allocations: u32,
    pub frees: u32,
    pub exhaustions: u32,
}

impl SizeClassStats {
    /// Get leaked buffer count
    pub fn leaked(&self) -> u32 {
        self.allocations.saturating_sub(self.frees)
    }
}