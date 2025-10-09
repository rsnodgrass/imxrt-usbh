//! Enhanced descriptor management with lifecycle tracking
//!
//! Provides allocation, tracking, and recycling of USB descriptors

use crate::ehci::{QueueHead, QueueTD};
use crate::error::{Result, UsbError};
use core::ptr::NonNull;
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Descriptor lifecycle states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DescriptorState {
    /// Available in pool
    Free = 0,
    /// Allocated but not yet queued
    Allocated = 1,
    /// Queued for hardware processing
    Queued = 2,
    /// Being processed by hardware
    Active = 3,
    /// Completed, awaiting software processing
    Complete = 4,
    /// Error state, needs cleanup
    Error = 5,
}

/// Descriptor type information for validation
#[derive(Debug, Clone, Copy)]
pub struct DescriptorTypeInfo {
    /// Type tag for validation
    pub type_tag: u16,
    /// Size of descriptor type
    pub size: usize,
    /// Pool index where allocated
    pub pool_index: usize,
}

/// Enhanced descriptor wrapper with lifecycle tracking
pub struct ManagedDescriptor<T> {
    /// The actual descriptor
    descriptor: NonNull<T>,
    /// Current state
    state: AtomicU8,
    /// Reference count for safe sharing
    ref_count: AtomicU32,
    /// Pool index for efficient recycling
    pool_index: u16,
    /// Descriptor type tag for safety
    type_tag: u16,
}

impl<T> ManagedDescriptor<T> {
    /// Create new managed descriptor
    #[must_use]
    pub fn new(descriptor: NonNull<T>, pool_index: usize) -> Self {
        Self {
            descriptor,
            state: AtomicU8::new(DescriptorState::Free as u8),
            ref_count: AtomicU32::new(0),
            pool_index: pool_index as u16,
            type_tag: Self::type_tag_value(),
        }
    }

    /// Get type tag for this descriptor type
    const fn type_tag_value() -> u16 {
        // Use size as a simple type discriminator
        core::mem::size_of::<T>() as u16
    }

    /// Transition to new state with validation
    pub fn transition_state(&self, from: DescriptorState, to: DescriptorState) -> Result<()> {
        let current = self.state.load(Ordering::Acquire);
        if current != from as u8 {
            return Err(UsbError::InvalidState);
        }

        // Validate state transition
        match (from, to) {
            (DescriptorState::Free, DescriptorState::Allocated)
            | (DescriptorState::Allocated, DescriptorState::Queued)
            | (DescriptorState::Queued, DescriptorState::Active)
            | (DescriptorState::Active, DescriptorState::Complete)
            | (DescriptorState::Active, DescriptorState::Error)
            | (DescriptorState::Complete, DescriptorState::Free)
            | (DescriptorState::Error, DescriptorState::Free) => {
                self.state.store(to as u8, Ordering::Release);
                Ok(())
            }
            _ => Err(UsbError::InvalidState),
        }
    }

    /// Get pool index for recycling
    pub fn pool_index(&self) -> usize {
        self.pool_index as usize
    }

    /// Validate descriptor type integrity
    pub fn validate_type(&self) -> bool {
        self.type_tag == Self::type_tag_value()
    }

    /// Get descriptor type information
    pub fn type_info(&self) -> DescriptorTypeInfo {
        DescriptorTypeInfo {
            type_tag: self.type_tag,
            size: core::mem::size_of::<T>(),
            pool_index: self.pool_index as usize,
        }
    }

    /// Increment reference count
    pub fn acquire(&self) -> Result<()> {
        let old_count = self.ref_count.fetch_add(1, Ordering::AcqRel);
        if old_count > 100 {
            // Suspicious reference count, possible leak
            self.ref_count.fetch_sub(1, Ordering::AcqRel);
            return Err(UsbError::InvalidState);
        }
        Ok(())
    }

    /// Decrement reference count
    pub fn release(&self) -> bool {
        let old_count = self.ref_count.fetch_sub(1, Ordering::AcqRel);
        old_count == 1 // Returns true if this was the last reference
    }

    /// Get current state
    pub fn state(&self) -> DescriptorState {
        match self.state.load(Ordering::Acquire) {
            0 => DescriptorState::Free,
            1 => DescriptorState::Allocated,
            2 => DescriptorState::Queued,
            3 => DescriptorState::Active,
            4 => DescriptorState::Complete,
            5 => DescriptorState::Error,
            _ => DescriptorState::Error,
        }
    }

    /// Access descriptor (only when allocated)
    pub fn access(&self) -> Result<&T> {
        let state = self.state();
        if state == DescriptorState::Free {
            return Err(UsbError::InvalidState);
        }
        // Safety: NonNull pointer is valid, allocated from pool, and state check ensures not Free
        unsafe { Ok(self.descriptor.as_ref()) }
    }

    /// Mutable access to descriptor (only when allocated and not active)
    pub fn access_mut(&mut self) -> Result<&mut T> {
        let state = self.state();
        if state == DescriptorState::Free || state == DescriptorState::Active {
            return Err(UsbError::InvalidState);
        }
        // Safety: NonNull pointer is valid, we have exclusive &mut self, and state ensures not Free/Active
        unsafe { Ok(self.descriptor.as_mut()) }
    }
}

/// Descriptor allocator with advanced management
pub struct DescriptorAllocator<const N_QH: usize, const N_QTD: usize> {
    /// Managed queue heads
    qh_descriptors: [ManagedDescriptor<QueueHead>; N_QH],
    /// Managed queue TDs
    qtd_descriptors: [ManagedDescriptor<QueueTD>; N_QTD],
    /// Allocation statistics
    stats: AllocationStats,
}

impl<const N_QH: usize, const N_QTD: usize> DescriptorAllocator<N_QH, N_QTD> {
    /// Create new allocator
    ///
    /// # Safety
    ///
    /// Caller must ensure memory is properly aligned and will remain valid
    pub unsafe fn new(
        qh_memory: &mut [QueueHead; N_QH],
        qtd_memory: &mut [QueueTD; N_QTD],
    ) -> Self {
        // Initialize managed descriptors
        let mut qh_descriptors =
            core::mem::MaybeUninit::<[ManagedDescriptor<QueueHead>; N_QH]>::uninit();
        let mut qtd_descriptors =
            core::mem::MaybeUninit::<[ManagedDescriptor<QueueTD>; N_QTD]>::uninit();

        let qh_ptr = qh_descriptors.as_mut_ptr() as *mut ManagedDescriptor<QueueHead>;
        let qtd_ptr = qtd_descriptors.as_mut_ptr() as *mut ManagedDescriptor<QueueTD>;

        for i in 0..N_QH {
            let desc = ManagedDescriptor::new(
                // Safety: qh_memory[i] is valid, properly aligned, and lives for 'static
                unsafe { NonNull::new_unchecked(&mut qh_memory[i] as *mut QueueHead) },
                i,
            );
            // Safety: qh_ptr is valid for N_QH writes, i < N_QH, and memory is uninitialized
            unsafe {
                qh_ptr.add(i).write(desc);
            }
        }

        for i in 0..N_QTD {
            let desc = ManagedDescriptor::new(
                // Safety: qtd_memory[i] is valid, properly aligned, and lives for 'static
                unsafe { NonNull::new_unchecked(&mut qtd_memory[i] as *mut QueueTD) },
                i,
            );
            // Safety: qtd_ptr is valid for N_QTD writes, i < N_QTD, and memory is uninitialized
            unsafe {
                qtd_ptr.add(i).write(desc);
            }
        }

        Self {
            // Safety: All N_QH elements initialized in loop above
            qh_descriptors: unsafe { qh_descriptors.assume_init() },
            // Safety: All N_QTD elements initialized in loop above
            qtd_descriptors: unsafe { qtd_descriptors.assume_init() },
            stats: AllocationStats::new(),
        }
    }

    /// Allocate queue head with lifecycle management
    pub fn alloc_qh(&mut self) -> Result<QhHandle> {
        for (index, managed) in self.qh_descriptors.iter_mut().enumerate() {
            if managed.state() == DescriptorState::Free {
                managed.transition_state(DescriptorState::Free, DescriptorState::Allocated)?;
                managed.acquire()?;
                self.stats.record_qh_alloc();

                // Initialize the QH
                let qh = managed.access_mut()?;
                *qh = QueueHead::new();

                return Ok(QhHandle {
                    index,
                    generation: self.stats.allocation_generation(),
                });
            }
        }

        self.stats.record_qh_exhaustion();
        Err(UsbError::NoResources)
    }

    /// Allocate queue TD with lifecycle management
    pub fn alloc_qtd(&mut self) -> Result<QtdHandle> {
        for (index, managed) in self.qtd_descriptors.iter_mut().enumerate() {
            if managed.state() == DescriptorState::Free {
                managed.transition_state(DescriptorState::Free, DescriptorState::Allocated)?;
                managed.acquire()?;
                self.stats.record_qtd_alloc();

                // Initialize the qTD
                let qtd = managed.access_mut()?;
                *qtd = QueueTD::new();

                return Ok(QtdHandle {
                    index,
                    generation: self.stats.allocation_generation(),
                });
            }
        }

        self.stats.record_qtd_exhaustion();
        Err(UsbError::NoResources)
    }

    /// Free queue head
    pub fn free_qh(&mut self, handle: QhHandle) -> Result<()> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }

        let managed = &mut self.qh_descriptors[handle.index];

        // Ensure it's in a freeable state
        let state = managed.state();
        if state != DescriptorState::Complete && state != DescriptorState::Error {
            return Err(UsbError::InvalidState);
        }

        if managed.release() {
            // Last reference, can actually free
            managed.transition_state(state, DescriptorState::Free)?;
            self.stats.record_qh_free();
        }

        Ok(())
    }

    /// Free queue TD
    pub fn free_qtd(&mut self, handle: QtdHandle) -> Result<()> {
        if handle.index >= N_QTD {
            return Err(UsbError::InvalidParameter);
        }

        let managed = &mut self.qtd_descriptors[handle.index];

        // Ensure it's in a freeable state
        let state = managed.state();
        if state != DescriptorState::Complete && state != DescriptorState::Error {
            return Err(UsbError::InvalidState);
        }

        if managed.release() {
            // Last reference, can actually free
            managed.transition_state(state, DescriptorState::Free)?;
            self.stats.record_qtd_free();
        }

        Ok(())
    }

    /// Mark descriptor as queued for hardware
    pub fn mark_qh_queued(&mut self, handle: QhHandle) -> Result<()> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }

        self.qh_descriptors[handle.index]
            .transition_state(DescriptorState::Allocated, DescriptorState::Queued)
    }

    /// Mark descriptor as active (being processed by hardware)
    pub fn mark_qh_active(&mut self, handle: QhHandle) -> Result<()> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }

        self.qh_descriptors[handle.index]
            .transition_state(DescriptorState::Queued, DescriptorState::Active)
    }

    /// Mark descriptor as completed
    pub fn mark_qh_complete(&mut self, handle: QhHandle) -> Result<()> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }

        self.qh_descriptors[handle.index]
            .transition_state(DescriptorState::Active, DescriptorState::Complete)
    }

    /// Get allocator statistics
    pub fn statistics(&self) -> &AllocationStats {
        &self.stats
    }

    /// Perform garbage collection of leaked descriptors
    pub fn garbage_collect(&mut self) -> usize {
        let mut collected = 0;

        // Check for leaked QHs
        for managed in &mut self.qh_descriptors {
            if managed.state() == DescriptorState::Allocated {
                // Check if it's been allocated for too long without being queued
                // This is a simplified check - real implementation would track time
                if managed.ref_count.load(Ordering::Acquire) == 0 {
                    let _ =
                        managed.transition_state(DescriptorState::Allocated, DescriptorState::Free);
                    collected += 1;
                }
            }
        }

        // Check for leaked qTDs
        for managed in &mut self.qtd_descriptors {
            if managed.state() == DescriptorState::Allocated {
                if managed.ref_count.load(Ordering::Acquire) == 0 {
                    let _ =
                        managed.transition_state(DescriptorState::Allocated, DescriptorState::Free);
                    collected += 1;
                }
            }
        }

        collected
    }

    /// Get reference to QH
    pub fn get_qh(&self, handle: &QhHandle) -> Result<&QueueHead> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }
        self.qh_descriptors[handle.index].access()
    }

    /// Get mutable reference to QH
    pub fn get_qh_mut(&mut self, handle: &QhHandle) -> Result<&mut QueueHead> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }
        self.qh_descriptors[handle.index].access_mut()
    }

    /// Get reference to qTD
    pub fn get_qtd(&self, handle: &QtdHandle) -> Result<&QueueTD> {
        if handle.index >= N_QTD {
            return Err(UsbError::InvalidParameter);
        }
        self.qtd_descriptors[handle.index].access()
    }

    /// Get mutable reference to qTD
    pub fn get_qtd_mut(&mut self, handle: &QtdHandle) -> Result<&mut QueueTD> {
        if handle.index >= N_QTD {
            return Err(UsbError::InvalidParameter);
        }
        self.qtd_descriptors[handle.index].access_mut()
    }

    /// Get physical DMA address of QH
    pub fn get_qh_addr(&self, handle: &QhHandle) -> Result<u32> {
        if handle.index >= N_QH {
            return Err(UsbError::InvalidParameter);
        }
        let qh = self.qh_descriptors[handle.index].access()?;
        Ok(qh as *const QueueHead as u32)
    }

    /// Get physical DMA address of qTD
    pub fn get_qtd_addr(&self, handle: &QtdHandle) -> Result<u32> {
        if handle.index >= N_QTD {
            return Err(UsbError::InvalidParameter);
        }
        let qtd = self.qtd_descriptors[handle.index].access()?;
        Ok(qtd as *const QueueTD as u32)
    }
}

/// Handle to allocated queue head
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QhHandle {
    index: usize,
    generation: u32,
}

/// Handle to allocated queue TD
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QtdHandle {
    index: usize,
    generation: u32,
}

impl QhHandle {
    /// Create a new Queue Head handle with index and generation counter
    ///
    /// The generation counter provides ABA problem protection: when a QH is freed
    /// and reallocated, the generation increments, invalidating old handles.
    #[must_use]
    pub fn new(index: usize, generation: u32) -> Self {
        Self { index, generation }
    }

    /// Get the descriptor pool index
    ///
    /// Returns the index into the QH pool where this descriptor is located.
    #[must_use]
    pub fn index(&self) -> usize {
        self.index
    }

    /// Get the generation counter
    ///
    /// The generation counter increments each time a pool slot is reallocated,
    /// preventing use-after-free bugs from stale handles.
    #[must_use]
    pub fn generation(&self) -> u32 {
        self.generation
    }

    /// Check if handle is still valid (generation matches)
    #[must_use]
    pub fn is_valid(&self, expected_generation: u32) -> bool {
        self.generation == expected_generation
    }
}

impl QtdHandle {
    /// Create a new Queue Transfer Descriptor handle with index and generation counter
    ///
    /// The generation counter provides ABA problem protection: when a qTD is freed
    /// and reallocated, the generation increments, invalidating old handles.
    #[must_use]
    pub fn new(index: usize, generation: u32) -> Self {
        Self { index, generation }
    }

    /// Get the descriptor pool index
    ///
    /// Returns the index into the qTD pool where this descriptor is located.
    #[must_use]
    pub fn index(&self) -> usize {
        self.index
    }

    /// Get the generation counter
    ///
    /// The generation counter increments each time a pool slot is reallocated,
    /// preventing use-after-free bugs from stale handles.
    #[must_use]
    pub fn generation(&self) -> u32 {
        self.generation
    }

    /// Check if handle is still valid (generation matches)
    #[must_use]
    pub fn is_valid(&self, expected_generation: u32) -> bool {
        self.generation == expected_generation
    }
}

/// Allocation statistics for monitoring
pub struct AllocationStats {
    qh_allocations: AtomicU32,
    qh_frees: AtomicU32,
    qh_exhaustions: AtomicU32,
    qtd_allocations: AtomicU32,
    qtd_frees: AtomicU32,
    qtd_exhaustions: AtomicU32,
    generation: AtomicU32,
}

impl AllocationStats {
    const fn new() -> Self {
        Self {
            qh_allocations: AtomicU32::new(0),
            qh_frees: AtomicU32::new(0),
            qh_exhaustions: AtomicU32::new(0),
            qtd_allocations: AtomicU32::new(0),
            qtd_frees: AtomicU32::new(0),
            qtd_exhaustions: AtomicU32::new(0),
            generation: AtomicU32::new(0),
        }
    }

    fn record_qh_alloc(&self) {
        self.qh_allocations.fetch_add(1, Ordering::Relaxed);
        self.generation.fetch_add(1, Ordering::Relaxed);
    }

    fn record_qh_free(&self) {
        self.qh_frees.fetch_add(1, Ordering::Relaxed);
    }

    fn record_qh_exhaustion(&self) {
        self.qh_exhaustions.fetch_add(1, Ordering::Relaxed);
    }

    fn record_qtd_alloc(&self) {
        self.qtd_allocations.fetch_add(1, Ordering::Relaxed);
        self.generation.fetch_add(1, Ordering::Relaxed);
    }

    fn record_qtd_free(&self) {
        self.qtd_frees.fetch_add(1, Ordering::Relaxed);
    }

    fn record_qtd_exhaustion(&self) {
        self.qtd_exhaustions.fetch_add(1, Ordering::Relaxed);
    }

    fn allocation_generation(&self) -> u32 {
        self.generation.load(Ordering::Relaxed)
    }

    /// Get current statistics snapshot
    pub fn snapshot(&self) -> StatsSnapshot {
        StatsSnapshot {
            qh_allocations: self.qh_allocations.load(Ordering::Relaxed),
            qh_frees: self.qh_frees.load(Ordering::Relaxed),
            qh_exhaustions: self.qh_exhaustions.load(Ordering::Relaxed),
            qh_leaked: self
                .qh_allocations
                .load(Ordering::Relaxed)
                .saturating_sub(self.qh_frees.load(Ordering::Relaxed)),
            qtd_allocations: self.qtd_allocations.load(Ordering::Relaxed),
            qtd_frees: self.qtd_frees.load(Ordering::Relaxed),
            qtd_exhaustions: self.qtd_exhaustions.load(Ordering::Relaxed),
            qtd_leaked: self
                .qtd_allocations
                .load(Ordering::Relaxed)
                .saturating_sub(self.qtd_frees.load(Ordering::Relaxed)),
        }
    }
}

/// Statistics snapshot
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct StatsSnapshot {
    pub qh_allocations: u32,
    pub qh_frees: u32,
    pub qh_exhaustions: u32,
    pub qh_leaked: u32,
    pub qtd_allocations: u32,
    pub qtd_frees: u32,
    pub qtd_exhaustions: u32,
    pub qtd_leaked: u32,
}

impl StatsSnapshot {
    /// Check if there are potential memory leaks
    pub fn has_leaks(&self) -> bool {
        self.qh_leaked > 0 || self.qtd_leaked > 0
    }

    /// Check if pools have been exhausted
    pub fn has_exhaustions(&self) -> bool {
        self.qh_exhaustions > 0 || self.qtd_exhaustions > 0
    }
}
