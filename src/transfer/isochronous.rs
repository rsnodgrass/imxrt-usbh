//! USB Isochronous transfer implementation
//! 
//! Implements USB 2.0 isochronous transfers for real-time streaming
//! applications like audio devices, video capture, and other 
//! time-sensitive data transfers that require guaranteed bandwidth.

use crate::error::{Result, UsbError};
use crate::dma::{QhHandle, QtdHandle, DescriptorAllocator, DmaBuffer, cache_ops};
use crate::transfer::Direction;
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

/// Isochronous transfer states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum IsochronousState {
    /// Initial state
    Idle = 0,
    /// Transfer scheduled and waiting for frame
    Scheduled = 1,
    /// Transfer in progress
    Active = 2,
    /// Transfer complete
    Complete = 3,
    /// Transfer failed (note: no retries for iso transfers)
    Failed = 4,
}

/// Microframe timing for high-speed isochronous transfers
#[derive(Debug, Clone, Copy)]
pub struct MicroframeTiming {
    /// Number of transactions per microframe (1-3 for high-speed)
    transactions_per_uframe: u8,
    /// Additional opportunities for transaction
    additional_opportunities: u8,
}

impl MicroframeTiming {
    /// Create timing for single transaction per microframe
    pub const fn single() -> Self {
        Self {
            transactions_per_uframe: 1,
            additional_opportunities: 0,
        }
    }
    
    /// Create timing for multiple transactions per microframe
    pub const fn multiple(transactions: u8, additional: u8) -> Self {
        let limited_transactions = if transactions > 3 { 3 } else { transactions };
        Self {
            transactions_per_uframe: limited_transactions,
            additional_opportunities: additional,
        }
    }
    
    pub fn additional_opportunities(&self) -> u8 {
        self.additional_opportunities
    }
    
    pub fn total_bandwidth_slots(&self) -> u8 {
        self.transactions_per_uframe + self.additional_opportunities
    }
    
    pub const fn double() -> Self {
        Self {
            transactions_per_uframe: 2,
            additional_opportunities: 1,
        }
    }
    
    /// Create timing for triple transaction per microframe (high-speed only)
    pub const fn triple() -> Self {
        Self {
            transactions_per_uframe: 3,
            additional_opportunities: 2,
        }
    }
}

/// Isochronous transfer context for real-time endpoint
pub struct IsochronousTransfer {
    /// Current state
    state: AtomicU8,
    /// Transfer direction
    direction: Direction,
    /// Device address
    device_address: u8,
    /// Endpoint number (without direction bit)
    endpoint: u8,
    /// Maximum packet size for this endpoint
    max_packet_size: u16,
    /// Data buffers for transfer (ping-pong buffering)
    data_buffers: [Option<DmaBuffer>; 2],
    /// Current buffer index (0 or 1)
    current_buffer: AtomicU8,
    /// Bytes transferred in current frame
    bytes_transferred: AtomicU32,
    /// Polling interval in frames (1 for isochronous)
    interval_frames: u8,
    /// Next scheduled frame number
    next_frame: AtomicU32,
    /// Microframe timing (for high-speed)
    microframe_timing: MicroframeTiming,
    /// Number of consecutive errors
    error_count: AtomicU8,
    /// Maximum consecutive errors before giving up
    max_errors: u8,
    /// Queue head handle
    qh_handle: Option<QhHandle>,
    /// Current qTD handles (one per microframe transaction)
    qtd_handles: [Option<QtdHandle>; 3],
    /// Whether this is a continuous streaming transfer
    is_streaming: bool,
    /// Frame counter for statistics
    frame_count: AtomicU32,
    /// Error frame count for statistics
    error_frame_count: AtomicU32,
}

impl IsochronousTransfer {
    /// Create new isochronous transfer
    pub fn new(
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer1: DmaBuffer,
        buffer2: DmaBuffer,
        microframe_timing: MicroframeTiming,
        is_streaming: bool,
    ) -> Self {
        Self {
            state: AtomicU8::new(IsochronousState::Idle as u8),
            direction,
            device_address,
            endpoint,
            max_packet_size,
            data_buffers: [Some(buffer1), Some(buffer2)],
            current_buffer: AtomicU8::new(0),
            bytes_transferred: AtomicU32::new(0),
            interval_frames: 1, // Always 1 for isochronous
            next_frame: AtomicU32::new(0),
            microframe_timing,
            error_count: AtomicU8::new(0),
            max_errors: 10, // Higher tolerance for isochronous
            qh_handle: None,
            qtd_handles: [None, None, None],
            is_streaming,
            frame_count: AtomicU32::new(0),
            error_frame_count: AtomicU32::new(0),
        }
    }
    
    /// Get current state
    pub fn state(&self) -> IsochronousState {
        match self.state.load(Ordering::Acquire) {
            0 => IsochronousState::Idle,
            1 => IsochronousState::Scheduled,
            2 => IsochronousState::Active,
            3 => IsochronousState::Complete,
            4 => IsochronousState::Failed,
            _ => IsochronousState::Failed,
        }
    }
    
    /// Transition to new state
    fn transition_state(&self, new_state: IsochronousState) {
        self.state.store(new_state as u8, Ordering::Release);
    }
    
    /// Schedule isochronous transfer for next frame
    pub fn schedule<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        current_frame: u32,
    ) -> Result<()> {
        if !matches!(self.state(), IsochronousState::Idle | IsochronousState::Complete) {
            return Err(UsbError::InvalidState);
        }
        
        // Calculate next frame for this transfer
        let next_frame = if self.state() == IsochronousState::Idle {
            // First scheduling - start a few frames in the future to allow setup
            current_frame + 3
        } else {
            // Streaming reschedule - next frame
            let last_frame = self.next_frame.load(Ordering::Acquire);
            last_frame + 1
        };
        
        self.next_frame.store(next_frame, Ordering::Release);
        
        // Allocate queue head for isochronous transfer if not already allocated
        if self.qh_handle.is_none() {
            let qh_handle = allocator.alloc_qh()?;
            self.qh_handle = Some(qh_handle);
        }
        
        // Prepare current DMA buffer for transfer
        let buffer_index = self.current_buffer.load(Ordering::Acquire) as usize;
        if let Some(ref mut buffer) = self.data_buffers[buffer_index] {
            match self.direction {
                Direction::Out => {
                    // CPU -> Device: clean cache to ensure data is in memory
                    cache_ops::prepare_for_dma_write(buffer.as_slice());
                }
                Direction::In => {
                    // Device -> CPU: invalidate cache so CPU will read from memory
                    cache_ops::prepare_for_dma_read(buffer.as_mut_slice());
                }
            }
        }
        
        self.transition_state(IsochronousState::Scheduled);
        
        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Isochronous transfer scheduled: addr={}, ep={}, frame={}, uframes={}",
            self.device_address,
            self.endpoint,
            next_frame,
            self.microframe_timing.transactions_per_uframe
        );
        
        Ok(())
    }
    
    /// Get polling interval in frames
    pub fn interval_frames(&self) -> u8 {
        self.interval_frames
    }
    
    /// Calculate next frame based on interval
    pub fn calculate_next_frame(&self, current_frame: u32) -> u32 {
        current_frame + (self.interval_frames as u32)
    }
    
    /// Check if transfer should be scheduled in this frame
    pub fn should_schedule_in_frame(&self, frame_number: u32) -> bool {
        let expected_frame = self.next_frame.load(Ordering::Relaxed);
        frame_number >= expected_frame
    }
    
    /// Start isochronous transfer when scheduled frame arrives
    pub fn start_if_ready<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        current_frame: u32,
    ) -> Result<bool> {
        if self.state() != IsochronousState::Scheduled {
            return Ok(false); // Not ready
        }
        
        let scheduled_frame = self.next_frame.load(Ordering::Acquire);
        if current_frame < scheduled_frame {
            return Ok(false); // Not time yet
        }
        
        // Time to start the transfer
        self.transition_state(IsochronousState::Active);
        self.execute_transfer(allocator)?;
        
        Ok(true) // Transfer started
    }
    
    /// Execute the isochronous transfer
    fn execute_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free previous qTDs if any
        for qtd_opt in self.qtd_handles.iter_mut() {
            if let Some(qtd) = qtd_opt.take() {
                allocator.free_qtd(qtd)?;
            }
        }
        
        // Allocate qTDs for microframe transactions
        let num_transactions = self.microframe_timing.transactions_per_uframe as usize;
        for i in 0..num_transactions {
            let qtd_handle = allocator.alloc_qtd()?;
            self.qtd_handles[i] = Some(qtd_handle);
        }
        
        // Reset transfer state
        self.bytes_transferred.store(0, Ordering::Release);
        self.frame_count.fetch_add(1, Ordering::Relaxed);
        
        let buffer_index = self.current_buffer.load(Ordering::Acquire) as usize;
        if let Some(ref buffer) = self.data_buffers[buffer_index] {
            // Configure qTDs for isochronous transfer
            let buffer_addr = buffer.dma_addr();
            let transfer_size_per_transaction = self.max_packet_size as u32;
            
            for i in 0..num_transactions {
                let transaction_buffer_addr = buffer_addr + (i as u32 * transfer_size_per_transaction);
                
                // This would configure the actual hardware qTD:
                // - Buffer pointer = transaction_buffer_addr
                // - Transfer size = transfer_size_per_transaction
                // - No data toggle (isochronous doesn't use data toggle)
                // - Endpoint = self.endpoint | (direction << 7)
                // - Device address = self.device_address
                // - PID = IN/OUT token
                // - Microframe mask for this transaction
                // - No error retry (isochronous accepts errors for timing)
                
                #[cfg(feature = "defmt")]
                defmt::trace!(
                    "Iso qTD {}: addr={:08x}, size={}, uframe={}",
                    i,
                    transaction_buffer_addr,
                    transfer_size_per_transaction,
                    i
                );
            }
            
            #[cfg(feature = "defmt")]
            defmt::debug!(
                "Isochronous transfer executing: addr={}, ep={}, dir={:?}, transactions={}",
                self.device_address,
                self.endpoint,
                self.direction,
                num_transactions
            );
        }
        
        Ok(())
    }
    
    /// Process transfer completion from interrupt handler
    pub fn process_completion<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        _allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        qtd_index: usize,
        status: u32,
    ) -> Result<bool> {
        if self.state() != IsochronousState::Active {
            return Err(UsbError::InvalidState);
        }
        
        if qtd_index >= self.microframe_timing.transactions_per_uframe as usize {
            return Err(UsbError::InvalidParameter);
        }
        
        // For isochronous transfers, we don't retry on errors
        // We accept the error and continue for timing guarantees
        
        let bytes_in_transaction = if status & 0x7C != 0 {
            // Error occurred in this transaction
            self.error_count.fetch_add(1, Ordering::AcqRel);
            self.error_frame_count.fetch_add(1, Ordering::Relaxed);
            0 // No data transferred in error case
        } else {
            // Success - get actual bytes transferred
            self.get_bytes_from_qtd(status)
        };
        
        // Accumulate bytes for this frame
        self.bytes_transferred.fetch_add(bytes_in_transaction, Ordering::AcqRel);
        
        // Check if all microframe transactions are complete
        let mut all_complete = true;
        for i in 0..self.microframe_timing.transactions_per_uframe as usize {
            if self.qtd_handles[i].is_some() {
                // This transaction hasn't completed yet
                all_complete = false;
                break;
            }
        }
        
        if all_complete {
            // All microframe transactions complete
            self.transition_state(IsochronousState::Complete);
            
            // For IN transfers, invalidate cache after DMA completion
            if self.direction == Direction::In {
                let buffer_index = self.current_buffer.load(Ordering::Acquire) as usize;
                if let Some(ref mut buffer) = self.data_buffers[buffer_index] {
                    cache_ops::prepare_for_dma_read(buffer.as_mut_slice());
                }
            }
            
            // Switch to next buffer for continuous streaming
            if self.is_streaming {
                let current = self.current_buffer.load(Ordering::Acquire);
                let next_buffer = 1 - current; // Toggle between 0 and 1
                self.current_buffer.store(next_buffer, Ordering::Release);
                
                // Prepare for next frame
                self.transition_state(IsochronousState::Idle);
                return Ok(true); // Needs rescheduling
            }
            
            return Ok(false); // One-time transfer complete
        }
        
        Ok(false) // More transactions pending
    }
    
    /// Clean up resources (used when transfer is cancelled)
    fn cleanup<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free qTDs if allocated
        for qtd_opt in self.qtd_handles.iter_mut() {
            if let Some(qtd) = qtd_opt.take() {
                allocator.free_qtd(qtd)?;
            }
        }
        
        // For non-streaming transfers, free queue head
        if !self.is_streaming {
            if let Some(qh) = self.qh_handle.take() {
                allocator.free_qh(qh)?;
            }
        }
        
        Ok(())
    }
    
    /// Extract bytes transferred from qTD status
    fn get_bytes_from_qtd(&self, status: u32) -> u32 {
        // Extract remaining bytes from qTD token field
        // Bits 30:16 contain total bytes remaining
        let remaining = (status >> 16) & 0x7FFF;
        let requested = self.max_packet_size as u32;
        
        // Bytes actually transferred = requested - remaining
        if remaining > requested {
            0 // Error case, shouldn't happen
        } else {
            requested - remaining
        }
    }
    
    /// Check if transfer is complete
    pub fn is_complete(&self) -> bool {
        self.state() == IsochronousState::Complete
    }
    
    /// Check if transfer failed
    pub fn is_failed(&self) -> bool {
        // For isochronous, we consider it "failed" only if too many consecutive errors
        let error_count = self.error_count.load(Ordering::Acquire);
        error_count >= self.max_errors
    }
    
    /// Check if transfer is ready to be scheduled
    pub fn is_ready_for_schedule(&self) -> bool {
        self.state() == IsochronousState::Idle
    }
    
    /// Check if transfer is scheduled and waiting
    pub fn is_scheduled(&self) -> bool {
        self.state() == IsochronousState::Scheduled
    }
    
    /// Get bytes transferred in current frame
    pub fn bytes_transferred(&self) -> u32 {
        self.bytes_transferred.load(Ordering::Acquire)
    }
    
    /// Get transfer direction
    pub fn direction(&self) -> Direction {
        self.direction
    }
    
    /// Get endpoint address with direction bit
    pub fn endpoint_address(&self) -> u8 {
        match self.direction {
            Direction::In => self.endpoint | 0x80,
            Direction::Out => self.endpoint & 0x7F,
        }
    }
    
    /// Get next scheduled frame number
    pub fn next_frame(&self) -> u32 {
        self.next_frame.load(Ordering::Acquire)
    }
    
    /// Get microframe timing configuration
    pub fn microframe_timing(&self) -> MicroframeTiming {
        self.microframe_timing
    }
    
    /// Get current buffer (for application to process data)
    pub fn current_buffer(&self) -> Option<&DmaBuffer> {
        let buffer_index = self.current_buffer.load(Ordering::Acquire) as usize;
        self.data_buffers[buffer_index].as_ref()
    }
    
    /// Get mutable current buffer (for application to fill data)
    pub fn current_buffer_mut(&mut self) -> Option<&mut DmaBuffer> {
        let buffer_index = self.current_buffer.load(Ordering::Acquire) as usize;
        self.data_buffers[buffer_index].as_mut()
    }
    
    /// Swap to the other buffer (for double buffering)
    pub fn swap_buffer(&self) {
        let current = self.current_buffer.load(Ordering::Acquire);
        let next_buffer = 1 - current; // Toggle between 0 and 1
        self.current_buffer.store(next_buffer, Ordering::Release);
    }
    
    /// Reset error count (useful for error recovery)
    pub fn reset_error_count(&mut self) {
        self.error_count.store(0, Ordering::Release);
    }
    
    /// Get transfer statistics
    pub fn statistics(&self) -> IsoTransferStats {
        IsoTransferStats {
            frames_processed: self.frame_count.load(Ordering::Relaxed),
            error_frames: self.error_frame_count.load(Ordering::Relaxed),
            current_error_count: self.error_count.load(Ordering::Relaxed),
        }
    }
    
    /// Take ownership of both data buffers
    pub fn take_buffers(&mut self) -> [Option<DmaBuffer>; 2] {
        [
            self.data_buffers[0].take(),
            self.data_buffers[1].take(),
        ]
    }
}

/// Isochronous transfer statistics
#[derive(Debug, Clone, Copy)]
pub struct IsoTransferStats {
    /// Total frames processed
    pub frames_processed: u32,
    /// Frames with errors
    pub error_frames: u32,
    /// Current consecutive error count
    pub current_error_count: u8,
}

impl IsoTransferStats {
    /// Calculate error rate
    pub fn error_rate(&self) -> f32 {
        if self.frames_processed > 0 {
            self.error_frames as f32 / self.frames_processed as f32
        } else {
            0.0
        }
    }
    
    /// Calculate success rate
    pub fn success_rate(&self) -> f32 {
        1.0 - self.error_rate()
    }
}

/// Isochronous transfer manager with frame-based scheduling
pub struct IsochronousTransferManager<const MAX_TRANSFERS: usize = 16> {
    /// Active transfers
    transfers: [Option<IsochronousTransfer>; MAX_TRANSFERS],
    /// Transfer statistics
    stats: IsochronousTransferStats,
    /// Current frame number (for scheduling)
    current_frame: AtomicU32,
}

impl<const MAX_TRANSFERS: usize> IsochronousTransferManager<MAX_TRANSFERS> {
    /// Create new isochronous transfer manager
    pub const fn new() -> Self {
        const NONE: Option<IsochronousTransfer> = None;
        Self {
            transfers: [NONE; MAX_TRANSFERS],
            stats: IsochronousTransferStats::new(),
            current_frame: AtomicU32::new(0),
        }
    }
    
    /// Submit new isochronous transfer
    pub fn submit(
        &mut self,
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer1: DmaBuffer,
        buffer2: DmaBuffer,
        microframe_timing: MicroframeTiming,
        is_streaming: bool,
    ) -> Result<usize> {
        // Find free slot
        for (index, slot) in self.transfers.iter_mut().enumerate() {
            if slot.is_none() {
                let transfer = IsochronousTransfer::new(
                    direction,
                    device_address,
                    endpoint,
                    max_packet_size,
                    buffer1,
                    buffer2,
                    microframe_timing,
                    is_streaming,
                );
                *slot = Some(transfer);
                self.stats.record_submission();
                return Ok(index);
            }
        }
        
        Err(UsbError::NoResources)
    }
    
    /// Update current frame number (called from frame interrupt)
    pub fn update_frame(&self, frame_number: u32) {
        self.current_frame.store(frame_number, Ordering::Release);
    }
    
    /// Process frame tick - schedule and start transfers as needed
    pub fn process_frame<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        let current_frame = self.current_frame.load(Ordering::Acquire);
        
        for transfer_opt in self.transfers.iter_mut() {
            if let Some(ref mut transfer) = transfer_opt {
                // Schedule transfers that are ready
                if transfer.is_ready_for_schedule() {
                    if let Err(e) = transfer.schedule(allocator, current_frame) {
                        #[cfg(feature = "defmt")]
                        defmt::warn!("Failed to schedule isochronous transfer: {:?}", e);
                    }
                }
                
                // Start transfers whose frame has arrived
                if transfer.is_scheduled() {
                    match transfer.start_if_ready(allocator, current_frame) {
                        Ok(true) => {
                            // Transfer started successfully
                        }
                        Ok(false) => {
                            // Not ready yet or couldn't start
                        }
                        Err(e) => {
                            #[cfg(feature = "defmt")]
                            defmt::warn!("Failed to start isochronous transfer: {:?}", e);
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// Process completion for a specific microframe transaction
    pub fn process_completion<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        transfer_index: usize,
        qtd_index: usize,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<()> {
        if transfer_index >= MAX_TRANSFERS {
            return Err(UsbError::InvalidParameter);
        }
        
        let needs_reschedule = if let Some(ref mut transfer) = self.transfers[transfer_index] {
            match transfer.process_completion(allocator, qtd_index, status) {
                Ok(reschedule) => reschedule,
                Err(e) => {
                    self.stats.record_failure();
                    return Err(e);
                }
            }
        } else {
            return Err(UsbError::InvalidParameter);
        };
        
        // Handle completion
        if let Some(ref transfer) = self.transfers[transfer_index] {
            if transfer.is_complete() {
                self.stats.record_completion();
                
                // If not streaming, remove the transfer
                if !needs_reschedule {
                    self.transfers[transfer_index] = None;
                }
            } else if transfer.is_failed() {
                self.stats.record_failure();
                self.transfers[transfer_index] = None;
            }
        }
        
        Ok(())
    }
    
    /// Cancel a transfer
    pub fn cancel_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        index: usize,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<[Option<DmaBuffer>; 2]> {
        if index >= MAX_TRANSFERS {
            return Err(UsbError::InvalidParameter);
        }
        
        if let Some(mut transfer) = self.transfers[index].take() {
            transfer.cleanup(allocator)?;
            self.stats.record_cancellation();
            Ok(transfer.take_buffers())
        } else {
            Ok([None, None])
        }
    }
    
    /// Get transfer by index
    pub fn get_transfer(&self, index: usize) -> Option<&IsochronousTransfer> {
        if index < MAX_TRANSFERS {
            self.transfers[index].as_ref()
        } else {
            None
        }
    }
    
    /// Get mutable transfer by index
    pub fn get_transfer_mut(&mut self, index: usize) -> Option<&mut IsochronousTransfer> {
        if index < MAX_TRANSFERS {
            self.transfers[index].as_mut()
        } else {
            None
        }
    }
    
    /// Get transfer statistics
    pub fn statistics(&self) -> &IsochronousTransferStats {
        &self.stats
    }
    
    /// Get count of active transfers
    pub fn active_count(&self) -> usize {
        self.transfers.iter()
            .filter(|t| t.is_some())
            .count()
    }
    
    /// Get count of streaming transfers
    pub fn streaming_count(&self) -> usize {
        self.transfers.iter()
            .filter_map(|t| t.as_ref())
            .filter(|t| t.is_streaming)
            .count()
    }
}

/// Isochronous transfer statistics for the manager
pub struct IsochronousTransferStats {
    submissions: AtomicU32,
    completions: AtomicU32,
    failures: AtomicU32,
    cancellations: AtomicU32,
    total_frames: AtomicU32,
    error_frames: AtomicU32,
}

impl IsochronousTransferStats {
    const fn new() -> Self {
        Self {
            submissions: AtomicU32::new(0),
            completions: AtomicU32::new(0),
            failures: AtomicU32::new(0),
            cancellations: AtomicU32::new(0),
            total_frames: AtomicU32::new(0),
            error_frames: AtomicU32::new(0),
        }
    }
    
    fn record_submission(&self) {
        self.submissions.fetch_add(1, Ordering::Relaxed);
    }
    
    fn record_completion(&self) {
        self.completions.fetch_add(1, Ordering::Relaxed);
    }
    
    fn record_failure(&self) {
        self.failures.fetch_add(1, Ordering::Relaxed);
    }
    
    fn record_cancellation(&self) {
        self.cancellations.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Record frame processing
    pub fn record_frame(&self, had_error: bool) {
        self.total_frames.fetch_add(1, Ordering::Relaxed);
        if had_error {
            self.error_frames.fetch_add(1, Ordering::Relaxed);
        }
    }
    
    /// Get overall success rate
    pub fn success_rate(&self) -> f32 {
        let completions = self.completions.load(Ordering::Relaxed) as f32;
        let total = (self.completions.load(Ordering::Relaxed) + 
                     self.failures.load(Ordering::Relaxed)) as f32;
        if total > 0.0 {
            completions / total
        } else {
            0.0
        }
    }
    
    /// Get frame error rate
    pub fn frame_error_rate(&self) -> f32 {
        let error_frames = self.error_frames.load(Ordering::Relaxed) as f32;
        let total_frames = self.total_frames.load(Ordering::Relaxed) as f32;
        if total_frames > 0.0 {
            error_frames / total_frames
        } else {
            0.0
        }
    }
    
    /// Get total submissions
    pub fn submissions(&self) -> u32 {
        self.submissions.load(Ordering::Relaxed)
    }
    
    /// Get total completions
    pub fn completions(&self) -> u32 {
        self.completions.load(Ordering::Relaxed)
    }
    
    /// Get total failures
    pub fn failures(&self) -> u32 {
        self.failures.load(Ordering::Relaxed)
    }
    
    /// Get total frames processed
    pub fn total_frames(&self) -> u32 {
        self.total_frames.load(Ordering::Relaxed)
    }
    
    /// Get error frames
    pub fn error_frames(&self) -> u32 {
        self.error_frames.load(Ordering::Relaxed)
    }
}

/// Helper functions for isochronous endpoint management
pub mod isochronous_endpoint {
    use super::*;
    
    /// Create an audio streaming IN transfer (typical for USB audio)
    pub fn create_audio_in_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer1: DmaBuffer,
        buffer2: DmaBuffer,
    ) -> IsochronousTransfer {
        IsochronousTransfer::new(
            Direction::In,
            device_address,
            endpoint,
            max_packet_size,
            buffer1,
            buffer2,
            MicroframeTiming::single(), // Most audio is single transaction
            true, // Streaming
        )
    }
    
    /// Create an audio streaming OUT transfer
    pub fn create_audio_out_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer1: DmaBuffer,
        buffer2: DmaBuffer,
    ) -> IsochronousTransfer {
        IsochronousTransfer::new(
            Direction::Out,
            device_address,
            endpoint,
            max_packet_size,
            buffer1,
            buffer2,
            MicroframeTiming::single(),
            true, // Streaming
        )
    }
    
    /// Create a high-bandwidth video streaming transfer
    pub fn create_video_in_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer1: DmaBuffer,
        buffer2: DmaBuffer,
        high_bandwidth: bool,
    ) -> IsochronousTransfer {
        let timing = if high_bandwidth {
            MicroframeTiming::triple() // High bandwidth video
        } else {
            MicroframeTiming::single() // Standard video
        };
        
        IsochronousTransfer::new(
            Direction::In,
            device_address,
            endpoint,
            max_packet_size,
            buffer1,
            buffer2,
            timing,
            true, // Streaming
        )
    }
}