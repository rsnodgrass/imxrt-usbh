//! USB Interrupt transfer implementation
//!
//! Implements USB 2.0 interrupt transfers for periodic data exchange
//! with HID devices (keyboards, mice), hub status changes, and other
//! low-bandwidth periodic endpoints.

use crate::dma::{cache_ops, DescriptorAllocator, DmaBuffer, QhHandle, QtdHandle};
use crate::error::{Result, UsbError};
use crate::transfer::{Direction, TransferState};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// Interrupt transfer states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum InterruptState {
    /// Initial state
    Idle = 0,
    /// Transfer scheduled and waiting for frame
    Scheduled = 1,
    /// Transfer in progress
    Active = 2,
    /// Transfer complete
    Complete = 3,
    /// Transfer failed
    Failed = 4,
    /// Transfer stalled (endpoint halted)
    Stalled = 5,
}

impl From<u8> for InterruptState {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl crate::transfer::TransferState for InterruptState {
    fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::Idle,
            1 => Self::Scheduled,
            2 => Self::Active,
            3 => Self::Complete,
            4 => Self::Failed,
            5 => Self::Stalled,
            _ => {
                #[cfg(feature = "defmt")]
                defmt::error!("Invalid InterruptState value: {}, defaulting to Failed", val);
                Self::Failed
            }
        }
    }

    fn to_u8(self) -> u8 {
        self as u8
    }
}

/// Interrupt transfer context for periodic endpoint
pub struct InterruptTransfer {
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
    /// Data buffer for transfer
    data_buffer: Option<DmaBuffer>,
    /// Bytes transferred in current transaction
    bytes_transferred: AtomicU32,
    /// Data toggle bit for this endpoint
    data_toggle: AtomicU8,
    /// Polling interval in frames (1ms for high-speed, 1-255ms for full/low-speed)
    interval_frames: u8,
    /// Next scheduled frame number
    next_frame: AtomicU32,
    /// Number of consecutive NAKs received
    nak_count: AtomicU8,
    /// Maximum consecutive NAKs before giving up
    max_naks: u8,
    /// Queue head handle
    qh_handle: Option<QhHandle>,
    /// Current qTD handle
    qtd_handle: Option<QtdHandle>,
    /// Whether this is a periodic recurring transfer
    is_periodic: bool,
    /// Transfer completion callback ID
    callback_id: Option<u32>,
}

impl InterruptTransfer {
    /// Create new interrupt transfer
    pub fn new(
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        data_buffer: DmaBuffer,
        interval_frames: u8,
        is_periodic: bool,
    ) -> Self {
        Self {
            state: AtomicU8::new(InterruptState::Idle as u8),
            direction,
            device_address,
            endpoint,
            max_packet_size,
            data_buffer: Some(data_buffer),
            bytes_transferred: AtomicU32::new(0),
            data_toggle: AtomicU8::new(0), // Initial toggle state
            interval_frames: interval_frames.max(1), // Minimum 1 frame interval
            next_frame: AtomicU32::new(0),
            nak_count: AtomicU8::new(0),
            max_naks: 3, // Standard for interrupt transfers
            qh_handle: None,
            qtd_handle: None,
            is_periodic,
            callback_id: None,
        }
    }

    /// Get current state
    pub fn state(&self) -> InterruptState {
        InterruptState::load_from(&self.state)
    }

    /// Transition to new state
    fn transition_state(&self, new_state: InterruptState) {
        new_state.store_into(&self.state);
    }

    /// Schedule interrupt transfer for next appropriate frame
    pub fn schedule<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        current_frame: u32,
    ) -> Result<()> {
        if !matches!(
            self.state(),
            InterruptState::Idle | InterruptState::Complete
        ) {
            return Err(UsbError::InvalidState);
        }

        // Calculate next frame for this transfer
        let next_frame = if self.state() == InterruptState::Idle {
            // First scheduling - start immediately or next frame
            current_frame + 1
        } else {
            // Periodic reschedule - add interval
            let last_frame = self.next_frame.load(Ordering::Acquire);
            last_frame + (self.interval_frames as u32)
        };

        self.next_frame.store(next_frame, Ordering::Release);

        // Allocate queue head for interrupt transfer if not already allocated
        if self.qh_handle.is_none() {
            let qh_handle = allocator.alloc_qh()?;
            self.qh_handle = Some(qh_handle);
        }

        // Prepare DMA buffer for transfer
        if let Some(ref mut buffer) = self.data_buffer {
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

        self.transition_state(InterruptState::Scheduled);

        #[cfg(feature = "defmt")]
        defmt::debug!(
            "Interrupt transfer scheduled: addr={}, ep={}, frame={}, interval={}",
            self.device_address,
            self.endpoint,
            next_frame,
            self.interval_frames
        );

        Ok(())
    }

    /// Start interrupt transfer when scheduled frame arrives
    pub fn start_if_ready<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        current_frame: u32,
    ) -> Result<bool> {
        if self.state() != InterruptState::Scheduled {
            return Ok(false); // Not ready
        }

        let scheduled_frame = self.next_frame.load(Ordering::Acquire);
        if current_frame < scheduled_frame {
            return Ok(false); // Not time yet
        }

        // Time to start the transfer
        self.transition_state(InterruptState::Active);
        self.execute_transfer(allocator)?;

        Ok(true) // Transfer started
    }

    /// Execute the interrupt transfer
    fn execute_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free previous qTD if any
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Allocate qTD for interrupt transfer
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // Reset transfer state
        self.bytes_transferred.store(0, Ordering::Release);
        self.nak_count.store(0, Ordering::Release);

        if let Some(ref buffer) = self.data_buffer {
            // Configure qTD for interrupt transfer
            let _buffer_addr = buffer.dma_addr();
            let _transfer_size = self.max_packet_size as u32;
            let _toggle = self.data_toggle.load(Ordering::Acquire);

            // This would configure the actual hardware qTD:
            // - Buffer pointer = _buffer_addr
            // - Transfer size = _transfer_size (max packet size)
            // - Data toggle = _toggle
            // - Endpoint = self.endpoint | (direction << 7)
            // - Device address = self.device_address
            // - PID = IN/OUT token
            // - Interrupt on complete = true

            #[cfg(feature = "defmt")]
            defmt::debug!(
                "Interrupt transfer executing: addr={}, ep={}, dir={:?}, size={}, toggle={}",
                self.device_address,
                self.endpoint,
                self.direction,
                _transfer_size,
                _toggle
            );
        }

        Ok(())
    }

    /// Process transfer completion from interrupt handler
    pub fn process_completion<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<bool> {
        if self.state() != InterruptState::Active {
            return Err(UsbError::InvalidState);
        }

        // Check for STALL condition first
        if status & (1 << 6) != 0 {
            // STALL bit
            self.transition_state(InterruptState::Stalled);
            return Err(UsbError::Stall);
        }

        // Check for NAK (device not ready)
        if status & (1 << 0) != 0 {
            // NAK bit (assuming bit 0 for NAK)
            let nak_count = self.nak_count.fetch_add(1, Ordering::AcqRel) + 1;

            if nak_count >= self.max_naks {
                // Too many NAKs - complete transfer with no data
                self.transition_state(InterruptState::Complete);

                // For periodic transfers, schedule next occurrence
                if self.is_periodic {
                    self.transition_state(InterruptState::Idle); // Ready for next schedule
                    return Ok(true); // Needs rescheduling
                }

                return Ok(false); // One-time transfer failed due to NAKs
            }

            // Retry the transfer (stay in Active state)
            return Ok(false);
        }

        // Check for other errors
        if status & 0x7C != 0 {
            // Error bits (excluding STALL and NAK)
            return self.handle_error(allocator, status);
        }

        // Success - update transfer progress
        let bytes_received = self.get_bytes_from_qtd(status);
        self.bytes_transferred
            .store(bytes_received, Ordering::Release);

        // Toggle data bit for next transfer
        let toggle = self.data_toggle.load(Ordering::Acquire);
        self.data_toggle.store(1 - toggle, Ordering::Release);

        // Transfer complete
        self.transition_state(InterruptState::Complete);

        // For IN transfers, invalidate cache after DMA completion
        if self.direction == Direction::In {
            if let Some(ref mut buffer) = self.data_buffer {
                cache_ops::prepare_for_dma_read(buffer.as_mut_slice());
            }
        }

        // For periodic transfers, prepare for next schedule
        if self.is_periodic {
            self.transition_state(InterruptState::Idle); // Ready for next schedule
            return Ok(true); // Needs rescheduling
        }

        Ok(false) // One-time transfer complete
    }

    /// Handle transfer error
    fn handle_error<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        _allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<bool> {
        // For interrupt transfers, we generally don't retry errors
        // Instead, we complete the transfer and let the application decide
        self.transition_state(InterruptState::Failed);

        // Determine specific error from status
        if status & (1 << 5) != 0 {
            return Err(UsbError::BufferOverflow);
        } else if status & (1 << 4) != 0 {
            return Err(UsbError::TransactionError);
        } else if status & (1 << 3) != 0 {
            return Err(UsbError::TransactionError);
        }
        return Err(UsbError::TransactionError);
    }

    /// Clean up resources (used when transfer is cancelled)
    fn cleanup<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free qTD if allocated
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // For non-periodic transfers, free queue head
        if !self.is_periodic {
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
        self.state() == InterruptState::Complete
    }

    /// Check if transfer failed
    pub fn is_failed(&self) -> bool {
        matches!(
            self.state(),
            InterruptState::Failed | InterruptState::Stalled
        )
    }

    /// Check if endpoint is stalled
    pub fn is_stalled(&self) -> bool {
        self.state() == InterruptState::Stalled
    }

    /// Check if transfer is ready to be scheduled
    pub fn is_ready_for_schedule(&self) -> bool {
        self.state() == InterruptState::Idle
    }

    /// Check if transfer is scheduled and waiting
    pub fn is_scheduled(&self) -> bool {
        self.state() == InterruptState::Scheduled
    }

    /// Get bytes transferred in last transaction
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

    /// Get polling interval in frames
    pub fn interval(&self) -> u8 {
        self.interval_frames
    }

    /// Get next scheduled frame number
    pub fn next_frame(&self) -> u32 {
        self.next_frame.load(Ordering::Acquire)
    }

    /// Clear endpoint stall condition
    ///
    /// This should be followed by a CLEAR_HALT control request to the device
    pub fn clear_stall(&mut self) {
        if self.state() == InterruptState::Stalled {
            self.transition_state(InterruptState::Idle);
            self.nak_count.store(0, Ordering::Release);
        }
    }

    /// Reset data toggle to DATA0
    ///
    /// Used after SET_CONFIGURATION or CLEAR_HALT
    pub fn reset_data_toggle(&mut self) {
        self.data_toggle.store(0, Ordering::Release);
    }

    /// Get current data toggle state
    pub fn data_toggle(&self) -> u8 {
        self.data_toggle.load(Ordering::Acquire)
    }

    /// Take ownership of data buffer
    pub fn take_buffer(&mut self) -> Option<DmaBuffer> {
        self.data_buffer.take()
    }

    /// Set completion callback ID
    pub fn set_callback(&mut self, callback_id: u32) {
        self.callback_id = Some(callback_id);
    }

    /// Get completion callback ID
    pub fn callback_id(&self) -> Option<u32> {
        self.callback_id
    }
}

/// Interrupt transfer manager with frame-based scheduling
pub struct InterruptTransferManager<const MAX_TRANSFERS: usize = 32> {
    /// Active transfers
    transfers: [Option<InterruptTransfer>; MAX_TRANSFERS],
    /// Transfer statistics
    stats: InterruptTransferStats,
    /// Current frame number (for scheduling)
    current_frame: AtomicU32,
}

impl<const MAX_TRANSFERS: usize> InterruptTransferManager<MAX_TRANSFERS> {
    /// Create new interrupt transfer manager
    pub const fn new() -> Self {
        const NONE: Option<InterruptTransfer> = None;
        Self {
            transfers: [NONE; MAX_TRANSFERS],
            stats: InterruptTransferStats::new(),
            current_frame: AtomicU32::new(0),
        }
    }

    /// Submit new interrupt transfer
    pub fn submit(
        &mut self,
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        data_buffer: DmaBuffer,
        interval_frames: u8,
        is_periodic: bool,
    ) -> Result<usize> {
        // Find free slot
        for (index, slot) in self.transfers.iter_mut().enumerate() {
            if slot.is_none() {
                let transfer = InterruptTransfer::new(
                    direction,
                    device_address,
                    endpoint,
                    max_packet_size,
                    data_buffer,
                    interval_frames,
                    is_periodic,
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
                        defmt::warn!("Failed to schedule interrupt transfer: {:?}", e);
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
                            defmt::warn!("Failed to start interrupt transfer: {:?}", e);
                        }
                    }
                }
            }
        }

        Ok(())
    }

    /// Process completion for a transfer
    pub fn process_completion<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        index: usize,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<()> {
        if index >= MAX_TRANSFERS {
            return Err(UsbError::InvalidParameter);
        }

        let needs_reschedule = if let Some(ref mut transfer) = self.transfers[index] {
            match transfer.process_completion(allocator, status) {
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
        if let Some(ref transfer) = self.transfers[index] {
            if transfer.is_complete() {
                self.stats.record_completion();

                // If not periodic, remove the transfer
                if !needs_reschedule {
                    self.transfers[index] = None;
                }
            } else if transfer.is_failed() {
                self.stats.record_failure();
                self.transfers[index] = None;
            }
        }

        Ok(())
    }

    /// Cancel a transfer
    pub fn cancel_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        index: usize,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<Option<DmaBuffer>> {
        if index >= MAX_TRANSFERS {
            return Err(UsbError::InvalidParameter);
        }

        if let Some(mut transfer) = self.transfers[index].take() {
            transfer.cleanup(allocator)?;
            self.stats.record_cancellation();
            Ok(transfer.take_buffer())
        } else {
            Ok(None)
        }
    }

    /// Get transfer by index
    pub fn get_transfer(&self, index: usize) -> Option<&InterruptTransfer> {
        if index < MAX_TRANSFERS {
            self.transfers[index].as_ref()
        } else {
            None
        }
    }

    /// Get mutable transfer by index
    pub fn get_transfer_mut(&mut self, index: usize) -> Option<&mut InterruptTransfer> {
        if index < MAX_TRANSFERS {
            self.transfers[index].as_mut()
        } else {
            None
        }
    }

    /// Clear endpoint stall for all transfers on an endpoint
    pub fn clear_endpoint_stall(&mut self, device_address: u8, endpoint: u8) {
        for transfer_opt in self.transfers.iter_mut() {
            if let Some(transfer) = transfer_opt {
                if transfer.device_address == device_address && transfer.endpoint == endpoint {
                    transfer.clear_stall();
                }
            }
        }
    }

    /// Reset data toggle for all transfers on an endpoint
    pub fn reset_endpoint_toggle(&mut self, device_address: u8, endpoint: u8) {
        for transfer_opt in self.transfers.iter_mut() {
            if let Some(transfer) = transfer_opt {
                if transfer.device_address == device_address && transfer.endpoint == endpoint {
                    transfer.reset_data_toggle();
                }
            }
        }
    }

    /// Get transfer statistics
    pub fn statistics(&self) -> &InterruptTransferStats {
        &self.stats
    }

    /// Get count of active transfers
    pub fn active_count(&self) -> usize {
        self.transfers.iter().filter(|t| t.is_some()).count()
    }

    /// Get count of scheduled transfers waiting for their frame
    pub fn scheduled_count(&self) -> usize {
        self.transfers
            .iter()
            .filter_map(|t| t.as_ref())
            .filter(|t| t.is_scheduled())
            .count()
    }
}

/// Interrupt transfer statistics
pub struct InterruptTransferStats {
    submissions: AtomicU32,
    completions: AtomicU32,
    failures: AtomicU32,
    cancellations: AtomicU32,
    nak_timeouts: AtomicU32,
    total_bytes: AtomicU32,
}

impl InterruptTransferStats {
    const fn new() -> Self {
        Self {
            submissions: AtomicU32::new(0),
            completions: AtomicU32::new(0),
            failures: AtomicU32::new(0),
            cancellations: AtomicU32::new(0),
            nak_timeouts: AtomicU32::new(0),
            total_bytes: AtomicU32::new(0),
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

    /// Record NAK timeout
    pub fn record_nak_timeout(&self) {
        self.nak_timeouts.fetch_add(1, Ordering::Relaxed);
    }

    /// Record bytes transferred
    pub fn record_bytes(&self, bytes: u32) {
        self.total_bytes.fetch_add(bytes, Ordering::Relaxed);
    }

    /// Get success rate (completions / (completions + failures))
    pub fn success_rate(&self) -> f32 {
        let completions = self.completions.load(Ordering::Relaxed) as f32;
        let total = (self.completions.load(Ordering::Relaxed)
            + self.failures.load(Ordering::Relaxed)) as f32;
        if total > 0.0 {
            completions / total
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

    /// Get total bytes transferred
    pub fn total_bytes(&self) -> u32 {
        self.total_bytes.load(Ordering::Relaxed)
    }

    /// Get NAK timeouts
    pub fn nak_timeouts(&self) -> u32 {
        self.nak_timeouts.load(Ordering::Relaxed)
    }
}

/// Helper functions for interrupt endpoint management
pub mod interrupt_endpoint {
    use super::*;

    /// Create a periodic interrupt IN transfer (typical for HID devices)
    pub fn create_periodic_in_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: DmaBuffer,
        interval_ms: u8,
    ) -> InterruptTransfer {
        InterruptTransfer::new(
            Direction::In,
            device_address,
            endpoint,
            max_packet_size,
            buffer,
            interval_ms,
            true, // Periodic
        )
    }

    /// Create a one-time interrupt IN transfer
    pub fn create_oneshot_in_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: DmaBuffer,
    ) -> InterruptTransfer {
        InterruptTransfer::new(
            Direction::In,
            device_address,
            endpoint,
            max_packet_size,
            buffer,
            1,     // Minimum interval
            false, // One-time
        )
    }

    /// Create a periodic interrupt OUT transfer
    pub fn create_periodic_out_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: DmaBuffer,
        interval_ms: u8,
    ) -> InterruptTransfer {
        InterruptTransfer::new(
            Direction::Out,
            device_address,
            endpoint,
            max_packet_size,
            buffer,
            interval_ms,
            true, // Periodic
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::ptr::NonNull;

    fn create_mock_buffer(size: usize) -> DmaBuffer {
        static mut TEST_BUFFER: [u8; 1024] = [0; 1024];
        unsafe {
            let ptr = NonNull::new_unchecked(TEST_BUFFER.as_mut_ptr());
            DmaBuffer::new(ptr, size, 0)
        }
    }

    #[test]
    fn test_interrupt_transfer_initial_state() {
        let buffer = create_mock_buffer(64);
        let transfer = InterruptTransfer::new(
            Direction::In,
            1,
            0x81,
            8, // HID keyboard typical max packet size
            buffer,
            10,   // 10ms interval
            true, // periodic
        );

        assert_eq!(transfer.state(), InterruptState::Idle);
        assert_eq!(transfer.bytes_transferred(), 0);
        assert_eq!(transfer.data_toggle(), 0);
        assert_eq!(transfer.direction(), Direction::In);
        assert_eq!(transfer.endpoint_address(), 0x81);
        assert_eq!(transfer.interval(), 10);
        assert!(transfer.is_ready_for_schedule());
    }

    #[test]
    fn test_interrupt_transfer_periodic_vs_oneshot() {
        // periodic transfer
        let buffer1 = create_mock_buffer(64);
        let periodic = InterruptTransfer::new(
            Direction::In,
            1,
            0x81,
            8,
            buffer1,
            10,
            true, // periodic
        );
        assert_eq!(periodic.is_periodic, true);

        // one-shot transfer
        let buffer2 = create_mock_buffer(64);
        let oneshot = InterruptTransfer::new(
            Direction::In,
            1,
            0x81,
            8,
            buffer2,
            1,
            false, // not periodic
        );
        assert_eq!(oneshot.is_periodic, false);
    }

    #[test]
    fn test_interrupt_transfer_state_transitions() {
        let buffer = create_mock_buffer(64);
        let transfer = InterruptTransfer::new(Direction::In, 1, 0x81, 8, buffer, 10, true);

        // idle → scheduled
        transfer.transition_state(InterruptState::Scheduled);
        assert_eq!(transfer.state(), InterruptState::Scheduled);
        assert!(transfer.is_scheduled());

        // scheduled → active
        transfer.transition_state(InterruptState::Active);
        assert_eq!(transfer.state(), InterruptState::Active);

        // active → complete
        transfer.transition_state(InterruptState::Complete);
        assert_eq!(transfer.state(), InterruptState::Complete);
        assert!(transfer.is_complete());
        assert!(!transfer.is_failed());
    }

    #[test]
    fn test_interrupt_transfer_stalled_state() {
        let buffer = create_mock_buffer(64);
        let mut transfer = InterruptTransfer::new(Direction::In, 1, 0x81, 8, buffer, 10, true);

        transfer.transition_state(InterruptState::Stalled);
        assert_eq!(transfer.state(), InterruptState::Stalled);
        assert!(transfer.is_stalled());
        assert!(transfer.is_failed());

        // clear stall
        transfer.clear_stall();
        assert_eq!(transfer.state(), InterruptState::Idle);
        assert!(!transfer.is_stalled());
        assert_eq!(transfer.nak_count.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn test_interrupt_transfer_nak_counting() {
        let buffer = create_mock_buffer(64);
        let transfer = InterruptTransfer::new(Direction::In, 1, 0x81, 8, buffer, 10, true);

        // initial NAK count
        assert_eq!(transfer.nak_count.load(Ordering::Relaxed), 0);

        // simulate NAKs
        transfer.nak_count.fetch_add(1, Ordering::Relaxed);
        assert_eq!(transfer.nak_count.load(Ordering::Relaxed), 1);

        transfer.nak_count.fetch_add(1, Ordering::Relaxed);
        assert_eq!(transfer.nak_count.load(Ordering::Relaxed), 2);

        transfer.nak_count.fetch_add(1, Ordering::Relaxed);
        assert_eq!(transfer.nak_count.load(Ordering::Relaxed), 3);

        // max_naks is 3, so threshold reached
        assert_eq!(
            transfer.nak_count.load(Ordering::Relaxed),
            transfer.max_naks
        );
    }

    #[test]
    fn test_interrupt_transfer_frame_scheduling() {
        let buffer = create_mock_buffer(64);
        let transfer = InterruptTransfer::new(
            Direction::In,
            1,
            0x81,
            8,
            buffer,
            10, // 10 frame interval
            true,
        );

        // set next frame
        transfer.next_frame.store(100, Ordering::Release);
        assert_eq!(transfer.next_frame(), 100);

        // next scheduled frame = current + interval
        transfer.next_frame.store(110, Ordering::Release);
        assert_eq!(transfer.next_frame(), 110);
    }

    #[test]
    fn test_interrupt_transfer_data_toggle() {
        let buffer = create_mock_buffer(64);
        let mut transfer = InterruptTransfer::new(Direction::In, 1, 0x81, 8, buffer, 10, true);

        assert_eq!(transfer.data_toggle(), 0);

        // toggle (simulating completion)
        let toggle = transfer.data_toggle.load(Ordering::Acquire);
        transfer.data_toggle.store(1 - toggle, Ordering::Release);
        assert_eq!(transfer.data_toggle(), 1);

        // reset
        transfer.reset_data_toggle();
        assert_eq!(transfer.data_toggle(), 0);
    }

    #[test]
    fn test_interrupt_manager_submission() {
        let mut manager = InterruptTransferManager::<8>::new();

        let buffer = create_mock_buffer(64);
        let result = manager.submit(Direction::In, 1, 0x81, 8, buffer, 10, true);

        assert!(result.is_ok());
        let index = result.unwrap();
        assert_eq!(index, 0);
        assert_eq!(manager.active_count(), 1);
    }

    #[test]
    fn test_interrupt_manager_pool_exhaustion() {
        let mut manager = InterruptTransferManager::<4>::new();

        // fill all 4 slots
        for i in 0..4 {
            let buffer = create_mock_buffer(64);
            let result = manager.submit(Direction::In, 1, 0x81, 8, buffer, 10, true);
            assert!(result.is_ok());
            assert_eq!(result.unwrap(), i);
        }

        assert_eq!(manager.active_count(), 4);

        // 5th should fail
        let buffer = create_mock_buffer(64);
        let result = manager.submit(Direction::In, 1, 0x81, 8, buffer, 10, true);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::NoResources)));
    }

    #[test]
    fn test_interrupt_manager_frame_update() {
        let manager = InterruptTransferManager::<8>::new();

        // update frame number
        manager.update_frame(1000);
        assert_eq!(manager.current_frame.load(Ordering::Relaxed), 1000);

        manager.update_frame(1001);
        assert_eq!(manager.current_frame.load(Ordering::Relaxed), 1001);
    }

    #[test]
    fn test_interrupt_manager_scheduled_count() {
        let mut manager = InterruptTransferManager::<8>::new();

        // submit and transition to scheduled
        let buffer1 = create_mock_buffer(64);
        let idx1 = manager
            .submit(Direction::In, 1, 0x81, 8, buffer1, 10, true)
            .unwrap();

        let buffer2 = create_mock_buffer(64);
        let idx2 = manager
            .submit(Direction::In, 1, 0x82, 8, buffer2, 10, true)
            .unwrap();

        // manually transition to scheduled for testing
        if let Some(ref transfer) = manager.transfers[idx1] {
            transfer.transition_state(InterruptState::Scheduled);
        }
        if let Some(ref transfer) = manager.transfers[idx2] {
            transfer.transition_state(InterruptState::Scheduled);
        }

        assert_eq!(manager.scheduled_count(), 2);
    }

    #[test]
    fn test_interrupt_manager_statistics() {
        let manager = InterruptTransferManager::<8>::new();
        let stats = manager.statistics();

        assert_eq!(stats.submissions(), 0);
        assert_eq!(stats.completions(), 0);
        assert_eq!(stats.failures(), 0);
        assert_eq!(stats.nak_timeouts(), 0);

        // simulate activity
        stats.record_submission();
        stats.record_submission();
        assert_eq!(stats.submissions(), 2);

        stats.record_completion();
        assert_eq!(stats.completions(), 1);

        stats.record_nak_timeout();
        assert_eq!(stats.nak_timeouts(), 1);

        stats.record_failure();
        assert_eq!(stats.failures(), 1);

        // success rate = 1 / (1 + 1) = 0.5
        assert_eq!(stats.success_rate(), 0.5);
    }

    #[test]
    fn test_interrupt_transfer_endpoint_address() {
        // IN endpoint
        let buffer1 = create_mock_buffer(64);
        let transfer_in = InterruptTransfer::new(Direction::In, 1, 0x01, 8, buffer1, 10, true);
        assert_eq!(transfer_in.endpoint_address(), 0x81);

        // OUT endpoint
        let buffer2 = create_mock_buffer(64);
        let transfer_out = InterruptTransfer::new(Direction::Out, 1, 0x02, 8, buffer2, 10, true);
        assert_eq!(transfer_out.endpoint_address(), 0x02);
    }

    #[test]
    fn test_interrupt_stats_byte_tracking() {
        let stats = InterruptTransferStats::new();

        assert_eq!(stats.total_bytes(), 0);

        stats.record_bytes(8);
        assert_eq!(stats.total_bytes(), 8);

        stats.record_bytes(8);
        assert_eq!(stats.total_bytes(), 16);
    }

    #[test]
    fn test_interrupt_transfer_callback() {
        let buffer = create_mock_buffer(64);
        let mut transfer = InterruptTransfer::new(Direction::In, 1, 0x81, 8, buffer, 10, true);

        assert_eq!(transfer.callback_id(), None);

        transfer.set_callback(42);
        assert_eq!(transfer.callback_id(), Some(42));
    }
}
