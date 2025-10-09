//! USB Bulk transfer implementation

use crate::dma::{cache_ops, DescriptorAllocator, DmaBuffer, QhHandle, QtdHandle};
use crate::error::{Result, UsbError};
use crate::transfer::{Direction, TransferState};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

fn get_current_tick_count() -> u32 {
    unsafe {
        let dwt = cortex_m::peripheral::DWT::PTR;
        (*dwt).cyccnt.read()
    }
}

fn ticks_to_ms(ticks: u32) -> u32 {
    crate::timing::cycles_to_ms(ticks)
}

/// Bulk transfer states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum BulkState {
    /// Initial state
    Idle = 0,
    /// Transfer in progress
    Active = 1,
    /// Transfer complete
    Complete = 2,
    /// Transfer failed
    Failed = 3,
    /// Transfer stalled (endpoint halted)
    Stalled = 4,
}

impl From<u8> for BulkState {
    fn from(val: u8) -> Self {
        Self::from_u8(val)
    }
}

impl crate::transfer::TransferState for BulkState {
    fn from_u8(val: u8) -> Self {
        match val {
            0 => Self::Idle,
            1 => Self::Active,
            2 => Self::Complete,
            3 => Self::Failed,
            4 => Self::Stalled,
            _ => {
                #[cfg(feature = "defmt")]
                defmt::error!("Invalid BulkState value: {}, defaulting to Failed", val);
                Self::Failed
            }
        }
    }

    fn to_u8(self) -> u8 {
        self as u8
    }
}

/// Bulk transfer context for a single endpoint
pub struct BulkTransfer {
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
    /// Total bytes to transfer
    total_bytes: u32,
    /// Bytes transferred so far
    bytes_transferred: AtomicU32,
    /// Data toggle bit for this endpoint
    data_toggle: AtomicU8,
    /// Number of retry attempts
    retry_count: AtomicU8,
    /// Maximum retry attempts
    max_retries: u8,
    /// Queue head handle
    qh_handle: Option<QhHandle>,
    /// Current qTD handle
    qtd_handle: Option<QtdHandle>,
    /// Transfer timeout in milliseconds
    timeout_ms: u32,
    /// Transfer start timestamp (implementation dependent)
    start_time: AtomicU32,
}

impl BulkTransfer {
    /// Create new bulk transfer
    pub fn new(
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        data_buffer: DmaBuffer,
        timeout_ms: u32,
    ) -> Self {
        let total_bytes = data_buffer.as_slice().len() as u32;

        Self {
            state: AtomicU8::new(BulkState::Idle as u8),
            direction,
            device_address,
            endpoint,
            max_packet_size,
            data_buffer: Some(data_buffer),
            total_bytes,
            bytes_transferred: AtomicU32::new(0),
            data_toggle: AtomicU8::new(0), // Initial toggle state
            retry_count: AtomicU8::new(0),
            max_retries: 3,
            qh_handle: None,
            qtd_handle: None,
            timeout_ms,
            start_time: AtomicU32::new(0),
        }
    }

    /// Get current state
    pub fn state(&self) -> BulkState {
        BulkState::load_from(&self.state)
    }

    /// Transition to new state
    fn transition_state(&self, new_state: BulkState) {
        new_state.store_into(&self.state);
    }

    /// Start bulk transfer
    pub fn start<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        if self.state() != BulkState::Idle {
            return Err(UsbError::InvalidState);
        }

        // Allocate queue head for bulk transfer
        let qh_handle = allocator.alloc_qh()?;
        self.qh_handle = Some(qh_handle);

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

        // Start transfer
        self.transition_state(BulkState::Active);
        self.start_transfer_timer();
        self.execute_transfer(allocator)?;

        Ok(())
    }

    /// Start transfer timer for timeout tracking
    fn start_transfer_timer(&self) {
        // Use a simple tick counter (implementation-specific)
        // In real implementation, this would use a hardware timer
        let current_time = get_current_tick_count();
        self.start_time.store(current_time, Ordering::Relaxed);
    }

    /// Check if transfer has timed out
    pub fn is_timed_out(&self) -> bool {
        if self.timeout_ms == 0 {
            return false; // No timeout set
        }

        let start = self.start_time.load(Ordering::Relaxed);
        if start == 0 {
            return false; // Transfer not started
        }

        let current = get_current_tick_count();
        let elapsed_ms = ticks_to_ms(current.saturating_sub(start));
        elapsed_ms >= self.timeout_ms
    }

    pub fn timeout_ms(&self) -> u32 {
        self.timeout_ms
    }

    pub fn set_timeout(&mut self, timeout_ms: u32) {
        self.timeout_ms = timeout_ms;
    }

    /// Execute the bulk transfer
    fn execute_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Allocate qTD for bulk transfer
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // Calculate transfer size for this qTD
        let transferred = self.bytes_transferred.load(Ordering::Acquire);
        let remaining = self.total_bytes - transferred;
        let _transfer_size = remaining.min(self.max_packet_size as u32);

        if let Some(ref buffer) = self.data_buffer {
            // Configure qTD for bulk transfer
            let _buffer_addr = buffer.dma_addr() + transferred;
            let _toggle = self.data_toggle.load(Ordering::Acquire);

            // This would configure the actual hardware qTD:
            // - Buffer pointer = _buffer_addr
            // - Transfer size = _transfer_size
            // - Data toggle = _toggle
            // - Endpoint = self.endpoint | (direction << 7)
            // - Device address = self.device_address
            // - PID = IN/OUT token

            // For bulk transfers, we don't need SETUP stage like control transfers
            // The qTD is configured directly for IN or OUT data phase

            #[cfg(feature = "defmt")]
            defmt::debug!(
                "Bulk transfer: addr={}, ep={}, dir={:?}, size={}, toggle={}",
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
    ) -> Result<()> {
        if self.state() != BulkState::Active {
            return Err(UsbError::InvalidState);
        }

        // Check for STALL condition first
        if status & (1 << 6) != 0 {
            // STALL bit
            self.transition_state(BulkState::Stalled);
            self.cleanup(allocator)?;
            return Err(UsbError::Stall);
        }

        // Check for other errors
        if status & 0x7C != 0 {
            // Error bits (excluding STALL)
            return self.handle_error(allocator, status);
        }

        // Success - update transfer progress
        let bytes_in_qtd = self.get_bytes_from_qtd(status);
        let new_total = self
            .bytes_transferred
            .fetch_add(bytes_in_qtd, Ordering::AcqRel)
            + bytes_in_qtd;

        // Toggle data bit for next transfer
        let toggle = self.data_toggle.load(Ordering::Acquire);
        self.data_toggle.store(1 - toggle, Ordering::Release);

        // Check if transfer is complete
        if new_total >= self.total_bytes || bytes_in_qtd < self.max_packet_size as u32 {
            // Transfer complete (all bytes transferred or short packet received)
            self.transition_state(BulkState::Complete);

            // For IN transfers, invalidate cache after DMA completion
            if self.direction == Direction::In {
                if let Some(ref mut buffer) = self.data_buffer {
                    cache_ops::prepare_for_dma_read(buffer.as_mut_slice());
                }
            }

            self.cleanup(allocator)?;
        } else {
            // More data to transfer - continue with next qTD
            self.execute_transfer(allocator)?;
        }

        Ok(())
    }

    /// Handle transfer error with retry logic
    fn handle_error<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<()> {
        let retry_count = self.retry_count.fetch_add(1, Ordering::AcqRel);

        if retry_count >= self.max_retries {
            // Max retries exceeded, fail transfer
            self.transition_state(BulkState::Failed);
            self.cleanup(allocator)?;

            // Return specific error based on status
            return if status & (1 << 5) != 0 {
                Err(UsbError::BufferOverflow)
            } else if status & (1 << 4) != 0 {
                Err(UsbError::TransactionError)
            } else if status & (1 << 3) != 0 {
                Err(UsbError::TransactionError)
            } else {
                Err(UsbError::TransactionError)
            };
        }

        // Free current qTD and retry
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Wait before retry (implementation dependent)
        // In a real system, this might schedule a delayed retry

        // Retry the transfer
        self.execute_transfer(allocator)?;

        Ok(())
    }

    /// Clean up resources
    fn cleanup<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free qTD if allocated
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Free queue head
        if let Some(qh) = self.qh_handle.take() {
            allocator.free_qh(qh)?;
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
        self.state() == BulkState::Complete
    }

    /// Check if transfer failed
    pub fn is_failed(&self) -> bool {
        matches!(self.state(), BulkState::Failed | BulkState::Stalled)
    }

    /// Check if endpoint is stalled
    pub fn is_stalled(&self) -> bool {
        self.state() == BulkState::Stalled
    }

    /// Get bytes transferred so far
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

    /// Clear endpoint stall condition
    ///
    /// This should be followed by a CLEAR_HALT control request to the device
    pub fn clear_stall(&mut self) {
        if self.state() == BulkState::Stalled {
            self.transition_state(BulkState::Idle);
            self.retry_count.store(0, Ordering::Release);
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
}

/// Bulk transfer manager for multiple concurrent transfers
pub struct BulkTransferManager<const MAX_TRANSFERS: usize = 16> {
    /// Active transfers
    transfers: [Option<BulkTransfer>; MAX_TRANSFERS],
    /// Transfer statistics
    stats: BulkTransferStats,
}

impl<const MAX_TRANSFERS: usize> BulkTransferManager<MAX_TRANSFERS> {
    /// Create new bulk transfer manager
    pub const fn new() -> Self {
        const NONE: Option<BulkTransfer> = None;
        Self {
            transfers: [NONE; MAX_TRANSFERS],
            stats: BulkTransferStats::new(),
        }
    }

    /// Submit new bulk transfer
    pub fn submit(
        &mut self,
        direction: Direction,
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        data_buffer: DmaBuffer,
        timeout_ms: u32,
    ) -> Result<usize> {
        // Find free slot
        for (index, slot) in self.transfers.iter_mut().enumerate() {
            if slot.is_none() {
                let transfer = BulkTransfer::new(
                    direction,
                    device_address,
                    endpoint,
                    max_packet_size,
                    data_buffer,
                    timeout_ms,
                );
                *slot = Some(transfer);
                self.stats.record_submission();
                return Ok(index);
            }
        }

        Err(UsbError::NoResources)
    }

    /// Start a submitted transfer
    pub fn start_transfer<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        index: usize,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        if index >= MAX_TRANSFERS {
            return Err(UsbError::InvalidParameter);
        }

        if let Some(ref mut transfer) = self.transfers[index] {
            transfer.start(allocator)?;
        } else {
            return Err(UsbError::InvalidParameter);
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

        if let Some(ref mut transfer) = self.transfers[index] {
            transfer.process_completion(allocator, status)?;

            if transfer.is_complete() {
                self.stats.record_completion();
                self.transfers[index] = None;
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
    pub fn get_transfer(&self, index: usize) -> Option<&BulkTransfer> {
        if index < MAX_TRANSFERS {
            self.transfers[index].as_ref()
        } else {
            None
        }
    }

    /// Get mutable transfer by index
    pub fn get_transfer_mut(&mut self, index: usize) -> Option<&mut BulkTransfer> {
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
    pub fn statistics(&self) -> &BulkTransferStats {
        &self.stats
    }

    /// Get count of active transfers
    pub fn active_count(&self) -> usize {
        self.transfers.iter().filter(|t| t.is_some()).count()
    }
}

/// Bulk transfer statistics
pub struct BulkTransferStats {
    submissions: AtomicU32,
    completions: AtomicU32,
    failures: AtomicU32,
    cancellations: AtomicU32,
    stalls: AtomicU32,
    total_bytes: AtomicU32,
}

impl BulkTransferStats {
    const fn new() -> Self {
        Self {
            submissions: AtomicU32::new(0),
            completions: AtomicU32::new(0),
            failures: AtomicU32::new(0),
            cancellations: AtomicU32::new(0),
            stalls: AtomicU32::new(0),
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

    fn record_stall(&self) {
        self.stalls.fetch_add(1, Ordering::Relaxed);
    }

    /// Record a STALL condition
    pub fn handle_stall(&self) {
        self.record_stall();
    }

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
}

/// Helper functions for bulk endpoint management
pub mod bulk_endpoint {
    use super::*;

    /// Create a bulk IN transfer
    pub fn create_in_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: DmaBuffer,
        timeout_ms: u32,
    ) -> BulkTransfer {
        BulkTransfer::new(
            Direction::In,
            device_address,
            endpoint,
            max_packet_size,
            buffer,
            timeout_ms,
        )
    }

    /// Create a bulk OUT transfer
    pub fn create_out_transfer(
        device_address: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: DmaBuffer,
        timeout_ms: u32,
    ) -> BulkTransfer {
        BulkTransfer::new(
            Direction::Out,
            device_address,
            endpoint,
            max_packet_size,
            buffer,
            timeout_ms,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::ptr::NonNull;

    // create a mock DMA buffer for testing (doesn't need real hardware)
    fn create_mock_buffer(size: usize) -> DmaBuffer {
        // allocate test buffer on heap (only for tests with std)
        // for no_std tests, we use a static buffer
        static mut TEST_BUFFER: [u8; 1024] = [0; 1024];
        unsafe {
            let ptr = NonNull::new_unchecked(TEST_BUFFER.as_mut_ptr());
            DmaBuffer::new(ptr, size, 0)
        }
    }

    #[test]
    fn test_bulk_transfer_initial_state() {
        let buffer = create_mock_buffer(512);
        let transfer = BulkTransfer::new(
            Direction::In,
            1,    // device_address
            0x81, // endpoint
            64,   // max_packet_size
            buffer,
            1000, // timeout_ms
        );

        assert_eq!(transfer.state(), BulkState::Idle);
        assert_eq!(transfer.bytes_transferred(), 0);
        assert_eq!(transfer.data_toggle(), 0);
        assert_eq!(transfer.direction(), Direction::In);
        assert_eq!(transfer.endpoint_address(), 0x81);
        assert_eq!(transfer.timeout_ms(), 1000);
    }

    #[test]
    fn test_bulk_transfer_state_transitions() {
        let buffer = create_mock_buffer(512);
        let transfer = BulkTransfer::new(Direction::Out, 2, 0x02, 64, buffer, 500);

        // initial state
        assert_eq!(transfer.state(), BulkState::Idle);

        // transition to active
        transfer.transition_state(BulkState::Active);
        assert_eq!(transfer.state(), BulkState::Active);

        // transition to complete
        transfer.transition_state(BulkState::Complete);
        assert_eq!(transfer.state(), BulkState::Complete);
        assert!(transfer.is_complete());
        assert!(!transfer.is_failed());
    }

    #[test]
    fn test_bulk_transfer_stalled_state() {
        let buffer = create_mock_buffer(256);
        let mut transfer = BulkTransfer::new(Direction::In, 1, 0x81, 64, buffer, 1000);

        // transition to stalled
        transfer.transition_state(BulkState::Stalled);
        assert_eq!(transfer.state(), BulkState::Stalled);
        assert!(transfer.is_stalled());
        assert!(transfer.is_failed());

        // clear stall
        transfer.clear_stall();
        assert_eq!(transfer.state(), BulkState::Idle);
        assert!(!transfer.is_stalled());
    }

    #[test]
    fn test_bulk_transfer_data_toggle() {
        let buffer = create_mock_buffer(512);
        let mut transfer = BulkTransfer::new(Direction::In, 1, 0x81, 64, buffer, 1000);

        // initial toggle is DATA0
        assert_eq!(transfer.data_toggle(), 0);

        // toggle to DATA1 (simulated by process_completion)
        let toggle = transfer.data_toggle.load(Ordering::Acquire);
        transfer.data_toggle.store(1 - toggle, Ordering::Release);
        assert_eq!(transfer.data_toggle(), 1);

        // toggle back to DATA0
        let toggle = transfer.data_toggle.load(Ordering::Acquire);
        transfer.data_toggle.store(1 - toggle, Ordering::Release);
        assert_eq!(transfer.data_toggle(), 0);

        // reset toggle
        transfer.reset_data_toggle();
        assert_eq!(transfer.data_toggle(), 0);
    }

    #[test]
    fn test_bulk_transfer_timeout_settings() {
        let buffer = create_mock_buffer(512);
        let mut transfer = BulkTransfer::new(Direction::In, 1, 0x81, 64, buffer, 1000);

        assert_eq!(transfer.timeout_ms(), 1000);

        transfer.set_timeout(2000);
        assert_eq!(transfer.timeout_ms(), 2000);

        // timeout of 0 means no timeout
        transfer.set_timeout(0);
        assert_eq!(transfer.timeout_ms(), 0);
        assert!(!transfer.is_timed_out()); // no timeout set
    }

    #[test]
    fn test_bulk_manager_submission() {
        let mut manager = BulkTransferManager::<8>::new();

        // submit first transfer
        let buffer1 = create_mock_buffer(512);
        let result1 = manager.submit(Direction::In, 1, 0x81, 64, buffer1, 1000);
        assert!(result1.is_ok());
        let index1 = result1.unwrap();
        assert_eq!(index1, 0); // first slot

        // submit second transfer
        let buffer2 = create_mock_buffer(256);
        let result2 = manager.submit(Direction::Out, 2, 0x02, 64, buffer2, 500);
        assert!(result2.is_ok());
        let index2 = result2.unwrap();
        assert_eq!(index2, 1); // second slot

        // verify active count
        assert_eq!(manager.active_count(), 2);
    }

    #[test]
    fn test_bulk_manager_pool_exhaustion() {
        let mut manager = BulkTransferManager::<4>::new();

        // fill all 4 slots
        for i in 0..4 {
            let buffer = create_mock_buffer(512);
            let result = manager.submit(Direction::In, 1, 0x81, 64, buffer, 1000);
            assert!(result.is_ok());
            assert_eq!(result.unwrap(), i);
        }

        assert_eq!(manager.active_count(), 4);

        // try to allocate 5th - should fail
        let buffer = create_mock_buffer(512);
        let result = manager.submit(Direction::In, 1, 0x81, 64, buffer, 1000);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::NoResources)));
    }

    #[test]
    fn test_bulk_manager_get_transfer() {
        let mut manager = BulkTransferManager::<8>::new();

        let buffer = create_mock_buffer(512);
        let index = manager
            .submit(Direction::In, 1, 0x81, 64, buffer, 1000)
            .unwrap();

        // get transfer by index
        let transfer = manager.get_transfer(index);
        assert!(transfer.is_some());
        assert_eq!(transfer.unwrap().state(), BulkState::Idle);

        // invalid index
        let invalid = manager.get_transfer(99);
        assert!(invalid.is_none());
    }

    #[test]
    fn test_bulk_manager_statistics() {
        let manager = BulkTransferManager::<8>::new();
        let stats = manager.statistics();

        // initial stats
        assert_eq!(stats.submissions(), 0);
        assert_eq!(stats.completions(), 0);
        assert_eq!(stats.failures(), 0);

        // simulate submissions
        stats.record_submission();
        stats.record_submission();
        assert_eq!(stats.submissions(), 2);

        // simulate completion
        stats.record_completion();
        assert_eq!(stats.completions(), 1);

        // simulate failure
        stats.record_failure();
        assert_eq!(stats.failures(), 1);

        // success rate = 1 / (1 + 1) = 0.5
        assert_eq!(stats.success_rate(), 0.5);
    }

    #[test]
    fn test_bulk_transfer_endpoint_address() {
        // IN endpoint
        let buffer1 = create_mock_buffer(512);
        let transfer_in = BulkTransfer::new(
            Direction::In,
            1,
            0x01, // endpoint number (no direction bit)
            64,
            buffer1,
            1000,
        );
        assert_eq!(transfer_in.endpoint_address(), 0x81); // IN = 0x80 | ep

        // OUT endpoint
        let buffer2 = create_mock_buffer(512);
        let transfer_out = BulkTransfer::new(
            Direction::Out,
            1,
            0x02, // endpoint number
            64,
            buffer2,
            1000,
        );
        assert_eq!(transfer_out.endpoint_address(), 0x02); // OUT = ep & 0x7F
    }

    #[test]
    fn test_bulk_stats_byte_tracking() {
        let stats = BulkTransferStats::new();

        assert_eq!(stats.total_bytes(), 0);

        stats.record_bytes(512);
        assert_eq!(stats.total_bytes(), 512);

        stats.record_bytes(256);
        assert_eq!(stats.total_bytes(), 768);
    }
}
