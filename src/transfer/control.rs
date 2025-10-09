//! USB Control transfer state machine implementation

use crate::dma::{DescriptorAllocator, QhHandle, QtdHandle};
use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};

/// USB Setup packet per USB 2.0 specification
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)] // USB spec field names
pub struct SetupPacket {
    /// Request type and direction
    pub bmRequestType: u8,
    /// Specific request
    pub bRequest: u8,
    /// Request-specific value
    pub wValue: u16,
    /// Request-specific index
    pub wIndex: u16,
    /// Data transfer length
    pub wLength: u16,
}

impl SetupPacket {
    /// Standard GET_DESCRIPTOR request
    pub const fn get_descriptor(
        desc_type: u8,
        desc_index: u8,
        language_id: u16,
        length: u16,
    ) -> Self {
        Self {
            bmRequestType: 0x80, // Device-to-host, standard, device
            bRequest: 0x06,      // GET_DESCRIPTOR
            wValue: ((desc_type as u16) << 8) | (desc_index as u16),
            wIndex: language_id,
            wLength: length,
        }
    }

    /// Standard SET_ADDRESS request
    pub const fn set_address(address: u8) -> Self {
        Self {
            bmRequestType: 0x00, // Host-to-device, standard, device
            bRequest: 0x05,      // SET_ADDRESS
            wValue: address as u16,
            wIndex: 0,
            wLength: 0,
        }
    }

    /// Standard SET_CONFIGURATION request
    pub const fn set_configuration(config_value: u8) -> Self {
        Self {
            bmRequestType: 0x00, // Host-to-device, standard, device
            bRequest: 0x09,      // SET_CONFIGURATION
            wValue: config_value as u16,
            wIndex: 0,
            wLength: 0,
        }
    }

    /// Standard CLEAR_FEATURE request for endpoint halt
    pub const fn clear_halt(endpoint: u8) -> Self {
        Self {
            bmRequestType: 0x02, // Host-to-device, standard, endpoint
            bRequest: 0x01,      // CLEAR_FEATURE
            wValue: 0,           // ENDPOINT_HALT
            wIndex: endpoint as u16,
            wLength: 0,
        }
    }
}

/// Control transfer states per USB 2.0 spec
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlState {
    /// Initial state
    Idle = 0,
    /// Setup stage in progress
    Setup = 1,
    /// Data IN stage (device to host)
    DataIn = 2,
    /// Data OUT stage (host to device)
    DataOut = 3,
    /// Status stage (handshake)
    Status = 4,
    /// Transfer complete
    Complete = 5,
    /// Transfer failed
    Failed = 6,
}

/// Control transfer context
pub struct ControlTransfer {
    /// Current state
    state: AtomicU8,
    /// Setup packet
    setup_packet: SetupPacket,
    /// Data buffer for IN/OUT transfers
    #[allow(dead_code)]
    data_buffer: Option<&'static mut [u8]>,
    /// Bytes transferred so far
    bytes_transferred: AtomicU32,
    /// Total bytes to transfer
    total_bytes: u32,
    /// Number of retry attempts
    retry_count: AtomicU8,
    /// Maximum retry attempts
    max_retries: u8,
    /// Queue head handle
    qh_handle: Option<QhHandle>,
    /// Current qTD handle
    qtd_handle: Option<QtdHandle>,
    /// Device address
    #[allow(dead_code)]
    device_address: u8,
    /// Endpoint number (usually 0 for control)
    #[allow(dead_code)]
    endpoint: u8,
    /// Maximum packet size
    max_packet_size: u16,
    /// Data toggle bit
    data_toggle: AtomicU8,
}

impl ControlTransfer {
    /// Create new control transfer
    pub fn new(
        setup_packet: SetupPacket,
        data_buffer: Option<&'static mut [u8]>,
        device_address: u8,
        max_packet_size: u16,
    ) -> Self {
        let total_bytes = if data_buffer.is_some() {
            setup_packet.wLength as u32
        } else {
            0
        };

        Self {
            state: AtomicU8::new(ControlState::Idle as u8),
            setup_packet,
            data_buffer,
            bytes_transferred: AtomicU32::new(0),
            total_bytes,
            retry_count: AtomicU8::new(0),
            max_retries: 3,
            qh_handle: None,
            qtd_handle: None,
            device_address,
            endpoint: 0, // Control endpoint
            max_packet_size,
            data_toggle: AtomicU8::new(0),
        }
    }

    /// Get current state
    pub fn state(&self) -> ControlState {
        match self.state.load(Ordering::Acquire) {
            0 => ControlState::Idle,
            1 => ControlState::Setup,
            2 => ControlState::DataIn,
            3 => ControlState::DataOut,
            4 => ControlState::Status,
            5 => ControlState::Complete,
            6 => ControlState::Failed,
            _ => ControlState::Failed,
        }
    }

    /// Transition to new state
    fn transition_state(&self, new_state: ControlState) {
        self.state.store(new_state as u8, Ordering::Release);
    }

    /// Start control transfer
    pub fn start<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        if self.state() != ControlState::Idle {
            return Err(UsbError::InvalidState);
        }

        // Allocate queue head for control transfer
        let qh_handle = allocator.alloc_qh()?;
        self.qh_handle = Some(qh_handle);

        // Start with SETUP stage
        self.transition_state(ControlState::Setup);
        self.execute_setup_stage(allocator)?;

        Ok(())
    }

    /// Execute SETUP stage
    fn execute_setup_stage<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Allocate qTD for SETUP packet
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // Configure qTD for SETUP packet
        // This would normally interact with the actual hardware descriptors
        // For now, we're setting up the state machine

        // SETUP always uses DATA0
        self.data_toggle.store(0, Ordering::Release);

        Ok(())
    }

    /// Process transfer completion from interrupt handler
    pub fn process_completion<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<()> {
        // Check for errors in status
        if status & 0x7C != 0 {
            // Error bits
            return self.handle_error(allocator, status);
        }

        match self.state() {
            ControlState::Setup => {
                // SETUP complete, move to DATA or STATUS stage
                if self.total_bytes > 0 {
                    if self.setup_packet.bmRequestType & 0x80 != 0 {
                        // Device-to-host (IN)
                        self.transition_state(ControlState::DataIn);
                        self.execute_data_in_stage(allocator)?;
                    } else {
                        // Host-to-device (OUT)
                        self.transition_state(ControlState::DataOut);
                        self.execute_data_out_stage(allocator)?;
                    }
                } else {
                    // No data stage, go directly to STATUS
                    self.transition_state(ControlState::Status);
                    self.execute_status_stage(allocator)?;
                }
            }
            ControlState::DataIn => {
                // Update bytes transferred
                let bytes_in_qtd = self.get_bytes_from_qtd(status);
                self.bytes_transferred
                    .fetch_add(bytes_in_qtd, Ordering::AcqRel);

                // Toggle data bit for next transfer
                let toggle = self.data_toggle.load(Ordering::Acquire);
                self.data_toggle.store(1 - toggle, Ordering::Release);

                if self.bytes_transferred.load(Ordering::Acquire) >= self.total_bytes {
                    // All data received, move to STATUS
                    self.transition_state(ControlState::Status);
                    self.execute_status_stage(allocator)?;
                } else {
                    // More data to receive
                    self.execute_data_in_stage(allocator)?;
                }
            }
            ControlState::DataOut => {
                // Update bytes transferred
                let bytes_out = self.get_bytes_from_qtd(status);
                self.bytes_transferred
                    .fetch_add(bytes_out, Ordering::AcqRel);

                // Toggle data bit for next transfer
                let toggle = self.data_toggle.load(Ordering::Acquire);
                self.data_toggle.store(1 - toggle, Ordering::Release);

                if self.bytes_transferred.load(Ordering::Acquire) >= self.total_bytes {
                    // All data sent, move to STATUS
                    self.transition_state(ControlState::Status);
                    self.execute_status_stage(allocator)?;
                } else {
                    // More data to send
                    self.execute_data_out_stage(allocator)?;
                }
            }
            ControlState::Status => {
                // Transfer complete!
                self.transition_state(ControlState::Complete);
                self.cleanup(allocator)?;
            }
            _ => {
                return Err(UsbError::InvalidState);
            }
        }

        Ok(())
    }

    /// Execute DATA IN stage
    fn execute_data_in_stage<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free previous qTD if any
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Allocate new qTD for DATA IN
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // Calculate transfer size for this qTD
        let remaining = self.total_bytes - self.bytes_transferred.load(Ordering::Acquire);
        let _transfer_size = remaining.min(self.max_packet_size as u32);

        // Configure qTD for DATA IN with current toggle bit
        let _toggle = self.data_toggle.load(Ordering::Acquire);
        // Hardware configuration would happen here

        Ok(())
    }

    /// Execute DATA OUT stage
    fn execute_data_out_stage<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free previous qTD if any
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Allocate new qTD for DATA OUT
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // Calculate transfer size for this qTD
        let remaining = self.total_bytes - self.bytes_transferred.load(Ordering::Acquire);
        let _transfer_size = remaining.min(self.max_packet_size as u32);

        // Configure qTD for DATA OUT with current toggle bit
        let _toggle = self.data_toggle.load(Ordering::Acquire);
        // Hardware configuration would happen here

        Ok(())
    }

    /// Execute STATUS stage
    fn execute_status_stage<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
    ) -> Result<()> {
        // Free previous qTD if any
        if let Some(qtd) = self.qtd_handle.take() {
            allocator.free_qtd(qtd)?;
        }

        // Allocate qTD for STATUS
        let qtd_handle = allocator.alloc_qtd()?;
        self.qtd_handle = Some(qtd_handle);

        // STATUS stage always uses DATA1
        self.data_toggle.store(1, Ordering::Release);

        // Direction is opposite of data stage
        // Zero-length packet for handshake

        Ok(())
    }

    /// Handle transfer error
    fn handle_error<const N_QH: usize, const N_QTD: usize>(
        &mut self,
        allocator: &mut DescriptorAllocator<N_QH, N_QTD>,
        status: u32,
    ) -> Result<()> {
        let retry_count = self.retry_count.fetch_add(1, Ordering::AcqRel);

        if retry_count >= self.max_retries {
            // Max retries exceeded, fail transfer
            self.transition_state(ControlState::Failed);
            self.cleanup(allocator)?;

            // Determine specific error from status
            if status & (1 << 6) != 0 {
                return Err(UsbError::Stall);
            } else if status & (1 << 5) != 0 {
                return Err(UsbError::BufferOverflow);
            } else if status & (1 << 4) != 0 {
                return Err(UsbError::TransactionError);
            } else if status & (1 << 3) != 0 {
                return Err(UsbError::TransactionError);
            }
            return Err(UsbError::TransactionError);
        }

        // Retry current stage
        match self.state() {
            ControlState::Setup => self.execute_setup_stage(allocator)?,
            ControlState::DataIn => self.execute_data_in_stage(allocator)?,
            ControlState::DataOut => self.execute_data_out_stage(allocator)?,
            ControlState::Status => self.execute_status_stage(allocator)?,
            _ => return Err(UsbError::InvalidState),
        }

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

    /// Get bytes transferred from qTD status
    fn get_bytes_from_qtd(&self, status: u32) -> u32 {
        // Extract remaining bytes from qTD token field
        // Bits 30:16 contain remaining bytes (decrements as transfer progresses)
        let remaining = (status >> 16) & 0x7FFF;
        // Calculate bytes transferred = original total - remaining
        self.total_bytes.saturating_sub(remaining)
    }

    /// Check if transfer is complete
    pub fn is_complete(&self) -> bool {
        self.state() == ControlState::Complete
    }

    /// Check if transfer failed
    pub fn is_failed(&self) -> bool {
        self.state() == ControlState::Failed
    }

    /// Get bytes transferred
    pub fn bytes_transferred(&self) -> u32 {
        self.bytes_transferred.load(Ordering::Acquire)
    }
}

/// Control transfer manager for multiple concurrent transfers
pub struct ControlTransferManager<const MAX_TRANSFERS: usize = 8> {
    /// Active transfers
    transfers: [Option<ControlTransfer>; MAX_TRANSFERS],
    /// Statistics
    stats: TransferStats,
}

impl<const MAX_TRANSFERS: usize> ControlTransferManager<MAX_TRANSFERS> {
    /// Create new transfer manager
    pub const fn new() -> Self {
        const NONE: Option<ControlTransfer> = None;
        Self {
            transfers: [NONE; MAX_TRANSFERS],
            stats: TransferStats::new(),
        }
    }

    /// Submit new control transfer
    pub fn submit(
        &mut self,
        setup_packet: SetupPacket,
        data_buffer: Option<&'static mut [u8]>,
        device_address: u8,
        max_packet_size: u16,
    ) -> Result<usize> {
        // Find free slot
        for (index, slot) in self.transfers.iter_mut().enumerate() {
            if slot.is_none() {
                let transfer = ControlTransfer::new(
                    setup_packet,
                    data_buffer,
                    device_address,
                    max_packet_size,
                );
                *slot = Some(transfer);
                self.stats.record_submission();
                return Ok(index);
            }
        }

        Err(UsbError::NoResources)
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

        if let Some(transfer) = &mut self.transfers[index] {
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

    /// Get transfer statistics
    pub fn statistics(&self) -> &TransferStats {
        &self.stats
    }
}

/// Transfer statistics
pub struct TransferStats {
    submissions: AtomicU32,
    completions: AtomicU32,
    failures: AtomicU32,
}

impl TransferStats {
    const fn new() -> Self {
        Self {
            submissions: AtomicU32::new(0),
            completions: AtomicU32::new(0),
            failures: AtomicU32::new(0),
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

    /// Get success rate
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
}
