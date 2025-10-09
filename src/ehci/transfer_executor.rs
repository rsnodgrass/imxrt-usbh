//! Transfer execution layer - bridges high-level transfers to EHCI hardware
//!
//! This module provides the critical glue between transfer manager APIs
//! and actual EHCI Queue Head/Queue Transfer Descriptor programming.

use crate::dma::DmaBuffer;
use crate::ehci::qh::{endpoint, QueueHead};
use crate::ehci::qtd::{token, QueueTD};
use crate::error::{Result, UsbError};
use crate::transfer::Direction;
use core::sync::atomic::{AtomicU32, Ordering};

/// Static pool of queue heads (32-byte aligned)
static mut QH_POOL: [QueueHead; 32] = [const { QueueHead::new() }; 32];

/// Static pool of queue TDs (32-byte aligned)
static mut QTD_POOL: [QueueTD; 64] = [const { QueueTD::new() }; 64];

/// QH allocation bitmap
static QH_ALLOCATED: AtomicU32 = AtomicU32::new(0);

/// qTD allocation bitmap
static QTD_ALLOCATED: AtomicU32 = AtomicU32::new(0);

/// Simple handle to allocated QH
#[derive(Debug, Clone, Copy)]
pub struct QhHandle {
    index: usize,
}

/// Simple handle to allocated qTD
#[derive(Debug, Clone, Copy)]
pub struct QtdHandle {
    index: usize,
}

/// RAII guard for automatic resource cleanup on error paths
///
/// Ensures QH and qTDs are freed even if transfer setup fails partway through.
/// Accesses allocation bitmaps directly to avoid borrowing conflicts.
struct ResourceGuard {
    qh: Option<QhHandle>,
    qtds: heapless::Vec<QtdHandle, 4>,
}

impl ResourceGuard {
    fn new() -> Self {
        Self {
            qh: None,
            qtds: heapless::Vec::new(),
        }
    }

    fn set_qh(&mut self, qh: QhHandle) {
        self.qh = Some(qh);
    }

    fn add_qtd(&mut self, qtd: QtdHandle) {
        let _ = self.qtds.push(qtd);
    }

    /// Disarm the guard - resources will not be freed on drop
    fn disarm(mut self) {
        self.qh = None;
        self.qtds.clear();
    }
}

impl Drop for ResourceGuard {
    fn drop(&mut self) {
        // Free all qTDs by clearing allocation bits directly
        for qtd in &self.qtds {
            QTD_ALLOCATED.fetch_and(!(1 << qtd.index), Ordering::AcqRel);
        }

        // Free QH if allocated
        if let Some(qh) = self.qh {
            QH_ALLOCATED.fetch_and(!(1 << qh.index), Ordering::AcqRel);
        }
    }
}

/// Transfer executor - wires transfer requests to EHCI hardware
pub struct TransferExecutor {
    /// EHCI operational register base address
    op_base: usize,
}

impl TransferExecutor {
    /// Create new transfer executor
    ///
    /// # Safety
    ///
    /// Caller must ensure op_base points to valid EHCI operational registers
    pub unsafe fn new(op_base: usize) -> Self {
        Self { op_base }
    }

    /// Allocate a QH from the pool
    fn alloc_qh(&self) -> Result<QhHandle> {
        loop {
            let current = QH_ALLOCATED.load(Ordering::Acquire);
            let free_bit = (!current).trailing_zeros();

            if free_bit >= 32 {
                #[cfg(feature = "defmt")]
                defmt::warn!("QH pool exhausted - all 32 queue heads allocated");
                return Err(UsbError::NoResources);
            }

            let new_value = current | (1 << free_bit);
            if QH_ALLOCATED
                .compare_exchange(current, new_value, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                // Reset the QH
                unsafe {
                    QH_POOL[free_bit as usize] = QueueHead::new();
                }
                return Ok(QhHandle {
                    index: free_bit as usize,
                });
            }
        }
    }

    /// Free a QH back to the pool
    fn free_qh(&self, handle: QhHandle) -> Result<()> {
        if handle.index >= 32 {
            return Err(UsbError::InvalidParameter);
        }

        QH_ALLOCATED.fetch_and(!(1 << handle.index), Ordering::AcqRel);
        Ok(())
    }

    /// Allocate a qTD from the pool
    fn alloc_qtd(&self) -> Result<QtdHandle> {
        loop {
            let current = QTD_ALLOCATED.load(Ordering::Acquire);
            let free_bit = (!current).trailing_zeros();

            if free_bit >= 64 {
                #[cfg(feature = "defmt")]
                defmt::warn!("qTD pool exhausted - all 64 transfer descriptors allocated");
                return Err(UsbError::NoResources);
            }

            let new_value = current | (1 << free_bit);
            if QTD_ALLOCATED
                .compare_exchange(current, new_value, Ordering::AcqRel, Ordering::Acquire)
                .is_ok()
            {
                // Reset the qTD
                unsafe {
                    QTD_POOL[free_bit as usize] = QueueTD::new();
                }
                return Ok(QtdHandle {
                    index: free_bit as usize,
                });
            }
        }
    }

    /// Free a qTD back to the pool
    fn free_qtd(&self, handle: QtdHandle) -> Result<()> {
        if handle.index >= 64 {
            return Err(UsbError::InvalidParameter);
        }

        QTD_ALLOCATED.fetch_and(!(1 << handle.index), Ordering::AcqRel);
        Ok(())
    }

    /// Get QH reference
    ///
    /// NOTE: Currently unused - mutable version get_qh_mut() is used instead.
    /// Safe to remove if no future use case for immutable access.
    #[allow(dead_code)]
    fn get_qh(&self, handle: &QhHandle) -> Result<&QueueHead> {
        if handle.index >= 32 {
            return Err(UsbError::InvalidParameter);
        }
        Ok(unsafe { &QH_POOL[handle.index] })
    }

    /// Get mutable QH reference
    fn get_qh_mut(&mut self, handle: &QhHandle) -> Result<&mut QueueHead> {
        if handle.index >= 32 {
            return Err(UsbError::InvalidParameter);
        }
        Ok(unsafe { &mut QH_POOL[handle.index] })
    }

    /// Get qTD reference
    fn get_qtd(&self, handle: &QtdHandle) -> Result<&QueueTD> {
        if handle.index >= 64 {
            return Err(UsbError::InvalidParameter);
        }
        Ok(unsafe { &QTD_POOL[handle.index] })
    }

    /// Get mutable qTD reference
    fn get_qtd_mut(&mut self, handle: &QtdHandle) -> Result<&mut QueueTD> {
        if handle.index >= 64 {
            return Err(UsbError::InvalidParameter);
        }
        Ok(unsafe { &mut QTD_POOL[handle.index] })
    }

    /// Get QH physical address
    fn get_qh_addr(&self, handle: &QhHandle) -> Result<u32> {
        if handle.index >= 32 {
            return Err(UsbError::InvalidParameter);
        }
        let qh = unsafe { &QH_POOL[handle.index] };
        Ok(qh as *const QueueHead as u32)
    }

    /// Get qTD physical address
    fn get_qtd_addr(&self, handle: &QtdHandle) -> Result<u32> {
        if handle.index >= 64 {
            return Err(UsbError::InvalidParameter);
        }
        let qtd = unsafe { &QTD_POOL[handle.index] };
        Ok(qtd as *const QueueTD as u32)
    }

    /// Execute control transfer (SETUP + DATA + STATUS)
    ///
    /// # Safety
    ///
    /// Buffer must remain valid until transfer completes
    pub unsafe fn execute_control_transfer(
        &mut self,
        device_addr: u8,
        max_packet_size: u16,
        setup_packet: &[u8; 8],
        data_buffer: Option<&mut DmaBuffer>,
        direction: Direction,
    ) -> Result<usize> {
        // Create RAII guard for automatic cleanup on error
        let mut guard = ResourceGuard::new();

        // Allocate QH for control endpoint
        let qh_handle = self.alloc_qh()?;
        guard.set_qh(qh_handle);

        // Initialize QH for control endpoint 0 (scope the borrow)
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            qh.init_endpoint(
                device_addr,
                0, // Control endpoint is always 0
                max_packet_size,
                endpoint::SPEED_HIGH, // Assume high-speed, adjust for hub devices
                true,                 // is_control
            )?;
        }

        // SETUP stage - always DATA0
        let setup_qtd_handle = self.alloc_qtd()?;
        guard.add_qtd(setup_qtd_handle);

        {
            let setup_qtd = self.get_qtd_mut(&setup_qtd_handle)?;
            unsafe {
                setup_qtd.init_transfer(
                    token::PID_SETUP,
                    false, // DATA0
                    Some((setup_packet.as_ptr(), 8)),
                    false, // no interrupt on SETUP
                )?;
            }
        }

        // DATA stage (if present)
        let data_qtd_handle = if let Some(buffer) = data_buffer.as_ref() {
            let qtd_handle = self.alloc_qtd()?;
            guard.add_qtd(qtd_handle);

            let pid = match direction {
                Direction::In => token::PID_IN,
                Direction::Out => token::PID_OUT,
            };

            // Prepare buffer for DMA
            if direction == Direction::Out {
                buffer.prepare_for_device();
            }

            {
                let qtd = self.get_qtd_mut(&qtd_handle)?;
                unsafe {
                    qtd.init_transfer(
                        pid,
                        true, // DATA1 for data stage
                        Some((buffer.as_ptr(), buffer.len())),
                        false, // no interrupt on DATA
                    )?;
                }
            }

            Some(qtd_handle)
        } else {
            None
        };

        // STATUS stage - always DATA1, opposite direction
        let status_qtd_handle = self.alloc_qtd()?;
        guard.add_qtd(status_qtd_handle);

        let status_pid = if data_buffer.is_some() {
            // If data stage present, status is opposite direction
            match direction {
                Direction::In => token::PID_OUT,
                Direction::Out => token::PID_IN,
            }
        } else {
            // No data stage, status is IN
            token::PID_IN
        };

        {
            let status_qtd = self.get_qtd_mut(&status_qtd_handle)?;
            unsafe {
                status_qtd.init_transfer(
                    status_pid, true, // DATA1
                    None, // Zero-length packet
                    true, // Interrupt on complete
                )?;
            }
        }

        // Link qTDs: SETUP → DATA → STATUS
        let setup_qtd_addr = self.get_qtd_addr(&setup_qtd_handle)?;
        let status_qtd_addr = self.get_qtd_addr(&status_qtd_handle)?;

        if let Some(data_handle) = data_qtd_handle.as_ref() {
            let data_qtd_addr = self.get_qtd_addr(data_handle)?;

            // SETUP → DATA
            let setup_qtd = self.get_qtd_mut(&setup_qtd_handle)?;
            setup_qtd.next_qtd.store(data_qtd_addr, Ordering::Release);

            // DATA → STATUS
            let data_qtd = self.get_qtd_mut(data_handle)?;
            data_qtd.next_qtd.store(status_qtd_addr, Ordering::Release);
        } else {
            // SETUP → STATUS (no data stage)
            let setup_qtd = self.get_qtd_mut(&setup_qtd_handle)?;
            setup_qtd.next_qtd.store(status_qtd_addr, Ordering::Release);
        }

        // Link QH to first qTD
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            unsafe {
                qh.link_qtd(setup_qtd_addr)?;
            }
        }

        // Add QH to async schedule
        self.link_qh_to_async_schedule(&qh_handle)?;

        // Enable async schedule if not already running
        self.enable_async_schedule()?;

        // Wait for transfer completion
        let bytes_transferred = self.wait_for_completion(&status_qtd_handle, data_buffer)?;

        // Clean up (unlink from schedule, then free resources)
        self.unlink_qh_from_async_schedule(&qh_handle)?;

        // Transfer successful - disarm guard and manually free resources
        // (Resources freed in reverse order: qTDs first, then QH)
        guard.disarm();
        self.free_qtd(status_qtd_handle)?;
        if let Some(data_handle) = data_qtd_handle {
            self.free_qtd(data_handle)?;
        }
        self.free_qtd(setup_qtd_handle)?;
        self.free_qh(qh_handle)?;

        Ok(bytes_transferred)
    }

    /// Execute bulk transfer
    pub fn execute_bulk_transfer(
        &mut self,
        device_addr: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: &mut DmaBuffer,
        direction: Direction,
        timeout_ms: u32,
    ) -> Result<usize> {
        // Allocate QH and qTD
        let qh_handle = self.alloc_qh()?;

        // Initialize QH for bulk endpoint
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            qh.init_endpoint(
                device_addr,
                endpoint & 0x7F, // Remove direction bit
                max_packet_size,
                endpoint::SPEED_HIGH,
                false, // not control
            )?;
        }

        let qtd_handle = self.alloc_qtd()?;

        let pid = match direction {
            Direction::In => token::PID_IN,
            Direction::Out => token::PID_OUT,
        };

        // Prepare buffer for DMA
        if direction == Direction::Out {
            buffer.prepare_for_device();
        }

        {
            let qtd = self.get_qtd_mut(&qtd_handle)?;
            unsafe {
                qtd.init_transfer(
                    pid,
                    false, // DATA0 initially (should track toggle)
                    Some((buffer.as_ptr(), buffer.len())),
                    true, // Interrupt on complete
                )?;
            }
        }

        // Link QH to qTD
        let qtd_addr = self.get_qtd_addr(&qtd_handle)?;
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            unsafe {
                qh.link_qtd(qtd_addr)?;
            }
        }

        // Add to async schedule
        self.link_qh_to_async_schedule(&qh_handle)?;
        self.enable_async_schedule()?;

        // Wait for completion with timeout
        let bytes_transferred =
            self.wait_for_completion_timeout(&qtd_handle, Some(buffer), timeout_ms)?;

        // Clean up
        self.unlink_qh_from_async_schedule(&qh_handle)?;
        self.free_qtd(qtd_handle)?;
        self.free_qh(qh_handle)?;

        Ok(bytes_transferred)
    }

    /// Execute interrupt transfer
    ///
    /// # Arguments
    ///
    /// * `device_addr` - USB device address
    /// * `endpoint` - Endpoint number
    /// * `max_packet_size` - Maximum packet size for endpoint
    /// * `buffer` - DMA buffer for data
    /// * `direction` - Transfer direction (In/Out)
    /// * `interval` - Polling interval in frames (1, 2, 4, 8, 16, etc.)
    ///
    pub fn execute_interrupt_transfer(
        &mut self,
        device_addr: u8,
        endpoint: u8,
        max_packet_size: u16,
        buffer: &mut DmaBuffer,
        direction: Direction,
        interval: u32,
    ) -> Result<usize> {
        // Allocate QH for interrupt endpoint
        let qh_handle = self.alloc_qh()?;

        // Initialize QH for interrupt endpoint
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            qh.init_endpoint(
                device_addr,
                endpoint & 0x7F,
                max_packet_size,
                endpoint::SPEED_HIGH,
                false, // not control
            )?;
        }

        let qtd_handle = self.alloc_qtd()?;

        let pid = match direction {
            Direction::In => token::PID_IN,
            Direction::Out => token::PID_OUT,
        };

        // Prepare buffer for DMA
        if direction == Direction::Out {
            buffer.prepare_for_device();
        }

        {
            let qtd = self.get_qtd_mut(&qtd_handle)?;
            unsafe {
                qtd.init_transfer(
                    pid,
                    false, // DATA0 initially
                    Some((buffer.as_ptr(), buffer.len())),
                    true, // Interrupt on complete
                )?;
            }
        }

        // Link QH to qTD
        let qtd_addr = self.get_qtd_addr(&qtd_handle)?;
        {
            let qh = self.get_qh_mut(&qh_handle)?;
            unsafe {
                qh.link_qtd(qtd_addr)?;
            }
        }

        // Link to periodic schedule
        self.link_qh_to_periodic_schedule(&qh_handle, interval)?;

        // Enable periodic schedule if not already running
        unsafe {
            super::periodic::enable_periodic_schedule(self.op_base)?;
        }

        // Wait for completion (interrupt transfers are periodic, poll for completion)
        let bytes_transferred =
            self.wait_for_completion_timeout(&qtd_handle, Some(buffer), 1000)?;

        // Clean up
        self.unlink_qh_from_periodic_schedule(&qh_handle, interval)?;
        self.free_qtd(qtd_handle)?;
        self.free_qh(qh_handle)?;

        Ok(bytes_transferred)
    }

    /// Link QH to periodic schedule
    fn link_qh_to_periodic_schedule(&mut self, qh_handle: &QhHandle, interval: u32) -> Result<()> {
        let qh_addr = self.get_qh_addr(qh_handle)?;

        // Get frame list and scheduler
        let frame_list = unsafe { super::periodic::get_frame_list() };
        let scheduler = super::periodic::get_scheduler();

        // Find optimal offset for load balancing
        let offset = scheduler.find_best_offset(interval);

        // Link QH to periodic schedule
        unsafe {
            frame_list.link_qh_periodic(interval, qh_addr)?;
        }

        // Update scheduler tracking
        scheduler.schedule_qh(interval, offset, 64); // Assume 64-byte packets for tracking

        Ok(())
    }

    /// Unlink QH from periodic schedule
    fn unlink_qh_from_periodic_schedule(
        &mut self,
        _qh_handle: &QhHandle,
        interval: u32,
    ) -> Result<()> {
        let frame_list = unsafe { super::periodic::get_frame_list() };
        let scheduler = super::periodic::get_scheduler();

        // Unlink from all frames (simplified - production should track offset)
        let mut frame = 0;
        while frame < super::periodic::FRAME_LIST_SIZE {
            let _ = frame_list.unlink_qh(frame); // Ignore errors
            frame += interval as usize;
        }

        // Update scheduler tracking (use offset 0 as we don't track it)
        scheduler.unschedule_qh(interval, 0, 64);

        Ok(())
    }

    /// Link QH to async schedule
    fn link_qh_to_async_schedule(&mut self, qh_handle: &QhHandle) -> Result<()> {
        let qh_addr = self.get_qh_addr(qh_handle)?;
        let op_base = self.op_base;

        // Read current async list head
        let asynclistaddr = unsafe { core::ptr::read_volatile((op_base + 0x18) as *const u32) };

        // Update QH based on whether async schedule exists
        {
            let qh = self.get_qh_mut(qh_handle)?;

            if asynclistaddr == 0 || (asynclistaddr & 1) != 0 {
                // No existing async schedule, make this QH the head
                qh.set_head_of_list();
                qh.horizontal_link.store(
                    qh_addr | QueueHead::TYPE_QH, // Point to self
                    Ordering::Release,
                );
            } else {
                // Insert into existing circular list
                let head_qh_addr = asynclistaddr & !0x1F;

                // Link new QH to current head
                qh.horizontal_link
                    .store(head_qh_addr | QueueHead::TYPE_QH, Ordering::Release);

                // Note: In production, should update previous QH to point to this one
                // For simplicity, just prepending to list
            }
        }

        // Set async list head if this is the first QH
        if asynclistaddr == 0 || (asynclistaddr & 1) != 0 {
            unsafe {
                cortex_m::asm::dmb();
                core::ptr::write_volatile((op_base + 0x18) as *mut u32, qh_addr);
                cortex_m::asm::dsb();
            }
        }

        Ok(())
    }

    /// Unlink QH from async schedule
    fn unlink_qh_from_async_schedule(&mut self, qh_handle: &QhHandle) -> Result<()> {
        // Simplified: Just reset the QH overlay
        let qh = self.get_qh_mut(qh_handle)?;
        qh.reset_overlay();

        // In production, should:
        // 1. Find previous QH in list
        // 2. Update its horizontal_link to skip this QH
        // 3. Use async advance doorbell to safely remove

        Ok(())
    }

    /// Enable async schedule
    fn enable_async_schedule(&self) -> Result<()> {
        let usbcmd_addr = self.op_base as *mut u32;

        unsafe {
            cortex_m::asm::dmb();
            let mut usbcmd = core::ptr::read_volatile(usbcmd_addr);
            usbcmd |= 1 << 5; // Async Schedule Enable
            core::ptr::write_volatile(usbcmd_addr, usbcmd);
            cortex_m::asm::dsb();
        }

        // Wait for schedule to actually start
        let usbsts_addr = (self.op_base + 0x04) as *const u32;
        let timeout = super::RegisterTimeout::new_ms(10);

        timeout.wait_for(|| {
            let usbsts = unsafe { core::ptr::read_volatile(usbsts_addr) };
            (usbsts & (1 << 15)) != 0 // Async Schedule Status
        })?;

        Ok(())
    }

    /// Wait for transfer completion
    fn wait_for_completion(
        &self,
        qtd_handle: &QtdHandle,
        data_buffer: Option<&mut DmaBuffer>,
    ) -> Result<usize> {
        self.wait_for_completion_timeout(qtd_handle, data_buffer, 1000)
    }

    /// Wait for transfer completion with timeout
    fn wait_for_completion_timeout(
        &self,
        qtd_handle: &QtdHandle,
        data_buffer: Option<&mut DmaBuffer>,
        timeout_ms: u32,
    ) -> Result<usize> {
        let qtd = self.get_qtd(qtd_handle)?;
        let timeout = super::RegisterTimeout::new_ms(timeout_ms);

        // Poll for completion
        timeout.wait_for(|| !qtd.is_active())?;

        // Check for errors
        if let Some(error) = qtd.has_error() {
            return Err(error);
        }

        // For IN transfers, invalidate cache after DMA completion
        if let Some(buffer) = data_buffer {
            buffer.prepare_for_cpu();
        }

        // Calculate bytes transferred
        let token = qtd.token.load(Ordering::Acquire);
        let total_bytes = (token >> token::TOTAL_BYTES_SHIFT) & token::TOTAL_BYTES_MASK;

        Ok(total_bytes as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transfer_executor_creation() {
        let executor = unsafe { TransferExecutor::new(0x402E_0200) };
        // Verify executor was created (basic smoke test)
        let _ = executor;
    }

    #[test]
    fn test_qh_allocation() {
        let executor = unsafe { TransferExecutor::new(0x402E_0200) };

        // Allocate a QH
        let handle = executor.alloc_qh().expect("Failed to allocate QH");

        // Verify we can get the QH
        let qh = executor.get_qh(&handle).expect("Failed to get QH");
        let _ = qh;

        // Free it
        executor.free_qh(handle).expect("Failed to free QH");
    }

    #[test]
    fn test_qtd_allocation() {
        let executor = unsafe { TransferExecutor::new(0x402E_0200) };

        // Allocate a qTD
        let handle = executor.alloc_qtd().expect("Failed to allocate qTD");

        // Verify we can get the qTD
        let qtd = executor.get_qtd(&handle).expect("Failed to get qTD");
        let _ = qtd;

        // Free it
        executor.free_qtd(handle).expect("Failed to free qTD");
    }
}
