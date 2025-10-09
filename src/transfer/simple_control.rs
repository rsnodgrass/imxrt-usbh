//! Simplified USB Control transfer implementation
//!
//! Streamlined control transfer without excessive state management

use crate::dma::{DmaBuffer, UsbMemoryPool};
use crate::ehci::TransferExecutor;
use crate::error::{Result, UsbError};
use crate::transfer::Direction;

/// USB Setup packet (USB 2.0 spec)
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
#[allow(non_snake_case)] // USB spec field names
#[allow(missing_docs)]
pub struct SetupPacket {
    pub bmRequestType: u8,
    pub bRequest: u8,
    pub wValue: u16,
    pub wIndex: u16,
    pub wLength: u16,
}

impl SetupPacket {
    /// GET_DESCRIPTOR request
    pub const fn get_descriptor(desc_type: u8, desc_index: u8, length: u16) -> Self {
        Self {
            bmRequestType: 0x80,
            bRequest: 0x06,
            wValue: ((desc_type as u16) << 8) | (desc_index as u16),
            wIndex: 0,
            wLength: length,
        }
    }

    /// SET_ADDRESS request
    pub const fn set_address(address: u8) -> Self {
        Self {
            bmRequestType: 0x00,
            bRequest: 0x05,
            wValue: address as u16,
            wIndex: 0,
            wLength: 0,
        }
    }

    /// SET_CONFIGURATION request
    pub const fn set_configuration(config: u8) -> Self {
        Self {
            bmRequestType: 0x00,
            bRequest: 0x09,
            wValue: config as u16,
            wIndex: 0,
            wLength: 0,
        }
    }
}

/// Simplified control transfer
pub struct SimpleControlTransfer {
    setup: SetupPacket,
    data_buffer: Option<DmaBuffer>,
    device_address: u8,
    endpoint: u8,
    max_packet_size: u16,
    completed: bool,
}

impl SimpleControlTransfer {
    /// Create new control transfer
    pub fn new(setup: SetupPacket, device_address: u8, max_packet_size: u16) -> Self {
        Self {
            setup,
            data_buffer: None,
            device_address,
            endpoint: 0,
            max_packet_size,
            completed: false,
        }
    }

    /// Execute control transfer synchronously using TransferExecutor
    pub fn execute(
        &mut self,
        executor: &mut TransferExecutor,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<usize> {
        // Allocate data buffer if needed
        let mut data_buffer = if self.setup.wLength > 0 {
            Some(memory_pool.alloc_buffer(self.setup.wLength as usize)?)
        } else {
            None
        };

        // Determine direction
        let direction = if self.setup.bmRequestType & 0x80 != 0 {
            Direction::In
        } else {
            Direction::Out
        };

        // Convert setup packet to byte array
        // Safety: Using transmute_copy to avoid alignment issues with packed struct
        let setup_bytes: [u8; 8] = unsafe { core::mem::transmute_copy(&self.setup) };

        // Execute control transfer via TransferExecutor
        // Safety: setup_bytes is valid for duration of call, data_buffer (if Some) is DmaBuffer
        // from pool that remains valid until stored in self.data_buffer after transfer completes
        let bytes_transferred = unsafe {
            executor.execute_control_transfer(
                self.device_address,
                self.max_packet_size,
                &setup_bytes,
                data_buffer.as_mut(),
                direction,
            )?
        };

        // Store buffer for later retrieval
        self.data_buffer = data_buffer;
        self.completed = true;
        Ok(bytes_transferred)
    }

    pub fn device_address(&self) -> u8 {
        self.device_address
    }

    pub fn endpoint(&self) -> u8 {
        self.endpoint
    }

    pub fn set_device_address(&mut self, address: u8) {
        self.device_address = address;
    }

    pub fn target_info(&self) -> (u8, u8) {
        (self.device_address, self.endpoint)
    }

    /// Get data buffer if transfer is complete
    pub fn get_data(&self) -> Option<&[u8]> {
        if self.completed {
            self.data_buffer.as_ref().map(|b| b.as_slice())
        } else {
            None
        }
    }
}

/// Simple control transfer executor using hardware TransferExecutor
pub struct ControlExecutor<'a> {
    executor: &'a mut TransferExecutor,
    memory_pool: &'a mut UsbMemoryPool,
}

impl<'a> ControlExecutor<'a> {
    /// Create new control executor
    pub fn new(executor: &'a mut TransferExecutor, memory_pool: &'a mut UsbMemoryPool) -> Self {
        Self {
            executor,
            memory_pool,
        }
    }

    /// Execute control transfer with retry
    pub fn execute_with_retry(
        &mut self,
        setup: SetupPacket,
        device_address: u8,
        max_packet_size: u16,
        max_retries: u8,
    ) -> Result<heapless::Vec<u8, 256>> {
        for attempt in 0..=max_retries {
            let mut transfer = SimpleControlTransfer::new(setup, device_address, max_packet_size);

            match transfer.execute(self.executor, self.memory_pool) {
                Ok(_) => {
                    // Copy data before returning
                    if let Some(data) = transfer.get_data() {
                        let mut result = heapless::Vec::new();
                        result.extend_from_slice(data).ok();
                        return Ok(result);
                    }
                    return Ok(heapless::Vec::new());
                }
                Err(e) if attempt < max_retries => {
                    // Retry on transient errors
                    match e {
                        UsbError::Timeout | UsbError::TransactionError | UsbError::Nak => continue,
                        _ => return Err(e),
                    }
                }
                Err(e) => return Err(e),
            }
        }

        Err(UsbError::Timeout)
    }

    /// Helper: Get device descriptor
    pub fn get_device_descriptor(&mut self, device_address: u8) -> Result<[u8; 18]> {
        let setup = SetupPacket::get_descriptor(0x01, 0, 18);
        let data = self.execute_with_retry(
            setup,
            device_address,
            64, // Default max packet size
            3,
        )?;

        if data.len() != 18 {
            return Err(UsbError::InvalidDescriptor);
        }

        let mut result = [0u8; 18];
        result.copy_from_slice(&data);
        Ok(result)
    }

    /// Helper: Set device address
    pub fn set_address(&mut self, new_address: u8) -> Result<()> {
        let setup = SetupPacket::set_address(new_address);
        self.execute_with_retry(
            setup, 0, // Address 0 for unconfigured device
            64, 3,
        )?;
        Ok(())
    }

    /// Helper: Set configuration
    pub fn set_configuration(&mut self, device_address: u8, config: u8) -> Result<()> {
        let setup = SetupPacket::set_configuration(config);
        self.execute_with_retry(setup, device_address, 64, 3)?;
        Ok(())
    }
}

// Using heapless for no_std compatibility
