//! Simplified USB Control transfer implementation
//! 
//! Streamlined control transfer without excessive state management

use crate::error::{Result, UsbError};
use crate::dma::{UsbMemoryPool, MemoryDmaBuffer};

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
    data_buffer: Option<MemoryDmaBuffer>,
    device_address: u8,
    endpoint: u8,
    max_packet_size: u16,
    completed: bool,
}

impl SimpleControlTransfer {
    /// Create new control transfer
    pub fn new(
        setup: SetupPacket,
        device_address: u8,
        max_packet_size: u16,
    ) -> Self {
        Self {
            setup,
            data_buffer: None,
            device_address,
            endpoint: 0,
            max_packet_size,
            completed: false,
        }
    }
    
    /// Execute control transfer synchronously
    pub fn execute(&mut self, memory_pool: &mut UsbMemoryPool) -> Result<usize> {
        // Allocate data buffer if needed
        if self.setup.wLength > 0 {
            let buffer = memory_pool.alloc_buffer(self.setup.wLength as usize)
                .ok_or(UsbError::NoResources)?;
            self.data_buffer = Some(buffer);
        }
        
        // Execute SETUP stage
        self.do_setup()?;
        
        // Execute DATA stage if present
        let bytes_transferred = if self.setup.wLength > 0 {
            if self.setup.bmRequestType & 0x80 != 0 {
                // IN transfer (device to host)
                self.do_data_in()?
            } else {
                // OUT transfer (host to device)
                self.do_data_out()?
            }
        } else {
            0
        };
        
        // Execute STATUS stage
        self.do_status()?;
        
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
    
    /// Execute SETUP stage
    fn do_setup(&mut self) -> Result<()> {
        // Queue SETUP packet with DATA0
        // Write setup packet to controller
        // This would interact with actual hardware
        let setup_addr = &self.setup as *const SetupPacket as u32;
        
        // Simplified: just validate the setup packet
        if setup_addr == 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        Ok(())
    }
    
    /// Execute DATA IN stage
    fn do_data_in(&mut self) -> Result<usize> {
        let buffer = self.data_buffer.as_mut()
            .ok_or(UsbError::InvalidState)?;
        
        // Prepare buffer for DMA
        buffer.prepare_for_cpu();
        
        let mut bytes_received = 0;
        let total_bytes = self.setup.wLength as usize;
        
        // Simple transfer loop
        while bytes_received < total_bytes {
            let chunk_size = (total_bytes - bytes_received)
                .min(self.max_packet_size as usize);
            
            // Queue IN transfer
            // Hardware interaction would go here
            
            bytes_received += chunk_size;
            
            // Short packet ends transfer
            if chunk_size < self.max_packet_size as usize {
                break;
            }
        }
        
        Ok(bytes_received)
    }
    
    /// Execute DATA OUT stage
    fn do_data_out(&mut self) -> Result<usize> {
        let buffer = self.data_buffer.as_ref()
            .ok_or(UsbError::InvalidState)?;
        
        // Prepare buffer for DMA
        buffer.prepare_for_device();
        
        let mut bytes_sent = 0;
        let total_bytes = self.setup.wLength as usize;
        
        // Simple transfer loop
        while bytes_sent < total_bytes {
            let chunk_size = (total_bytes - bytes_sent)
                .min(self.max_packet_size as usize);
            
            // Queue OUT transfer
            // Hardware interaction would go here
            
            bytes_sent += chunk_size;
        }
        
        Ok(bytes_sent)
    }
    
    /// Execute STATUS stage
    fn do_status(&mut self) -> Result<()> {
        // STATUS is always DATA1, opposite direction of data stage
        // Zero-length packet
        
        // Hardware interaction would go here
        Ok(())
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

/// Simple control transfer executor
pub struct ControlExecutor;

impl ControlExecutor {
    /// Execute control transfer with retry
    pub fn execute_with_retry(
        setup: SetupPacket,
        device_address: u8,
        max_packet_size: u16,
        memory_pool: &mut UsbMemoryPool,
        max_retries: u8,
    ) -> Result<heapless::Vec<u8, 256>> {
        for attempt in 0..=max_retries {
            let mut transfer = SimpleControlTransfer::new(
                setup,
                device_address,
                max_packet_size,
            );
            
            match transfer.execute(memory_pool) {
                Ok(_) => {
                    // Copy data before returning
                    if let Some(data) = transfer.get_data() {
                        let mut result = heapless::Vec::new();
                        result.extend_from_slice(data).ok();
                        return Ok(result);
                    } else {
                        return Ok(heapless::Vec::new());
                    }
                }
                Err(e) if attempt < max_retries => {
                    // Retry on transient errors
                    match e {
                        UsbError::Timeout | 
                        UsbError::TransactionError |
                        UsbError::Nak => continue,
                        _ => return Err(e),
                    }
                }
                Err(e) => return Err(e),
            }
        }
        
        Err(UsbError::Timeout)
    }
    
    /// Helper: Get device descriptor
    pub fn get_device_descriptor(
        device_address: u8,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<[u8; 18]> {
        let setup = SetupPacket::get_descriptor(0x01, 0, 18);
        let data = Self::execute_with_retry(
            setup,
            device_address,
            64, // Default max packet size
            memory_pool,
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
    pub fn set_address(
        new_address: u8,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<()> {
        let setup = SetupPacket::set_address(new_address);
        Self::execute_with_retry(
            setup,
            0, // Address 0 for unconfigured device
            64,
            memory_pool,
            3,
        )?;
        Ok(())
    }
    
    /// Helper: Set configuration
    pub fn set_configuration(
        device_address: u8,
        config: u8,
        memory_pool: &mut UsbMemoryPool,
    ) -> Result<()> {
        let setup = SetupPacket::set_configuration(config);
        Self::execute_with_retry(
            setup,
            device_address,
            64,
            memory_pool,
            3,
        )?;
        Ok(())
    }
}

// Using heapless for no_std compatibility