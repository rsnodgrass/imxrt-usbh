//! USB device enumeration
//! 
//! Handles device detection, enumeration, and configuration.
//! Example focus: MIDI keyboard enumeration

use crate::error::{Result, UsbError};
use crate::transfer::simple_control::{ControlExecutor, SetupPacket};
use crate::dma::memory::UsbMemoryPool;
use crate::ehci::controller::EhciController;

/// USB device class codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DeviceClass {
    /// Audio device (includes MIDI)
    Audio = 0x01,
    /// Communications device
    Cdc = 0x02,
    /// Human interface device
    Hid = 0x03,
    /// Mass storage device
    MassStorage = 0x08,
    /// Hub
    Hub = 0x09,
    /// Vendor specific
    VendorSpecific = 0xFF,
}

impl DeviceClass {
    /// Create from u8 value
    pub fn from_u8(value: u8) -> Self {
        match value {
            0x01 => Self::Audio,
            0x02 => Self::Cdc,
            0x03 => Self::Hid,
            0x08 => Self::MassStorage,
            0x09 => Self::Hub,
            0xFF => Self::VendorSpecific,
            _ => Self::VendorSpecific, // Default to vendor specific for unknown
        }
    }
}

/// USB device descriptor (first 18 bytes)
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct DeviceDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub bcd_usb: u16,
    pub b_device_class: u8,
    pub b_device_sub_class: u8,
    pub b_device_protocol: u8,
    pub b_max_packet_size0: u8,
    pub id_vendor: u16,
    pub id_product: u16,
    pub bcd_device: u16,
    pub i_manufacturer: u8,
    pub i_product: u8,
    pub i_serial_number: u8,
    pub b_num_configurations: u8,
}

impl DeviceDescriptor {
    /// Parse from raw bytes
    pub fn from_bytes(data: &[u8]) -> Result<Self> {
        if data.len() < 18 {
            return Err(UsbError::InvalidDescriptor);
        }
        
        // Validate descriptor type
        if data[1] != 0x01 {
            return Err(UsbError::InvalidDescriptor);
        }
        
        unsafe {
            Ok(core::ptr::read_unaligned(data.as_ptr() as *const Self))
        }
    }
    
    /// Check if this is a MIDI device
    pub fn is_midi_device(&self) -> bool {
        // Class 0x01 is Audio, subclass 0x03 is MIDI Streaming
        self.b_device_class == 0x01 || 
        // Some MIDI devices report as vendor-specific at device level
        // but have Audio class interfaces
        self.b_device_class == 0x00 || 
        self.b_device_class == 0xFF
    }
    
    /// Get device class
    pub fn device_class(&self) -> DeviceClass {
        DeviceClass::from_u8(self.b_device_class)
    }
}

/// Configuration descriptor header
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct ConfigurationDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub w_total_length: u16,
    pub b_num_interfaces: u8,
    pub b_configuration_value: u8,
    pub i_configuration: u8,
    pub bm_attributes: u8,
    pub b_max_power: u8,
}

/// Interface descriptor
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct InterfaceDescriptor {
    pub b_length: u8,
    pub b_descriptor_type: u8,
    pub b_interface_number: u8,
    pub b_alternate_setting: u8,
    pub b_num_endpoints: u8,
    pub b_interface_class: u8,
    pub b_interface_sub_class: u8,
    pub b_interface_protocol: u8,
    pub i_interface: u8,
}

impl InterfaceDescriptor {
    /// Check if this is a MIDI streaming interface
    pub fn is_midi_streaming(&self) -> bool {
        // Class 0x01 (Audio), SubClass 0x03 (MIDI Streaming)
        self.b_interface_class == 0x01 && self.b_interface_sub_class == 0x03
    }
}

/// Enumerated device information
#[derive(Debug, Clone)]
pub struct EnumeratedDevice {
    /// Device address (1-127)
    pub address: u8,
    /// Device descriptor
    pub device_desc: DeviceDescriptor,
    /// Selected configuration
    pub config_value: u8,
    /// Device class
    pub class: DeviceClass,
    /// Is this a MIDI device?
    pub is_midi: bool,
    /// Maximum packet size for control endpoint
    pub max_packet_size: u16,
}

/// USB device enumerator
pub struct DeviceEnumerator<'a, const N_PORTS: usize, State> 
where
    [(); N_PORTS]: Sized,
{
    controller: &'a mut EhciController<N_PORTS, State>,
    memory_pool: &'a mut UsbMemoryPool,
    next_address: u8,
}

impl<'a, const N_PORTS: usize, State> DeviceEnumerator<'a, N_PORTS, State>
where
    [(); N_PORTS]: Sized,
{
    /// Create new enumerator
    pub fn new(
        controller: &'a mut EhciController<N_PORTS, State>,
        memory_pool: &'a mut UsbMemoryPool,
    ) -> Self {
        Self {
            controller,
            memory_pool,
            next_address: 1, // USB addresses start at 1
        }
    }
    
    /// Enumerate a newly connected device
    pub fn enumerate_device(&mut self) -> Result<EnumeratedDevice> {
        #[cfg(feature = "defmt")]
        defmt::info!("Starting device enumeration");
        
        // Step 1: Get first 8 bytes of device descriptor at address 0
        let partial_desc = self.get_partial_descriptor()?;
        let max_packet_size = partial_desc[7] as u16;
        
        #[cfg(feature = "defmt")]
        defmt::info!("Device max packet size: {}", max_packet_size);
        
        // Step 2: Assign device address
        let device_address = self.assign_address(max_packet_size)?;
        
        // Step 3: Get full device descriptor at new address
        let device_desc = self.get_full_device_descriptor(device_address, max_packet_size)?;
        
        // Check if it's a MIDI device
        let is_midi = device_desc.is_midi_device();
        
        #[cfg(feature = "defmt")]
        if is_midi {
            // Copy values to avoid packed field reference
            let vid = device_desc.id_vendor;
            let pid = device_desc.id_product;
            defmt::info!("MIDI device detected! VID={:#x} PID={:#x}", vid, pid);
        }
        
        // Step 4: Get configuration descriptor
        let config = self.get_configuration(device_address, max_packet_size)?;
        
        // Step 5: Set configuration
        self.set_configuration(device_address, config, max_packet_size)?;
        
        // Step 6: For MIDI devices, look for MIDI streaming interfaces
        if is_midi {
            self.find_midi_interfaces(device_address, max_packet_size)?;
        }
        
        Ok(EnumeratedDevice {
            address: device_address,
            device_desc,
            config_value: config,
            class: device_desc.device_class(),
            is_midi,
            max_packet_size,
        })
    }
    
    /// Get first 8 bytes of device descriptor
    fn get_partial_descriptor(&mut self) -> Result<[u8; 8]> {
        let setup = SetupPacket::get_descriptor(0x01, 0, 8);
        let data = ControlExecutor::execute_with_retry(
            setup,
            0, // Address 0 for unconfigured device
            64, // Default max packet size
            self.memory_pool,
            3,
        )?;
        
        if data.len() < 8 {
            return Err(UsbError::InvalidDescriptor);
        }
        
        let mut result = [0u8; 8];
        result.copy_from_slice(&data[..8]);
        Ok(result)
    }
    
    /// Assign address to device
    fn assign_address(&mut self, _max_packet_size: u16) -> Result<u8> {
        if self.next_address > 127 {
            return Err(UsbError::NoResources);
        }
        
        let address = self.next_address;
        self.next_address += 1;
        
        // Send SET_ADDRESS request
        ControlExecutor::set_address(address, self.memory_pool)?;
        
        // USB spec requires 2ms recovery time after SET_ADDRESS
        self.delay_ms(2);
        
        #[cfg(feature = "defmt")]
        defmt::info!("Assigned address {}", address);
        
        Ok(address)
    }
    
    /// Get full device descriptor
    fn get_full_device_descriptor(
        &mut self,
        address: u8,
        max_packet_size: u16,
    ) -> Result<DeviceDescriptor> {
        let setup = SetupPacket::get_descriptor(0x01, 0, 18);
        let data = ControlExecutor::execute_with_retry(
            setup,
            address,
            max_packet_size,
            self.memory_pool,
            3,
        )?;
        
        DeviceDescriptor::from_bytes(&data)
    }
    
    /// Get configuration value
    fn get_configuration(&mut self, address: u8, max_packet_size: u16) -> Result<u8> {
        // Get configuration descriptor header first
        let setup = SetupPacket::get_descriptor(0x02, 0, 9);
        let data = ControlExecutor::execute_with_retry(
            setup,
            address,
            max_packet_size,
            self.memory_pool,
            3,
        )?;
        
        if data.len() < 9 {
            return Err(UsbError::InvalidDescriptor);
        }
        
        // Return configuration value (typically 1)
        Ok(data[5])
    }
    
    /// Set device configuration
    fn set_configuration(
        &mut self,
        address: u8,
        config: u8,
        _max_packet_size: u16,
    ) -> Result<()> {
        ControlExecutor::set_configuration(address, config, self.memory_pool)?;
        
        #[cfg(feature = "defmt")]
        defmt::info!("Set configuration {}", config);
        
        Ok(())
    }
    
    /// Find MIDI streaming interfaces
    fn find_midi_interfaces(&mut self, address: u8, max_packet_size: u16) -> Result<()> {
        // Get full configuration descriptor
        let setup = SetupPacket::get_descriptor(0x02, 0, 255);
        let data = ControlExecutor::execute_with_retry(
            setup,
            address,
            max_packet_size,
            self.memory_pool,
            3,
        )?;
        
        // Parse interfaces looking for MIDI streaming
        let mut offset = 9; // Skip config descriptor
        while offset + 9 <= data.len() {
            let desc_len = data[offset] as usize;
            let desc_type = data[offset + 1];
            
            if desc_type == 0x04 && desc_len >= 9 { // Interface descriptor
                let interface = unsafe {
                    core::ptr::read_unaligned(
                        data[offset..].as_ptr() as *const InterfaceDescriptor
                    )
                };
                
                if interface.is_midi_streaming() {
                    #[cfg(feature = "defmt")]
                    defmt::info!("Found MIDI streaming interface {}", 
                               interface.b_interface_number);
                }
            }
            
            offset += desc_len;
        }
        
        Ok(())
    }
    
    /// Delay in milliseconds
    fn delay_ms(&self, ms: u32) {
        cortex_m::asm::delay(ms * 600_000); // Assuming 600MHz CPU
    }
}

// Example usage for MIDI keyboard enumeration:
// 
// let mut device = enumerator.enumerate_device()?;
// 
// if device.is_midi {
//     println!("MIDI keyboard connected!");
//     println!("Vendor ID: {:04x}", device.device_desc.id_vendor);
//     println!("Product ID: {:04x}", device.device_desc.id_product);
//     
//     // Now ready to set up MIDI bulk endpoints for data transfer
// }