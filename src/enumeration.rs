//! USB device enumeration
//!
//! Handles device detection, enumeration, and configuration.
//! Example focus: MIDI keyboard enumeration

use crate::dma::memory::UsbMemoryPool;
use crate::ehci::controller::EhciController;
use crate::ehci::TransferExecutor;
use crate::error::{Result, UsbError};
use crate::transfer::simple_control::{ControlExecutor, SetupPacket};

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
///
/// Standard USB 2.0 device descriptor containing device identification
/// and configuration information. See USB 2.0 Specification Section 9.6.1.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct DeviceDescriptor {
    /// Size of this descriptor in bytes (always 18)
    pub b_length: u8,
    /// Descriptor type (always 0x01 for DEVICE)
    pub b_descriptor_type: u8,
    /// USB specification release number in BCD (e.g., 0x0200 for USB 2.0)
    pub bcd_usb: u16,
    /// Class code (assigned by USB-IF). 0x00 means class specified in interface descriptors
    pub b_device_class: u8,
    /// Subclass code (assigned by USB-IF)
    pub b_device_sub_class: u8,
    /// Protocol code (assigned by USB-IF)
    pub b_device_protocol: u8,
    /// Maximum packet size for endpoint 0 (valid values: 8, 16, 32, 64)
    pub b_max_packet_size0: u8,
    /// Vendor ID (assigned by USB-IF)
    pub id_vendor: u16,
    /// Product ID (assigned by manufacturer)
    pub id_product: u16,
    /// Device release number in BCD
    pub bcd_device: u16,
    /// Index of manufacturer string descriptor (0 if no string)
    pub i_manufacturer: u8,
    /// Index of product string descriptor (0 if no string)
    pub i_product: u8,
    /// Index of serial number string descriptor (0 if no string)
    pub i_serial_number: u8,
    /// Number of possible configurations
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

        unsafe { Ok(core::ptr::read_unaligned(data.as_ptr() as *const Self)) }
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
///
/// Describes one possible device configuration. A device may have multiple
/// configurations but only one is active at a time. See USB 2.0 Spec Section 9.6.3.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct ConfigurationDescriptor {
    /// Size of this descriptor in bytes (always 9)
    pub b_length: u8,
    /// Descriptor type (always 0x02 for CONFIGURATION)
    pub b_descriptor_type: u8,
    /// Total length of data returned for this configuration (includes all interface/endpoint descriptors)
    pub w_total_length: u16,
    /// Number of interfaces supported by this configuration
    pub b_num_interfaces: u8,
    /// Value to use as argument to SET_CONFIGURATION to select this configuration
    pub b_configuration_value: u8,
    /// Index of string descriptor describing this configuration (0 if no string)
    pub i_configuration: u8,
    /// Configuration characteristics (bit 7: reserved=1, bit 6: self-powered, bit 5: remote wakeup)
    pub bm_attributes: u8,
    /// Maximum power consumption in 2mA units (e.g., 50 = 100mA)
    pub b_max_power: u8,
}

/// Interface descriptor
///
/// Describes a specific interface within a configuration. Interfaces group
/// related endpoints for a particular function. See USB 2.0 Spec Section 9.6.5.
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct InterfaceDescriptor {
    /// Size of this descriptor in bytes (always 9)
    pub b_length: u8,
    /// Descriptor type (always 0x04 for INTERFACE)
    pub b_descriptor_type: u8,
    /// Number identifying this interface (zero-based)
    pub b_interface_number: u8,
    /// Value used to select alternate setting for this interface
    pub b_alternate_setting: u8,
    /// Number of endpoints used by this interface (excluding endpoint 0)
    pub b_num_endpoints: u8,
    /// Class code (assigned by USB-IF). 0xFF = vendor-specific
    pub b_interface_class: u8,
    /// Subclass code (assigned by USB-IF)
    pub b_interface_sub_class: u8,
    /// Protocol code (assigned by USB-IF)
    pub b_interface_protocol: u8,
    /// Index of string descriptor describing this interface (0 if no string)
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
    transfer_executor: &'a mut TransferExecutor,
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
        transfer_executor: &'a mut TransferExecutor,
    ) -> Self {
        Self {
            controller,
            memory_pool,
            transfer_executor,
            next_address: 1, // USB addresses start at 1
        }
    }

    pub fn controller(&mut self) -> &mut EhciController<N_PORTS, State> {
        self.controller
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
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        let data = executor.execute_with_retry(
            setup, 0,  // Address 0 for unconfigured device
            64, // Default max packet size
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
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        executor.set_address(address)?;

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
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        let data = executor.execute_with_retry(setup, address, max_packet_size, 3)?;

        DeviceDescriptor::from_bytes(&data)
    }

    /// Get configuration value
    fn get_configuration(&mut self, address: u8, max_packet_size: u16) -> Result<u8> {
        // Get configuration descriptor header first
        let setup = SetupPacket::get_descriptor(0x02, 0, 9);
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        let data = executor.execute_with_retry(setup, address, max_packet_size, 3)?;

        if data.len() < 9 {
            return Err(UsbError::InvalidDescriptor);
        }

        // Return configuration value (typically 1)
        Ok(data[5])
    }

    /// Set device configuration
    fn set_configuration(&mut self, address: u8, config: u8, _max_packet_size: u16) -> Result<()> {
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        executor.set_configuration(address, config)?;

        #[cfg(feature = "defmt")]
        defmt::info!("Set configuration {}", config);

        Ok(())
    }

    /// Find MIDI streaming interfaces
    fn find_midi_interfaces(&mut self, address: u8, max_packet_size: u16) -> Result<()> {
        // Get full configuration descriptor
        let setup = SetupPacket::get_descriptor(0x02, 0, 255);
        let mut executor = ControlExecutor::new(self.transfer_executor, self.memory_pool);
        let data = executor.execute_with_retry(setup, address, max_packet_size, 3)?;

        // Parse interfaces looking for MIDI streaming
        let mut offset = 9; // Skip config descriptor
        while offset + 9 <= data.len() {
            let desc_len = data[offset] as usize;
            let desc_type = data[offset + 1];

            if desc_type == 0x04 && desc_len >= 9 {
                // Interface descriptor
                let interface = unsafe {
                    core::ptr::read_unaligned(data[offset..].as_ptr() as *const InterfaceDescriptor)
                };

                if interface.is_midi_streaming() {
                    #[cfg(feature = "defmt")]
                    defmt::info!(
                        "Found MIDI streaming interface {}",
                        interface.b_interface_number
                    );
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_valid_device_descriptor() {
        // valid USB 2.0 device descriptor (18 bytes)
        let desc_bytes = [
            0x12, // bLength
            0x01, // bDescriptorType (DEVICE)
            0x00, 0x02, // bcdUSB (2.0)
            0x00, // bDeviceClass (defined at interface)
            0x00, // bDeviceSubClass
            0x00, // bDeviceProtocol
            0x40, // bMaxPacketSize0 (64)
            0x83, 0x04, // idVendor (0x0483)
            0x40, 0x00, // idProduct (0x0040)
            0x00, 0x01, // bcdDevice (1.0)
            0x01, // iManufacturer
            0x02, // iProduct
            0x03, // iSerialNumber
            0x01, // bNumConfigurations
        ];

        let result = DeviceDescriptor::from_bytes(&desc_bytes);
        assert!(result.is_ok());

        let desc = result.unwrap();
        assert_eq!(desc.b_length, 18);
        assert_eq!(desc.b_descriptor_type, 1);
        assert_eq!(desc.bcd_usb, 0x0200);
        assert_eq!(desc.b_device_class, 0);
        assert_eq!(desc.b_max_packet_size0, 64);
        assert_eq!(desc.id_vendor, 0x0483);
        assert_eq!(desc.id_product, 0x0040);
        assert_eq!(desc.b_num_configurations, 1);
    }

    #[test]
    fn test_parse_device_descriptor_too_short() {
        // only 10 bytes instead of required 18
        let short_bytes = [0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x83, 0x04];

        let result = DeviceDescriptor::from_bytes(&short_bytes);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::InvalidDescriptor)));
    }

    #[test]
    fn test_parse_device_descriptor_wrong_type() {
        // descriptor type is 0x02 (config) instead of 0x01 (device)
        let wrong_type_bytes = [
            0x12, // bLength
            0x02, // bDescriptorType (WRONG - should be 0x01)
            0x00, 0x02, // bcdUSB
            0x00, 0x00, 0x00, 0x40, 0x83, 0x04, 0x40, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
        ];

        let result = DeviceDescriptor::from_bytes(&wrong_type_bytes);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::InvalidDescriptor)));
    }

    #[test]
    fn test_device_class_identification() {
        // audio device
        let mut desc_bytes = [0u8; 18];
        desc_bytes[0] = 18;
        desc_bytes[1] = 1;
        desc_bytes[4] = 0x01; // Audio class
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.device_class(), DeviceClass::Audio);

        // HID device
        desc_bytes[4] = 0x03;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.device_class(), DeviceClass::Hid);

        // Mass Storage
        desc_bytes[4] = 0x08;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.device_class(), DeviceClass::MassStorage);

        // Hub
        desc_bytes[4] = 0x09;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.device_class(), DeviceClass::Hub);

        // Vendor Specific
        desc_bytes[4] = 0xFF;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.device_class(), DeviceClass::VendorSpecific);
    }

    #[test]
    fn test_midi_device_detection() {
        let mut desc_bytes = [0u8; 18];
        desc_bytes[0] = 18;
        desc_bytes[1] = 1;

        // class 0x01 (Audio) should be detected as potential MIDI
        desc_bytes[4] = 0x01;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert!(desc.is_midi_device());

        // class 0x00 (interface-defined) should be detected as potential MIDI
        desc_bytes[4] = 0x00;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert!(desc.is_midi_device());

        // class 0xFF (vendor-specific) should be detected as potential MIDI
        desc_bytes[4] = 0xFF;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert!(desc.is_midi_device());

        // class 0x03 (HID) should NOT be MIDI
        desc_bytes[4] = 0x03;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert!(!desc.is_midi_device());
    }

    #[test]
    fn test_device_class_from_u8() {
        assert_eq!(DeviceClass::from_u8(0x01), DeviceClass::Audio);
        assert_eq!(DeviceClass::from_u8(0x02), DeviceClass::Cdc);
        assert_eq!(DeviceClass::from_u8(0x03), DeviceClass::Hid);
        assert_eq!(DeviceClass::from_u8(0x08), DeviceClass::MassStorage);
        assert_eq!(DeviceClass::from_u8(0x09), DeviceClass::Hub);
        assert_eq!(DeviceClass::from_u8(0xFF), DeviceClass::VendorSpecific);

        // unknown class defaults to vendor specific
        assert_eq!(DeviceClass::from_u8(0x99), DeviceClass::VendorSpecific);
    }

    #[test]
    fn test_interface_descriptor_midi_streaming() {
        // create interface descriptor with MIDI streaming class
        #[repr(C, packed)]
        struct TestInterfaceDesc {
            b_length: u8,
            b_descriptor_type: u8,
            b_interface_number: u8,
            b_alternate_setting: u8,
            b_num_endpoints: u8,
            b_interface_class: u8,
            b_interface_sub_class: u8,
            b_interface_protocol: u8,
            i_interface: u8,
        }

        // MIDI streaming interface: class 0x01, subclass 0x03
        let midi_interface = TestInterfaceDesc {
            b_length: 9,
            b_descriptor_type: 0x04, // Interface
            b_interface_number: 1,
            b_alternate_setting: 0,
            b_num_endpoints: 2,
            b_interface_class: 0x01,     // Audio
            b_interface_sub_class: 0x03, // MIDI Streaming
            b_interface_protocol: 0,
            i_interface: 0,
        };

        let desc_bytes =
            unsafe { core::slice::from_raw_parts(&midi_interface as *const _ as *const u8, 9) };

        let interface =
            unsafe { core::ptr::read_unaligned(desc_bytes.as_ptr() as *const InterfaceDescriptor) };

        assert!(interface.is_midi_streaming());

        // non-MIDI interface
        let hid_interface = TestInterfaceDesc {
            b_length: 9,
            b_descriptor_type: 0x04,
            b_interface_number: 0,
            b_alternate_setting: 0,
            b_num_endpoints: 1,
            b_interface_class: 0x03,     // HID
            b_interface_sub_class: 0x01, // Boot
            b_interface_protocol: 0x01,  // Keyboard
            i_interface: 0,
        };

        let desc_bytes =
            unsafe { core::slice::from_raw_parts(&hid_interface as *const _ as *const u8, 9) };

        let interface =
            unsafe { core::ptr::read_unaligned(desc_bytes.as_ptr() as *const InterfaceDescriptor) };

        assert!(!interface.is_midi_streaming());
    }

    #[test]
    fn test_parse_malformed_descriptor_empty() {
        let empty: [u8; 0] = [];
        let result = DeviceDescriptor::from_bytes(&empty);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::InvalidDescriptor)));
    }

    #[test]
    fn test_parse_malformed_descriptor_garbage() {
        // random garbage data
        let garbage = [
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF,
        ];

        let result = DeviceDescriptor::from_bytes(&garbage);
        // descriptor type is 0xFF (not 0x01), should fail
        assert!(result.is_err());
    }

    #[test]
    fn test_device_descriptor_usb_versions() {
        let mut desc_bytes = [0u8; 18];
        desc_bytes[0] = 18;
        desc_bytes[1] = 1;

        // USB 1.0
        desc_bytes[2] = 0x00;
        desc_bytes[3] = 0x01;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.bcd_usb, 0x0100);

        // USB 1.1
        desc_bytes[2] = 0x10;
        desc_bytes[3] = 0x01;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.bcd_usb, 0x0110);

        // USB 2.0
        desc_bytes[2] = 0x00;
        desc_bytes[3] = 0x02;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.bcd_usb, 0x0200);

        // USB 3.0
        desc_bytes[2] = 0x00;
        desc_bytes[3] = 0x03;
        let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
        assert_eq!(desc.bcd_usb, 0x0300);
    }

    #[test]
    fn test_device_descriptor_max_packet_sizes() {
        let mut desc_bytes = [0u8; 18];
        desc_bytes[0] = 18;
        desc_bytes[1] = 1;

        // valid max packet sizes for USB 2.0 (8, 16, 32, 64)
        for size in [8u8, 16, 32, 64] {
            desc_bytes[7] = size;
            let desc = DeviceDescriptor::from_bytes(&desc_bytes).unwrap();
            assert_eq!(desc.b_max_packet_size0, size);
        }
    }
}
