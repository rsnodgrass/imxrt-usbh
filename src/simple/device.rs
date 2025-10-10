//! USB Device abstraction

use crate::enumeration::{DeviceClass, EnumeratedDevice};
use crate::error::Result;
use crate::transfer::Direction;
use crate::transfer::simple_control::{ControlExecutor, SetupPacket};
use crate::dma::UsbMemoryPool;
use crate::ehci::TransferExecutor;
use heapless::Vec;

/// Maximum number of endpoints per device
const MAX_ENDPOINTS: usize = 16;

/// Represents an enumerated USB device
///
/// This struct encapsulates all information about a connected USB device,
/// including its descriptors, endpoints, and configuration.
#[derive(Debug)]
pub struct UsbDevice {
    /// Device address (1-127)
    address: u8,
    /// Device class code
    class: DeviceClass,
    /// Vendor ID
    vendor_id: u16,
    /// Product ID
    product_id: u16,
    /// Device release number
    device_release: u16,
    /// USB specification release
    usb_release: u16,
    /// Maximum packet size for endpoint 0
    max_packet_ep0: u8,
    /// Configuration value
    configuration_value: u8,
    /// Endpoint information
    endpoints: Vec<EndpointInfo, MAX_ENDPOINTS>,
}

/// Endpoint information
#[derive(Debug, Clone, Copy)]
pub struct EndpointInfo {
    /// Endpoint address (includes direction bit)
    pub address: u8,
    /// Endpoint attributes (transfer type)
    pub attributes: u8,
    /// Maximum packet size
    pub max_packet_size: u16,
    /// Polling interval (for interrupt endpoints)
    pub interval: u8,
    /// Interface number this endpoint belongs to
    pub interface_num: u8,
}

impl EndpointInfo {
    /// Get endpoint number (0-15)
    pub fn number(&self) -> u8 {
        self.address & 0x0F
    }

    /// Get transfer direction
    pub fn direction(&self) -> Direction {
        if self.address & 0x80 != 0 {
            Direction::In
        } else {
            Direction::Out
        }
    }

    /// Check if this is an IN endpoint
    pub fn is_in(&self) -> bool {
        self.address & 0x80 != 0
    }

    /// Check if this is an OUT endpoint
    pub fn is_out(&self) -> bool {
        self.address & 0x80 == 0
    }

    /// Get transfer type
    pub fn transfer_type(&self) -> TransferType {
        match self.attributes & 0x03 {
            0 => TransferType::Control,
            1 => TransferType::Isochronous,
            2 => TransferType::Bulk,
            3 => TransferType::Interrupt,
            _ => unreachable!(),
        }
    }
}

/// USB transfer types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferType {
    /// Control transfer
    Control,
    /// Isochronous transfer
    Isochronous,
    /// Bulk transfer
    Bulk,
    /// Interrupt transfer
    Interrupt,
}

impl UsbDevice {
    /// Create device from enumerated device
    ///
    /// Parses full configuration descriptor to extract endpoint information.
    pub(crate) fn from_enumerated(
        enum_dev: EnumeratedDevice,
        memory_pool: &mut UsbMemoryPool,
        transfer_executor: &mut TransferExecutor,
    ) -> Result<Self> {
        let mut endpoints = Vec::new();

        // Get full configuration descriptor with all interfaces/endpoints
        let setup = SetupPacket::get_descriptor(0x02, 0, 255);
        let mut executor = ControlExecutor::new(transfer_executor, memory_pool);
        let data = executor.execute_with_retry(
            setup,
            enum_dev.address,
            enum_dev.max_packet_size,
            3,
        )?;

        // Parse descriptors
        let mut offset = 9; // Skip configuration descriptor header
        let mut current_interface = 0;

        while offset + 2 <= data.len() {
            let desc_len = data[offset] as usize;
            let desc_type = data[offset + 1];

            if offset + desc_len > data.len() {
                break;
            }

            match desc_type {
                0x04 => {
                    // Interface descriptor
                    if desc_len >= 9 {
                        current_interface = data[offset + 2];
                    }
                }
                0x05 => {
                    // Endpoint descriptor
                    if desc_len >= 7 {
                        let _ = endpoints.push(EndpointInfo {
                            address: data[offset + 2],
                            attributes: data[offset + 3],
                            max_packet_size: u16::from_le_bytes([
                                data[offset + 4],
                                data[offset + 5],
                            ]),
                            interval: data[offset + 6],
                            interface_num: current_interface,
                        });
                    }
                }
                _ => {}
            }

            offset += desc_len;
        }

        Ok(Self {
            address: enum_dev.address,
            class: enum_dev.class,
            vendor_id: enum_dev.device_desc.id_vendor,
            product_id: enum_dev.device_desc.id_product,
            device_release: enum_dev.device_desc.bcd_device,
            usb_release: enum_dev.device_desc.bcd_usb,
            max_packet_ep0: enum_dev.device_desc.b_max_packet_size0,
            configuration_value: enum_dev.config_value,
            endpoints,
        })
    }

    /// Get device address
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Get device class
    pub fn class(&self) -> DeviceClass {
        self.class
    }

    /// Get vendor ID
    pub fn vendor_id(&self) -> u16 {
        self.vendor_id
    }

    /// Get product ID
    pub fn product_id(&self) -> u16 {
        self.product_id
    }

    /// Get device release number
    pub fn device_release(&self) -> u16 {
        self.device_release
    }

    /// Get USB specification release
    pub fn usb_release(&self) -> u16 {
        self.usb_release
    }

    /// Get maximum packet size for endpoint 0
    pub fn max_packet_ep0(&self) -> u8 {
        self.max_packet_ep0
    }

    /// Get configuration value
    pub fn configuration_value(&self) -> u8 {
        self.configuration_value
    }

    /// Get all endpoints
    pub fn endpoints(&self) -> &[EndpointInfo] {
        &self.endpoints
    }

    /// Find endpoint by address
    pub fn find_endpoint_by_address(&self, address: u8) -> Option<&EndpointInfo> {
        self.endpoints.iter().find(|ep| ep.address == address)
    }

    /// Find first endpoint matching criteria
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use imxrt_usbh::simple::{UsbDevice, TransferType};
    /// # use imxrt_usbh::transfer::Direction;
    /// # fn example(device: &UsbDevice) {
    /// // Find first interrupt IN endpoint
    /// if let Some(ep) = device.find_endpoint(Direction::In, TransferType::Interrupt) {
    ///     println!("Interrupt IN endpoint: 0x{:02X}", ep.address);
    /// }
    /// # }
    /// ```
    pub fn find_endpoint(&self, direction: Direction, transfer_type: TransferType) -> Option<&EndpointInfo> {
        self.endpoints.iter().find(|ep| {
            let dir_match = match direction {
                Direction::In => ep.is_in(),
                Direction::Out => ep.is_out(),
            };
            dir_match && ep.transfer_type() == transfer_type
        })
    }

    /// Find all endpoints matching criteria
    pub fn find_endpoints(&self, direction: Direction, transfer_type: TransferType) -> impl Iterator<Item = &EndpointInfo> {
        self.endpoints.iter().filter(move |ep| {
            let dir_match = match direction {
                Direction::In => ep.is_in(),
                Direction::Out => ep.is_out(),
            };
            dir_match && ep.transfer_type() == transfer_type
        })
    }

    /// Check if this is an HID device
    pub fn is_hid(&self) -> bool {
        self.class == DeviceClass::Hid
    }

    /// Check if this is a mass storage device
    pub fn is_mass_storage(&self) -> bool {
        self.class == DeviceClass::MassStorage
    }

    /// Check if this is an audio device
    pub fn is_audio(&self) -> bool {
        self.class == DeviceClass::Audio
    }

    /// Check if this is a CDC device
    pub fn is_cdc(&self) -> bool {
        self.class == DeviceClass::Cdc
    }

    /// Check if this is a hub
    pub fn is_hub(&self) -> bool {
        self.class == DeviceClass::Hub
    }
}
