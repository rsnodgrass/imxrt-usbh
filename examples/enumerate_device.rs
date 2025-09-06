//! USB Device Enumeration Example
//!
//! This example demonstrates how to enumerate a USB device using the imxrt-usbh library.
//! It shows the complete enumeration sequence from port reset to configuration.

#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

use imxrt_usbh::{UsbHost, Result};
use imxrt_usbh::ehci::Ehci;
use imxrt_usbh::enumeration::{DeviceEnumerator, DeviceDescriptor, ConfigurationDescriptor};
use imxrt_usbh::transfer::{ControlTransfer, SetupPacket, Direction};

/// USB Standard Request Codes
mod usb_requests {
    pub const GET_STATUS: u8 = 0x00;
    pub const CLEAR_FEATURE: u8 = 0x01;
    pub const SET_FEATURE: u8 = 0x03;
    pub const SET_ADDRESS: u8 = 0x05;
    pub const GET_DESCRIPTOR: u8 = 0x06;
    pub const SET_DESCRIPTOR: u8 = 0x07;
    pub const GET_CONFIGURATION: u8 = 0x08;
    pub const SET_CONFIGURATION: u8 = 0x09;
}

/// USB Descriptor Types
mod descriptor_types {
    pub const DEVICE: u16 = 0x0100;
    pub const CONFIGURATION: u16 = 0x0200;
    pub const STRING: u16 = 0x0300;
    pub const INTERFACE: u16 = 0x0400;
    pub const ENDPOINT: u16 = 0x0500;
}

/// Example device information structure
struct UsbDeviceInfo {
    address: u8,
    vendor_id: u16,
    product_id: u16,
    device_class: u8,
    num_configurations: u8,
    manufacturer_string: Option<heapless::String<64>>,
    product_string: Option<heapless::String<64>>,
}

#[entry]
fn main() -> ! {
    // Initialize the USB host controller
    let mut usb_host = unsafe { UsbHost::new() }.expect("Failed to initialize USB host");
    
    // Initialize EHCI controller
    let mut ehci = unsafe { Ehci::new() }.expect("Failed to initialize EHCI");
    
    // Enable port power and wait for device connection
    ehci.enable_port_power(0);
    delay_ms(100); // Wait for power to stabilize
    
    // Main enumeration loop
    loop {
        // Check if a device is connected
        if let Some(port_status) = ehci.get_port_status(0) {
            if port_status.is_connected() && !port_status.is_enabled() {
                // New device detected, start enumeration
                match enumerate_device(&mut ehci, 0) {
                    Ok(device_info) => {
                        print_device_info(&device_info);
                    }
                    Err(e) => {
                        // Handle enumeration error
                        handle_error(e);
                    }
                }
            }
        }
        
        delay_ms(100);
    }
}

/// Enumerate a newly connected USB device
fn enumerate_device(ehci: &mut Ehci, port: u8) -> Result<UsbDeviceInfo> {
    // Step 1: Reset the port
    ehci.reset_port(port)?;
    delay_ms(50); // USB 2.0 spec requires 10ms minimum
    
    // Step 2: Get the first 8 bytes of device descriptor
    let mut device_desc_buf = [0u8; 8];
    let setup = SetupPacket::new(
        0x80,  // Device-to-host, standard, device
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::DEVICE,
        0,
        8,
    );
    
    control_transfer(ehci, 0, &setup, Some(&mut device_desc_buf))?;
    
    // Extract max packet size from partial descriptor
    let max_packet_size = device_desc_buf[7];
    
    // Step 3: Reset the port again (some devices need this)
    ehci.reset_port(port)?;
    delay_ms(20);
    
    // Step 4: Set device address
    let device_address = allocate_address();
    let setup = SetupPacket::new(
        0x00,  // Host-to-device, standard, device
        usb_requests::SET_ADDRESS,
        device_address as u16,
        0,
        0,
    );
    
    control_transfer(ehci, 0, &setup, None)?;
    delay_ms(2); // Allow device to process address change
    
    // Step 5: Get full device descriptor using new address
    let mut device_desc_buf = [0u8; 18];
    let setup = SetupPacket::new(
        0x80,
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::DEVICE,
        0,
        18,
    );
    
    control_transfer(ehci, device_address, &setup, Some(&mut device_desc_buf))?;
    
    // Parse device descriptor
    let device_desc = parse_device_descriptor(&device_desc_buf);
    
    // Step 6: Get configuration descriptor (first 9 bytes to get total length)
    let mut config_desc_buf = [0u8; 9];
    let setup = SetupPacket::new(
        0x80,
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::CONFIGURATION | 0, // Configuration index 0
        0,
        9,
    );
    
    control_transfer(ehci, device_address, &setup, Some(&mut config_desc_buf))?;
    
    // Extract total configuration length
    let total_length = u16::from_le_bytes([config_desc_buf[2], config_desc_buf[3]]);
    
    // Step 7: Get full configuration descriptor
    let mut config_desc_buf = heapless::Vec::<u8, 256>::new();
    config_desc_buf.resize(total_length as usize, 0).ok();
    
    let setup = SetupPacket::new(
        0x80,
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::CONFIGURATION | 0,
        0,
        total_length,
    );
    
    control_transfer(ehci, device_address, &setup, Some(config_desc_buf.as_mut_slice()))?;
    
    // Step 8: Set configuration (usually configuration 1)
    let setup = SetupPacket::new(
        0x00,
        usb_requests::SET_CONFIGURATION,
        1, // Configuration value (usually 1)
        0,
        0,
    );
    
    control_transfer(ehci, device_address, &setup, None)?;
    
    // Step 9: Get manufacturer string (optional)
    let manufacturer_string = if device_desc.i_manufacturer > 0 {
        get_string_descriptor(ehci, device_address, device_desc.i_manufacturer)?
    } else {
        None
    };
    
    // Step 10: Get product string (optional)
    let product_string = if device_desc.i_product > 0 {
        get_string_descriptor(ehci, device_address, device_desc.i_product)?
    } else {
        None
    };
    
    Ok(UsbDeviceInfo {
        address: device_address,
        vendor_id: device_desc.id_vendor,
        product_id: device_desc.id_product,
        device_class: device_desc.b_device_class,
        num_configurations: device_desc.b_num_configurations,
        manufacturer_string,
        product_string,
    })
}

/// Perform a control transfer
fn control_transfer(
    ehci: &mut Ehci,
    address: u8,
    setup: &SetupPacket,
    data: Option<&mut [u8]>,
) -> Result<()> {
    // This would use the actual EHCI control transfer implementation
    // For demonstration, we show the structure
    
    // Create control transfer
    let transfer = ControlTransfer::new(address, 0, 64); // Endpoint 0, max packet 64
    
    // Execute transfer with setup packet and optional data
    transfer.execute(setup, data)?;
    
    Ok(())
}

/// Parse device descriptor from raw bytes
fn parse_device_descriptor(data: &[u8]) -> DeviceDescriptor {
    DeviceDescriptor {
        b_length: data[0],
        b_descriptor_type: data[1],
        bcd_usb: u16::from_le_bytes([data[2], data[3]]),
        b_device_class: data[4],
        b_device_sub_class: data[5],
        b_device_protocol: data[6],
        b_max_packet_size0: data[7],
        id_vendor: u16::from_le_bytes([data[8], data[9]]),
        id_product: u16::from_le_bytes([data[10], data[11]]),
        bcd_device: u16::from_le_bytes([data[12], data[13]]),
        i_manufacturer: data[14],
        i_product: data[15],
        i_serial_number: data[16],
        b_num_configurations: data[17],
    }
}

/// Get string descriptor
fn get_string_descriptor(
    ehci: &mut Ehci,
    address: u8,
    index: u8,
) -> Result<Option<heapless::String<64>>> {
    // First get the string descriptor length
    let mut lang_buf = [0u8; 4];
    let setup = SetupPacket::new(
        0x80,
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::STRING | 0, // Language ID descriptor
        0,
        4,
    );
    
    control_transfer(ehci, address, &setup, Some(&mut lang_buf))?;
    
    // Get the actual string descriptor
    let mut string_buf = [0u8; 64];
    let setup = SetupPacket::new(
        0x80,
        usb_requests::GET_DESCRIPTOR,
        descriptor_types::STRING | (index as u16),
        0x0409, // English (US) language ID
        64,
    );
    
    control_transfer(ehci, address, &setup, Some(&mut string_buf))?;
    
    // Parse Unicode string (UTF-16LE)
    let length = string_buf[0] as usize;
    if length < 2 {
        return Ok(None);
    }
    
    let mut result = heapless::String::<64>::new();
    for i in (2..length).step_by(2) {
        if i + 1 < length {
            let ch = u16::from_le_bytes([string_buf[i], string_buf[i + 1]]);
            if ch < 128 {
                result.push(ch as u8 as char).ok();
            }
        }
    }
    
    Ok(Some(result))
}

/// Allocate a new device address
fn allocate_address() -> u8 {
    // In a real implementation, this would track allocated addresses
    // For this example, we use a simple counter
    static mut NEXT_ADDRESS: u8 = 1;
    unsafe {
        let addr = NEXT_ADDRESS;
        NEXT_ADDRESS += 1;
        if NEXT_ADDRESS > 127 {
            NEXT_ADDRESS = 1;
        }
        addr
    }
}

/// Print device information
fn print_device_info(info: &UsbDeviceInfo) {
    // In a real embedded system, this might use RTT or UART
    // For demonstration purposes, we show the structure
    
    defmt::info!("USB Device Enumerated:");
    defmt::info!("  Address: {}", info.address);
    defmt::info!("  VID:PID: {:04x}:{:04x}", info.vendor_id, info.product_id);
    defmt::info!("  Class: {}", info.device_class);
    
    if let Some(ref manufacturer) = info.manufacturer_string {
        defmt::info!("  Manufacturer: {}", manufacturer.as_str());
    }
    
    if let Some(ref product) = info.product_string {
        defmt::info!("  Product: {}", product.as_str());
    }
}

/// Handle enumeration errors
fn handle_error(error: imxrt_usbh::UsbError) {
    defmt::error!("Enumeration failed: {:?}", error);
    
    // In a real system, might retry or report to higher layer
    delay_ms(1000);
}

/// Simple delay function (would use hardware timer in real implementation)
fn delay_ms(ms: u32) {
    cortex_m::asm::delay(600_000 * ms); // Assuming 600MHz clock
}

/// Placeholder for missing types (would come from main library)
mod placeholders {
    #[derive(Debug)]
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
}

use placeholders::DeviceDescriptor;