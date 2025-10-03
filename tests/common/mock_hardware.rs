//! Mock hardware helpers for testing without real USB devices
//!
//! Provides mock DMA buffers, USB descriptors, and test data builders.

#![no_std]

use core::ptr::NonNull;
use imxrt_usbh::dma::DmaBuffer;

/// Create a mock DMA buffer for testing (using static memory)
///
/// # Safety
/// Uses static mutable memory, safe for single-threaded tests
pub fn create_mock_buffer(size: usize, index: usize) -> DmaBuffer {
    static mut TEST_BUFFERS: [[u8; 1024]; 8] = [[0; 1024]; 8];

    assert!(index < 8, "Test buffer index out of range");
    assert!(size <= 1024, "Test buffer size too large");

    unsafe {
        let ptr = NonNull::new_unchecked(TEST_BUFFERS[index].as_mut_ptr());
        DmaBuffer::new(ptr, size, index)
    }
}

/// Create a standard USB 2.0 device descriptor for testing
pub fn create_test_device_descriptor() -> [u8; 18] {
    [
        0x12,       // bLength
        0x01,       // bDescriptorType (DEVICE)
        0x00, 0x02, // bcdUSB (2.0)
        0x00,       // bDeviceClass (defined at interface level)
        0x00,       // bDeviceSubClass
        0x00,       // bDeviceProtocol
        0x40,       // bMaxPacketSize0 (64 bytes)
        0x83, 0x04, // idVendor (0x0483 - STMicroelectronics)
        0x40, 0x00, // idProduct (0x0040)
        0x00, 0x01, // bcdDevice (1.0)
        0x01,       // iManufacturer
        0x02,       // iProduct
        0x03,       // iSerialNumber
        0x01,       // bNumConfigurations
    ]
}

/// Create a standard USB configuration descriptor for testing
pub fn create_test_config_descriptor() -> [u8; 9] {
    [
        0x09,       // bLength
        0x02,       // bDescriptorType (CONFIGURATION)
        0x09, 0x00, // wTotalLength (9 bytes - just header for testing)
        0x01,       // bNumInterfaces
        0x01,       // bConfigurationValue
        0x00,       // iConfiguration
        0x80,       // bmAttributes (bus powered)
        0x32,       // bMaxPower (100mA)
    ]
}

/// Create a MIDI device descriptor (Audio class) for testing
pub fn create_midi_device_descriptor() -> [u8; 18] {
    [
        0x12,       // bLength
        0x01,       // bDescriptorType (DEVICE)
        0x00, 0x02, // bcdUSB (2.0)
        0x01,       // bDeviceClass (Audio)
        0x00,       // bDeviceSubClass
        0x00,       // bDeviceProtocol
        0x40,       // bMaxPacketSize0 (64 bytes)
        0x82, 0x08, // idVendor (0x0882 - Roland)
        0x01, 0x00, // idProduct
        0x00, 0x01, // bcdDevice (1.0)
        0x01,       // iManufacturer
        0x02,       // iProduct
        0x03,       // iSerialNumber
        0x01,       // bNumConfigurations
    ]
}

/// Create an HID keyboard device descriptor for testing
pub fn create_hid_device_descriptor() -> [u8; 18] {
    [
        0x12,       // bLength
        0x01,       // bDescriptorType (DEVICE)
        0x00, 0x02, // bcdUSB (2.0)
        0x00,       // bDeviceClass (defined at interface)
        0x00,       // bDeviceSubClass
        0x00,       // bDeviceProtocol
        0x08,       // bMaxPacketSize0 (8 bytes for low-speed)
        0x6D, 0x04, // idVendor (0x046D - Logitech)
        0x16, 0xC0, // idProduct (0xC016 - Keyboard)
        0x10, 0x00, // bcdDevice (0.16)
        0x01,       // iManufacturer
        0x02,       // iProduct
        0x00,       // iSerialNumber
        0x01,       // bNumConfigurations
    ]
}

/// Assert that a buffer address is properly aligned for DMA (32-byte cache line)
#[inline]
pub fn assert_buffer_aligned(addr: usize) {
    assert_eq!(
        addr & 0x1F,
        0,
        "Buffer at {:#x} is not 32-byte aligned",
        addr
    );
}

/// Assert that two buffer ranges do not overlap
#[inline]
pub fn assert_no_overlap(addr1: usize, size1: usize, addr2: usize, size2: usize) {
    let end1 = addr1 + size1;
    let end2 = addr2 + size2;

    let no_overlap = (end1 <= addr2) || (end2 <= addr1);

    assert!(
        no_overlap,
        "Buffers overlap: [{:#x}..{:#x}] and [{:#x}..{:#x}]",
        addr1, end1, addr2, end2
    );
}

/// Helper to create a test transfer state for verification
pub struct TestTransferState {
    pub device_address: u8,
    pub endpoint: u8,
    pub max_packet_size: u16,
    pub timeout_ms: u32,
}

impl TestTransferState {
    pub fn new() -> Self {
        Self {
            device_address: 1,
            endpoint: 0x81,
            max_packet_size: 64,
            timeout_ms: 1000,
        }
    }

    pub fn with_address(mut self, addr: u8) -> Self {
        self.device_address = addr;
        self
    }

    pub fn with_endpoint(mut self, ep: u8) -> Self {
        self.endpoint = ep;
        self
    }

    pub fn with_max_packet_size(mut self, size: u16) -> Self {
        self.max_packet_size = size;
        self
    }
}
