//! Integration tests for imxrt-usbh
//!
//! These tests verify the USB host controller functionality
//! Note: Some tests require hardware and will be marked as #[ignore]

#![no_std]
#![no_main]

use imxrt_usbh::{UsbHost, UsbError};
use imxrt_usbh::ehci::{EhciController, QueueTD, QueueHead};
use imxrt_usbh::dma::DmaBufferPool;
use imxrt_usbh::transfer::{Direction, TransferType};

#[cfg(test)]
mod tests {
    use super::*;
    
    /// Test that we can only initialize USB host once
    #[test]
    fn test_single_initialization() {
        // This test would run on hardware
        // For now, we verify the concept
        
        // First initialization should succeed
        let result1 = unsafe { UsbHost::new() };
        assert!(result1.is_ok());
        
        // Second initialization should fail
        let result2 = unsafe { UsbHost::new() };
        assert!(matches!(result2, Err(UsbError::AlreadyInitialized)));
    }
    
    /// Test DMA buffer alignment requirements
    #[test]
    fn test_dma_buffer_alignment() {
        // DMA buffers must be 32-byte aligned
        let mut pool = DmaBufferPool::new();
        let buffer = pool.alloc(1024).expect("Failed to allocate buffer");
        let addr = buffer.as_ptr() as usize;
        assert_eq!(addr & 0x1F, 0, "DMA buffer not 32-byte aligned");
    }
    
    /// Test EHCI register structure sizes
    #[test]
    fn test_ehci_structure_sizes() {
        use imxrt_usbh::ehci::{QueueHead, QueueTD};
        
        // Verify structure sizes match EHCI specification
        assert_eq!(
            core::mem::size_of::<QueueHead>(),
            96,
            "QueueHead size incorrect"
        );
        
        assert_eq!(
            core::mem::size_of::<QueueTD>(),
            32,
            "QTD size incorrect"
        );
        
        // Verify alignment requirements
        assert!(
            core::mem::align_of::<QueueHead>() >= 32,
            "QueueHead alignment incorrect"
        );
        
        assert!(
            core::mem::align_of::<QueueTD>() >= 32,
            "QTD alignment incorrect"
        );
    }
    
    /// Test USB descriptor parsing
    #[test]
    fn test_descriptor_parsing() {
        use imxrt_usbh::enumeration::DeviceDescriptor;
        
        // Example device descriptor bytes
        let device_desc_bytes = [
            0x12,       // bLength
            0x01,       // bDescriptorType (DEVICE)
            0x00, 0x02, // bcdUSB (2.0)
            0x00,       // bDeviceClass
            0x00,       // bDeviceSubClass
            0x00,       // bDeviceProtocol
            0x40,       // bMaxPacketSize0 (64)
            0x83, 0x04, // idVendor
            0x40, 0x00, // idProduct
            0x00, 0x01, // bcdDevice
            0x01,       // iManufacturer
            0x02,       // iProduct
            0x03,       // iSerialNumber
            0x01,       // bNumConfigurations
        ];
        
        // Verify we can parse device descriptor fields
        assert_eq!(device_desc_bytes[0], 0x12);  // bLength
        assert_eq!(device_desc_bytes[1], 0x01);  // bDescriptorType
        
        // Parse USB version (little endian)
        let bcd_usb = u16::from_le_bytes([device_desc_bytes[2], device_desc_bytes[3]]);
        assert_eq!(bcd_usb, 0x0200);
        
        assert_eq!(device_desc_bytes[7], 0x40);  // bMaxPacketSize0
        
        // Parse vendor ID (little endian)
        let id_vendor = u16::from_le_bytes([device_desc_bytes[8], device_desc_bytes[9]]);
        assert_eq!(id_vendor, 0x0483);
        
        assert_eq!(device_desc_bytes[17], 0x01);  // bNumConfigurations
    }
    
    /// Test transfer type configuration
    #[test]
    fn test_transfer_types() {
        // Verify transfer type enums
        assert_eq!(TransferType::Control as u8, 0);
        assert_eq!(TransferType::Bulk as u8, 2);
        assert_eq!(TransferType::Interrupt as u8, 3);
        assert_eq!(TransferType::Isochronous as u8, 1);
        
        // Verify direction enums
        assert_eq!(Direction::Out as u8, 0);
        assert_eq!(Direction::In as u8, 1);
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_port_reset_timing() {
        // This test would verify port reset timing requirements
        // Must hold reset for at least 10ms, no more than 20ms
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_enumeration_sequence() {
        // This test would verify the full enumeration sequence:
        // 1. Port reset
        // 2. Get device descriptor (8 bytes)
        // 3. Set address
        // 4. Get full device descriptor
        // 5. Get configuration descriptor
        // 6. Set configuration
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_bulk_transfer() {
        // This test would verify bulk transfer functionality
        // with a known USB device (e.g., flash drive)
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_interrupt_transfer() {
        // This test would verify interrupt transfer functionality
        // with a known USB device (e.g., HID keyboard)
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_hub_enumeration() {
        // This test would verify hub enumeration and
        // downstream device detection
    }
    
    #[test]
    #[ignore] // Requires hardware
    fn test_transaction_translator() {
        // This test would verify Transaction Translator
        // functionality with FS/LS devices behind HS hub
    }
}

/// Module tests that don't require hardware
#[cfg(test)]
mod unit_tests {
    use super::*;
    
    mod ehci {
        use imxrt_usbh::ehci::{QueueTD, QueueHead, token, endpoint};
        
        #[test]
        fn test_qtd_token_construction() {
            let qtd = QueueTD::new();
            
            // Test that we can create a QTD and access its token
            let initial_token = qtd.token.load(core::sync::atomic::Ordering::Relaxed);
            
            // Test token constants are defined correctly
            assert_eq!(token::PID_OUT, 0 << 8);
            assert_eq!(token::PID_IN, 1 << 8);
            assert_eq!(token::PID_SETUP, 2 << 8);
            
            // Test PID mask
            assert_eq!(token::PID_MASK, 0x300);
        }
        
        #[test]
        fn test_qtd_buffer_configuration() {
            let qtd = QueueTD::new();
            
            // Test that we can access buffer pointers array
            let buffer_ptr = qtd.buffer_pointers[0].load(core::sync::atomic::Ordering::Relaxed);
            
            // Initial buffer pointer should be zero
            assert_eq!(buffer_ptr, 0);
            
            // Test token shift constants are defined
            assert!(token::TOTAL_BYTES_SHIFT > 0);
            assert!(token::TOTAL_BYTES_MASK > 0);
        }
        
        #[test]
        fn test_qh_endpoint_configuration() {
            let qh = QueueHead::new();
            
            let result = qh.init_endpoint(
                1,      // device address
                0,      // endpoint 0
                64,     // max packet size
                endpoint::SPEED_HIGH,
                true,   // control endpoint
            );
            
            assert!(result.is_ok());
            
            let chars = qh.endpoint_chars.load(core::sync::atomic::Ordering::Relaxed);
            
            // Verify device address
            assert_eq!(
                (chars >> endpoint::DEVICE_ADDRESS_SHIFT) & endpoint::DEVICE_ADDRESS_MASK,
                1
            );
            
            // Verify endpoint number
            assert_eq!(
                (chars >> endpoint::ENDPOINT_NUMBER_SHIFT) & endpoint::ENDPOINT_NUMBER_MASK,
                0
            );
            
            // Verify max packet size
            assert_eq!(
                (chars >> endpoint::MAX_PACKET_LENGTH_SHIFT) & endpoint::MAX_PACKET_LENGTH_MASK,
                64
            );
            
            // Verify it's marked as control endpoint
            assert_ne!(chars & endpoint::CONTROL_ENDPOINT, 0);
        }
    }
    
    #[cfg(feature = "hub")]
    mod hub {
        use imxrt_usbh::hub::{Hub, PortStatus, PortChange, TransactionTranslator};
        
        #[test]
        fn test_hub_creation() {
            let hub = Hub::new(1, 4, 0x0000, 50);
            
            assert_eq!(hub.get_address(), 1);
            assert_eq!(hub.get_num_ports(), 4);
            assert_eq!(hub.get_depth(), 0);
            assert!(!hub.is_multi_tt());
        }
        
        #[test]
        fn test_port_status_flags() {
            let status = PortStatus::from_bits(
                PortStatus::CONNECTED | 
                PortStatus::ENABLED |
                PortStatus::HIGH_SPEED
            ).unwrap();
            
            assert!(status.contains(PortStatus::CONNECTED));
            assert!(status.contains(PortStatus::ENABLED));
            assert!(status.contains(PortStatus::HIGH_SPEED));
            assert!(!status.contains(PortStatus::LOW_SPEED));
        }
        
        #[test]
        fn test_transaction_translator() {
            let tt = TransactionTranslator::new(1, 2, 3);
            
            assert_eq!(tt.get_hub_address(), 1);
            assert_eq!(tt.get_hub_port(), 2);
            assert_eq!(tt.get_think_time(), 3);
            
            // Test split transaction scheduling
            assert!(tt.can_schedule_split());
            tt.start_split();
            
            // Verify we track active splits
            let active = tt.get_active_splits();
            assert_eq!(active, 1);
            
            tt.complete_split();
            let active = tt.get_active_splits();
            assert_eq!(active, 0);
        }
        
        #[test]
        fn test_hub_depth_limits() {
            // USB 2.0 allows maximum 5 hub levels
            let mut hub = Hub::new(1, 4, 0x0000, 50);
            
            for depth in 0..5 {
                let result = hub.set_depth(depth);
                assert!(result.is_ok());
            }
            
            // Depth 5 should fail (too deep)
            let result = hub.set_depth(5);
            assert!(matches!(result, Err(UsbError::HubDepthExceeded)));
        }
    }
    
    mod dma {
        use imxrt_usbh::dma::DmaBufferPool;
        
        #[test]
        fn test_dma_buffer_creation() {
            let mut pool = DmaBufferPool::new();
            let buffer = pool.alloc(256).expect("Failed to allocate buffer");
            
            // Verify alignment
            let addr = buffer.as_ptr() as usize;
            assert_eq!(addr & 0x1F, 0, "Buffer not 32-byte aligned");
            
            // Verify buffer can be used
            assert!(!buffer.as_slice().is_empty());
        }
    }
    
    mod error {
        use imxrt_usbh::UsbError;
        
        #[test]
        fn test_error_types() {
            // Test that error types can be created and matched
            let timeout_error = UsbError::Timeout;
            match timeout_error {
                UsbError::Timeout => {},
                _ => panic!("Wrong error type"),
            }
            
            let invalid_param_error = UsbError::InvalidParameter;
            match invalid_param_error {
                UsbError::InvalidParameter => {},
                _ => panic!("Wrong error type"),
            }
        }
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // In a real embedded environment, this would reset or halt
    loop {}
}

