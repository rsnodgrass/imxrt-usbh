//! Integration tests for imxrt-usbh
//!
//! These tests verify the USB host controller functionality
//! Note: Some tests require hardware and will be marked as #[ignore]

#![no_std]
#![no_main]

use imxrt_usbh::{UsbHost, UsbError, Result};
use imxrt_usbh::ehci::{Ehci, OperationalRegs};
use imxrt_usbh::phy::UsbPhy;
use imxrt_usbh::dma::{DmaBuffer, DmaAllocator};
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
        let buffer = DmaBuffer::<1024>::new();
        let addr = buffer.as_ptr() as usize;
        assert_eq!(addr & 0x1F, 0, "DMA buffer not 32-byte aligned");
    }
    
    /// Test EHCI register structure sizes
    #[test]
    fn test_ehci_structure_sizes() {
        use imxrt_usbh::ehci::qh::QueueHead;
        use imxrt_usbh::ehci::qtd::QueueTransferDescriptor;
        
        // Verify structure sizes match EHCI specification
        assert_eq!(
            core::mem::size_of::<QueueHead>(),
            96,
            "QueueHead size incorrect"
        );
        
        assert_eq!(
            core::mem::size_of::<QueueTransferDescriptor>(),
            32,
            "QTD size incorrect"
        );
        
        // Verify alignment requirements
        assert!(
            core::mem::align_of::<QueueHead>() >= 32,
            "QueueHead alignment incorrect"
        );
        
        assert!(
            core::mem::align_of::<QueueTransferDescriptor>() >= 32,
            "QTD alignment incorrect"
        );
    }
    
    /// Test USB descriptor parsing
    #[test]
    fn test_descriptor_parsing() {
        use imxrt_usbh::enumeration::{DeviceDescriptor, ConfigurationDescriptor};
        
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
        
        // Verify we can parse device descriptor
        let desc = unsafe {
            core::ptr::read(device_desc_bytes.as_ptr() as *const DeviceDescriptor)
        };
        
        assert_eq!(desc.b_length, 0x12);
        assert_eq!(desc.b_descriptor_type, 0x01);
        assert_eq!(desc.bcd_usb, 0x0200);
        assert_eq!(desc.b_max_packet_size0, 0x40);
        assert_eq!(desc.id_vendor, 0x0483);
        assert_eq!(desc.b_num_configurations, 0x01);
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
        use imxrt_usbh::ehci::qtd::{QueueTransferDescriptor, token};
        use imxrt_usbh::ehci::qh::{QueueHead, endpoint};
        
        #[test]
        fn test_qtd_token_construction() {
            let mut qtd = QueueTransferDescriptor::new();
            
            // Test PID encoding
            qtd.set_pid(token::PID_OUT);
            let token = qtd.token.load(core::sync::atomic::Ordering::Relaxed);
            assert_eq!(token & token::PID_MASK, token::PID_OUT);
            
            qtd.set_pid(token::PID_IN);
            let token = qtd.token.load(core::sync::atomic::Ordering::Relaxed);
            assert_eq!(token & token::PID_MASK, token::PID_IN);
            
            qtd.set_pid(token::PID_SETUP);
            let token = qtd.token.load(core::sync::atomic::Ordering::Relaxed);
            assert_eq!(token & token::PID_MASK, token::PID_SETUP);
        }
        
        #[test]
        fn test_qtd_buffer_configuration() {
            let mut qtd = QueueTransferDescriptor::new();
            let buffer_addr = 0x2020_0000u32;
            let length = 512u32;
            
            unsafe {
                qtd.set_buffer(buffer_addr, length);
            }
            
            // Verify buffer pointer was set
            assert_eq!(
                qtd.buffer_pointers[0].load(core::sync::atomic::Ordering::Relaxed),
                buffer_addr
            );
            
            // Verify length was set in token
            let token = qtd.token.load(core::sync::atomic::Ordering::Relaxed);
            let stored_length = (token >> token::TOTAL_BYTES_SHIFT) & token::TOTAL_BYTES_MASK;
            assert_eq!(stored_length, length);
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
        use imxrt_usbh::dma::{DmaBuffer, DescriptorAllocator, DmaPool};
        
        #[test]
        fn test_dma_buffer_creation() {
            let buffer = DmaBuffer::<256>::new();
            
            // Verify alignment
            let addr = buffer.as_ptr() as usize;
            assert_eq!(addr & 0x1F, 0, "Buffer not 32-byte aligned");
            
            // Verify size
            assert_eq!(buffer.len(), 256);
        }
        
        #[test]
        fn test_descriptor_pool() {
            let mut pool = DmaPool::<32, 16>::new(); // 16 descriptors of 32 bytes
            
            // Allocate all descriptors
            let mut descriptors = Vec::new();
            for _ in 0..16 {
                let desc = pool.allocate();
                assert!(desc.is_some());
                descriptors.push(desc.unwrap());
            }
            
            // Pool should be exhausted
            let desc = pool.allocate();
            assert!(desc.is_none());
            
            // Free one descriptor
            pool.free(descriptors.pop().unwrap());
            
            // Should be able to allocate again
            let desc = pool.allocate();
            assert!(desc.is_some());
        }
    }
    
    mod error {
        use imxrt_usbh::error::{UsbError, ErrorMetrics};
        
        #[test]
        fn test_error_metrics() {
            let metrics = ErrorMetrics::new();
            
            metrics.increment_nak();
            metrics.increment_nak();
            metrics.increment_stall();
            metrics.increment_babble();
            
            assert_eq!(metrics.get_nak_count(), 2);
            assert_eq!(metrics.get_stall_count(), 1);
            assert_eq!(metrics.get_babble_count(), 1);
            assert_eq!(metrics.get_timeout_count(), 0);
            
            metrics.reset();
            
            assert_eq!(metrics.get_nak_count(), 0);
            assert_eq!(metrics.get_stall_count(), 0);
        }
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // In a real embedded environment, this would reset or halt
    loop {}
}