//! Integration tests for imxrt-usbh
//!
//! These tests verify cross-module interactions and high-level functionality.

#![no_std]
#![cfg(test)]

extern crate std;

use imxrt_usbh::{UsbHost, UsbError, Result};

/// Test USB host initialization constraints
#[test]
fn test_single_initialization() {
    // This verifies the singleton pattern works correctly
    let _host1 = unsafe { UsbHost::new() }.expect("First initialization should succeed");
    
    // Second initialization should fail
    let result2 = unsafe { UsbHost::new() };
    assert!(matches!(result2, Err(UsbError::AlreadyInitialized)));
}

/// Test EHCI data structure sizes match specification
#[test]
fn test_ehci_structure_compliance() {
    use imxrt_usbh::ehci::qh::QueueHead;
    use imxrt_usbh::ehci::qtd::QueueTD;
    
    // EHCI spec requires specific sizes and alignments
    assert_eq!(core::mem::size_of::<QueueHead>(), 96);
    assert!(core::mem::align_of::<QueueHead>() >= 32);
    
    assert!(core::mem::size_of::<QueueTD>() >= 64);
    assert_eq!(core::mem::align_of::<QueueTD>(), 32);
}

/// Test DMA buffer alignment requirements
#[test]
fn test_dma_buffer_requirements() {
    use imxrt_usbh::dma::DmaBuffer;
    
    // Test various buffer sizes
    let buffer_64 = DmaBuffer::<64>::new();
    let buffer_512 = DmaBuffer::<512>::new();
    let buffer_4k = DmaBuffer::<4096>::new();
    
    // All should be 32-byte aligned for EHCI
    assert_eq!(buffer_64.as_ptr() as usize & 0x1F, 0);
    assert_eq!(buffer_512.as_ptr() as usize & 0x1F, 0);
    assert_eq!(buffer_4k.as_ptr() as usize & 0x1F, 0);
    
    // Verify sizes
    assert_eq!(buffer_64.len(), 64);
    assert_eq!(buffer_512.len(), 512);
    assert_eq!(buffer_4k.len(), 4096);
}

/// Test transfer type and direction enums
#[test]  
fn test_usb_protocol_constants() {
    use imxrt_usbh::transfer::{TransferType, Direction};
    
    // USB 2.0 specification values
    assert_eq!(TransferType::Control as u8, 0);
    assert_eq!(TransferType::Isochronous as u8, 1);
    assert_eq!(TransferType::Bulk as u8, 2);
    assert_eq!(TransferType::Interrupt as u8, 3);
    
    assert_eq!(Direction::Out as u8, 0);
    assert_eq!(Direction::In as u8, 1);
}

/// Test USB standard request packet construction  
#[test]
fn test_setup_packet_construction() {
    use imxrt_usbh::transfer::SetupPacket;
    
    // GET_DESCRIPTOR device request
    let packet = SetupPacket::new(
        0x80,  // Device-to-host, standard, device
        0x06,  // GET_DESCRIPTOR
        0x0100, // Device descriptor
        0,     // wIndex
        18,    // wLength
    );
    
    assert_eq!(packet.request_type, 0x80);
    assert_eq!(packet.request, 0x06);
    assert_eq!(packet.value, 0x0100);
    assert_eq!(packet.index, 0);
    assert_eq!(packet.length, 18);
}

/// Test error handling and recovery mechanisms
#[test]
fn test_error_handling_integration() {
    use imxrt_usbh::error::{ErrorMetrics, ErrorClassification};
    
    let metrics = ErrorMetrics::new();
    
    // Test error counting
    metrics.increment_nak();
    metrics.increment_stall();
    metrics.increment_timeout();
    
    assert_eq!(metrics.get_nak_count(), 1);
    assert_eq!(metrics.get_stall_count(), 1);
    assert_eq!(metrics.get_timeout_count(), 1);
    
    // Test error classification
    let classification = ErrorClassification::from_usb_error(&UsbError::Stall);
    assert_eq!(classification.severity(), 2); // Medium severity
    assert!(classification.is_retryable());
}

#[cfg(feature = "hub")]
mod hub_integration_tests {
    use super::*;
    use imxrt_usbh::hub::{Hub, PortStatus, TransactionTranslator};
    
    /// Test hub and transaction translator integration
    #[test]
    fn test_hub_transaction_translator_integration() {
        let hub = Hub::new(1, 4, 0x0200, 50);
        
        // Configure hub for split transactions
        let mut hub = hub;
        hub.set_multi_tt(true);
        
        // Add transaction translators for each port
        for port in 1..=4 {
            let result = hub.configure_port_tt(port, 2); // 2 microframes think time
            assert!(result.is_ok());
        }
        
        // Verify TT configuration
        if let Some(tt) = hub.get_port_tt(1) {
            assert_eq!(tt.get_hub_address(), 1);
            assert_eq!(tt.get_hub_port(), 1);
            assert_eq!(tt.get_think_time(), 2);
        } else {
            panic!("Transaction Translator not configured");
        }
    }
    
    /// Test hub depth limits
    #[test]
    fn test_hub_depth_compliance() {
        let mut hub = Hub::new(1, 4, 0x0200, 50);
        
        // USB 2.0 allows maximum 5 hub tiers
        for depth in 0..5 {
            let result = hub.set_depth(depth);
            assert!(result.is_ok(), "Hub depth {} should be valid", depth);
        }
        
        // Depth 5 should fail (tier 6)
        let result = hub.set_depth(5);
        assert!(matches!(result, Err(UsbError::HubDepthExceeded)));
    }
    
    /// Test port status interpretation
    #[test]
    fn test_port_status_integration() {
        let hub = Hub::new(1, 4, 0x0200, 50);
        
        // Test different device speeds
        let hs_status = PortStatus::CONNECTED | PortStatus::ENABLED | PortStatus::HIGH_SPEED;
        assert_eq!(hub.get_port_speed(hs_status), 2); // High-speed
        
        let ls_status = PortStatus::CONNECTED | PortStatus::ENABLED | PortStatus::LOW_SPEED;
        assert_eq!(hub.get_port_speed(ls_status), 1); // Low-speed
        
        let fs_status = PortStatus::CONNECTED | PortStatus::ENABLED;
        assert_eq!(hub.get_port_speed(fs_status), 0); // Full-speed (default)
    }
}

/// Test descriptor allocation and management
#[test]
fn test_descriptor_pool_integration() {
    use imxrt_usbh::dma::{DescriptorAllocator, DmaPool};
    
    // Test QH and QTD allocation from the same allocator
    let mut allocator = DescriptorAllocator::<8, 16>::new();
    
    // Allocate some QHs
    let qh1 = allocator.alloc_qh();
    let qh2 = allocator.alloc_qh();
    assert!(qh1.is_some());
    assert!(qh2.is_some());
    
    // Allocate some QTDs
    let qtd1 = allocator.alloc_qtd();
    let qtd2 = allocator.alloc_qtd();
    assert!(qtd1.is_some());
    assert!(qtd2.is_some());
    
    // Free and reallocate
    allocator.free_qh(qh1.unwrap());
    let qh3 = allocator.alloc_qh();
    assert!(qh3.is_some());
}

/// Test transfer manager coordination
#[test] 
fn test_transfer_manager_integration() {
    use imxrt_usbh::transfer::TransferManager;
    
    let manager = TransferManager::new();
    
    // Test that manager starts in a clean state
    assert_eq!(manager.active_transfers(), 0);
    assert_eq!(manager.completed_transfers(), 0);
    assert_eq!(manager.failed_transfers(), 0);
}

/// Test enumeration state machine
#[test]
fn test_enumeration_state_transitions() {
    use imxrt_usbh::enumeration::{EnumerationState, DeviceEnumerator};
    
    // Test initial state
    let state = EnumerationState::new();
    assert!(matches!(state.current_state(), EnumerationState::Idle));
    
    // Test state transitions
    let mut state = state;
    state.start_reset();
    assert!(matches!(state.current_state(), EnumerationState::Resetting { .. }));
    
    state.reset_complete();
    assert!(matches!(state.current_state(), EnumerationState::GetDescriptor { .. }));
}

#[cfg(feature = "alloc")]
mod alloc_integration_tests {
    use super::*;
    use std::vec::Vec;
    
    /// Test integration with allocator for larger transfers
    #[test] 
    fn test_large_transfer_buffers() {
        // Test that we can handle large transfers with allocation
        let large_buffer: Vec<u8> = vec![0; 64 * 1024]; // 64KB
        assert_eq!(large_buffer.len(), 64 * 1024);
        
        // Verify buffer alignment for DMA (if needed)
        let addr = large_buffer.as_ptr() as usize;
        if addr & 0x1F != 0 {
            // Would need special allocation for DMA alignment
            println!("Warning: Large buffer not DMA-aligned");
        }
    }
}

#[cfg(feature = "rtic-support")]
mod rtic_integration_tests {
    use super::*;
    
    /// Test RTIC integration points
    #[test]
    fn test_rtic_resource_sharing() {
        // Test that our atomic structures work with RTIC sharing
        use imxrt_usbh::ehci::qh::QueueHead;
        use core::sync::atomic::Ordering;
        
        let qh = QueueHead::new();
        
        // Simulate shared access patterns that RTIC might use
        let initial_chars = qh.endpoint_chars.load(Ordering::Acquire);
        qh.endpoint_chars.store(initial_chars | 0x1000, Ordering::Release);
        
        let updated_chars = qh.endpoint_chars.load(Ordering::Acquire);
        assert_ne!(initial_chars, updated_chars);
    }
}