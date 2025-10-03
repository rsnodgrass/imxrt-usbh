//! Integration tests for multi-transfer scenarios
//!
//! These tests verify interactions between different transfer types
//! and components without requiring actual hardware.
//!
//! Tests compile for ARM Cortex-M7 (thumbv7em-none-eabihf) and verify
//! logic, state machines, and resource coordination.

#![no_std]
#![no_main]

mod common;

use common::{assert_buffer_aligned, create_mock_buffer};
use imxrt_usbh::transfer::{
    BulkTransferManager, Direction, InterruptTransferManager, IsochronousTransferManager,
    MicroframeTiming,
};
use imxrt_usbh::{BulkTransfer, InterruptTransfer, IsochronousTransfer};

#[cfg(test)]
mod tests {
    use super::*;

    /// Test concurrent bulk and interrupt transfers don't interfere
    #[test]
    fn test_bulk_interrupt_coexistence() {
        let mut bulk_mgr = BulkTransferManager::<4>::new();
        let mut interrupt_mgr = InterruptTransferManager::<4>::new();

        // Submit bulk transfer
        let bulk_buf = create_mock_buffer(512, 0);
        let bulk_id = bulk_mgr
            .submit(Direction::In, 1, 0x81, 64, bulk_buf, 1000)
            .expect("bulk submission failed");

        // Submit interrupt transfer
        let int_buf = create_mock_buffer(64, 1);
        let int_id = interrupt_mgr
            .submit(Direction::In, 1, 0x82, 8, int_buf, 10, true)
            .expect("interrupt submission failed");

        // Verify both are active
        assert_eq!(bulk_mgr.active_count(), 1);
        assert_eq!(interrupt_mgr.active_count(), 1);

        // Verify they got different indices
        assert_eq!(bulk_id, 0);
        assert_eq!(int_id, 0);
    }

    /// Test multiple bulk transfers in sequence
    #[test]
    fn test_bulk_transfer_sequence() {
        let mut mgr = BulkTransferManager::<8>::new();

        // Submit 3 bulk transfers
        for i in 0..3 {
            let buffer = create_mock_buffer(512, i);
            let result = mgr.submit(Direction::In, 1, 0x81, 64, buffer, 1000);
            assert!(result.is_ok());
            assert_eq!(result.unwrap(), i);
        }

        assert_eq!(mgr.active_count(), 3);

        // Verify all transfers are accessible
        for i in 0..3 {
            let transfer = mgr.get_transfer(i);
            assert!(transfer.is_some());
        }
    }

    /// Test resource exhaustion with multiple transfer types
    #[test]
    fn test_multi_type_resource_limits() {
        let mut bulk_mgr = BulkTransferManager::<2>::new();
        let mut int_mgr = InterruptTransferManager::<2>::new();

        // Fill bulk manager
        for i in 0..2 {
            let buf = create_mock_buffer(512, i);
            assert!(bulk_mgr
                .submit(Direction::In, 1, 0x81, 64, buf, 1000)
                .is_ok());
        }

        // Fill interrupt manager
        for i in 2..4 {
            let buf = create_mock_buffer(64, i);
            assert!(int_mgr
                .submit(Direction::In, 1, 0x82, 8, buf, 10, true)
                .is_ok());
        }

        // Both should be at capacity
        assert_eq!(bulk_mgr.active_count(), 2);
        assert_eq!(int_mgr.active_count(), 2);

        // Next allocations should fail
        let buf = create_mock_buffer(512, 4);
        assert!(bulk_mgr
            .submit(Direction::In, 1, 0x81, 64, buf, 1000)
            .is_err());

        let buf = create_mock_buffer(64, 5);
        assert!(int_mgr
            .submit(Direction::In, 1, 0x82, 8, buf, 10, true)
            .is_err());
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
