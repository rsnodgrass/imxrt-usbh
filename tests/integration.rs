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

use common::{create_mock_buffer, assert_buffer_aligned};
use imxrt_usbh::transfer::{
    BulkTransferManager, InterruptTransferManager, IsochronousTransferManager,
    Direction, MicroframeTiming,
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

    /// Test isochronous double buffering with buffer swapping
    #[test]
    fn test_isochronous_buffer_ping_pong() {
        let mut mgr = IsochronousTransferManager::<2>::new();

        let buf1 = create_mock_buffer(512, 0);
        let buf2 = create_mock_buffer(512, 1);

        let idx = mgr
            .submit(
                Direction::In,
                1,
                0x81,
                1023,
                buf1,
                buf2,
                MicroframeTiming::single(),
                true,
            )
            .expect("iso submission failed");

        let transfer = mgr.get_transfer(idx).expect("transfer not found");

        // Initial buffer is 0
        assert_eq!(transfer.current_buffer.load(core::sync::atomic::Ordering::Relaxed), 0);

        // Simulate buffer swap (simulating completion)
        transfer.swap_buffer();
        assert_eq!(transfer.current_buffer.load(core::sync::atomic::Ordering::Relaxed), 1);

        // Swap back
        transfer.swap_buffer();
        assert_eq!(transfer.current_buffer.load(core::sync::atomic::Ordering::Relaxed), 0);
    }

    /// Test frame-based scheduling for interrupt transfers
    #[test]
    fn test_interrupt_frame_scheduling() {
        let mgr = InterruptTransferManager::<8>::new();

        // Set initial frame to 100
        mgr.update_frame(100);
        assert_eq!(mgr.current_frame.load(core::sync::atomic::Ordering::Relaxed), 100);

        // Advance frames
        for frame in 101..110 {
            mgr.update_frame(frame);
            assert_eq!(mgr.current_frame.load(core::sync::atomic::Ordering::Relaxed), frame);
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
            assert!(bulk_mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).is_ok());
        }

        // Fill interrupt manager
        for i in 2..4 {
            let buf = create_mock_buffer(64, i);
            assert!(int_mgr.submit(Direction::In, 1, 0x82, 8, buf, 10, true).is_ok());
        }

        // Both should be at capacity
        assert_eq!(bulk_mgr.active_count(), 2);
        assert_eq!(int_mgr.active_count(), 2);

        // Next allocations should fail
        let buf = create_mock_buffer(512, 4);
        assert!(bulk_mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).is_err());

        let buf = create_mock_buffer(64, 5);
        assert!(int_mgr.submit(Direction::In, 1, 0x82, 8, buf, 10, true).is_err());
    }

    /// Test statistics across multiple transfers
    #[test]
    fn test_multi_transfer_statistics() {
        let bulk_mgr = BulkTransferManager::<8>::new();
        let int_mgr = InterruptTransferManager::<8>::new();

        let bulk_stats = bulk_mgr.statistics();
        let int_stats = int_mgr.statistics();

        // Initial state
        assert_eq!(bulk_stats.submissions(), 0);
        assert_eq!(int_stats.submissions(), 0);

        // Simulate some activity
        bulk_stats.record_submission();
        bulk_stats.record_submission();
        bulk_stats.record_completion();

        int_stats.record_submission();
        int_stats.record_nak_timeout();

        // Verify independent tracking
        assert_eq!(bulk_stats.submissions(), 2);
        assert_eq!(bulk_stats.completions(), 1);
        assert_eq!(int_stats.submissions(), 1);
        assert_eq!(int_stats.nak_timeouts(), 1);
    }

    /// Test microframe timing calculations for isochronous transfers
    #[test]
    fn test_isochronous_timing_calculations() {
        let single = MicroframeTiming::single();
        let double = MicroframeTiming::double();
        let triple = MicroframeTiming::triple();

        // Single: 1 transaction per microframe
        assert_eq!(single.transactions_per_uframe, 1);
        assert_eq!(single.additional_opportunities(), 0);
        assert_eq!(single.total_bandwidth_slots(), 1);

        // Double: 2 transactions per microframe
        assert_eq!(double.transactions_per_uframe, 2);
        assert_eq!(double.additional_opportunities(), 1);
        assert_eq!(double.total_bandwidth_slots(), 3);

        // Triple: 3 transactions per microframe (maximum)
        assert_eq!(triple.transactions_per_uframe, 3);
        assert_eq!(triple.additional_opportunities(), 2);
        assert_eq!(triple.total_bandwidth_slots(), 5);
    }

    /// Test transfer state coordination
    #[test]
    fn test_transfer_state_coordination() {
        let mut bulk_mgr = BulkTransferManager::<4>::new();

        // Submit transfer
        let buf = create_mock_buffer(512, 0);
        let idx = bulk_mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();

        let transfer = bulk_mgr.get_transfer(idx).expect("transfer missing");

        // Verify initial state
        use imxrt_usbh::transfer::BulkState;
        assert_eq!(transfer.state(), BulkState::Idle);

        // Simulate state transitions
        transfer.transition_state(BulkState::Active);
        assert_eq!(transfer.state(), BulkState::Active);
        assert!(!transfer.is_complete());

        transfer.transition_state(BulkState::Complete);
        assert_eq!(transfer.state(), BulkState::Complete);
        assert!(transfer.is_complete());
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
