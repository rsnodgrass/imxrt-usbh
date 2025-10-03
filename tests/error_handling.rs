//! Error handling and recovery tests
//!
//! Tests verify error detection, classification, and recovery logic
//! without requiring actual hardware.

#![no_std]
#![no_main]

mod common;

use common::create_mock_buffer;
use imxrt_usbh::transfer::{
    BulkTransferManager, InterruptTransferManager, Direction,
    BulkState, InterruptState,
};
use imxrt_usbh::UsbError;

#[cfg(test)]
mod tests {
    use super::*;

    /// Test NAK handling and retry logic for bulk transfers
    #[test]
    fn test_bulk_nak_handling() {
        let mut mgr = BulkTransferManager::<4>::new();
        let buf = create_mock_buffer(512, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Simulate NAK response (device not ready)
        // In real scenario, transfer would remain active and retry
        transfer.transition_state(BulkState::Active);

        // NAK doesn't cause failure, transfer stays active
        assert_eq!(transfer.state(), BulkState::Active);
        assert!(!transfer.is_failed());
    }

    /// Test STALL condition handling and recovery
    #[test]
    fn test_bulk_stall_recovery() {
        let mut mgr = BulkTransferManager::<4>::new();
        let buf = create_mock_buffer(512, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Simulate STALL condition (endpoint halted)
        transfer.transition_state(BulkState::Stalled);
        assert_eq!(transfer.state(), BulkState::Stalled);
        assert!(transfer.is_stalled());
        assert!(transfer.is_failed());

        // Clear stall to recover
        let mut transfer_mut = mgr.get_transfer_mut(idx).unwrap();
        transfer_mut.clear_stall();

        assert_eq!(transfer_mut.state(), BulkState::Idle);
        assert!(!transfer_mut.is_stalled());
    }

    /// Test timeout detection logic
    #[test]
    fn test_transfer_timeout_logic() {
        let mut mgr = BulkTransferManager::<4>::new();
        let buf = create_mock_buffer(512, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Verify timeout value is set
        assert_eq!(transfer.timeout_ms(), 1000);

        // Test zero timeout (no timeout)
        let mut transfer_mut = mgr.get_transfer_mut(idx).unwrap();
        transfer_mut.set_timeout(0);
        assert_eq!(transfer_mut.timeout_ms(), 0);

        // Test custom timeout
        transfer_mut.set_timeout(5000);
        assert_eq!(transfer_mut.timeout_ms(), 5000);
    }

    /// Test interrupt transfer NAK counting and threshold
    #[test]
    fn test_interrupt_nak_threshold() {
        let mut mgr = InterruptTransferManager::<4>::new();
        let buf = create_mock_buffer(64, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 8, buf, 10, true).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Initial NAK count is 0
        assert_eq!(transfer.nak_count.load(core::sync::atomic::Ordering::Relaxed), 0);

        // Simulate NAK responses
        for expected_count in 1..=3 {
            transfer.nak_count.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            assert_eq!(
                transfer.nak_count.load(core::sync::atomic::Ordering::Relaxed),
                expected_count
            );
        }

        // max_naks is 3, threshold reached
        assert_eq!(transfer.nak_count.load(core::sync::atomic::Ordering::Relaxed), transfer.max_naks);
    }

    /// Test error state transitions
    #[test]
    fn test_bulk_error_state_transitions() {
        let mut mgr = BulkTransferManager::<4>::new();
        let buf = create_mock_buffer(512, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Normal: Idle → Active → Complete
        transfer.transition_state(BulkState::Active);
        transfer.transition_state(BulkState::Complete);
        assert!(transfer.is_complete());

        // Error path: Active → Failed
        let buf2 = create_mock_buffer(512, 1);
        let idx2 = mgr.submit(Direction::In, 1, 0x81, 64, buf2, 1000).unwrap();
        let transfer2 = mgr.get_transfer(idx2).unwrap();

        transfer2.transition_state(BulkState::Active);
        transfer2.transition_state(BulkState::Failed);
        assert_eq!(transfer2.state(), BulkState::Failed);
        assert!(transfer2.is_failed());
        assert!(!transfer2.is_complete());
    }

    /// Test interrupt error state transitions
    #[test]
    fn test_interrupt_error_state_transitions() {
        let mut mgr = InterruptTransferManager::<4>::new();
        let buf = create_mock_buffer(64, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 8, buf, 10, true).unwrap();
        let transfer = mgr.get_transfer(idx).unwrap();

        // Normal: Idle → Scheduled → Active → Complete
        transfer.transition_state(InterruptState::Scheduled);
        transfer.transition_state(InterruptState::Active);
        transfer.transition_state(InterruptState::Complete);
        assert!(transfer.is_complete());

        // Error path: Scheduled → Stalled
        let buf2 = create_mock_buffer(64, 1);
        let idx2 = mgr.submit(Direction::In, 1, 0x81, 8, buf2, 10, true).unwrap();
        let transfer2 = mgr.get_transfer(idx2).unwrap();

        transfer2.transition_state(InterruptState::Scheduled);
        transfer2.transition_state(InterruptState::Stalled);
        assert!(transfer2.is_stalled());
        assert!(transfer2.is_failed());
    }

    /// Test statistics tracking for errors
    #[test]
    fn test_error_statistics_tracking() {
        let mgr = BulkTransferManager::<8>::new();
        let stats = mgr.statistics();

        // Initial state - no errors
        assert_eq!(stats.failures(), 0);
        assert_eq!(stats.submissions(), 0);

        // Record some operations
        stats.record_submission();
        stats.record_submission();
        stats.record_completion();
        stats.record_failure();

        assert_eq!(stats.submissions(), 2);
        assert_eq!(stats.completions(), 1);
        assert_eq!(stats.failures(), 1);

        // Success rate = 1 / (1 + 1) = 0.5
        assert_eq!(stats.success_rate(), 0.5);
    }

    /// Test interrupt NAK timeout statistics
    #[test]
    fn test_interrupt_nak_timeout_stats() {
        let mgr = InterruptTransferManager::<8>::new();
        let stats = mgr.statistics();

        // Initial state
        assert_eq!(stats.nak_timeouts(), 0);

        // Simulate NAK timeouts
        stats.record_nak_timeout();
        stats.record_nak_timeout();

        assert_eq!(stats.nak_timeouts(), 2);
    }

    /// Test data toggle reset on error recovery
    #[test]
    fn test_data_toggle_reset_on_error() {
        let mut mgr = BulkTransferManager::<4>::new();
        let buf = create_mock_buffer(512, 0);

        let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
        let transfer_mut = mgr.get_transfer_mut(idx).unwrap();

        // Set data toggle to DATA1
        transfer_mut.data_toggle.store(1, core::sync::atomic::Ordering::Release);
        assert_eq!(transfer_mut.data_toggle(), 1);

        // After error recovery, toggle should reset to DATA0
        transfer_mut.reset_data_toggle();
        assert_eq!(transfer_mut.data_toggle(), 0);
    }

    /// Test buffer overflow detection (size validation)
    #[test]
    fn test_buffer_size_validation() {
        use imxrt_usbh::dma::DmaBufferPool;

        let mut pool = DmaBufferPool::new();

        // Valid sizes should succeed
        assert!(pool.alloc(512).is_ok());
        assert!(pool.alloc(256).is_ok());
        assert!(pool.alloc(64).is_ok());

        // Oversized buffer should fail
        let result = pool.alloc(513);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::InvalidParameter)));
    }

    /// Test concurrent error handling doesn't interfere
    #[test]
    fn test_independent_error_handling() {
        let mut mgr = BulkTransferManager::<4>::new();

        // Create two transfers
        let buf1 = create_mock_buffer(512, 0);
        let idx1 = mgr.submit(Direction::In, 1, 0x81, 64, buf1, 1000).unwrap();

        let buf2 = create_mock_buffer(512, 1);
        let idx2 = mgr.submit(Direction::In, 1, 0x82, 64, buf2, 1000).unwrap();

        // Transfer 1 fails
        let transfer1 = mgr.get_transfer(idx1).unwrap();
        transfer1.transition_state(BulkState::Failed);

        // Transfer 2 succeeds
        let transfer2 = mgr.get_transfer(idx2).unwrap();
        transfer2.transition_state(BulkState::Active);
        transfer2.transition_state(BulkState::Complete);

        // Verify independent states
        assert!(mgr.get_transfer(idx1).unwrap().is_failed());
        assert!(mgr.get_transfer(idx2).unwrap().is_complete());
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
