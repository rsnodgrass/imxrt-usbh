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

}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
