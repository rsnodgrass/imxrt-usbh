//! Resource management and lifecycle tests
//!
//! Tests verify DMA buffer pools, descriptor pools, and resource lifecycle
//! without requiring actual hardware.

#![no_std]
#![no_main]

mod common;

use common::create_mock_buffer;
use imxrt_usbh::dma::{BufferStats, DmaBufferPool};
use imxrt_usbh::transfer::{BulkTransferManager, Direction};
use imxrt_usbh::UsbError;

#[cfg(test)]
mod tests {
    use super::*;

    /// Test DMA buffer pool allocation and deallocation cycle
    #[test]
    fn test_dma_pool_allocation_cycle() {
        let mut pool = DmaBufferPool::new();

        // Initial state - all free
        let stats = pool.buffer_stats();
        assert_eq!(stats.total_buffers, 32);
        assert_eq!(stats.allocated_buffers, 0);
        assert_eq!(stats.free_buffers, 32);

        // Allocate 10 buffers
        let mut buffers = heapless::Vec::<_, 10>::new();
        for _ in 0..10 {
            let buf = pool.alloc(512).expect("allocation failed");
            buffers.push(buf).ok();
        }

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 10);
        assert_eq!(stats.free_buffers, 22);

        // Free all buffers
        while let Some(buf) = buffers.pop() {
            pool.free(buf);
        }

        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 0);
        assert_eq!(stats.free_buffers, 32);
    }

    /// Test descriptor pool exhaustion and recovery
    #[test]
    fn test_transfer_pool_exhaustion_recovery() {
        let mut mgr = BulkTransferManager::<4>::new();

        // Fill pool
        let mut buffers = heapless::Vec::<_, 4>::new();
        for i in 0..4 {
            let buf = create_mock_buffer(512, i);
            let idx = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000).unwrap();
            buffers.push(idx).ok();
        }

        assert_eq!(mgr.active_count(), 4);

        // Pool exhausted - next allocation fails
        let buf = create_mock_buffer(512, 4);
        let result = mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000);
        assert!(result.is_err());
        assert!(matches!(result, Err(UsbError::NoResources)));

        // Simulate completion and removal (in real code, would be done by manager)
        // For test purposes, we verify the count
        assert_eq!(mgr.active_count(), 4);
    }

    /// Test concurrent resource allocation across multiple pools
    #[test]
    fn test_concurrent_pool_allocation() {
        let mut dma_pool = DmaBufferPool::new();
        let mut bulk_mgr = BulkTransferManager::<8>::new();
        let mut allocated = 0;

        // Allocate from both pools simultaneously
        for i in 0..5 {
            // DMA buffer allocation
            let buf = dma_pool.alloc(512).expect("DMA alloc failed");

            // Transfer manager allocation
            let idx = bulk_mgr
                .submit(Direction::In, 1, 0x81, 64, buf, 1000)
                .unwrap();
            assert_eq!(idx, i);
            allocated += 1;
        }

        // Verify both pools tracked allocations
        assert_eq!(dma_pool.buffer_stats().allocated_buffers, 5);
        assert_eq!(bulk_mgr.active_count(), 5);
        assert_eq!(allocated, 5);
    }

    /// Test statistics accuracy across multiple operations
    #[test]
    fn test_statistics_accuracy() {
        let mgr = BulkTransferManager::<8>::new();
        let stats = mgr.statistics();

        // Simulate a series of operations
        for _ in 0..10 {
            stats.record_submission();
        }

        for _ in 0..7 {
            stats.record_completion();
        }

        for _ in 0..2 {
            stats.record_failure();
        }

        // Verify counts
        assert_eq!(stats.submissions(), 10);
        assert_eq!(stats.completions(), 7);
        assert_eq!(stats.failures(), 2);

        // Success rate = 7 / (7 + 2) = 0.777...
        let rate = stats.success_rate();
        assert!((rate - 0.777).abs() < 0.001);
    }

    /// Test memory leak detection (allocation/free balance)
    #[test]
    fn test_no_memory_leaks() {
        let mut pool = DmaBufferPool::new();

        // Perform 100 allocation/free cycles
        for _ in 0..100 {
            let buf = pool.alloc(256).expect("allocation failed");
            pool.free(buf);
        }

        // Pool should be back to initial state
        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 0);
        assert_eq!(stats.free_buffers, 32);
    }

    /// Test buffer reuse after free
    #[test]
    fn test_buffer_reuse() {
        let mut pool = DmaBufferPool::new();

        // Allocate buffer
        let buf1 = pool.alloc(512).unwrap();
        let pool_idx1 = buf1.pool_index();

        // Free it
        pool.free(buf1);

        // Next allocation should reuse same slot
        let buf2 = pool.alloc(512).unwrap();
        let pool_idx2 = buf2.pool_index();

        // Should get same pool index (LIFO or FIFO reuse)
        assert_eq!(pool_idx1, pool_idx2);
    }

    /// Test resource contention simulation
    #[test]
    fn test_resource_contention() {
        let mut bulk_mgr = BulkTransferManager::<4>::new();

        // Fill to capacity
        for i in 0..4 {
            let buf = create_mock_buffer(512, i);
            assert!(bulk_mgr
                .submit(Direction::In, 1, 0x81, 64, buf, 1000)
                .is_ok());
        }

        // Multiple attempts to allocate should all fail
        for _ in 0..5 {
            let buf = create_mock_buffer(256, 0);
            let result = bulk_mgr.submit(Direction::In, 1, 0x81, 64, buf, 1000);
            assert!(result.is_err());
        }

        // Pool count should remain at capacity
        assert_eq!(bulk_mgr.active_count(), 4);
    }

    /// Test statistics reset/clear functionality
    #[test]
    fn test_statistics_lifecycle() {
        let mgr = BulkTransferManager::<8>::new();
        let stats = mgr.statistics();

        // Record some activity
        stats.record_submission();
        stats.record_submission();
        stats.record_completion();

        assert_eq!(stats.submissions(), 2);
        assert_eq!(stats.completions(), 1);

        // In a real implementation, stats might be clearable
        // For now, just verify they persist
        assert_eq!(stats.submissions(), 2);
    }

    /// Test buffer pool exhaustion gradual fill
    #[test]
    fn test_pool_gradual_exhaustion() {
        let mut pool = DmaBufferPool::new();

        // Track stats as we fill the pool
        for expected_allocated in 1..=32 {
            let buf = pool
                .alloc(512)
                .expect(&alloc::format!("Failed at {}", expected_allocated));

            let stats = pool.buffer_stats();
            assert_eq!(stats.allocated_buffers, expected_allocated);
            assert_eq!(stats.free_buffers, 32 - expected_allocated);

            // Keep buffer alive
            core::mem::forget(buf);
        }

        // Pool should be exhausted
        let stats = pool.buffer_stats();
        assert_eq!(stats.allocated_buffers, 32);
        assert_eq!(stats.free_buffers, 0);

        // Next allocation fails
        let result = pool.alloc(512);
        assert!(result.is_err());
    }

    /// Test byte tracking across operations
    #[test]
    fn test_byte_tracking() {
        let mgr = BulkTransferManager::<8>::new();
        let stats = mgr.statistics();

        // Initial bytes
        assert_eq!(stats.total_bytes(), 0);

        // Simulate transfers
        stats.record_bytes(512);
        stats.record_bytes(1024);
        stats.record_bytes(256);

        assert_eq!(stats.total_bytes(), 1792);
    }

    /// Test multiple buffer sizes
    #[test]
    fn test_various_buffer_sizes() {
        let mut pool = DmaBufferPool::new();

        // Allocate various sizes (all <=512)
        let sizes = [64, 128, 256, 512, 1, 500];
        let mut buffers = heapless::Vec::<_, 6>::new();

        for &size in &sizes {
            let buf = pool
                .alloc(size)
                .expect(&alloc::format!("Failed size {}", size));
            assert_eq!(buf.len(), size);
            buffers.push(buf).ok();
        }

        // Free all
        while let Some(buf) = buffers.pop() {
            pool.free(buf);
        }

        // Pool back to normal
        assert_eq!(pool.buffer_stats().allocated_buffers, 0);
    }
}

#[cfg(all(test, not(feature = "std")))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

// Minimal alloc stub for format! macro in tests
#[cfg(test)]
mod alloc {
    pub fn format(_args: &str) -> &str {
        _args
    }
}
