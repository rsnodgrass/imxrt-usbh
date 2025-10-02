//! Performance monitoring and diagnostics
//! 
//! Provides zero-cost performance counters and diagnostics for optimization

use core::sync::atomic::{AtomicU32, Ordering};

/// Performance counters for USB operations
pub struct PerfCounters {
    /// Total number of transfers initiated
    pub transfers_total: AtomicU32,
    
    /// Number of successful transfers
    pub transfers_success: AtomicU32,
    
    /// Number of failed transfers
    pub transfers_failed: AtomicU32,
    
    /// Number of transfer retries
    pub transfer_retries: AtomicU32,
    
    /// Total bytes transferred (lower 32 bits)
    pub bytes_transferred: AtomicU32,
    
    /// Interrupt processing time (in CPU cycles)
    pub interrupt_cycles: AtomicU32,
    
    /// Number of interrupts processed
    pub interrupt_count: AtomicU32,
    
    /// Peak interrupt processing time
    pub interrupt_peak_cycles: AtomicU32,
}

impl PerfCounters {
    /// Create new performance counter set
    pub const fn new() -> Self {
        Self {
            transfers_total: AtomicU32::new(0),
            transfers_success: AtomicU32::new(0),
            transfers_failed: AtomicU32::new(0),
            transfer_retries: AtomicU32::new(0),
            bytes_transferred: AtomicU32::new(0),
            interrupt_cycles: AtomicU32::new(0),
            interrupt_count: AtomicU32::new(0),
            interrupt_peak_cycles: AtomicU32::new(0),
        }
    }
    
    /// Record a successful transfer
    #[inline(always)]
    pub fn record_transfer_success(&self, bytes: usize) {
        self.transfers_total.fetch_add(1, Ordering::Relaxed);
        self.transfers_success.fetch_add(1, Ordering::Relaxed);

        // Use saturating add to prevent silent overflow
        let _ = self.bytes_transferred.fetch_update(
            Ordering::Relaxed,
            Ordering::Relaxed,
            |current| current.checked_add(bytes as u32).or(Some(u32::MAX))
        );
    }
    
    /// Record a failed transfer
    #[inline(always)]
    pub fn record_transfer_failure(&self) {
        self.transfers_total.fetch_add(1, Ordering::Relaxed);
        self.transfers_failed.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Record a transfer retry
    #[inline(always)]
    pub fn record_retry(&self) {
        self.transfer_retries.fetch_add(1, Ordering::Relaxed);
    }
    
    /// Record interrupt processing time
    #[inline(always)]
    pub fn record_interrupt(&self, cycles: u32) {
        self.interrupt_count.fetch_add(1, Ordering::Relaxed);
        self.interrupt_cycles.fetch_add(cycles, Ordering::Relaxed);
        
        // Update peak if this interrupt took longer
        let current_peak = self.interrupt_peak_cycles.load(Ordering::Relaxed);
        if cycles > current_peak {
            let _ = self.interrupt_peak_cycles.compare_exchange_weak(
                current_peak,
                cycles,
                Ordering::Relaxed,
                Ordering::Relaxed,
            );
        }
    }
    
    /// Get current statistics snapshot
    pub fn snapshot(&self) -> PerfSnapshot {
        PerfSnapshot {
            transfers_total: self.transfers_total.load(Ordering::Relaxed),
            transfers_success: self.transfers_success.load(Ordering::Relaxed),
            transfers_failed: self.transfers_failed.load(Ordering::Relaxed),
            transfer_retries: self.transfer_retries.load(Ordering::Relaxed),
            bytes_transferred: self.bytes_transferred.load(Ordering::Relaxed),
            interrupt_count: self.interrupt_count.load(Ordering::Relaxed),
            interrupt_total_cycles: self.interrupt_cycles.load(Ordering::Relaxed),
            interrupt_peak_cycles: self.interrupt_peak_cycles.load(Ordering::Relaxed),
        }
    }
    
    /// Reset all counters
    pub fn reset(&self) {
        self.transfers_total.store(0, Ordering::Relaxed);
        self.transfers_success.store(0, Ordering::Relaxed);
        self.transfers_failed.store(0, Ordering::Relaxed);
        self.transfer_retries.store(0, Ordering::Relaxed);
        self.bytes_transferred.store(0, Ordering::Relaxed);
        self.interrupt_cycles.store(0, Ordering::Relaxed);
        self.interrupt_count.store(0, Ordering::Relaxed);
        self.interrupt_peak_cycles.store(0, Ordering::Relaxed);
    }
}

/// Immutable snapshot of performance counters
#[derive(Debug, Clone, Copy)]
#[allow(missing_docs)]
pub struct PerfSnapshot {
    pub transfers_total: u32,
    pub transfers_success: u32,
    pub transfers_failed: u32,
    pub transfer_retries: u32,
    pub bytes_transferred: u32,
    pub interrupt_count: u32,
    pub interrupt_total_cycles: u32,
    pub interrupt_peak_cycles: u32,
}

impl PerfSnapshot {
    /// Calculate success rate as percentage
    pub fn success_rate(&self) -> f32 {
        if self.transfers_total == 0 {
            return 100.0;
        }
        (self.transfers_success as f32 / self.transfers_total as f32) * 100.0
    }
    
    /// Calculate average interrupt processing time in cycles
    pub fn avg_interrupt_cycles(&self) -> f32 {
        if self.interrupt_count == 0 {
            return 0.0;
        }
        self.interrupt_total_cycles as f32 / self.interrupt_count as f32
    }
    
    /// Calculate throughput in bytes per second (needs timing info)
    pub fn throughput_bps(&self, elapsed_ms: u32) -> u32 {
        if elapsed_ms == 0 {
            return 0;
        }
        (self.bytes_transferred * 1000) / elapsed_ms
    }
    
    /// Check if performance is within acceptable bounds
    pub fn is_healthy(&self) -> bool {
        let success_rate = self.success_rate();
        let avg_interrupt_us = self.avg_interrupt_cycles() / 600.0; // Assuming 600MHz CPU
        
        // Performance is healthy if:
        // - Success rate > 95%
        // - Average interrupt time < 10μs  
        // - Peak interrupt time < 50μs
        success_rate > 95.0 && 
        avg_interrupt_us < 10.0 && 
        (self.interrupt_peak_cycles as f32 / 600.0) < 50.0
    }
}

/// RAII timer for measuring interrupt processing time
pub struct InterruptTimer {
    start_cycles: u32,
    counters: &'static PerfCounters,
}

impl InterruptTimer {
    /// Start timing an interrupt
    #[inline(always)]
    pub fn start(counters: &'static PerfCounters) -> Self {
        let start_cycles = cortex_m::peripheral::DWT::cycle_count();
        Self {
            start_cycles,
            counters,
        }
    }
}

impl Drop for InterruptTimer {
    #[inline(always)]
    fn drop(&mut self) {
        let end_cycles = cortex_m::peripheral::DWT::cycle_count();
        let elapsed = end_cycles.wrapping_sub(self.start_cycles);
        self.counters.record_interrupt(elapsed);
    }
}

/// Global performance counters instance
pub static PERF_COUNTERS: PerfCounters = PerfCounters::new();

/// Macro for easy interrupt timing
#[macro_export]
macro_rules! time_interrupt {
    ($body:block) => {
        {
            let _timer = $crate::perf::InterruptTimer::start(&$crate::perf::PERF_COUNTERS);
            $body
        }
    };
}

#[cfg(feature = "defmt")]
impl defmt::Format for PerfSnapshot {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "PerfSnapshot {{ success_rate: {}%, avg_interrupt: {}μs, peak_interrupt: {}μs, throughput: {}KB/s }}",
            self.success_rate() as u32,
            (self.avg_interrupt_cycles() / 600.0) as u32,
            (self.interrupt_peak_cycles as f32 / 600.0) as u32,
            self.bytes_transferred / 1024
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_perf_counters() {
        let counters = PerfCounters::new();
        
        // Record some operations
        counters.record_transfer_success(1024);
        counters.record_transfer_failure();
        counters.record_retry();
        counters.record_interrupt(1200); // 2μs at 600MHz
        
        let snapshot = counters.snapshot();
        
        assert_eq!(snapshot.transfers_total, 2);
        assert_eq!(snapshot.transfers_success, 1);
        assert_eq!(snapshot.transfers_failed, 1);
        assert_eq!(snapshot.transfer_retries, 1);
        assert_eq!(snapshot.bytes_transferred, 1024);
        assert_eq!(snapshot.interrupt_count, 1);
        assert_eq!(snapshot.success_rate(), 50.0);
    }
}