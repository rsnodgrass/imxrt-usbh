//! Safety monitoring and protection mechanisms
//! 
//! Provides stack overflow detection, bounds checking, and runtime safety validation

use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};

/// Stack monitoring for overflow detection
pub struct StackMonitor {
    /// Stack base address (top of stack)
    stack_base: u32,
    /// Stack size in bytes
    stack_size: u32,
    /// High water mark for stack usage
    high_water_mark: AtomicU32,
    /// Stack overflow detected flag
    overflow_detected: AtomicBool,
    /// Guard region size in bytes
    guard_size: u32,
}

impl StackMonitor {
    /// Create new stack monitor
    /// 
    /// # Safety
    /// 
    /// Caller must provide correct stack boundaries
    pub const unsafe fn new(stack_base: u32, stack_size: u32) -> Self {
        Self {
            stack_base,
            stack_size,
            high_water_mark: AtomicU32::new(0),
            overflow_detected: AtomicBool::new(false),
            guard_size: 256, // 256-byte guard region
        }
    }
    
    /// Initialize stack canary pattern for detection
    pub unsafe fn init_canary(&self) {
        let stack_end = self.stack_base - self.stack_size;
        let current_sp = cortex_m::register::msp::read() as u32;
        
        // Fill unused stack with canary pattern
        let canary_start = stack_end as *mut u32;
        let canary_end = (current_sp - self.guard_size) as *mut u32;
        let canary_pattern = 0xDEADBEEF;
        
        unsafe {
            let mut ptr = canary_start;
            while ptr < canary_end {
                core::ptr::write_volatile(ptr, canary_pattern);
                ptr = ptr.add(1);
            }
        }
    }
    
    /// Check for stack overflow
    #[inline(always)]
    pub fn check_stack(&self) -> bool {
        let current_sp = cortex_m::register::msp::read() as u32;
        let stack_end = self.stack_base - self.stack_size;
        
        // Check if we're in the guard region
        if current_sp <= stack_end + self.guard_size {
            self.overflow_detected.store(true, Ordering::Release);
            
            #[cfg(feature = "defmt")]
            defmt::error!("Stack overflow! SP={:#x}, limit={:#x}", 
                         current_sp, stack_end + self.guard_size);
            
            return true;
        }
        
        // Update high water mark
        let stack_used = self.stack_base - current_sp;
        let current_high = self.high_water_mark.load(Ordering::Relaxed);
        if stack_used > current_high {
            self.high_water_mark.store(stack_used, Ordering::Relaxed);
        }
        
        false
    }
    
    /// Get stack usage statistics
    pub fn get_usage(&self) -> StackUsage {
        StackUsage {
            current: self.stack_base - cortex_m::register::msp::read() as u32,
            high_water_mark: self.high_water_mark.load(Ordering::Relaxed),
            total_size: self.stack_size,
            overflow_detected: self.overflow_detected.load(Ordering::Relaxed),
        }
    }
    
    /// Check canary values for corruption
    pub unsafe fn verify_canary(&self) -> bool {
        let stack_end = self.stack_base - self.stack_size;
        let canary_start = stack_end as *const u32;
        let canary_end = (stack_end + self.guard_size) as *const u32;
        let canary_pattern = 0xDEADBEEF;
        
        unsafe {
            let mut ptr = canary_start;
            while ptr < canary_end {
                if core::ptr::read_volatile(ptr) != canary_pattern {
                    #[cfg(feature = "defmt")]
                    defmt::error!("Stack canary corrupted at {:#x}", ptr as u32);
                    return false;
                }
                ptr = ptr.add(1);
            }
        }
        true
    }
}

/// Stack usage statistics
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StackUsage {
    /// Current stack usage in bytes
    pub current: u32,
    /// Maximum stack usage observed
    pub high_water_mark: u32,
    /// Total stack size
    pub total_size: u32,
    /// Overflow detected flag
    pub overflow_detected: bool,
}

impl StackUsage {
    /// Get stack usage percentage
    pub fn usage_percent(&self) -> u8 {
        ((self.high_water_mark as u64 * 100) / self.total_size as u64) as u8
    }
    
    /// Check if stack usage is critical (>75%)
    pub fn is_critical(&self) -> bool {
        self.usage_percent() > 75
    }
}

/// DMA buffer bounds checking
pub struct BoundsChecker;

impl BoundsChecker {
    /// Validate buffer pointer and length
    pub fn validate_buffer(ptr: *const u8, len: usize) -> Result<()> {
        if ptr.is_null() {
            return Err(UsbError::InvalidParameter);
        }
        
        let addr = ptr as usize;
        
        // Check for address overflow
        if addr.checked_add(len).is_none() {
            return Err(UsbError::BufferOverflow);
        }
        
        // Additional validation can be added here
        Ok(())
    }
    
    /// Validate DMA buffer is in correct region
    pub fn validate_dma_buffer(addr: usize, _len: usize) -> Result<()> {
        // For now, just validate alignment and non-null
        // In production, would check against actual DMA region bounds
        if addr == 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        // Check alignment
        if addr & (crate::dma::DMA_ALIGNMENT - 1) != 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        Ok(())
    }
    
    /// Validate alignment requirements
    pub fn validate_alignment(addr: usize, alignment: usize) -> Result<()> {
        // Check alignment is power of 2
        if alignment == 0 || (alignment & (alignment - 1)) != 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        // Check address alignment
        if addr & (alignment - 1) != 0 {
            return Err(UsbError::InvalidParameter);
        }
        
        Ok(())
    }
}

/// Critical section with timeout protection
pub struct TimedCriticalSection {
    start_cycles: u32,
    max_cycles: u32,
}

impl TimedCriticalSection {
    /// Create new timed critical section
    pub fn new(max_duration_us: u32) -> Self {
        Self {
            start_cycles: cortex_m::peripheral::DWT::cycle_count(),
            max_cycles: max_duration_us * 600, // 600MHz CPU
        }
    }
    
    /// Execute function in critical section with timeout
    pub fn execute<F, R>(&self, f: F) -> Result<R>
    where
        F: FnOnce() -> R,
    {
        let result = cortex_m::interrupt::free(|_| f());
        
        let elapsed = cortex_m::peripheral::DWT::cycle_count()
            .wrapping_sub(self.start_cycles);
        
        if elapsed > self.max_cycles {
            #[cfg(feature = "defmt")]
            defmt::error!("Critical section timeout: {} cycles (max {})", 
                         elapsed, self.max_cycles);
            return Err(UsbError::Timeout);
        }
        
        Ok(result)
    }
}

/// Deadline monitoring for real-time operations
pub struct DeadlineMonitor {
    deadline_cycles: u32,
    start_cycles: u32,
    missed_deadlines: AtomicU32,
}

impl DeadlineMonitor {
    /// Create new deadline monitor
    pub fn new(deadline_us: u32) -> Self {
        Self {
            deadline_cycles: deadline_us * 600, // 600MHz
            start_cycles: cortex_m::peripheral::DWT::cycle_count(),
            missed_deadlines: AtomicU32::new(0),
        }
    }
    
    /// Check if deadline was met
    pub fn check_deadline(&self) -> bool {
        let elapsed = cortex_m::peripheral::DWT::cycle_count()
            .wrapping_sub(self.start_cycles);
        
        if elapsed > self.deadline_cycles {
            self.missed_deadlines.fetch_add(1, Ordering::Relaxed);
            
            #[cfg(feature = "defmt")]
            defmt::warn!("Deadline missed: {}us > {}us", 
                        elapsed / 600, self.deadline_cycles / 600);
            
            false
        } else {
            true
        }
    }
    
    /// Get number of missed deadlines
    pub fn missed_count(&self) -> u32 {
        self.missed_deadlines.load(Ordering::Relaxed)
    }
}

/// Global stack monitor instance
/// 
/// # Safety
/// 
/// Must be initialized with correct stack boundaries at startup
pub static mut STACK_MONITOR: StackMonitor = unsafe {
    // Default values - must be updated at runtime
    StackMonitor::new(0x2000_8000, 0x4000) // 16KB stack at 0x20004000-0x20008000
};

/// Initialize safety monitoring
/// 
/// # Safety
/// 
/// Must be called once at system startup with correct stack boundaries
pub unsafe fn init_safety_monitoring(stack_base: u32, stack_size: u32) {
    unsafe {
        // Update stack monitor with actual values
        STACK_MONITOR.stack_base = stack_base;
        STACK_MONITOR.stack_size = stack_size;
        
        // Initialize stack canary
        STACK_MONITOR.init_canary();
    }
    
    // Enable DWT cycle counter for timing
    let mut peripherals = cortex_m::Peripherals::take().unwrap();
    peripherals.DCB.enable_trace();
    peripherals.DWT.enable_cycle_counter();
    
    #[cfg(feature = "defmt")]
    defmt::info!("Safety monitoring initialized: stack {:#x}..{:#x}", 
                 stack_base - stack_size, stack_base);
}

/// Check all safety monitors
#[inline(always)]
pub fn check_safety() -> bool {
    unsafe {
        // Check stack overflow
        if STACK_MONITOR.check_stack() {
            return false;
        }
        
        // Additional safety checks can be added here
        true
    }
}

/// Macro for adding safety checks to functions
#[macro_export]
macro_rules! with_safety_check {
    ($body:expr) => {{
        if !$crate::safety::check_safety() {
            return Err($crate::error::UsbError::InvalidState);
        }
        let result = $body;
        if !$crate::safety::check_safety() {
            return Err($crate::error::UsbError::InvalidState);
        }
        result
    }};
}