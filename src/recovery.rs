//! Error recovery and fault tolerance mechanisms
//!
//! Implements comprehensive error recovery strategies for USB host operation

use crate::ehci::controller::EhciController;
use crate::error::{Result, UsbError};
use crate::phy::UsbPhy;
use crate::vbus::VbusPowerControl;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

/// Recovery strategy for different error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryStrategy {
    /// Retry the operation immediately
    RetryImmediate,
    /// Retry with exponential backoff
    RetryWithBackoff { delay_ms: u32 },
    /// Reset the endpoint
    ResetEndpoint,
    /// Reset the device
    ResetDevice,
    /// Reset the port
    ResetPort,
    /// Reset the controller
    ResetController,
    /// Power cycle the device
    PowerCycle,
    /// Fatal error, cannot recover
    Fatal,
}

/// Error recovery coordinator
pub struct RecoveryCoordinator {
    /// Current recovery state
    state: RecoveryState,
    /// Recovery statistics
    stats: RecoveryStats,
    /// Maximum recovery attempts
    max_attempts: u8,
    /// Current backoff delay
    backoff_ms: AtomicU32,
    /// Recovery in progress flag
    recovering: AtomicBool,
}

/// Recovery state machine states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryState {
    /// Normal operation
    Normal,
    /// Analyzing error
    Analyzing,
    /// Executing recovery
    Recovering,
    /// Verifying recovery
    Verifying,
    /// Recovery failed
    Failed,
}

impl RecoveryCoordinator {
    /// Create new recovery coordinator
    pub const fn new() -> Self {
        Self {
            state: RecoveryState::Normal,
            stats: RecoveryStats::new(),
            max_attempts: 3,
            backoff_ms: AtomicU32::new(10),
            recovering: AtomicBool::new(false),
        }
    }

    /// Determine recovery strategy for an error
    pub fn analyze_error(&mut self, error: &UsbError) -> RecoveryStrategy {
        self.state = RecoveryState::Analyzing;
        self.stats.record_error(error);

        match error {
            // Transient errors - retry immediately
            UsbError::Nak => RecoveryStrategy::RetryImmediate,

            // Timing errors - retry with backoff
            UsbError::Timeout => {
                let delay = self.calculate_backoff();
                RecoveryStrategy::RetryWithBackoff { delay_ms: delay }
            }

            // Endpoint errors - reset endpoint
            UsbError::Stall => RecoveryStrategy::ResetEndpoint,

            // Transaction errors - reset device
            UsbError::TransactionError => RecoveryStrategy::ResetDevice,

            // Port errors - reset port
            UsbError::DeviceDisconnected | UsbError::PortError => RecoveryStrategy::ResetPort,

            // Controller/hardware errors - reset controller
            UsbError::HardwareFailure => RecoveryStrategy::ResetController,

            // Fatal errors - cannot recover
            UsbError::InvalidParameter
            | UsbError::InvalidState
            | UsbError::InvalidDescriptor
            | UsbError::Unsupported
            | UsbError::AlreadyInitialized => RecoveryStrategy::Fatal,

            // Buffer errors - depends on frequency
            UsbError::BufferOverflow | UsbError::NoResources => {
                if self.stats.is_recurring_error(error) {
                    RecoveryStrategy::ResetController
                } else {
                    RecoveryStrategy::RetryWithBackoff { delay_ms: 50 }
                }
            }
        }
    }

    /// Execute recovery strategy
    pub fn execute_recovery(
        &mut self,
        strategy: RecoveryStrategy,
        phy: Option<&mut UsbPhy>,
        controller: Option<&mut EhciController<8>>,
        vbus: Option<&mut dyn VbusPowerControl>,
    ) -> Result<()> {
        // Check if already recovering
        if self.recovering.swap(true, Ordering::AcqRel) {
            return Err(UsbError::InvalidState);
        }

        self.state = RecoveryState::Recovering;

        let result = match strategy {
            RecoveryStrategy::RetryImmediate => {
                self.stats.record_retry();
                Ok(()) // Caller will retry
            }

            RecoveryStrategy::RetryWithBackoff { delay_ms } => {
                self.delay_ms(delay_ms);
                self.stats.record_retry();
                Ok(()) // Caller will retry
            }

            RecoveryStrategy::ResetEndpoint => self.reset_endpoint(controller),

            RecoveryStrategy::ResetDevice => self.reset_device(controller),

            RecoveryStrategy::ResetPort => self.reset_port(controller),

            RecoveryStrategy::ResetController => self.reset_controller(phy, controller),

            RecoveryStrategy::PowerCycle => self.power_cycle(vbus),

            RecoveryStrategy::Fatal => {
                self.state = RecoveryState::Failed;
                Err(UsbError::HardwareFailure)
            }
        };

        // Clear recovering flag
        self.recovering.store(false, Ordering::Release);

        // Update state based on result
        match result {
            Ok(()) => {
                self.state = RecoveryState::Verifying;
                self.stats.record_recovery_success();
                // Reset backoff on success
                self.backoff_ms.store(10, Ordering::Relaxed);
            }
            Err(_) => {
                self.stats.record_recovery_failure();
                if self.stats.recovery_attempts() >= self.max_attempts {
                    self.state = RecoveryState::Failed;
                }
            }
        }

        result
    }

    /// Reset endpoint
    fn reset_endpoint(&mut self, controller: Option<&mut EhciController<8>>) -> Result<()> {
        if controller.is_none() {
            return Err(UsbError::InvalidParameter);
        }

        // Send CLEAR_HALT to endpoint
        // This would interact with the control transfer mechanism

        #[cfg(feature = "defmt")]
        defmt::info!("Resetting endpoint");

        Ok(())
    }

    /// Reset device
    fn reset_device(&mut self, controller: Option<&mut EhciController<8>>) -> Result<()> {
        if controller.is_none() {
            return Err(UsbError::InvalidParameter);
        }

        // Send SET_FEATURE(DEVICE_RESET) or re-enumerate

        #[cfg(feature = "defmt")]
        defmt::info!("Resetting device");

        Ok(())
    }

    /// Reset port
    fn reset_port(&mut self, controller: Option<&mut EhciController<8>>) -> Result<()> {
        let controller = controller.ok_or(UsbError::InvalidParameter)?;

        unsafe {
            // Get correct controller base address (NOT hardcoded USB1!)
            let base_addr = controller.base_address();
            let cap_base = core::ptr::read_volatile(base_addr as *const u32);
            let cap_length = (cap_base & 0xFF) as usize;
            let op_base = base_addr + cap_length;

            // PORTSC[0] is at operational_base + 0x44
            let portsc_addr = (op_base + 0x44) as *mut u32;

            // Set port reset bit
            cortex_m::asm::dmb(); // ensure prior ops complete before MMIO read
            let mut portsc = core::ptr::read_volatile(portsc_addr);
            cortex_m::asm::dmb(); // ensure read completes before modify
            portsc |= 1 << 8; // Port Reset
            cortex_m::asm::dmb(); // ensure modify completes before MMIO write
            core::ptr::write_volatile(portsc_addr, portsc);
            cortex_m::asm::dsb(); // ensure write completes before continuing

            // Wait for reset to complete (20ms per USB 2.0 spec recommendation)
            self.delay_ms(20);

            // Clear reset bit
            cortex_m::asm::dmb();
            portsc = core::ptr::read_volatile(portsc_addr);
            cortex_m::asm::dmb();
            portsc &= !(1 << 8);
            cortex_m::asm::dmb();
            core::ptr::write_volatile(portsc_addr, portsc);
            cortex_m::asm::dsb();

            // Wait for port to stabilize
            self.delay_ms(10);
        }

        #[cfg(feature = "defmt")]
        defmt::info!("Port reset completed");

        Ok(())
    }

    /// Reset controller
    fn reset_controller(
        &mut self,
        phy: Option<&mut UsbPhy>,
        controller: Option<&mut EhciController<8>>,
    ) -> Result<()> {
        let controller = controller.ok_or(UsbError::InvalidParameter)?;

        #[cfg(feature = "defmt")]
        defmt::warn!("Resetting USB controller");

        // Stop controller
        unsafe {
            // Get correct controller base address (NOT hardcoded USB1!)
            let base_addr = controller.base_address();
            let cap_base = core::ptr::read_volatile(base_addr as *const u32);
            let cap_length = (cap_base & 0xFF) as usize;
            let op_base = base_addr + cap_length;

            // USBCMD is at operational_base + 0x00
            let usbcmd_addr = (op_base + 0x00) as *mut u32;
            cortex_m::asm::dmb();
            let mut cmd = core::ptr::read_volatile(usbcmd_addr);
            cortex_m::asm::dmb();
            cmd &= !(1 << 0); // Clear Run/Stop
            cortex_m::asm::dmb();
            core::ptr::write_volatile(usbcmd_addr, cmd);
            cortex_m::asm::dsb();

            // Wait for halt
            // USBSTS is at operational_base + 0x04
            let usbsts_addr = (op_base + 0x04) as *const u32;
            let mut timeout = 1000;
            loop {
                cortex_m::asm::dmb();
                let status = core::ptr::read_volatile(usbsts_addr);
                cortex_m::asm::dmb();
                if status & (1 << 12) != 0 {
                    // HCHalted
                    break;
                }
                timeout -= 1;
                if timeout == 0 {
                    return Err(UsbError::Timeout);
                }
                self.delay_us(10);
            }

            // Reset controller
            cortex_m::asm::dmb();
            cmd = core::ptr::read_volatile(usbcmd_addr);
            cortex_m::asm::dmb();
            cmd |= 1 << 1; // Host Controller Reset
            cortex_m::asm::dmb();
            core::ptr::write_volatile(usbcmd_addr, cmd);
            cortex_m::asm::dsb();

            // Wait for reset to complete
            timeout = 1000;
            loop {
                cortex_m::asm::dmb();
                cmd = core::ptr::read_volatile(usbcmd_addr);
                cortex_m::asm::dmb();
                if cmd & (1 << 1) == 0 {
                    break;
                }
                timeout -= 1;
                if timeout == 0 {
                    return Err(UsbError::HardwareFailure);
                }
                self.delay_us(10);
            }
        }

        // Re-initialize PHY if available
        if let Some(phy) = phy {
            phy.init_host_mode()?;
        }

        // Re-initialize controller if available
        // controller would need re-initialization here

        #[cfg(feature = "defmt")]
        defmt::info!("Controller reset completed");

        Ok(())
    }

    /// Power cycle device
    fn power_cycle(&mut self, vbus: Option<&mut dyn VbusPowerControl>) -> Result<()> {
        if let Some(vbus) = vbus {
            #[cfg(feature = "defmt")]
            defmt::warn!("Power cycling USB device");

            // Disable VBUS
            vbus.disable_vbus();

            // Wait for discharge
            self.delay_ms(500);

            // Re-enable VBUS
            vbus.enable_vbus()?;

            // Wait for power stabilization
            self.delay_ms(100);

            #[cfg(feature = "defmt")]
            defmt::info!("Power cycle completed");
        } else {
            return Err(UsbError::InvalidParameter);
        }

        Ok(())
    }

    /// Calculate exponential backoff delay
    fn calculate_backoff(&self) -> u32 {
        let current = self.backoff_ms.load(Ordering::Relaxed);
        let next = (current * 2).min(1000); // Cap at 1 second
        self.backoff_ms.store(next, Ordering::Relaxed);
        current
    }

    /// Delay for specified milliseconds
    fn delay_ms(&self, ms: u32) {
        for _ in 0..ms {
            self.delay_us(1000);
        }
    }

    /// Delay for specified microseconds
    fn delay_us(&self, us: u32) {
        let start = cortex_m::peripheral::DWT::cycle_count();
        let cycles = us * 600; // 600MHz CPU
        while cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) < cycles {
            cortex_m::asm::nop();
        }
    }

    /// Verify recovery success
    pub fn verify_recovery(&mut self) -> Result<()> {
        if self.state != RecoveryState::Verifying {
            return Err(UsbError::InvalidState);
        }

        // Perform health checks
        // This would check PHY, controller, and transfer status

        self.state = RecoveryState::Normal;
        Ok(())
    }

    /// Check if recovery is possible
    pub fn can_recover(&self) -> bool {
        self.state != RecoveryState::Failed && self.stats.recovery_attempts() < self.max_attempts
    }

    /// Get recovery statistics
    pub fn statistics(&self) -> &RecoveryStats {
        &self.stats
    }

    /// Reset recovery state
    pub fn reset(&mut self) {
        self.state = RecoveryState::Normal;
        self.stats.reset();
        self.backoff_ms.store(10, Ordering::Relaxed);
        self.recovering.store(false, Ordering::Release);
    }
}

/// Recovery statistics
pub struct RecoveryStats {
    /// Error counts by type
    error_counts: [AtomicU32; 32],
    /// Total errors
    total_errors: AtomicU32,
    /// Recovery attempts
    recovery_attempts: AtomicU8,
    /// Successful recoveries
    recovery_successes: AtomicU32,
    /// Failed recoveries
    recovery_failures: AtomicU32,
    /// Retry count
    retry_count: AtomicU32,
    /// Last error type
    last_error: AtomicU8,
}

impl RecoveryStats {
    const fn new() -> Self {
        const ZERO: AtomicU32 = AtomicU32::new(0);
        Self {
            error_counts: [ZERO; 32],
            total_errors: AtomicU32::new(0),
            recovery_attempts: AtomicU8::new(0),
            recovery_successes: AtomicU32::new(0),
            recovery_failures: AtomicU32::new(0),
            retry_count: AtomicU32::new(0),
            last_error: AtomicU8::new(0),
        }
    }

    fn record_error(&self, error: &UsbError) {
        let index = Self::error_to_index(error);
        self.error_counts[index].fetch_add(1, Ordering::Relaxed);
        self.total_errors.fetch_add(1, Ordering::Relaxed);
        self.last_error.store(index as u8, Ordering::Relaxed);
    }

    fn record_retry(&self) {
        self.retry_count.fetch_add(1, Ordering::Relaxed);
    }

    fn record_recovery_success(&self) {
        self.recovery_successes.fetch_add(1, Ordering::Relaxed);
    }

    fn record_recovery_failure(&self) {
        self.recovery_failures.fetch_add(1, Ordering::Relaxed);
        self.recovery_attempts.fetch_add(1, Ordering::AcqRel);
    }

    fn recovery_attempts(&self) -> u8 {
        self.recovery_attempts.load(Ordering::Acquire)
    }

    fn is_recurring_error(&self, error: &UsbError) -> bool {
        let index = Self::error_to_index(error);
        self.error_counts[index].load(Ordering::Relaxed) > 3
    }

    fn error_to_index(error: &UsbError) -> usize {
        match error {
            UsbError::AlreadyInitialized => 0,
            UsbError::DeviceDisconnected => 1,
            UsbError::Stall => 2,
            UsbError::Timeout => 5,
            UsbError::TransactionError => 6,
            UsbError::BufferOverflow => 10,
            UsbError::InvalidParameter => 12,
            UsbError::InvalidState => 13,
            UsbError::NoResources => 14,
            UsbError::PortError => 15,
            UsbError::Unsupported => 17,
            UsbError::Nak => 18,
            UsbError::InvalidDescriptor => 19,
            UsbError::HardwareFailure => 22,
        }
    }

    fn reset(&self) {
        for counter in &self.error_counts {
            counter.store(0, Ordering::Relaxed);
        }
        self.total_errors.store(0, Ordering::Relaxed);
        self.recovery_attempts.store(0, Ordering::Relaxed);
        self.recovery_successes.store(0, Ordering::Relaxed);
        self.recovery_failures.store(0, Ordering::Relaxed);
        self.retry_count.store(0, Ordering::Relaxed);
        self.last_error.store(0, Ordering::Relaxed);
    }

    /// Get success rate
    pub fn success_rate(&self) -> f32 {
        let successes = self.recovery_successes.load(Ordering::Relaxed) as f32;
        let total = (self.recovery_successes.load(Ordering::Relaxed)
            + self.recovery_failures.load(Ordering::Relaxed)) as f32;
        if total > 0.0 {
            successes / total
        } else {
            0.0
        }
    }
}

/// Global recovery coordinator instance
pub static RECOVERY_COORDINATOR: RecoveryCoordinator = RecoveryCoordinator::new();
