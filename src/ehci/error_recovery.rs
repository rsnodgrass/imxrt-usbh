//! EHCI Error Recovery Mechanisms
//!
//! Comprehensive error detection and recovery for USB host operations.
//! Handles various error conditions per EHCI 1.0 specification.

use crate::error::{Result, UsbError};
use core::sync::atomic::{AtomicU32, Ordering};

/// Error recovery statistics
pub struct RecoveryStats {
    /// Total errors encountered
    pub total_errors: AtomicU32,
    /// Successful recoveries
    pub recoveries: AtomicU32,
    /// Failed recoveries
    pub failures: AtomicU32,
    /// Port resets performed
    pub port_resets: AtomicU32,
    /// Controller resets performed
    pub controller_resets: AtomicU32,
}

impl RecoveryStats {
    pub const fn new() -> Self {
        Self {
            total_errors: AtomicU32::new(0),
            recoveries: AtomicU32::new(0),
            failures: AtomicU32::new(0),
            port_resets: AtomicU32::new(0),
            controller_resets: AtomicU32::new(0),
        }
    }

    pub fn record_error(&self) {
        self.total_errors.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_recovery(&self) {
        self.recoveries.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_failure(&self) {
        self.failures.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_port_reset(&self) {
        self.port_resets.fetch_add(1, Ordering::Relaxed);
    }

    pub fn record_controller_reset(&self) {
        self.controller_resets.fetch_add(1, Ordering::Relaxed);
    }

    pub fn success_rate(&self) -> f32 {
        let total = self.total_errors.load(Ordering::Relaxed) as f32;
        let recovered = self.recoveries.load(Ordering::Relaxed) as f32;
        if total > 0.0 {
            recovered / total
        } else {
            0.0
        }
    }
}

/// Error types and their recovery strategies
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorType {
    /// Transaction error (CRC, timeout, etc.)
    TransactionError,
    /// USB protocol stall
    Stall,
    /// Data buffer error
    BufferError,
    /// Babble detection (device sending too much data)
    Babble,
    /// System error (internal controller error)
    SystemError,
    /// Host controller halted unexpectedly
    HostHalted,
    /// Port overcurrent condition
    Overcurrent,
}

/// Recovery strategy for different error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryAction {
    /// Retry the transfer
    Retry,
    /// Reset the endpoint
    ResetEndpoint,
    /// Reset the port
    ResetPort,
    /// Reset the controller
    ResetController,
    /// Abort and report fatal error
    Fatal,
}

impl ErrorType {
    /// Get recommended recovery action for this error
    pub fn recovery_action(&self) -> RecoveryAction {
        match self {
            ErrorType::TransactionError => RecoveryAction::Retry,
            ErrorType::Stall => RecoveryAction::ResetEndpoint,
            ErrorType::BufferError => RecoveryAction::Retry,
            ErrorType::Babble => RecoveryAction::ResetPort,
            ErrorType::SystemError => RecoveryAction::ResetController,
            ErrorType::HostHalted => RecoveryAction::ResetController,
            ErrorType::Overcurrent => RecoveryAction::Fatal,
        }
    }

    /// Maximum retry attempts before escalating
    pub fn max_retries(&self) -> u8 {
        match self {
            ErrorType::TransactionError => 3,
            ErrorType::BufferError => 3,
            ErrorType::Stall => 0, // Don't retry stalls
            _ => 1,
        }
    }
}

/// Error recovery manager
pub struct ErrorRecovery {
    stats: RecoveryStats,
    op_base: usize,
}

impl ErrorRecovery {
    /// Create new error recovery manager
    pub fn new(op_base: usize) -> Self {
        Self {
            stats: RecoveryStats::new(),
            op_base,
        }
    }

    /// Attempt recovery from an error
    pub fn recover(&self, error_type: ErrorType, retry_count: u8) -> Result<RecoveryAction> {
        self.stats.record_error();

        let action = if retry_count < error_type.max_retries() {
            RecoveryAction::Retry
        } else {
            error_type.recovery_action()
        };

        #[cfg(feature = "defmt")]
        defmt::debug!("Error recovery: {:?} -> {:?} (retry {})", error_type, action, retry_count);

        match action {
            RecoveryAction::Retry => {
                self.stats.record_recovery();
                Ok(RecoveryAction::Retry)
            }
            RecoveryAction::ResetEndpoint => {
                self.reset_endpoint()?;
                self.stats.record_recovery();
                Ok(RecoveryAction::ResetEndpoint)
            }
            RecoveryAction::ResetPort => {
                self.reset_port()?;
                self.stats.record_port_reset();
                self.stats.record_recovery();
                Ok(RecoveryAction::ResetPort)
            }
            RecoveryAction::ResetController => {
                self.reset_controller()?;
                self.stats.record_controller_reset();
                self.stats.record_recovery();
                Ok(RecoveryAction::ResetController)
            }
            RecoveryAction::Fatal => {
                self.stats.record_failure();
                Err(UsbError::InvalidState)
            }
        }
    }

    /// Reset an endpoint (clear halt condition)
    fn reset_endpoint(&self) -> Result<()> {
        // Endpoint reset is typically done via CLEAR_FEATURE control transfer
        // This would be called by higher-level code
        #[cfg(feature = "defmt")]
        defmt::debug!("Endpoint reset requested");
        Ok(())
    }

    /// Reset a USB port
    fn reset_port(&self) -> Result<()> {
        #[cfg(feature = "defmt")]
        defmt::debug!("Port reset requested");

        // Safety: Direct hardware access for port reset
        unsafe {
            let portsc = (self.op_base + 0x44) as *mut u32; // PORTSC[0]

            // Read current status
            cortex_m::asm::dmb();
            let mut status = core::ptr::read_volatile(portsc);

            // Set port reset bit (bit 8)
            status |= 1 << 8;
            core::ptr::write_volatile(portsc, status);
            cortex_m::asm::dsb();

            // Wait for reset (minimum 50ms per USB spec)
            cortex_m::asm::delay(50 * 600_000); // 50ms @ 600MHz

            // Clear port reset bit
            status = core::ptr::read_volatile(portsc);
            status &= !(1 << 8);
            core::ptr::write_volatile(portsc, status);
            cortex_m::asm::dsb();

            // Wait for reset recovery (10ms per USB spec)
            cortex_m::asm::delay(10 * 600_000); // 10ms @ 600MHz
        }

        Ok(())
    }

    /// Reset the USB controller
    fn reset_controller(&self) -> Result<()> {
        #[cfg(feature = "defmt")]
        defmt::warn!("Controller reset requested - this will drop all transfers");

        // Safety: Direct hardware access for controller reset
        unsafe {
            let usbcmd = self.op_base as *mut u32;

            // Issue controller reset (bit 1)
            cortex_m::asm::dmb();
            let mut cmd = core::ptr::read_volatile(usbcmd);
            cmd |= 1 << 1; // HC Reset
            core::ptr::write_volatile(usbcmd, cmd);
            cortex_m::asm::dsb();

            // Wait for reset to complete (timeout 250ms)
            let start = cortex_m::peripheral::DWT::cycle_count();
            let timeout = 250 * 600_000; // 250ms @ 600MHz

            loop {
                cortex_m::asm::dmb();
                let cmd = core::ptr::read_volatile(usbcmd);

                // Reset complete when bit 1 clears
                if (cmd & (1 << 1)) == 0 {
                    break;
                }

                if cortex_m::peripheral::DWT::cycle_count().wrapping_sub(start) > timeout {
                    return Err(UsbError::Timeout);
                }

                cortex_m::asm::nop();
            }
        }

        #[cfg(feature = "defmt")]
        defmt::info!("Controller reset complete");

        Ok(())
    }

    /// Check for and handle system errors
    pub fn check_system_errors(&self) -> Result<()> {
        // Safety: Read USB status register
        unsafe {
            let usbsts = (self.op_base + 0x04) as *const u32;
            cortex_m::asm::dmb();
            let status = core::ptr::read_volatile(usbsts);

            // Check for host controller halted (bit 12)
            if (status & (1 << 12)) != 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("Host controller halted unexpectedly");
                return self.recover(ErrorType::HostHalted, 0).map(|_| ());
            }

            // Check for host system error (bit 4)
            if (status & (1 << 4)) != 0 {
                #[cfg(feature = "defmt")]
                defmt::error!("Host system error detected");

                // Clear error bit
                core::ptr::write_volatile(usbsts as *mut u32, 1 << 4);
                cortex_m::asm::dsb();

                return self.recover(ErrorType::SystemError, 0).map(|_| ());
            }
        }

        Ok(())
    }

    /// Get recovery statistics
    pub fn statistics(&self) -> &RecoveryStats {
        &self.stats
    }
}

/// Parse qTD token to detect errors
pub fn parse_qtd_errors(token: u32) -> Option<ErrorType> {
    // Check error bits in token (bits 6-3)
    if (token & (1 << 6)) != 0 {
        Some(ErrorType::Stall) // Halted
    } else if (token & (1 << 5)) != 0 {
        Some(ErrorType::BufferError) // Data buffer error
    } else if (token & (1 << 4)) != 0 {
        Some(ErrorType::Babble) // Babble detected
    } else if (token & (1 << 3)) != 0 {
        Some(ErrorType::TransactionError) // Transaction error
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_error_type_recovery() {
        assert_eq!(ErrorType::TransactionError.recovery_action(), RecoveryAction::Retry);
        assert_eq!(ErrorType::Stall.recovery_action(), RecoveryAction::ResetEndpoint);
        assert_eq!(ErrorType::Babble.recovery_action(), RecoveryAction::ResetPort);
    }

    #[test]
    fn test_max_retries() {
        assert_eq!(ErrorType::TransactionError.max_retries(), 3);
        assert_eq!(ErrorType::Stall.max_retries(), 0);
    }

    #[test]
    fn test_qtd_error_parsing() {
        assert_eq!(parse_qtd_errors(1 << 6), Some(ErrorType::Stall));
        assert_eq!(parse_qtd_errors(1 << 5), Some(ErrorType::BufferError));
        assert_eq!(parse_qtd_errors(1 << 4), Some(ErrorType::Babble));
        assert_eq!(parse_qtd_errors(1 << 3), Some(ErrorType::TransactionError));
        assert_eq!(parse_qtd_errors(0), None);
    }
}
