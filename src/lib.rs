#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs)]

//! USB Host driver for i.MX RT1062 (Teensy 4.x)
//!
//! This driver provides USB host functionality for the i.MX RT1062 microcontroller,
//! specifically targeting Teensy 4.0 and 4.1 boards.
//!
//! # Learning Path
//!
//! This is a low-level USB host driver. For learning and getting started:
//!
//! 1. **Start with examples**: Check the `examples/` directory for educational code
//! 2. **Understand transfers**: Learn about [`transfer`] modules for different USB transfer types
//! 3. **Advanced control**: Use [`ehci`] module for direct hardware control
//!
//! # Core Components
//!
//! - [`ehci`] - EHCI USB host controller interface
//! - [`phy`] - USB PHY management and calibration
//! - [`transfer`] - USB transfer types (bulk, interrupt, control, isochronous)
//! - [`dma`] - DMA buffer management with cache coherency
//! - [`enumeration`] - USB device enumeration and descriptor parsing
//! - [`error`] - Comprehensive error types with recovery guidance

#[cfg(feature = "defmt")]
use defmt as _;

/// Hardware timing constants and utilities for i.MX RT1062 USB operations
///
/// All timing values are derived from:
/// - i.MX RT1060 Reference Manual
/// - USB 2.0 Specification
/// - ARM Cortex-M7 operating at 600MHz
pub mod timing {
    /// CPU frequency for i.MX RT1062 (Teensy 4.x) in MHz
    /// Default is 600MHz. Override with compile-time configuration if needed.
    pub const CPU_FREQ_MHZ: u32 = 600;

    /// Convert microseconds to CPU cycles
    #[inline(always)]
    pub const fn us_to_cycles(us: u32) -> u32 {
        us * CPU_FREQ_MHZ
    }

    /// Convert milliseconds to CPU cycles
    #[inline(always)]
    pub const fn ms_to_cycles(ms: u32) -> u32 {
        ms * CPU_FREQ_MHZ * 1000
    }

    /// Convert CPU cycles to microseconds
    #[inline(always)]
    pub const fn cycles_to_us(cycles: u32) -> u32 {
        cycles / CPU_FREQ_MHZ
    }

    /// Convert CPU cycles to milliseconds
    #[inline(always)]
    pub const fn cycles_to_ms(cycles: u32) -> u32 {
        cycles / (CPU_FREQ_MHZ * 1000)
    }

    // === USB PHY Timing (from RM 66.5.1) ===

    /// PLL lock timeout (10ms max per RM 14.5.3)
    pub const PLL_LOCK_TIMEOUT_US: u32 = 10_000;

    /// PHY reset hold time (minimum 1μs per RM 66.5.1)
    pub const PHY_RESET_HOLD_TIME_US: u32 = 10;

    /// PHY power-up stabilization time (1ms per RM 66.5.1)
    pub const PHY_POWER_STABILIZATION_US: u32 = 1_000;

    /// Clock stabilization time after PLL lock (100μs per RM 14.5.3)
    pub const CLOCK_STABILIZATION_US: u32 = 100;

    /// USB PHY calibration timeout (5ms max per RM 66.5.1.5)
    pub const PHY_CALIBRATION_TIMEOUT_US: u32 = 5_000;

    // === EHCI Controller Timing ===

    /// HC reset timeout (250ms per EHCI spec 1.0)
    pub const HC_RESET_TIMEOUT_US: u32 = 250_000;

    /// Port reset timeout (50ms per USB 2.0 spec)
    pub const PORT_RESET_TIMEOUT_US: u32 = 50_000;

    /// Port reset assertion time (20ms minimum per USB 2.0 spec 7.1.7.5)
    pub const PORT_RESET_HOLD_MS: u32 = 20;

    /// Schedule enable/disable timeout (~2ms)
    pub const SCHEDULE_TIMEOUT_US: u32 = 2_000;

    /// Controller halt timeout (16 microframes = ~2ms)
    pub const HALT_TIMEOUT_US: u32 = 2_000;

    // === VBUS Power Timing ===

    /// VBUS power good delay (10ms per USB spec)
    pub const POWER_GOOD_DELAY_US: u32 = 10_000;

    /// Minimum time between overcurrent recovery attempts (1 second)
    pub const OVERCURRENT_RECOVERY_DELAY_MS: u32 = 1000;

    // === USB Protocol Timing ===

    /// Device response timeout for SET_ADDRESS (100ms per USB 2.0 spec)
    pub const DEVICE_SET_ADDRESS_TIMEOUT_MS: u32 = 100;

    /// Hub power-on delay (100ms minimum per USB 2.0 spec 11.11.1.1)
    pub const HUB_POWER_ON_DELAY_MS: u32 = 100;
}

/// Re-export CPU_FREQ_MHZ at crate root for backward compatibility
pub use timing::CPU_FREQ_MHZ;

pub mod dma;
pub mod ehci;
pub mod enumeration;
pub mod error;
pub mod perf;
pub mod phy;
pub mod recovery;
pub mod safety;
pub mod transfer;
pub mod vbus;

#[cfg(feature = "rtic-support")]
pub mod rtic;

#[cfg(test)]
mod lib_test;

#[cfg(feature = "hub")]
pub mod hub;

pub use error::{Result, UsbError};
pub use transfer::{
    BulkTransfer, BulkTransferManager, Direction, InterruptTransfer, InterruptTransferManager,
    IsochronousTransfer, IsochronousTransferManager, MicroframeTiming, TransferType,
};

// Type aliases for improved readability
/// USB device address (0-127)
pub type DeviceAddress = u8;
/// USB endpoint number (0-15, direction encoded separately)
pub type EndpointNumber = u8;
/// Maximum packet size for an endpoint
pub type MaxPacketSize = u16;
/// Transfer result type
pub type TransferResult<T = ()> = Result<T>;

use core::sync::atomic::{AtomicBool, Ordering};

/// Global flag for USB host initialization state
static USB_HOST_INITIALIZED: AtomicBool = AtomicBool::new(false);

/// USB Host controller instance
pub struct UsbHost {
    _private: (),
}

impl UsbHost {
    /// Initialize the USB host controller
    ///
    /// # Safety
    ///
    /// This function must only be called once during system initialization.
    /// Caller must ensure exclusive access to USB peripherals.
    pub unsafe fn new() -> Result<Self> {
        if USB_HOST_INITIALIZED.swap(true, Ordering::Acquire) {
            return Err(UsbError::AlreadyInitialized);
        }

        Ok(Self { _private: () })
    }
}
