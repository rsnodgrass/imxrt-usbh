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

pub mod ehci;
pub mod phy;
pub mod dma;
pub mod error;
pub mod vbus;
pub mod transfer;
pub mod perf;
pub mod recovery;
pub mod safety;
pub mod enumeration;

#[cfg(feature = "rtic-support")]
pub mod rtic;

#[cfg(test)]
mod lib_test;

#[cfg(feature = "hub")]
pub mod hub;

pub use error::{UsbError, Result};
pub use transfer::{
    BulkTransfer, BulkTransferManager, 
    InterruptTransfer, InterruptTransferManager,
    IsochronousTransfer, IsochronousTransferManager, MicroframeTiming,
    Direction, TransferType
};

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
