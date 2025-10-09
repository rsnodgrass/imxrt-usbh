//! USB transfer management and state machines
//!
//! Implements control, bulk, interrupt, and isochronous transfer types.

use crate::error::UsbError;
use core::sync::atomic::{AtomicU8, Ordering};

pub mod bulk;
pub mod control;
pub mod interrupt;
pub mod isochronous;
pub mod simple_control;

/// Trait for transfer state enums with atomic storage
///
/// Reduces boilerplate for state conversions across transfer types.
/// All transfer state enums (BulkState, InterruptState, etc.) can implement this.
pub trait TransferState: Copy + Sized {
    /// Convert from u8 representation
    fn from_u8(val: u8) -> Self;

    /// Convert to u8 representation
    fn to_u8(self) -> u8;

    /// Load state from atomic storage
    fn load_from(atomic: &AtomicU8) -> Self {
        Self::from_u8(atomic.load(Ordering::Acquire))
    }

    /// Store state to atomic storage
    fn store_into(self, atomic: &AtomicU8) {
        atomic.store(self.to_u8(), Ordering::Release);
    }
}

pub use bulk::{bulk_endpoint, BulkState, BulkTransfer, BulkTransferManager};
pub use interrupt::{
    interrupt_endpoint, InterruptState, InterruptTransfer, InterruptTransferManager,
};
pub use isochronous::{
    isochronous_endpoint, IsochronousState, IsochronousTransfer, IsochronousTransferManager,
    MicroframeTiming,
};

/// USB transfer types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum TransferType {
    Control,
    Bulk,
    Interrupt,
    Isochronous,
}

/// Transfer direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum Direction {
    In,
    Out,
}

/// USB Setup packet for control transfers
#[repr(C)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub struct SetupPacket {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

impl SetupPacket {
    /// Create GET_DESCRIPTOR request
    pub fn get_descriptor(desc_type: u8, desc_index: u8, language_id: u16, length: u16) -> Self {
        Self {
            request_type: 0x80, // Device-to-host, standard, device
            request: 0x06,      // GET_DESCRIPTOR
            value: ((desc_type as u16) << 8) | (desc_index as u16),
            index: language_id,
            length,
        }
    }

    /// Create SET_ADDRESS request
    pub fn set_address(address: u8) -> Self {
        Self {
            request_type: 0x00, // Host-to-device, standard, device
            request: 0x05,      // SET_ADDRESS
            value: address as u16,
            index: 0,
            length: 0,
        }
    }

    /// Create SET_CONFIGURATION request
    pub fn set_configuration(configuration: u8) -> Self {
        Self {
            request_type: 0x00, // Host-to-device, standard, device
            request: 0x09,      // SET_CONFIGURATION
            value: configuration as u16,
            index: 0,
            length: 0,
        }
    }

    /// Check if this is an IN transfer
    pub fn is_in(&self) -> bool {
        (self.request_type & 0x80) != 0
    }
}

/// Control transfer state machine states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[allow(missing_docs)]
pub enum ControlState {
    Setup { attempt: u8 },
    DataIn { remaining: usize, attempt: u8 },
    DataOut { remaining: usize, attempt: u8 },
    Status { attempt: u8 },
    Complete,
    Failed(UsbError),
}

/// Placeholder for future implementation
#[allow(missing_docs)]
pub struct TransferManager {
    // Will contain qTD/qH management
}

impl TransferManager {
    pub const fn new() -> Self {
        Self {}
    }
}
