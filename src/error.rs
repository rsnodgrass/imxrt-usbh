//! Simplified USB error types

use core::fmt;

/// USB operation result type
pub type Result<T> = core::result::Result<T, UsbError>;

/// Simplified USB error types - focused on recovery actions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UsbError {
    // Initialization errors (non-recoverable)
    /// Already initialized
    AlreadyInitialized,
    /// Hardware failure (PHY/PLL/calibration)
    HardwareFailure,
    
    // Connection errors (recoverable by re-enumeration)
    /// Device disconnected or not responding
    DeviceDisconnected,
    /// Port error or power issue
    PortError,
    
    // Transfer errors (recoverable by retry)
    /// Transfer timeout
    Timeout,
    /// Device NAK'd - retry later
    Nak,
    /// USB stall - need endpoint clear
    Stall,
    /// Transaction error (CRC, babble, etc)
    TransactionError,
    
    // Resource errors (may recover when resources free)
    /// No available descriptors/buffers
    NoResources,
    
    // Programming errors (non-recoverable)
    /// Invalid parameter or state
    InvalidParameter,
    /// Invalid state for operation
    InvalidState,
    /// Unsupported operation
    Unsupported,
    
    // Data errors (non-recoverable for this transfer)
    /// Invalid USB descriptor
    InvalidDescriptor,
    /// Buffer overflow
    BufferOverflow,
}

impl fmt::Display for UsbError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlreadyInitialized => write!(f, "Already initialized"),
            Self::HardwareFailure => write!(f, "Hardware failure"),
            Self::DeviceDisconnected => write!(f, "Device disconnected"),
            Self::PortError => write!(f, "Port error"),
            Self::Timeout => write!(f, "Timeout"),
            Self::Nak => write!(f, "NAK"),
            Self::Stall => write!(f, "Stall"),
            Self::TransactionError => write!(f, "Transaction error"),
            Self::NoResources => write!(f, "No resources"),
            Self::InvalidParameter => write!(f, "Invalid parameter"),
            Self::InvalidState => write!(f, "Invalid state"),
            Self::Unsupported => write!(f, "Unsupported"),
            Self::InvalidDescriptor => write!(f, "Invalid descriptor"),
            Self::BufferOverflow => write!(f, "Buffer overflow"),
        }
    }
}

impl UsbError {
    /// Check if error is recoverable by retry
    pub fn is_retryable(&self) -> bool {
        matches!(self, 
            Self::Timeout | 
            Self::Nak | 
            Self::TransactionError |
            Self::NoResources
        )
    }
    
    /// Check if error requires re-enumeration
    pub fn needs_reenumeration(&self) -> bool {
        matches!(self,
            Self::DeviceDisconnected |
            Self::PortError
        )
    }
    
    /// Check if error requires endpoint clear
    pub fn needs_endpoint_clear(&self) -> bool {
        matches!(self, Self::Stall)
    }
    
    /// Get recommended retry delay in milliseconds
    pub fn retry_delay_ms(&self) -> Option<u32> {
        match self {
            Self::Nak => Some(1),              // NAK: retry quickly
            Self::Timeout => Some(10),         // Timeout: small backoff
            Self::TransactionError => Some(5), // Transaction: medium retry
            Self::NoResources => Some(50),     // Resources: wait longer
            _ => None,
        }
    }
}