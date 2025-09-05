//! USB error types

use core::fmt;

/// USB operation result type
pub type Result<T> = core::result::Result<T, UsbError>;

/// USB error types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UsbError {
    /// USB host already initialized
    AlreadyInitialized,
    /// Device not found or disconnected
    DeviceDisconnected,
    /// USB stall condition
    Stall,
    /// Data toggle mismatch
    DataToggleMismatch,
    /// CRC error in data
    CrcError,
    /// Timeout waiting for response
    Timeout,
    /// Babble detected (device sent too much data)
    Babble,
    /// Transaction error (CRC, timeout, bad PID)
    TransactionError,
    /// Missed microframe
    MissedMicroframe,
    /// Host system error (DMA error)
    HostSystemError,
    /// Buffer overflow
    BufferOverflow,
    /// Buffer underflow
    BufferUnderflow,
    /// Invalid parameter
    InvalidParameter,
    /// Invalid state for operation
    InvalidState,
    /// No available resources (descriptors, buffers)
    NoResources,
    /// VBUS over-current condition
    VbusOverCurrent,
    /// VBUS power failure
    VbusPowerFailure,
    /// Unsupported operation
    Unsupported,
    /// Device NAK'd transaction
    Nak,
    /// Invalid descriptor
    InvalidDescriptor,
    /// Endpoint halted
    EndpointHalted,
    /// Port error
    PortError,
}

impl fmt::Display for UsbError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::AlreadyInitialized => write!(f, "USB host already initialized"),
            Self::DeviceDisconnected => write!(f, "Device disconnected"),
            Self::Stall => write!(f, "USB stall"),
            Self::DataToggleMismatch => write!(f, "Data toggle mismatch"),
            Self::CrcError => write!(f, "CRC error"),
            Self::Timeout => write!(f, "Timeout"),
            Self::Babble => write!(f, "Babble detected"),
            Self::TransactionError => write!(f, "Transaction error"),
            Self::MissedMicroframe => write!(f, "Missed microframe"),
            Self::HostSystemError => write!(f, "Host system error"),
            Self::BufferOverflow => write!(f, "Buffer overflow"),
            Self::BufferUnderflow => write!(f, "Buffer underflow"),
            Self::InvalidParameter => write!(f, "Invalid parameter"),
            Self::InvalidState => write!(f, "Invalid state"),
            Self::NoResources => write!(f, "No resources available"),
            Self::VbusOverCurrent => write!(f, "VBUS over-current"),
            Self::VbusPowerFailure => write!(f, "VBUS power failure"),
            Self::Unsupported => write!(f, "Unsupported operation"),
            Self::Nak => write!(f, "Device NAK"),
            Self::InvalidDescriptor => write!(f, "Invalid descriptor"),
            Self::EndpointHalted => write!(f, "Endpoint halted"),
            Self::PortError => write!(f, "Port error"),
        }
    }
}