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
        matches!(
            self,
            Self::Timeout | Self::Nak | Self::TransactionError | Self::NoResources
        )
    }

    /// Check if error requires re-enumeration
    pub fn needs_reenumeration(&self) -> bool {
        matches!(self, Self::DeviceDisconnected | Self::PortError)
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

    /// Get human-readable description with troubleshooting steps
    ///
    /// This provides educational context for each error, helping users
    /// understand what went wrong and how to fix it.
    ///
    /// # Example
    ///
    /// ```rust
    /// use imxrt_usbh::error::UsbError;
    ///
    /// let error = UsbError::DeviceDisconnected;
    /// println!("Error: {}", error.description_with_help());
    /// // Prints: "Device disconnected. Check USB cable connection and device power."
    /// ```
    pub fn description_with_help(&self) -> &'static str {
        match self {
            Self::AlreadyInitialized =>
                "USB controller already initialized. Only one instance allowed at a time.",
            Self::HardwareFailure =>
                "USB hardware failure. Check power supply and crystal oscillator. Reset may be required.",
            Self::DeviceDisconnected =>
                "Device disconnected. Check USB cable connection and device power.",
            Self::PortError =>
                "USB port error. Device may be drawing too much power or cable is faulty.",
            Self::Timeout =>
                "Operation timed out. Device may be unresponsive. Try re-enumeration or check device compatibility.",
            Self::Nak =>
                "Device sent NAK (not ready). This is normal - the library will retry automatically.",
            Self::Stall =>
                "Device sent STALL. Feature may be unsupported or endpoint needs to be cleared.",
            Self::TransactionError =>
                "USB transaction error. Check signal integrity - cable may be too long or noisy.",
            Self::NoResources =>
                "No available USB resources. Too many devices connected or memory exhausted.",
            Self::InvalidParameter =>
                "Invalid parameter. Check endpoint addresses and buffer sizes.",
            Self::InvalidState =>
                "Invalid USB state. Device may need to be re-enumerated.",
            Self::Unsupported =>
                "Unsupported operation. Feature not implemented or device incompatible.",
            Self::InvalidDescriptor =>
                "Invalid USB descriptor. Device may be faulty or use non-standard descriptors.",
            Self::BufferOverflow =>
                "Buffer overflow. Increase buffer size or check device documentation for data sizes.",
        }
    }

    /// Get the error category for organizational purposes
    ///
    /// This helps classify errors by their nature, which is useful for
    /// error handling strategies and debugging.
    pub fn category(&self) -> ErrorCategory {
        match self {
            Self::AlreadyInitialized | Self::HardwareFailure => ErrorCategory::Initialization,
            Self::DeviceDisconnected | Self::PortError => ErrorCategory::Connection,
            Self::Timeout | Self::Nak | Self::Stall | Self::TransactionError => {
                ErrorCategory::Transfer
            }
            Self::NoResources => ErrorCategory::Resource,
            Self::InvalidParameter | Self::InvalidState | Self::Unsupported => {
                ErrorCategory::Programming
            }
            Self::InvalidDescriptor | Self::BufferOverflow => ErrorCategory::Data,
        }
    }

    /// Get suggested next steps for error resolution
    ///
    /// This provides concrete actions a user can take to resolve or work around the error.
    pub fn suggested_actions(&self) -> &'static [&'static str] {
        match self {
            Self::AlreadyInitialized => &[
                "Ensure only one SimpleUsbHost instance exists",
                "Check if USB was already initialized elsewhere in your code",
            ],
            Self::HardwareFailure => &[
                "Check 5V power supply stability",
                "Verify USB host cable is connected to Teensy pins 30/31",
                "Reset the Teensy board",
                "Check for hardware damage",
            ],
            Self::DeviceDisconnected => &[
                "Check USB cable connection",
                "Verify device is powered on",
                "Try a different USB cable",
                "Check if device is compatible with USB 2.0",
            ],
            Self::PortError => &[
                "Check if device is drawing too much power (>500mA)",
                "Try a different USB cable",
                "Verify VBUS power switching circuit",
            ],
            Self::Timeout => &[
                "Try re-enumerating the device",
                "Check device compatibility",
                "Increase timeout values if using low-level API",
                "Verify device is responding to other requests",
            ],
            Self::Nak => &[
                "This is normal - device is busy",
                "Library will retry automatically",
                "If persistent, check device documentation",
            ],
            Self::Stall => &[
                "Check if endpoint/feature is supported",
                "Consult device documentation",
                "Try clearing the endpoint halt condition",
                "Verify request parameters are correct",
            ],
            Self::TransactionError => &[
                "Check USB cable quality and length",
                "Reduce electromagnetic interference",
                "Verify signal integrity",
                "Try a different USB port or cable",
            ],
            Self::NoResources => &[
                "Disconnect unused devices",
                "Reduce number of concurrent transfers",
                "Check for memory leaks in your code",
            ],
            Self::InvalidParameter => &[
                "Check endpoint addresses (0x81, 0x02, etc.)",
                "Verify buffer sizes match device requirements",
                "Consult device documentation for valid parameters",
            ],
            Self::InvalidState => &[
                "Re-enumerate the device",
                "Check device state before operations",
                "Verify initialization sequence was completed",
            ],
            Self::Unsupported => &[
                "Check if feature is implemented in this library version",
                "Consult device documentation for alternative approaches",
                "Consider using low-level API if available",
            ],
            Self::InvalidDescriptor => &[
                "Device may be faulty or non-compliant",
                "Try a different device for testing",
                "Check device documentation for known issues",
            ],
            Self::BufferOverflow => &[
                "Increase buffer size in your code",
                "Check device documentation for maximum data sizes",
                "Verify endpoint maximum packet sizes",
            ],
        }
    }
}

/// Error categories for organizational and handling purposes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ErrorCategory {
    /// Initialization-related errors
    Initialization,
    /// Connection and port-related errors
    Connection,
    /// Transfer operation errors
    Transfer,
    /// Resource availability errors
    Resource,
    /// Programming/parameter errors
    Programming,
    /// Data validation errors
    Data,
}
