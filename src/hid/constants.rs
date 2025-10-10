//! HID protocol constants
//!
//! Standard constants from USB HID Specification 1.11 and HID Usage Tables 1.12

/// HID device class code
pub const HID_CLASS: u8 = 0x03;

/// HID subclass codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidSubclass {
    /// No subclass
    None = 0x00,
    /// Boot interface subclass
    Boot = 0x01,
}

/// HID protocol codes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidProtocol {
    /// No protocol
    None = 0x00,
    /// Keyboard protocol
    Keyboard = 0x01,
    /// Mouse protocol
    Mouse = 0x02,
}

/// HID class-specific descriptor types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidDescriptorType {
    /// HID descriptor
    Hid = 0x21,
    /// Report descriptor
    Report = 0x22,
    /// Physical descriptor
    Physical = 0x23,
}

/// HID class-specific requests
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidRequest {
    /// Get report
    GetReport = 0x01,
    /// Get idle rate
    GetIdle = 0x02,
    /// Get protocol
    GetProtocol = 0x03,
    /// Set report
    SetReport = 0x09,
    /// Set idle rate
    SetIdle = 0x0A,
    /// Set protocol
    SetProtocol = 0x0B,
}

/// HID report types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidReportType {
    /// Input report
    Input = 0x01,
    /// Output report
    Output = 0x02,
    /// Feature report
    Feature = 0x03,
}

/// HID protocol modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum HidProtocolMode {
    /// Boot protocol (simplified, standard reports)
    Boot = 0x00,
    /// Report protocol (full HID report descriptors)
    Report = 0x01,
}
