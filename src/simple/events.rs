//! USB Host events and filtering

use crate::enumeration::DeviceClass;

/// Events that can occur during USB host operations
#[derive(Debug, Clone, Copy)]
pub enum HostEvent {
    /// Device connected and enumerated
    DeviceConnected {
        /// Device address assigned
        address: u8,
        /// Device class
        class: DeviceClass,
    },
    /// Device disconnected
    DeviceDisconnected {
        /// Address of disconnected device
        address: u8,
    },
    /// Error occurred
    Error {
        /// Device address (if known)
        device: Option<u8>,
        /// Error code
        error: crate::error::UsbError,
    },
}

/// Filter for selective device enumeration
///
/// Use this to wait for specific types of devices instead of
/// accepting any connected device.
///
/// # Example
///
/// ```no_run
/// use imxrt_usbh::simple::DeviceFilter;
/// use imxrt_usbh::enumeration::DeviceClass;
///
/// // Only accept HID keyboards
/// let filter = DeviceFilter::new()
///     .class(DeviceClass::Hid);
///
/// // Only accept specific vendor/product
/// let filter = DeviceFilter::new()
///     .vendor_id(0x046D)  // Logitech
///     .product_id(0xC077); // M105 mouse
/// ```
#[derive(Debug, Clone, Copy, Default)]
pub struct DeviceFilter {
    /// Filter by device class
    pub class: Option<DeviceClass>,
    /// Filter by vendor ID
    pub vendor_id: Option<u16>,
    /// Filter by product ID
    pub product_id: Option<u16>,
    /// Filter by subclass
    pub subclass: Option<u8>,
    /// Filter by protocol
    pub protocol: Option<u8>,
}

impl DeviceFilter {
    /// Create new empty filter (matches all devices)
    pub const fn new() -> Self {
        Self {
            class: None,
            vendor_id: None,
            product_id: None,
            subclass: None,
            protocol: None,
        }
    }

    /// Filter by device class
    pub fn class(mut self, class: DeviceClass) -> Self {
        self.class = Some(class);
        self
    }

    /// Filter by vendor ID
    pub fn vendor_id(mut self, vid: u16) -> Self {
        self.vendor_id = Some(vid);
        self
    }

    /// Filter by product ID
    pub fn product_id(mut self, pid: u16) -> Self {
        self.product_id = Some(pid);
        self
    }

    /// Filter by subclass
    pub fn subclass(mut self, subclass: u8) -> Self {
        self.subclass = Some(subclass);
        self
    }

    /// Filter by protocol
    pub fn protocol(mut self, protocol: u8) -> Self {
        self.protocol = Some(protocol);
        self
    }

    /// Check if device matches this filter
    pub fn matches(&self, device: &super::UsbDevice) -> bool {
        if let Some(class) = self.class {
            if device.class() != class {
                return false;
            }
        }
        if let Some(vid) = self.vendor_id {
            if device.vendor_id() != vid {
                return false;
            }
        }
        if let Some(pid) = self.product_id {
            if device.product_id() != pid {
                return false;
            }
        }
        // Note: subclass and protocol would need to be added to UsbDevice
        // if we want to filter by them
        true
    }
}
