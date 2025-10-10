//! HID mouse support
//!
//! Implements HID Boot Protocol mouse support (USB HID Spec 1.11, Appendix B.2)

use bitflags::bitflags;

/// HID mouse report (boot protocol)
///
/// Standard 3-byte mouse report format:
/// - Byte 0: Button states
/// - Byte 1: X displacement (signed)
/// - Byte 2: Y displacement (signed)
///
/// Some mice include additional bytes for scroll wheel and extra buttons.
/// See USB HID Specification 1.11, Appendix B.2
#[derive(Debug, Clone, Copy)]
pub struct MouseReport {
    /// Button states
    pub buttons: MouseButtons,
    /// X displacement (relative movement)
    pub x: i8,
    /// Y displacement (relative movement)
    pub y: i8,
    /// Scroll wheel displacement (if present)
    pub wheel: i8,
}

impl MouseReport {
    /// Parse report from raw data
    ///
    /// Supports both 3-byte (basic) and 4-byte (with scroll) reports.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use imxrt_usbh::hid::MouseReport;
    ///
    /// let data: [u8; 4] = [0x01, 0x05, 0xFB, 0x00];
    /// let report = MouseReport::parse(&data);
    /// // Left button pressed, moved right 5 and up 5
    /// ```
    pub fn parse(data: &[u8]) -> Self {
        assert!(data.len() >= 3, "Mouse report must be at least 3 bytes");

        Self {
            buttons: MouseButtons::from_bits_truncate(data[0]),
            x: data[1] as i8,
            y: data[2] as i8,
            wheel: if data.len() >= 4 { data[3] as i8 } else { 0 },
        }
    }

    /// Check if mouse moved
    pub fn has_movement(&self) -> bool {
        self.x != 0 || self.y != 0 || self.wheel != 0
    }

    /// Check if any button is pressed
    pub fn has_button_press(&self) -> bool {
        !self.buttons.is_empty()
    }
}

bitflags! {
    /// Mouse button states
    ///
    /// Standard buttons 1-3 are defined by the boot protocol.
    /// Buttons 4-8 may be present on extended mice.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct MouseButtons: u8 {
        /// Button 1 (left button)
        const LEFT   = 0b00000001;
        /// Button 2 (right button)
        const RIGHT  = 0b00000010;
        /// Button 3 (middle button / wheel click)
        const MIDDLE = 0b00000100;
        /// Button 4 (side button)
        const BUTTON_4 = 0b00001000;
        /// Button 5 (side button)
        const BUTTON_5 = 0b00010000;
        /// Button 6 (extra button)
        const BUTTON_6 = 0b00100000;
        /// Button 7 (extra button)
        const BUTTON_7 = 0b01000000;
        /// Button 8 (extra button)
        const BUTTON_8 = 0b10000000;
    }
}

impl MouseButtons {
    /// Check if left button is pressed
    pub fn left(&self) -> bool {
        self.contains(Self::LEFT)
    }

    /// Check if right button is pressed
    pub fn right(&self) -> bool {
        self.contains(Self::RIGHT)
    }

    /// Check if middle button is pressed
    pub fn middle(&self) -> bool {
        self.contains(Self::MIDDLE)
    }

    /// Get number of buttons pressed
    pub fn count(&self) -> u8 {
        self.bits().count_ones() as u8
    }
}
